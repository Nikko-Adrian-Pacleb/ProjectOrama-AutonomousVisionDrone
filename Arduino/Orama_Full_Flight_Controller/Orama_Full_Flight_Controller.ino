// Used for minimal PID test
// Takes in throttle input from a single channel from controller
// mixes throttle input to 4 ESC's on pins 2,3,4,5
// Serial is set up 115200

// ====================== LIBRARIES =======================
#include "Drone.h"
#include <Wire.h>
#include <Servo.h>
#include "REG.h"
#include "wit_c_sdk.h"

// ======================== PINS ==========================
#define MOTOR_FL 2
#define MOTOR_FR 3
#define MOTOR_BL 4
#define MOTOR_BR 5

#define RC_THROTTLE_PIN 9

// ==================== DRONE OBJECT ======================
Drone quad;

// =============== IMU STATE & VARIABLES ==================
#define ACC_UPDATE   0x01
#define GYRO_UPDATE  0x02
#define ANGLE_UPDATE 0x04
#define READ_UPDATE  0x80

static volatile uint8_t s_cDataUpdate = 0;
static uint8_t s_cCmd = 0xff;

static float pitchDeg = 0.0f; // +nose up
static float rollDeg  = 0.0f; // +right wing down
static float yawRate  = 0.0f; // °/s (use gyro Z)

float fAcc[3]   = {0,0,0};  // g
float fGyro[3]  = {0,0,0};  // °/s
float fAngle[3] = {0,0,0};  // roll, pitch, yaw in °

// ====================== RC ARMING =======================
uint32_t haveSignalMs = 0, lostSignalMs = 0;

// ======================= HELPERS ========================
static inline float microsToSec(uint32_t us){ return us * 1e-6f; }
static inline float angleFromRaw(int16_t raw){ return raw * 180.0f  / 32768.0f; }
static inline float gyroFromRaw (int16_t raw){ return raw * 2000.0f / 32768.0f; }
static inline float accFromRaw  (int16_t raw){ return raw * 16.0f   / 32768.0f; }

// ================ Pre-Defined Functions==================
static void CmdProcess(void);
static void AutoScanSensor(void);
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
static void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum);
static int32_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t *data, uint32_t length);
static int32_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t* data, uint32_t length);
static void Delayms(uint16_t ucMs);

// ======================== SETUP =========================
void setup() {
  Wire.begin();
  Wire.setClock(100000);               // standard I2C, safer for many JY/Wit modules
#if defined(TWBR) || ARDUINO >= 10607
  Wire.setTimeout(5000);
#endif
  Serial.begin(115200);

  quad.begin(MOTOR_FL, MOTOR_FR, MOTOR_BL, MOTOR_BR, &Serial);
  quad.setDebug(true);
  quad.disarm();

  pinMode(RC_THROTTLE_PIN, INPUT);

  // Register I2C functions and callbacks BEFORE init/scan
  WitI2cFuncRegister(IICwriteBytes, IICreadBytes);
  WitRegisterCallBack(CopeSensorData);
  WitDelayMsRegister(Delayms);

  AutoScanSensor();
}

// ======================== LOOP ==========================
void loop() {
  static uint32_t lastMicros = micros();
  uint32_t now = micros();
  float dt = microsToSec(now - lastMicros);
  lastMicros = now;
  if (dt <= 0.0f || dt > 0.1f) dt = 0.005f; // cap dt

  // ----- IMU Poll (non-blocking) -----
  static uint32_t lastImu = 0;
  if (millis() - lastImu >= 5) {             // ~200 Hz
    lastImu = millis();
    // Read Acc + Gyro (AX..AZ, GX..GZ) then Angles (Roll..Yaw)
    WitReadReg(AX,   6);  // 3 regs (X,Y,Z) = 6 bytes each group handled by SDK
    WitReadReg(GX,   6);
    WitReadReg(Roll, 3);  // roll, pitch, yaw
  }

  // If any new data landed, convert using your scale factors
  if (s_cDataUpdate & (ACC_UPDATE | GYRO_UPDATE | ANGLE_UPDATE)) {
    // Convert Acc
    if (s_cDataUpdate & ACC_UPDATE) {
      fAcc[0] = accFromRaw((int16_t)sReg[AX+0]);
      fAcc[1] = accFromRaw((int16_t)sReg[AX+1]);
      fAcc[2] = accFromRaw((int16_t)sReg[AX+2]);
    }
    // Convert Gyro
    if (s_cDataUpdate & GYRO_UPDATE) {
      fGyro[0] = gyroFromRaw((int16_t)sReg[GX+0]);
      fGyro[1] = gyroFromRaw((int16_t)sReg[GX+1]);
      fGyro[2] = gyroFromRaw((int16_t)sReg[GX+2]);
    }
    // Convert Euler angles
    if (s_cDataUpdate & ANGLE_UPDATE) {
      fAngle[0] = -angleFromRaw((int16_t)sReg[Roll+0]);  // roll
      fAngle[1] = angleFromRaw((int16_t)sReg[Roll+1]);  // pitch
      fAngle[2] = angleFromRaw((int16_t)sReg[Roll+2]);  // yaw
    }

    // Drive the control signals you actually use
    rollDeg  = fAngle[0];
    pitchDeg = fAngle[1];
    yawRate  = fAngle[2];   // use gyro Z as yaw rate

    // clear the bits we just consumed
    s_cDataUpdate &= ~(ACC_UPDATE | GYRO_UPDATE | ANGLE_UPDATE);
  }

  // --- Read RC throttle pulse (blocking but simple) ---
  unsigned long pulse = pulseIn(RC_THROTTLE_PIN, HIGH, 25000UL); // timeout 25ms
  bool hasSignal = (pulse >= 900 && pulse <= 2100);
  uint32_t tNow = millis();

  if (hasSignal) {
    haveSignalMs += (uint16_t)(dt * 1000.0f);
    lostSignalMs = 0;
  } else {
    lostSignalMs += (uint16_t)(dt * 1000.0f);
    haveSignalMs = 0;
  }

  // --- Arming/disarming conditions ---
  if (!quad.isArmed() && hasSignal && haveSignalMs > 1000) {
    quad.arm();
  }
  if (quad.isArmed() && (!hasSignal || lostSignalMs > 500)) {
    quad.disarm();
  }
quad.arm();
  // Map throttle pulse to ESC µs
  int throttleUs = 1000;
  if (hasSignal) {
    if      (pulse < 1050) throttleUs = 1000;  // deadband
    else if (pulse > 1500) throttleUs = 1500;  // cap while tuning
    else                   throttleUs = (int)pulse;
  }
  quad.setBaseThrottle(throttleUs);

  // Level setpoints and zero yaw-rate
  quad.setSetpoints(0.0f, 0.0f, 0.0f);

  // Update mixer and PID with latest IMU values
  quad.update(pitchDeg, rollDeg, yawRate, dt);

  // Optional: heartbeat
  // static uint32_t lastBeat = 0;
  // if (millis() - lastBeat > 500) {
  //   lastBeat = millis();
  //   Serial.print(F("[beat] RC=")); Serial.print(hasSignal ? (int)pulse : -1);
  //   Serial.print(F("  THR=")); Serial.print(throttleUs);
  //   Serial.print(F("  ARM=")); Serial.println(quad.isArmed() ? 1 : 0);
  //   Serial.print(F(" IMU P,R,Yr="));
  //   Serial.print(pitchDeg,1); Serial.print(',');
  //   Serial.print(rollDeg,1);  Serial.print(',');
  //   Serial.println(yawRate,1);
  // }
}


// --------------- Camera Control Functions --------------- //
int16_t smoothAngle(int16_t newAngle) {
   return (int16_t)((3L * newAngle + 7L * smoothedAngle) / 10L);  // alpha = 0.3
}

/** This function runs the PID through a custom made PID. 
  * @param target is the number that we want the pid value to reach
  * @param current is the number that the value is currently.
  */
int16_t pidControl(int16_t target, int16_t current) {
  unsigned long now = millis();
  uint16_t dtMs = now - lastTime;
  if (dtMs == 0) dtMs = 1;
  lastTime = now;

  int16_t error = target - current;

  integral += (int32_t)error * dtMs;                                             // integrate over time
  int16_t derivative = (int16_t)(((int32_t)(error - prevError) * 1000) / dtMs);  // scale derivative

  int32_t output = (int32_t)gkp * error / 10 + (int32_t)gki * integral / 100000 + (int32_t)gkd * derivative / 100;

  prevError = error;

  return (int16_t)output;
}

// --------------- TOF Sensor Functions --------------- //
/** @brief This function starts up each TOF sensor using the multiplexer
  * @param range_mode chooses the precision and range of the sensor
  * @param high_speed Chooses true or false on whether you want to use high speed mode
  * @param sensor This is the sensor object that will contain the info for each sensor.
  */
void sensor_init(VL53L1X::DistanceMode range_mode, bool high_speed, VL53L1X sensor) {
  Wire.begin();
  // Start ranging on all sensors
  for (int i = 0; i < 4; i++) {
    sensors[i].startContinuous(50); // 50ms interval
  }
}

uint16_t* getAllTOFSensorData() {
  for (int i = 1; i <= 4; i++) {
    TOFData[i] = sensors[i].read();
  }
  return TOFData;
}

// --------------- LiDAR Sensor Functions --------------- //
/** Starts and connects the LiDAR through UART.
 * @return int available - number that is either 0 or 1 to tell if the serial is off or running.
*/
int lidar_init() {
  Serial1.begin(115200);

  if(Serial1.available()) {
    if (Serial1.read() == HEADER) {
      uart[0] = HEADER;
      if (Serial1.read() == HEADER) {
        uart[1] = HEADER;
        return 1;
      }
    }
  }
  return 0;
}

int getLiDARData() {
  if(Serial1.available()) {
    if (Serial1.read() == HEADER) {
      uart[0] = HEADER;
      if (Serial1.read() == HEADER) {
        uart[1] = HEADER;
        for (int i = 2; i < 9; i++) uart[i] = Serial1.read();
        for (int i = 0; i < 8; i++) check += uart[i];
        if (uart[8] == (check & 0xff)) {
          dist = uart[2] + uart[3] * 256;
          //strength = uart[4] + uart[5] * 256;  //if we want to use this we can but right now just using the distance.
        }
      }
    }
  }
  return dist;

// ================= WITMOTION FUNCTIONS ==================
static void AutoScanSensor(void) {
  for (int i = 0; i < 0x7F; i++) {
    WitInit(WIT_PROTOCOL_I2C, i);
    int retry = 2;
    do {
      s_cDataUpdate = 0;
      WitReadReg(AX, 3);     // try reading some accel
      Delayms(5);
      if (s_cDataUpdate & ACC_UPDATE) {
        Serial.print(F("Found IMU @ 0x")); Serial.println(i, HEX);
        return;
      }
    } while (retry--);
  }
  Serial.println(F("IMU not found"));
}

static void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum) {
  // Set flags based on which block the SDK says was updated
  // (REG.h provides AX,GX,Roll… enumerations, contiguous for XYZ)
  if (uiReg == AX)   s_cDataUpdate |= ACC_UPDATE;
  if (uiReg == GX)   s_cDataUpdate |= GYRO_UPDATE;
  if (uiReg == Roll) s_cDataUpdate |= ANGLE_UPDATE;
}

static int32_t IICreadBytes(uint8_t dev,uint8_t reg,uint8_t *data,uint32_t length){
  Wire.beginTransmission(dev);
  Wire.write(reg);
  if(Wire.endTransmission(false)!=0) return 0;
  Wire.requestFrom(dev,(uint8_t)length);
  unsigned long start=millis();
  while(Wire.available()<(int)length){ if(millis()-start>100) return 0; }
  for(uint32_t i=0;i<length;i++) data[i]=Wire.read();
  return 1;
}

static int32_t IICwriteBytes(uint8_t dev,uint8_t reg,uint8_t *data,uint32_t length){
  Wire.beginTransmission(dev);
  Wire.write(reg);
  Wire.write(data,length);
  return Wire.endTransmission()==0;
}

static void Delayms(uint16_t ms){ delay(ms); }
