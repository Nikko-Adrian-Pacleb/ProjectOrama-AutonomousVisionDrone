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

// ====================== IMU STATE =======================
#define ANGLE_UPDATE 0x04
static volatile char s_cDataUpdate = 0;
static float pitchDeg = 0.0f;
static float rollDeg = 0.0f;
static float yawRate = 0.0f;

// ====================== RC ARMING =======================
uint32_t haveSignalMs = 0, lostSignalMs = 0;

// ======================= HELPERS ========================
static inline float microsToSec(uint32_t us) {return us * 1e-6f;}
static inline float angleFromRaw(int16_t raw) {return raw * 180.0f / 32768.0f;}
static inline float gyroFromRaw(int16_t raw) {return raw * 2000.0f / 32768.0f;}

// ======================== SETUP =========================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println(F("\n== Drone Object + WitMotion IMU =="));

  // Drone object functions to start
  quad.begin(MOTOR_FL, MOTOR_FR, MOTOR_BL, MOTOR_BR, &Serial);
  quad.setDebug(true);

  // Start disarmed
  quad.disarm();

  pinMode(RC_THROTTLE_PIN, INPUT);

  Wire.begin();
  Wire.setClock(400000);
  WitI2cFuncRegister(IICreadBytes, IICwriteBytes);
  WitDelayMsRegister(Delayms);
  WitRegisterCallBack(CopeSensorData);

  // Default address 0x68; if can't find use autoscan
  WitInit(WIT_PROTOCOL_I2C, 0x68);
  Delayms(50);
  s_cDataUpdate = 0;
  WitReadReg(AX, 12);
  Delayms(50);
  if (!s_cDataUpdate) {
    Serial.println(F("[IMU] Auto-Scan..."));
    AutoScanSensor();
  } else {
    Serial.println(F("[IMU] Found at 0x68"));
  }
}

// ======================== LOOP ==========================
void loop() {
  static uint32_t lastMicros = micros();
  uint32_t now = micros();
  float dt = microsToSec(now - lastMicros);
  lastMicros = now;
  if (dt <= 0.0f || dt > 0.1f) dt = 0.005f; // safety (cap dt)

  // ----- IMU Poll -----
  static uint32_t lastImu = 0;
  if (millis() - lastImu >= 5) {
    lastImu = millis();
    if (WitReadReg(Roll, 3) == WIT_HAL_OK) {
      rollDeg = angleFromRaw(sReg[Roll]);
      pitchDeg = angleFromRaw(sReg[Pitch]);
    }
    if (witReadReg(GX, 3) == WIT_HAL_OK) {
      yawRate = gyroFromRaw(sReg[GZ]);
    }
  }

  // --- Read RC throttle pulse (blocking but simple) ---
  // Expect ~1000–2000 µs. Timeout 25ms to avoid long stalls.
  unsigned long pulse = pulseIn(RC_THROTTLE_PIN, HIGH, 25000UL);

  bool hasSignal = (pulse >= 900 && pulse <= 2100);
  uint32_t tNow = millis();

  if (hasSignal) {
    haveSignalMs += (uint16_t) (dt * 1000.0f);
    lostSignalMs = 0;
  } else {
    lostSignalMs += (uint16_t) (dt * 1000.0f);
    haveSignalMs = 0;
  }

  // --- Arming/disarming conditions ---
  if (!quad.isArmed() && hasSignal && haveSignalMs > 1000) {
    quad.arm();
  }
  if (quad.isArmed() && (!hasSignal || lostSignalMs > 500)) {
    quad.disarm();
  }

  // Map throttle pulse to ESC µs with a floor to keep props from spinning when low
  int throttleUs = 1000; // default min
  if (hasSignal) {
    // deadband below 1050, cap at 1900 by default (safer while tuning)
    if      (pulse < 1050) throttleUs = 1000;
    else if (pulse > 1500) throttleUs = 1500;
    else                   throttleUs = (int)pulse;
  }
  quad.setBaseThrottle(throttleUs);

  // SetPoints
  // For now we hold level (0°, 0°) and zero yaw-rate:
  quad.setSetpoints(0.0f, 0.0f, 0.0f);

  //update mixer and PID
  quad.update(pitchDeg, rollDeg, yawRate, dt);

  // Optional: lightweight heartbeat debug
  static uint32_t lastBeat = 0;
  if (millis() - lastBeat > 500) {
    lastBeat = millis();
    Serial.print(F("[beat] RC=")); Serial.print(hasSignal ? (int)pulse : -1);
    Serial.print(F("  THR=")); Serial.print(throttleUs);
    Serial.print(F("  ARM=")); Serial.println(quad.isArmed() ? 1 : 0);
    Serial.print(F(" IMU P,R,Yr="));
    Serial.print(pitchDeg,1); Serial.print(',');
    Serial.print(rollDeg,1); Serial.print(',');
    Serial.print(yawRate,1);
  }
}


// ================= WITMOTION FUNCTIONS ==================
static void AutoScanSensor(void) {
  for (int i=0;i<0x7F;i++) {
    WitInit(WIT_PROTOCOL_I2C, i);
    int retry=2;
    do {
      s_cDataUpdate=0;
      WitReadReg(AX,3);
      delay(5);
      if (s_cDataUpdate) {
        Serial.print(F("Found IMU @ 0x")); Serial.println(i,HEX);
        return;
      }
    } while(retry--);
  }
  Serial.println(F("IMU not found"));
}

static void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum) {
  if (uiReg == Yaw) s_cDataUpdate |= ANGLE_UPDATE;
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





