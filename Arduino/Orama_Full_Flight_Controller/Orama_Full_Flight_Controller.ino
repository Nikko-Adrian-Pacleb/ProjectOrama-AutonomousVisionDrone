#include <Wire.h>       // 5% of dynamic memory (113 bytes)
#include <Servo.h>      // 2% of dynamic memory (37 bytes)
#include "REG.h"        // so far doesn't use any program or SRAM space
#include "wit_c_sdk.h"  // so far doesn't use any program or SRAM space

// ###################### DEBUG VARIABLE ########################
bool DEBUG = false;
// ##############################################################


Servo gimbalServo;  // use 264 bytes program space and 4 bytes of SRAM
#define SERVO_PIN 13

// IMU data update flags
#define ACC_UPDATE 0x01
#define GYRO_UPDATE 0x02
#define ANGLE_UPDATE 0x04
#define MAG_UPDATE 0x08
#define READ_UPDATE 0x80
static char s_cDataUpdate = 0, s_cCmd = 0xff;

// pins for the motors and analog switch between manual and autonomous
#define FRONT_LEFT_MOTOR 2
#define FRONT_RIGHT_MOTOR 3
#define BACK_LEFT_MOTOR 4
#define BACK_RIGHT_MOTOR 5
#define MANUAL_SWITCH 6

// Pins for the reciever
#define CH1 7
#define CH2 8
#define CH3 9
#define CH4 10
#define CH5 11
#define CH6 12


// variables used to replace all delay functions and just use deltaTime
unsigned long lastSensorRead = 0;
const unsigned long sensorInterval = 20; // ms

// Gimbal PID Controller parameters
int16_t gkp = 2.2;    // Proportional gain
int16_t gki = 0.105;  // Integral gain
int16_t gkd = 0.04;   // Derivative gain

// Motor PID Controller parameters
int16_t mkp = 2.2;    // Proportional gain
int16_t mki = 0.105;  // Integral gain
int16_t mkd = 0.04;   // Derivative gain

int32_t integral = 0;
int16_t prevError = 0;
unsigned long lastTime = 0;

//IMU data for Gimbal
int16_t rollAngle = 0;      // tenths of degrees
int16_t smoothedAngle = 0;  // exponential smoothing


void setup() {
  Wire.begin();           // uses 174 bytes of program space along with 69 bytes of SRAM
  Wire.setClock(400000);  // uses 6 bytes of program space
  Serial.begin(115200);   // uses 970 bytes of program space along with 175 bytes of SRAM

  gimbalServo.attach(SERVO_PIN, 500, 2400);  // uses 254 bytes of program space along with 0 bytes of SRAM
  gimbalServo.write(90);              // sets camera at level // uses 356 bytes of program space along with 0 bytes of SRAM

  WitInit(WIT_PROTOCOL_I2C, 0x68); // uses 0 of both
  WitI2cFuncRegister(IICwriteBytes, IICreadBytes); // uses 1434 bytes of program space along with 34 bytes of SRAM
  WitRegisterCallBack(CopeSensorData); // uses 22 bytes of program space along with 1 bytes of SRAM
  WitDelayMsRegister(Delayms); // uses 104 bytes of program space along with 0 bytes of SRAM

  //delay(100); // uses 20 bytes of program space along with 0 bytes of SRAM
  //WitReadReg(AX, 12); // uses 106 bytes of program space along with 266 bytes of SRAM
  delay(1000);
  WitReadReg(AX, 12);
  delay(100);
  if(s_cDataUpdate == 0) {
    Serial.println("Sensor not found at 0x34, trying auto-scan...");
    AutoScanSensor();
  }
}

void loop() {
  
  unsigned long now = millis();

  // Sensor read timing using delta time not delays
  if (now - lastSensorRead >= sensorInterval) {
    lastSensorRead = now; // uses 4 bytes of SRAM
    WitReadReg(AX, 12); // only uses program space because it has been called before.
  }

  // only checks the sensor when the delta time is correct. Allows for the program to keep running. This is for the camera gimbal.
  if (s_cDataUpdate) {
    rollAngle = (int16_t)((sReg[Roll] * 1800L) / 32768L); // tenths of degrees --------------------- uses 288 bytes of SRAM ----------------------------------------
    smoothedAngle = smoothAngle(rollAngle);
    int16_t correction = pidControl(0, smoothedAngle);
    int16_t servoAngle = 900 - correction;

    if (servoAngle < 450) servoAngle = 450;
    if (servoAngle > 1800) servoAngle = 1800;

    gimbalServo.write(servoAngle / 10);
    s_cDataUpdate = 0;
  }

}

static void AutoScanSensor(void) {
  int i, iRetry;
  
  for(i = 0; i < 0x7F; i++) {
    WitInit(WIT_PROTOCOL_I2C, i);
    iRetry = 2;
    do {
      s_cDataUpdate = 0;
      WitReadReg(AX, 3);
      delay(5);
      if(s_cDataUpdate != 0) {
        Serial.print("find 0x");
        Serial.print(i, HEX);
        Serial.print(" addr sensor\r\n");
        return;
      }
      iRetry--;
    }while(iRetry);		
  }
  Serial.print("can not find sensor\r\n");
  Serial.print("please check your connection\r\n");
}

int16_t smoothAngle(int16_t newAngle) {
   return (int16_t)((3L * newAngle + 7L * smoothedAngle) / 10L);  // alpha = 0.3
}


int16_t pidControl(int16_t target, int16_t current) { // Doesn't use any SRAM or program space currently
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

static void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum) { // Doesn't use any SRAM or program space currently
  if (uiReg == Yaw) s_cDataUpdate |= ANGLE_UPDATE;
}

static int32_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t *data, uint32_t length) { // Doesn't use any SRAM or program space currently
  Wire.beginTransmission(dev);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0;
  Wire.requestFrom(dev, length);
  unsigned long start = millis();
  while (Wire.available() < length) {
    if (millis() - start > 100) return 0;
  }
  for (uint32_t i = 0; i < length; i++) data[i] = Wire.read();
  return 1;
}

static int32_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t *data, uint32_t length) { // Doesn't use any SRAM or program space currently
  Wire.beginTransmission(dev);
  Wire.write(reg);
  Wire.write(data, length);
  return Wire.endTransmission() == 0;
}

static void Delayms(uint16_t ms) {
  delay(ms);
}



