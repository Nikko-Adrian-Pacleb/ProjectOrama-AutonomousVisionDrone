//TODO: need to write a function for the motor PID control.
//      Need to implement the LiDAR sensor into the code for automatic landing sequence. // STARTED
//      Need to implement the code for object avoidance using the TOF sensors.

// dynamic memory is looking okay. I think we should be fine.

#include <Wire.h>       // 5% of dynamic memory (113 bytes)
#include <Servo.h>      // 2% of dynamic memory (37 bytes)
#include <SoftwareSerial.h>   //header file of software serial port
#include "REG.h"        
#include "wit_c_sdk.h"  
#include <VL53L1X.h>


// ###################### DEBUG VARIABLE ########################
bool DEBUG = false;
// ##############################################################

// TOF Object initilization
VL53L1X sensors[4];

// LiDAR variables and softwareserial object
SoftwareSerial Serial1(2, 3); //define software serial port name as Serial1 and define pin2 as RX & pin3 as TX
 
int dist;                     //actual distance measurements of LiDAR
int strength;                 //signal strength of LiDAR
int check;                    //save check value
int uart[9];                   //save data measured by LiDAR
const int HEADER = 0x59;      //frame header of data package

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

// control the ESC using servo objects and write microseconds to them.
Servo FrontLeftProp;
Servo FrontRightProp;
Servo BackLeftProp;
Servo BackRightProp;

// Pins for the reciever
#define CH1 7
#define CH2 8
#define CH3 9
#define CH4 10
#define CH5 11
#define CH6 12


// data that will get recieved from the RPi and is used to control the drone based on what the computer vision sees.
struct controlData {
  int throttle; // a potentiometer value like 0 - 1684 or whatever that value is.
  int switchMode; // use as a boolean so either 0 or 1
  // have to think about what else to put in the struct.

};

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

//TOF Variables
int16_t TOFData[] = {0, 0, 0, 0}; // not sure how nikko wanted to put this into a struct. It would be very similar.
const uint8_t sensorAddresses[4] = {0x30, 0x31, 0x32, 0x33}; // these will be found after hooking up the I2C multiplexer.


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


// --------------- IMU Functions --------------- //
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

static void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum) { 
  if (uiReg == Yaw) s_cDataUpdate |= ANGLE_UPDATE;
}

static int32_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t *data, uint32_t length) {
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

static int32_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t *data, uint32_t length) { 
  Wire.beginTransmission(dev);
  Wire.write(reg);
  Wire.write(data, length);
  return Wire.endTransmission() == 0;
}

static void Delayms(uint16_t ms) {
  delay(ms);
}


// --------------- Camera Control Functions --------------- //
int16_t smoothAngle(int16_t newAngle) {
   return (int16_t)((3L * newAngle + 7L * smoothedAngle) / 10L);  // alpha = 0.3
}

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
// --------------- Motor Control Functions --------------- //


/** Starts all 4 ESC's so that you can actually control them.
 * void function
*/
void motor_init() {
  FrontLeftProp.attach(FRONT_LEFT_MOTOR);
  FrontRightProp.attach(FRONT_RIGHT_MOTOR);
  BackLeftProp.attach(BACK_LEFT_MOTOR);
  BackRightProp.attach(BACK_RIGHT_MOTOR);

  FrontLeftProp.writeMicroseconds(1000);
  FrontRightProp.writeMicroseconds(1000);
  BackLeftProp.writeMicroseconds(1000);
  BackRightProp.writeMicroseconds(1000);
}

int16_t motorPIDControl(int16_t target, int16_t current) { // need to adapt it to 3 axis and 4 motors instead of just 1 axis and 1 motor.
  
}


// --------------- TOF Sensor Functions --------------- //
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
}



/** Automatically lands the drone using the LiDAR sensor. 
  * @return void
*/
void autoLand() {

 /* 1. need to first check the distance that the drone is currently from the ground.
    2. If the distance is really large start descending at fastest speed allowed.
    3. If not then use the percentage to find the descending speed.
    4. then write the correct speed to the motors and constantly adjusting till the distance stops changing.
    5. once the distance stops changing stop the motors.

    //coudln't do any reasearch or testing on this so someone will have to test and see if it works.
  */

  //almost positive the way that this works is not correct. but I hope you get the idea.
  
  int groundDist = getLiDARData();
  int lastDist = 0;
  if (groundDist > 3000) {
    //need to write to the motors through the PID to make sure that it lands flat.
    //not quite sure what value will start to lower the drone. This has to be found once we have the total weight and have tried flying it on a string or something once the PID works.
    int lastDist = getLiDARData();
  }
  while (groundDist > 20) { // need to write it so that it uses a last distance to make sure that the drone is still descending.

  }
  

}








