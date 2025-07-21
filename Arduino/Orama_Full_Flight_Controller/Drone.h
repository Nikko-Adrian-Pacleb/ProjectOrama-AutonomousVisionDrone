#ifndef DRONE_H
#define DRONE_H

#include <Arduino.h>
#include <Servo.h>
#include <VL53L1X.h>

enum DroneState {
  HOVER, // Stay in place
  LIFT, // Go up
  DESCENT, // Go down
  PITCH_FORWARD, // Move forward
  PITCH_BACKWARD, // Move Backward
  ROLL_LEFT, // Go left still facing front
  ROLL_RIGHT, // Go right still facing front
  YAW_LEFT, // Face left
  YAW_RIGHT // Face right
};

class Drone {
  private:
    // ----- Software -----
    DroneState dronteState;
    float pitch, roll, yaw;
    uint8_t multiplexerAddress;
    int throttle;

    // ----- Hardware -----
    Servo frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
  	VL53L1X tofSensor;
    // 1 Lidar
    // Controller
    // RaspberryPi
    // If we have enough pins. Emergency Switch
    // If we have enough pins. Light indicator

  public:
    Drone();

    void update();

    void arm();
    void disarm();
    bool isArmed();

    void emergencyStop();

    void setOrientationValues(float pitchAngle, float rollAngle, float yawAngle);
    float getPitch();
    float getRoll();
    float getYaw();

    void setDroneState(DroneState droneState);
    DroneState getDroneState();

    // ----- MOTORS -----
  	void initializeMotors(int FrontLeftPin, int FrontRightPin, int BackLeftPin, int BackRightPin);
    void setThrottle(int throttleValue);
    int getCurrentThrottle();
  
  	// ----- Multiplexer -----
  	void setI2CAddress(uint8_t address);
  	void selectI2CPort(uint8_t portNumber);
  
  	// ----- TOF -----
  	bool initializeTOFSensor(uint8_t portNumber);
  	bool isDistanceSafe(DroneState droneState);
  	int getTOFDistance(uint8_t portNumber);
};

#endif