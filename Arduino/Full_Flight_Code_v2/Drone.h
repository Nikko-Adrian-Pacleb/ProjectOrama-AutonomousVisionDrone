#ifndef DRONE_H
#define DRONE_H

#include <Arduino.h>
#include <Servo.h>

// --- Basic quad state (extend later if you want) ---
enum DroneState {
  DISARMED = 0,
  ARMED
};

// --- Tiny PID helper (no dynamic alloc, dt in seconds) ---
struct PID {
  float kp = 0.0f, ki = 0.0f, kd = 0.0f;
  float iTerm = 0.0f;
  float prevErr = 0.0f;

  void setGains(float _kp, float _ki, float _kd) {
    kp = _kp; ki = _ki; kd = _kd;
  }

  void reset() {
    iTerm = 0.0f;
    prevErr = 0.0f;
  }

  // returns control effort
  float update(float setpoint, float measured, float dt, float iLimit = 200.0f) {
    float err = setpoint - measured;
    iTerm += err * dt * ki;
    // anti-windup clamp
    if (iTerm > iLimit) iTerm = iLimit;
    if (iTerm < -iLimit) iTerm = -iLimit;
    float d = (dt > 0.0f) ? (err - prevErr) / dt : 0.0f;
    prevErr = err;
    return kp * err + iTerm + kd * d;
  }
};

class Drone {
public:
  Drone();

  // Attach motor pins (write in microseconds to ESCs)
  void begin(uint8_t pinFL, uint8_t pinFR, uint8_t pinBL, uint8_t pinBR, HardwareSerial* dbg = &Serial);

  // Arming / disarming
  void arm();
  void disarm();
  bool isArmed() const { return state == ARMED; }

  // PID gain setters
  void setPitchPID(float kp, float ki, float kd);
  void setRollPID (float kp, float ki, float kd);
  void setYawPID  (float kp, float ki, float kd);

  // Optional: set desired attitudes (deg) and yaw rate (deg/s) if you add sticks later
  void setSetpoints(float pitchDeg, float rollDeg, float yawRateDegS);

  // Base throttle in µs (1000–2000)
  void setBaseThrottle(int throttleUs);

  // Update stabilization + motor mix.
  // Provide measured pitch/roll in degrees, yaw rate in deg/s, and dt in seconds.
  void update(float pitchDeg, float rollDeg, float yawRateDegS, float dt);

  // Debug
  void setDebug(bool enabled);

private:
  // ESC outputs
  Servo mFL, mFR, mBL, mBR;
  uint8_t pinFL=255, pinFR=255, pinBL=255, pinBR=255;

  // State
  DroneState state = DISARMED;
  int baseThrottleUs = 1000; // µs
  bool debugEnabled = true;
  HardwareSerial* debugPort = &Serial;

  // Setpoints
  float spPitch = 0.0f;   // deg
  float spRoll  = 0.0f;   // deg
  float spYawRate = 0.0f; // deg/s

  // Controllers
  PID pidPitch, pidRoll, pidYaw;

  // Helpers
  static int clampUs(int x);
  void writeAll(int usFL, int usFR, int usBL, int usBR);
  void stopAll();
};

#endif
