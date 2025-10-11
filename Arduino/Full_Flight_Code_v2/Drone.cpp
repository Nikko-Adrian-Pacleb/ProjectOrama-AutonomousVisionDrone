#include "Drone.h"

Drone::Drone() {}

void Drone::begin(uint8_t _pinFL, uint8_t _pinFR, uint8_t _pinBL, uint8_t _pinBR, HardwareSerial* dbg) {
  pinFL = _pinFL; pinFR = _pinFR; pinBL = _pinBL; pinBR = _pinBR;
  debugPort = dbg ? dbg : &Serial;

  mFL.attach(pinFL);
  mFR.attach(pinFR);
  mBL.attach(pinBL);
  mBR.attach(pinBR);

  // Send safe minimum
  stopAll();

  // Default “gentle but responsive” gains — tune these!
  setPitchPID(3.0f, 0.0f, 0.10f);
  setRollPID (3.0f, 0.0f, 0.10f);
  setYawPID  (2.0f, 0.0f, 0.05f);

  if (debugEnabled) {
    debugPort->println(F("[Drone] Motors attached. Awaiting ARM."));
  }
}

void Drone::arm() {
  state = ARMED;
  pidPitch.reset();
  pidRoll.reset();
  pidYaw.reset();
  if (debugEnabled) debugPort->println(F("[Drone] ARMED"));
}

void Drone::disarm() {
  state = DISARMED;
  stopAll();
  if (debugEnabled) debugPort->println(F("[Drone] DISARMED"));
}

void Drone::setPitchPID(float kp, float ki, float kd) { pidPitch.setGains(kp, ki, kd); }
void Drone::setRollPID (float kp, float ki, float kd) { pidRoll .setGains(kp, ki, kd); }
void Drone::setYawPID  (float kp, float ki, float kd) { pidYaw  .setGains(kp, ki, kd); }

void Drone::setSetpoints(float pitchDeg, float rollDeg, float yawRateDegS) {
  spPitch = pitchDeg;
  spRoll = rollDeg;
  spYawRate = yawRateDegS;
}

void Drone::setBaseThrottle(int throttleUs) {
  baseThrottleUs = clampUs(throttleUs);
}

void Drone::setDebug(bool enabled) {
  debugEnabled = enabled;
}

void Drone::update(float pitchDeg, float rollDeg, float yawRateDegS, float dt) {
  if (state != ARMED) {
    stopAll();
    return;
  }

  // PID corrections (units map directly to microseconds after a scale)
  float uPitch = pidPitch.update(spPitch, rollDeg, dt); // Note: common frames use pitch correction from measured PITCH.
  float uRoll  = pidRoll .update(spRoll,  pitchDeg, dt); // Swap if your IMU axes are opposite.
  float uYaw   = pidYaw  .update(spYawRate, yawRateDegS, dt);

  // Optional overall sensitivity scale (tune down if too aggressive)
  const float kMix = 10.0f;      // deg -> µs scale for pitch/roll
  const float kYaw = 8.0f;       // deg/s -> µs scale for yaw

  int cP = (int)(kMix * uPitch);
  int cR = (int)(kMix * uRoll );
  int cY = (int)(kYaw * uYaw  );

  // X-quad mix:
  // FrontLeft  (+pitch, +roll, -yaw)
  // FrontRight (+pitch, -roll, +yaw)
  // BackLeft   (-pitch, +roll, +yaw)
  // BackRight  (-pitch, -roll, -yaw)
  int usFL = clampUs(baseThrottleUs + cP + cR - cY);
  int usFR = clampUs(baseThrottleUs + cP - cR + cY);
  int usBL = clampUs(baseThrottleUs - cP + cR + cY);
  int usBR = clampUs(baseThrottleUs - cP - cR - cY);

  writeAll(usFL, usFR, usBL, usBR);

  if (debugEnabled && debugPort) {
    static uint32_t lastPrint = 0;
    uint32_t now = millis();
    if (now - lastPrint >= 50) { // ~20Hz debug
      lastPrint = now;
      debugPort->print(F("THR=")); debugPort->print(baseThrottleUs);
      debugPort->print(F("  Meas[P,R,Yr]="));
      debugPort->print(pitchDeg, 2); debugPort->print(',');
      debugPort->print(rollDeg, 2);  debugPort->print(',');
      debugPort->print(yawRateDegS, 2);
      debugPort->print(F("  U[P,R,Y]="));
      debugPort->print(cP); debugPort->print(',');
      debugPort->print(cR); debugPort->print(',');
      debugPort->print(cY);
      debugPort->print(F("  Motors[FL,FR,BL,BR]="));
      debugPort->print(usFL); debugPort->print(',');
      debugPort->print(usFR); debugPort->print(',');
      debugPort->print(usBL); debugPort->print(',');
      debugPort->println(usBR);
    }
  }
}

int Drone::clampUs(int x) {
  if (x < 1000) return 1000;
  if (x > 2000) return 2000;
  return x;
}

void Drone::writeAll(int usFL, int usFR, int usBL, int usBR) {
  mFL.writeMicroseconds(usFL);
  mFR.writeMicroseconds(usFR);
  mBL.writeMicroseconds(usBL);
  mBR.writeMicroseconds(usBR);
}

void Drone::stopAll() {
  // Safe minimum (adjust to your ESC min-throttle if needed)
  mFL.writeMicroseconds(1000);
  mFR.writeMicroseconds(1000);
  mBL.writeMicroseconds(1000);
  mBR.writeMicroseconds(1000);
}
