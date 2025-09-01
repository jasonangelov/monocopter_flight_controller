#include "ControlSystem.h"
#include "Config.h"
#include <math.h>

ControlSystem::ControlSystem() {
  // Attitude defaults
  Kp_roll = 1.0f;
  Ki_roll = 0.0f;
  Kd_roll = 1.0f;
  Kp_pitch = 1.0f;
  Ki_pitch = 0.0f;
  Kd_pitch = 1.0f;
  deadzone_deg = 1.0f;
  
  // Fin defaults
  finGain_us = 10.0f;
  finLPF_alpha = 0.3f;
  finMax_us = 300;
  finDeadband = 0;
  
  // Yaw defaults
  yawEnabled = true;
  yawKp = 0.02f;
  yawKi = 0.00f;
  yawKd = 0.10f;
  yawLPF_alpha = 0.001f;
  yawLimit_us = 100;
  yawSign = -1;
  
  // Altitude defaults
  altTarget_cm = 100.0f;
  altKp = 4.0f;
  altKi = 0.0f;
  altKd = 2.0f;
  altLimit_us = 100;
  altLPF_alpha = 0.15f;
  throttle_us = ESC_HOVER;
  
  reset();
}

void ControlSystem::init() {
  reset();
}

void ControlSystem::reset() {
  i_roll = i_pitch = 0;
  prev_roll = prev_pitch = 0;
  finR_us = finP_us = 0;
  
  yawI_accum = 0;
  yawPrevErr = 0;
  yawGzFilt_deg = 0;
  yawDiff_us = 0;
  
  altI_accum = 0;
  heightFilt_cm = 0;
  altOut_us = 0;
}

float ControlSystem::constrainF(float v, float lo, float hi) const {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

void ControlSystem::resetYaw() {
  yawI_accum = 0;
  yawPrevErr = 0;
  yawGzFilt_deg = 0;
  yawDiff_us = 0;
}

void ControlSystem::updateAttitude(float roll, float pitch, float dt) {
  bool inDead = (fabsf(roll) < deadzone_deg && fabsf(pitch) < deadzone_deg);
  bool freezeFins = (Kp_roll == 0.0f && Kp_pitch == 0.0f);
  
  if (freezeFins) {
    i_roll = i_pitch = 0.0f;
  }
  
  float targetR_us = 0.0f;
  float targetP_us = 0.0f;
  
  if (!freezeFins && !inDead) {
    float errR = -roll;
    float errP = -pitch;
    
    i_roll += errR * dt;
    i_pitch += errP * dt;
    
    float d_roll = (roll - prev_roll) / dt;
    float d_pitch = (pitch - prev_pitch) / dt;
    prev_roll = roll;
    prev_pitch = pitch;
    
    float uR = Kp_roll * errR + Ki_roll * i_roll - Kd_roll * d_roll;
    float uP = Kp_pitch * errP + Ki_pitch * i_pitch - Kd_pitch * d_pitch;
    
    targetR_us = constrainF(uR * finGain_us, -finMax_us, +finMax_us);
    targetP_us = constrainF(uP * finGain_us, -finMax_us, +finMax_us);
    
    if (fabsf(targetR_us) < finDeadband) targetR_us = 0.0f;
    if (fabsf(targetP_us) < finDeadband) targetP_us = 0.0f;
  }
  
  // Apply LPF
  finR_us += finLPF_alpha * (targetR_us - finR_us);
  finP_us += finLPF_alpha * (targetP_us - finP_us);
}

void ControlSystem::updateYaw(float gyroZ, float dt) {
  if (!yawEnabled) {
    yawDiff_us = 0;
    resetYaw();
    return;
  }
  
  // LPF on measurement
  yawGzFilt_deg += yawLPF_alpha * (gyroZ - yawGzFilt_deg);
  
  // PID (target = 0 deg/s)
  float err = -yawGzFilt_deg;
  yawI_accum += err * dt;
  
  // Anti-windup
  float iTerm_us = constrainF(yawKi * yawI_accum, -yawLimit_us, yawLimit_us);
  
  float dErr = (err - yawPrevErr) / dt;
  yawPrevErr = err;
  
  float p_us = yawKp * err;
  float d_us = yawKd * dErr;
  
  float u = p_us + iTerm_us + d_us;
  u = constrainF(u, -yawLimit_us, yawLimit_us);
  
  yawDiff_us = (int)lrintf(yawSign * u);
}

void ControlSystem::updateAltitude(float heightM, uint8_t quality, uint32_t ageMs,
                                   float roll, float pitch, float dt) {
  float heightRaw_cm = NAN;
  bool lidarFresh = (ageMs <= ALT_VALID_MS);
  
  if (lidarFresh && quality > 0 && heightM > 0.03f) {
    float rollRad = roll * DEG2RAD;
    float pitchRad = pitch * DEG2RAD;
    float cosTilt = cosf(rollRad) * cosf(pitchRad);
    if (cosTilt < 0.1f) cosTilt = 0.1f;
    heightRaw_cm = heightM * 100.0f * cosTilt;
  }
  
  if (!isnan(heightRaw_cm)) {
    static float prevFilt = 0.0f;
    heightFilt_cm += altLPF_alpha * (heightRaw_cm - heightFilt_cm);
    
    float err_cm = altTarget_cm - heightFilt_cm;
    float dHeight_cm_s = (heightFilt_cm - prevFilt) / dt;
    prevFilt = heightFilt_cm;
    
    altI_accum += err_cm * dt;
    float iTerm_us = constrainF(altKi * altI_accum, -altLimit_us, +altLimit_us);
    float p_us = altKp * err_cm;
    float d_us = -altKd * dHeight_cm_s;
    
    altOut_us = constrainF(p_us + iTerm_us + d_us, -altLimit_us, +altLimit_us);
  } else {
    altOut_us *= 0.95f;  // Decay when no data
  }
}

void ControlSystem::setAttitudeGains(float kp, float ki, float kd) {
  Kp_roll = Kp_pitch = kp;
  Ki_roll = Ki_pitch = ki;
  Kd_roll = Kd_pitch = kd;
  if (kp == 0.0f) {
    i_roll = i_pitch = 0;
    finR_us = finP_us = 0;
  }
}

void ControlSystem::setRollGains(float kp, float ki, float kd) {
  Kp_roll = kp;
  Ki_roll = ki;
  Kd_roll = kd;
  if (Kp_roll == 0.0f && Kp_pitch == 0.0f) {
    i_roll = i_pitch = 0;
    finR_us = finP_us = 0;
  }
}

void ControlSystem::setPitchGains(float kp, float ki, float kd) {
  Kp_pitch = kp;
  Ki_pitch = ki;
  Kd_pitch = kd;
  if (Kp_roll == 0.0f && Kp_pitch == 0.0f) {
    i_roll = i_pitch = 0;
    finR_us = finP_us = 0;
  }
}

void ControlSystem::setYawGains(float kp, float ki, float kd) {
  yawKp = kp;
  yawKi = ki;
  yawKd = kd;
  if (ki == 0) yawI_accum = 0;
}

void ControlSystem::setAltitudeGains(float kp, float ki, float kd) {
  altKp = kp;
  altKi = ki;
  altKd = kd;
  if (ki == 0) altI_accum = 0;
}

void ControlSystem::getAttitudeGains(float& kpr, float& kir, float& kdr,
                                     float& kpp, float& kip, float& kdp) const {
  kpr = Kp_roll;
  kir = Ki_roll;
  kdr = Kd_roll;
  kpp = Kp_pitch;
  kip = Ki_pitch;
  kdp = Kd_pitch;
}

void ControlSystem::getYawParams(float& kp, float& ki, float& kd, float& lpf,
                                 int& limit, bool& enabled, bool& inverted) const {
  kp = yawKp;
  ki = yawKi;
  kd = yawKd;
  lpf = yawLPF_alpha;
  limit = yawLimit_us;
  enabled = yawEnabled;
  inverted = (yawSign < 0);
}

void ControlSystem::getAltitudeParams(float& target, float& kp, float& ki, float& kd,
                                      float& lpf, int& limit, int& throttleOut) const {
  target = altTarget_cm;
  kp = altKp;
  ki = altKi;
  kd = altKd;
  lpf = altLPF_alpha;
  limit = altLimit_us;
  throttleOut = throttle_us;
}

void ControlSystem::getFinParams(float& gain, float& lpf, int& max, int& deadband) const {
  gain = finGain_us;
  lpf = finLPF_alpha;
  max = finMax_us;
  deadband = finDeadband;
}