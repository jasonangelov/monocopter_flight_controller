#ifndef CONTROL_SYSTEM_H
#define CONTROL_SYSTEM_H

#include <Arduino.h>

class ControlSystem {
public:
  ControlSystem();
  void init();
  
  // Update methods
  void updateAttitude(float roll, float pitch, float dt);
  void updateYaw(float gyroZ, float dt);
  void updateAltitude(float heightM, uint8_t quality, uint32_t ageMs, 
                      float roll, float pitch, float dt);
  
  // Output getters
  float getRollCommand() const { return finR_us; }
  float getPitchCommand() const { return finP_us; }
  int getThrottleCommand() const { return throttle_us + (int)lrintf(altOut_us); }
  int getYawDifferential() const { return yawDiff_us; }
  
  // Parameter setters (for telnet)
  void setAttitudeGains(float kp, float ki, float kd);
  void setRollGains(float kp, float ki, float kd);
  void setPitchGains(float kp, float ki, float kd);
  void setDeadzone(float deg) { deadzone_deg = deg; }
  
  void setYawEnabled(bool en) { yawEnabled = en; if (!en) resetYaw(); }
  void setYawGains(float kp, float ki, float kd);
  void setYawLPF(float alpha) { yawLPF_alpha = alpha; }
  void setYawLimit(int us) { yawLimit_us = us; }
  void setYawInvert(bool inv) { yawSign = inv ? -1 : 1; }
  
  void setAltitudeTarget(float cm) { altTarget_cm = cm; altI_accum = 0; }
  void setAltitudeGains(float kp, float ki, float kd);
  void setAltitudeLPF(float alpha) { altLPF_alpha = alpha; }
  void setAltitudeLimit(int us) { altLimit_us = us; }
  void setThrottle(int us) { throttle_us = us; }
  
  void setFinGain(float us) { finGain_us = us; }
  void setFinLPF(float alpha) { finLPF_alpha = alpha; }
  void setFinMax(int us) { finMax_us = us; }
  void setFinDeadband(int us) { finDeadband = us; }
  
  void reset();
  
  // Parameter getters (for status)
  void getAttitudeGains(float& kpr, float& kir, float& kdr, 
                        float& kpp, float& kip, float& kdp) const;
  void getYawParams(float& kp, float& ki, float& kd, float& lpf, 
                    int& limit, bool& enabled, bool& inverted) const;
  void getAltitudeParams(float& target, float& kp, float& ki, float& kd, 
                         float& lpf, int& limit, int& throttle) const;
  void getFinParams(float& gain, float& lpf, int& max, int& deadband) const;
  float getDeadzone() const { return deadzone_deg; }
  float getFilteredHeight() const { return heightFilt_cm; }
  float getAltitudeOutput() const { return altOut_us; }
  float getYawRate() const { return yawGzFilt_deg; }
  
private:
  // Attitude control
  float Kp_roll, Ki_roll, Kd_roll;
  float Kp_pitch, Ki_pitch, Kd_pitch;
  float deadzone_deg;
  float i_roll, i_pitch;
  float prev_roll, prev_pitch;
  
  // Fin shaping
  float finGain_us;
  float finLPF_alpha;
  int finMax_us;
  int finDeadband;
  float finR_us, finP_us;
  
  // Yaw control
  bool yawEnabled;
  float yawKp, yawKi, yawKd;
  float yawLPF_alpha;
  int yawLimit_us;
  int yawSign;
  float yawI_accum;
  float yawPrevErr;
  float yawGzFilt_deg;
  int yawDiff_us;
  
  // Altitude control
  float altTarget_cm;
  float altKp, altKi, altKd;
  int altLimit_us;
  float altI_accum;
  float heightFilt_cm;
  float altOut_us;
  float altLPF_alpha;
  int throttle_us;
  
  // Helper methods
  float constrainF(float v, float lo, float hi) const;
  void resetYaw();
};

#endif