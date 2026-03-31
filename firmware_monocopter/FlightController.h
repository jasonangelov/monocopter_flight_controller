#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include <stdint.h>

#include "ControlSystem.h"

namespace monocopter {

struct ImuSample {
  float roll_deg = 0.0f;
  float pitch_deg = 0.0f;
  float yaw_rate_deg_s = 0.0f;
};

struct RangefinderSample {
  float height_m = 0.0f;
  uint8_t quality = 0;
  uint32_t age_ms = 0;
};

struct OpticalFlowSample {
  float vx_mps = 0.0f;
  float vy_mps = 0.0f;
  uint8_t quality = 0;
  uint32_t age_ms = 0;
};

struct ControlInputs {
  bool control_enabled = false;
  float loop_dt_s = 0.0f;
  ImuSample imu;
  RangefinderSample rangefinder;
  OpticalFlowSample optical_flow;
};

struct ControlOutputs {
  bool motors_enabled = false;
  float roll_fin_us = 0.0f;
  float pitch_fin_us = 0.0f;
  int throttle_us = 0;
  int yaw_diff_us = 0;
};

struct DiagnosticsSnapshot {
  float loop_dt_s = 0.0f;
  uint32_t range_age_ms = 0;
  uint32_t flow_age_ms = 0;
  uint8_t range_quality = 0;
  uint8_t flow_quality = 0;
  float filtered_height_cm = 0.0f;
  float altitude_output_us = 0.0f;
  float yaw_rate_filtered_deg_s = 0.0f;
  float roll_fin_us = 0.0f;
  float pitch_fin_us = 0.0f;
  int throttle_us = 0;
  int yaw_diff_us = 0;
};

struct ControlTickResult {
  ControlOutputs outputs;
  DiagnosticsSnapshot diagnostics;
};

class FlightController {
public:
  void init();
  void reset();

  ControlTickResult step(const ControlInputs& inputs);

  ControlSystem& control() { return control_; }
  const ControlSystem& control() const { return control_; }

private:
  ControlSystem control_;
};

}  // namespace monocopter

#endif
