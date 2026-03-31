#include "FlightController.h"

namespace monocopter {

void FlightController::init() {
  control_.init();
}

void FlightController::reset() {
  control_.reset();
}

ControlTickResult FlightController::step(const ControlInputs& inputs) {
  ControlTickResult result;
  result.outputs.motors_enabled = inputs.control_enabled;

  if (inputs.control_enabled) {
    control_.updateAttitude(inputs.imu.roll_deg, inputs.imu.pitch_deg, inputs.loop_dt_s);
    control_.updateYaw(inputs.imu.yaw_rate_deg_s, inputs.loop_dt_s);
    control_.updateAltitude(inputs.rangefinder.height_m,
                            inputs.rangefinder.quality,
                            inputs.rangefinder.age_ms,
                            inputs.imu.roll_deg,
                            inputs.imu.pitch_deg,
                            inputs.loop_dt_s);

    result.outputs.roll_fin_us = control_.getRollCommand();
    result.outputs.pitch_fin_us = control_.getPitchCommand();
    result.outputs.throttle_us = control_.getThrottleCommand();
    result.outputs.yaw_diff_us = control_.getYawDifferential();
  }

  result.diagnostics.loop_dt_s = inputs.loop_dt_s;
  result.diagnostics.range_age_ms = inputs.rangefinder.age_ms;
  result.diagnostics.flow_age_ms = inputs.optical_flow.age_ms;
  result.diagnostics.range_quality = inputs.rangefinder.quality;
  result.diagnostics.flow_quality = inputs.optical_flow.quality;
  result.diagnostics.filtered_height_cm = control_.getFilteredHeight();
  result.diagnostics.altitude_output_us = control_.getAltitudeOutput();
  result.diagnostics.yaw_rate_filtered_deg_s = control_.getYawRate();
  result.diagnostics.roll_fin_us = result.outputs.roll_fin_us;
  result.diagnostics.pitch_fin_us = result.outputs.pitch_fin_us;
  result.diagnostics.throttle_us = result.outputs.throttle_us;
  result.diagnostics.yaw_diff_us = result.outputs.yaw_diff_us;

  return result;
}

}  // namespace monocopter
