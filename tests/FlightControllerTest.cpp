#include <gtest/gtest.h>

#include "FlightController.h"

namespace {

TEST(FlightControllerTest, DisabledControllerKeepsOutputsAtZero) {
  monocopter::FlightController controller;
  controller.init();

  monocopter::ControlInputs inputs;
  inputs.control_enabled = false;
  inputs.loop_dt_s = 0.005f;
  inputs.rangefinder = monocopter::RangefinderSample{0.5f, 100, 10};
  inputs.optical_flow = monocopter::OpticalFlowSample{0.1f, -0.2f, 80, 12};

  const monocopter::ControlTickResult result = controller.step(inputs);

  EXPECT_FALSE(result.outputs.motors_enabled);
  EXPECT_FLOAT_EQ(result.outputs.roll_fin_us, 0.0f);
  EXPECT_FLOAT_EQ(result.outputs.pitch_fin_us, 0.0f);
  EXPECT_EQ(result.outputs.throttle_us, 0);
  EXPECT_EQ(result.outputs.yaw_diff_us, 0);
  EXPECT_FLOAT_EQ(result.diagnostics.loop_dt_s, 0.005f);
  EXPECT_EQ(result.diagnostics.range_age_ms, 10u);
  EXPECT_EQ(result.diagnostics.flow_age_ms, 12u);
}

TEST(FlightControllerTest, EnabledControllerReturnsControlOutputsAndDiagnostics) {
  monocopter::FlightController controller;
  controller.init();

  monocopter::ControlInputs inputs;
  inputs.control_enabled = true;
  inputs.loop_dt_s = 0.005f;
  inputs.imu = monocopter::ImuSample{6.0f, -2.0f, 25.0f};
  inputs.rangefinder = monocopter::RangefinderSample{0.8f, 90, 15};
  inputs.optical_flow = monocopter::OpticalFlowSample{0.2f, 0.1f, 70, 18};

  const monocopter::ControlTickResult result = controller.step(inputs);

  EXPECT_TRUE(result.outputs.motors_enabled);
  EXPECT_NE(result.outputs.roll_fin_us, 0.0f);
  EXPECT_NE(result.outputs.pitch_fin_us, 0.0f);
  EXPECT_GT(result.outputs.throttle_us, 0);
  EXPECT_EQ(result.diagnostics.range_quality, 90);
  EXPECT_EQ(result.diagnostics.flow_quality, 70);
  EXPECT_FLOAT_EQ(result.diagnostics.roll_fin_us, result.outputs.roll_fin_us);
  EXPECT_FLOAT_EQ(result.diagnostics.pitch_fin_us, result.outputs.pitch_fin_us);
  EXPECT_EQ(result.diagnostics.throttle_us, result.outputs.throttle_us);
  EXPECT_EQ(result.diagnostics.yaw_diff_us, result.outputs.yaw_diff_us);
}

}  // namespace
