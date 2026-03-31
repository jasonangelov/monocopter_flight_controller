#include <gtest/gtest.h>

#include "ControlSystem.h"

namespace {

TEST(ControlSystemTest, KeepsFinsCenteredInsideDeadzone) {
  ControlSystem control;
  control.init();

  control.updateAttitude(0.5f, -0.5f, 0.01f);

  EXPECT_FLOAT_EQ(control.getRollCommand(), 0.0f);
  EXPECT_FLOAT_EQ(control.getPitchCommand(), 0.0f);
}

TEST(ControlSystemTest, ProducesFilteredRollCommandOutsideDeadzone) {
  ControlSystem control;
  control.init();
  control.setRollGains(1.0f, 0.0f, 0.0f);
  control.setPitchGains(0.0f, 0.0f, 0.0f);

  control.updateAttitude(10.0f, 0.0f, 0.01f);

  EXPECT_NEAR(control.getRollCommand(), -30.0f, 1e-4f);
  EXPECT_FLOAT_EQ(control.getPitchCommand(), 0.0f);
}

TEST(ControlSystemTest, DisablingYawClearsDifferentialState) {
  ControlSystem control;
  control.init();

  control.updateYaw(100.0f, 0.02f);
  EXPECT_NE(control.getYawDifferential(), 0);

  control.setYawEnabled(false);
  control.updateYaw(100.0f, 0.02f);

  EXPECT_EQ(control.getYawDifferential(), 0);
  EXPECT_FLOAT_EQ(control.getYawRate(), 0.0f);
}

TEST(ControlSystemTest, InvalidAltitudeSampleDecaysPreviousOutput) {
  ControlSystem control;
  control.init();

  control.updateAltitude(0.5f, 100, 0, 0.0f, 0.0f, 0.1f);
  ASSERT_NEAR(control.getAltitudeOutput(), 100.0f, 1e-4f);

  control.updateAltitude(0.5f, 0, 1000, 0.0f, 0.0f, 0.1f);

  EXPECT_NEAR(control.getAltitudeOutput(), 95.0f, 1e-4f);
}

TEST(ControlSystemTest, ResetClearsAltitudeDerivativeHistory) {
  ControlSystem control;
  control.init();
  control.setAltitudeGains(1.0f, 0.0f, 2.0f);
  control.setAltitudeLimit(1000);

  control.updateAltitude(0.5f, 100, 0, 0.0f, 0.0f, 0.1f);
  ASSERT_NEAR(control.getAltitudeOutput(), -57.5f, 1e-3f);

  control.reset();
  control.updateAltitude(0.5f, 100, 0, 0.0f, 0.0f, 0.1f);

  EXPECT_NEAR(control.getAltitudeOutput(), -57.5f, 1e-3f);
}

}  // namespace
