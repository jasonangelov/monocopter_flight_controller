#include <gtest/gtest.h>

#include "ActuatorMixer.h"

namespace {

TEST(ActuatorMixerTest, MapsFinCommandsSymmetricallyWithOffsets) {
  monocopter::ActuatorMixer mixer;

  const monocopter::FinOutputs outputs = mixer.mixFins(50.0f, -20.0f);

  EXPECT_EQ(outputs.fin1_us, 1450);
  EXPECT_EQ(outputs.fin2_us, 1420);
  EXPECT_EQ(outputs.fin3_us, 1525);
  EXPECT_EQ(outputs.fin4_us, 1555);
}

TEST(ActuatorMixerTest, ClampsFinCommandsToServoRange) {
  monocopter::ActuatorMixer mixer;

  const monocopter::FinOutputs outputs = mixer.mixFins(500.0f, 500.0f);

  EXPECT_EQ(outputs.fin1_us, 1650);
  EXPECT_EQ(outputs.fin2_us, 1150);
  EXPECT_EQ(outputs.fin3_us, 1325);
  EXPECT_EQ(outputs.fin4_us, 1825);
}

TEST(ActuatorMixerTest, AppliesMotorYawMixingAndTrims) {
  monocopter::ActuatorMixer mixer;

  const monocopter::MotorOutputs outputs = mixer.mixMotors(1500, 80, 10, 20);

  EXPECT_EQ(outputs.esc1_us, 1410);
  EXPECT_EQ(outputs.esc2_us, 1560);
}

TEST(ActuatorMixerTest, ClampsMotorOutputsToEscLimits) {
  monocopter::ActuatorMixer mixer;

  const monocopter::MotorOutputs outputs = mixer.mixMotors(1000, 500, 0, 0);

  EXPECT_EQ(outputs.esc1_us, 1000);
  EXPECT_EQ(outputs.esc2_us, 1500);
}

}  // namespace
