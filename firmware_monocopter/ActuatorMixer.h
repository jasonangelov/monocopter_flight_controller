#ifndef ACTUATOR_MIXER_H
#define ACTUATOR_MIXER_H

#include "FlightControlConfig.h"

namespace monocopter {

struct FinOutputs {
  int fin1_us;
  int fin2_us;
  int fin3_us;
  int fin4_us;
};

struct MotorOutputs {
  int esc1_us;
  int esc2_us;
};

struct MixerConfig {
  int servoNeutralUs = kServoNeutralUs;
  int servoRangeUs = kServoRangeUs;
  int escOffUs = kEscOffUs;
  int escMinUs = kEscMinUs;
  int escMaxUs = kEscMaxUs;
  int fin1OffsetUs = kFin1OffsetUs;
  int fin2OffsetUs = kFin2OffsetUs;
  int fin3OffsetUs = kFin3OffsetUs;
  int fin4OffsetUs = kFin4OffsetUs;
};

class ActuatorMixer {
public:
  explicit ActuatorMixer(MixerConfig config = {});

  FinOutputs mixFins(float rollCmd_us, float pitchCmd_us) const;
  MotorOutputs mixMotors(int base_us, int yawDiff_us, int escTrim1_us, int escTrim2_us) const;

  int escOffUs() const { return config_.escOffUs; }

private:
  MixerConfig config_;

  int clampInt(int value, int lo, int hi) const;
  int servoUsFromCommand(float cmd_us) const;
};

}  // namespace monocopter

#endif
