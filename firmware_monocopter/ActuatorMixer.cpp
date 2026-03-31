#include "ActuatorMixer.h"

#include <math.h>

namespace monocopter {

ActuatorMixer::ActuatorMixer(MixerConfig config) : config_(config) {
}

int ActuatorMixer::clampInt(int value, int lo, int hi) const {
  return (value < lo) ? lo : (value > hi) ? hi : value;
}

int ActuatorMixer::servoUsFromCommand(float cmd_us) const {
  float constrained = cmd_us;
  if (constrained < -config_.servoRangeUs) {
    constrained = -config_.servoRangeUs;
  } else if (constrained > config_.servoRangeUs) {
    constrained = config_.servoRangeUs;
  }

  return config_.servoNeutralUs + static_cast<int>(lrintf(constrained));
}

FinOutputs ActuatorMixer::mixFins(float rollCmd_us, float pitchCmd_us) const {
  const int fin1 = servoUsFromCommand(+rollCmd_us) + config_.fin1OffsetUs;
  const int fin2 = servoUsFromCommand(-pitchCmd_us) + config_.fin2OffsetUs;
  const int fin3 = servoUsFromCommand(-rollCmd_us) + config_.fin3OffsetUs;
  const int fin4 = servoUsFromCommand(+pitchCmd_us) + config_.fin4OffsetUs;

  return FinOutputs{fin1, fin2, fin3, fin4};
}

MotorOutputs ActuatorMixer::mixMotors(int base_us, int yawDiff_us,
                                      int escTrim1_us, int escTrim2_us) const {
  int esc1 = base_us - (escTrim1_us + yawDiff_us);
  int esc2 = base_us - (escTrim2_us - yawDiff_us);

  esc1 = clampInt(esc1, config_.escMinUs, config_.escMaxUs);
  esc2 = clampInt(esc2, config_.escMinUs, config_.escMaxUs);

  return MotorOutputs{esc1, esc2};
}

}  // namespace monocopter
