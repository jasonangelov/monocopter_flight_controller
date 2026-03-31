#include "Actuators.h"
#include "Config.h"

Actuators::Actuators() : mixer() {
  escTrim1_us = 0;
  escTrim2_us = 0;
}

void Actuators::init() {
  // Initialize servos at 50 Hz
  fin1.setPeriodHertz(50);
  fin1.attach(PIN_S1, SERVO_MIN, SERVO_MAX);
  
  fin2.setPeriodHertz(50);
  fin2.attach(PIN_S2, SERVO_MIN, SERVO_MAX);
  
  fin3.setPeriodHertz(50);
  fin3.attach(PIN_S3, SERVO_MIN, SERVO_MAX);
  
  fin4.setPeriodHertz(50);
  fin4.attach(PIN_S4, SERVO_MIN, SERVO_MAX);
  
  writeFins(0, 0);
  
  // Initialize ESCs at 250 Hz
  esc1.setPeriodHertz(250);
  esc1.attach(PIN_ESC1, ESC_OFF, 2000);
  
  esc2.setPeriodHertz(250);
  esc2.attach(PIN_ESC2, ESC_OFF, 2000);
  
  motorsOff();
}

void Actuators::writeFins(float rollCmd_us, float pitchCmd_us) {
  const monocopter::FinOutputs outputs = mixer.mixFins(rollCmd_us, pitchCmd_us);

  fin1.writeMicroseconds(outputs.fin1_us);
  fin2.writeMicroseconds(outputs.fin2_us);
  fin3.writeMicroseconds(outputs.fin3_us);
  fin4.writeMicroseconds(outputs.fin4_us);
}

void Actuators::motorsOff() {
  esc1.writeMicroseconds(mixer.escOffUs());
  esc2.writeMicroseconds(mixer.escOffUs());
}

void Actuators::setMotors(int base_us, int yawDiff_us) {
  const monocopter::MotorOutputs outputs =
      mixer.mixMotors(base_us, yawDiff_us, escTrim1_us, escTrim2_us);

  esc1.writeMicroseconds(outputs.esc1_us);
  esc2.writeMicroseconds(outputs.esc2_us);
}

void Actuators::setESCTrims(int trim1, int trim2) {
  escTrim1_us = trim1;
  escTrim2_us = trim2;
}

void Actuators::getESCTrims(int& trim1, int& trim2) const {
  trim1 = escTrim1_us;
  trim2 = escTrim2_us;
}
