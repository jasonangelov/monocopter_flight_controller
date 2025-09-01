#include "Actuators.h"
#include "Config.h"

Actuators::Actuators() {
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

int Actuators::usFromCmd(float cmd) const {
  float constrained = (cmd < -SERVO_RANGE) ? -SERVO_RANGE : 
                     (cmd > SERVO_RANGE) ? SERVO_RANGE : cmd;
  return SERVO_NEUT + (int)lrintf(constrained);
}

void Actuators::writeFins(float rollCmd_us, float pitchCmd_us) {
  int u1 = usFromCmd(+rollCmd_us);
  int u3 = usFromCmd(-rollCmd_us);
  int u2 = usFromCmd(-pitchCmd_us);
  int u4 = usFromCmd(+pitchCmd_us);
  
  // Apply mechanical offsets
  fin1.writeMicroseconds(u1 - 100);
  fin2.writeMicroseconds(u2 - 100);
  fin3.writeMicroseconds(u3 + 75);
  fin4.writeMicroseconds(u4 + 75);
}

void Actuators::motorsOff() {
  esc1.writeMicroseconds(ESC_OFF);
  esc2.writeMicroseconds(ESC_OFF);
}

void Actuators::setMotors(int base_us, int yawDiff_us) {
  // Apply tilt compensation
  int t1 = base_us - (escTrim1_us + yawDiff_us);
  int t2 = base_us - (escTrim2_us - yawDiff_us);
  
  t1 = constrain(t1, 1000, 2000);
  t2 = constrain(t2, 1000, 2000);
  
  esc1.writeMicroseconds(t1);
  esc2.writeMicroseconds(t2);
}

void Actuators::setESCTrims(int trim1, int trim2) {
  escTrim1_us = trim1;
  escTrim2_us = trim2;
}

void Actuators::getESCTrims(int& trim1, int& trim2) const {
  trim1 = escTrim1_us;
  trim2 = escTrim2_us;
}
