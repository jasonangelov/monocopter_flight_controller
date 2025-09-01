#ifndef ACTUATORS_H
#define ACTUATORS_H

#include <ESP32Servo.h>

class Actuators {
public:
  Actuators();
  void init();
  
  void writeFins(float rollCmd_us, float pitchCmd_us);
  void setMotors(int base_us, int yawDiff_us);
  void motorsOff();
  
  void setESCTrims(int trim1, int trim2);
  void getESCTrims(int& trim1, int& trim2) const;
  
private:
  Servo fin1, fin2, fin3, fin4;
  Servo esc1, esc2;
  
  int escTrim1_us;
  int escTrim2_us;
  
  int usFromCmd(float cmd) const;
};

#endif
