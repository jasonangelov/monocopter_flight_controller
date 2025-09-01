#include "TelnetInterface.h"
#include "ControlSystem.h"
#include "Actuators.h"
#include "MSPProtocol.h"

TelnetInterface::TelnetInterface() : telnetServer(23) {
}

void TelnetInterface::begin() {
  telnetServer.begin();
  telnetServer.setNoDelay(true);
  Serial.println("Telnet server ready on port 23");
}

void TelnetInterface::sendTelemetry(float roll, float pitch, float height, int throttle) {
  telnet.printf("angles(deg): roll %.2f  pitch %.2f  height %.1fcm  throttle %d\r\n",
                roll, pitch, height * 100.0f, throttle);
}

void TelnetInterface::update(bool& ctrlEnabled, ControlSystem& control,
                             Actuators& actuators, MSPProtocol& msp) {
  if (!telnet || !telnet.connected()) {
    telnet = telnetServer.accept();
    if (telnet) {
      telnet.print("Monocopter ESP32 > ");
    }
    return;
  }
  
  while (telnet.available()) {
    char c = telnet.read();
    if (c == '\r' || c == '\n') {
      cmd.trim();
      cmd.toLowerCase();
      
      int sp = cmd.indexOf(' ');
      String name = (sp == -1) ? cmd : cmd.substring(0, sp);
      String arg = (sp == -1) ? "" : cmd.substring(sp + 1);
      arg.trim();
      
      processCommand(name, arg, ctrlEnabled, control, actuators, msp);
      
      cmd = "";
      telnet.print("> ");
    } else {
      cmd += c;
    }
  }
}

void TelnetInterface::processCommand(const String& name, const String& arg,
                                     bool& ctrlEnabled, ControlSystem& control,
                                     Actuators& actuators, MSPProtocol& msp) {
  
  if (name == "start") {
    control.reset();
    ctrlEnabled = true;
    telnet.println("✔ motors ON, control active");
    
  } else if (name == "stop") {
    ctrlEnabled = false;
    actuators.writeFins(0, 0);
    actuators.motorsOff();
    telnet.println("✖ motors OFF");
    
  } else if (name == "status") {
    // Get all parameters
    float kpr, kir, kdr, kpp, kip, kdp;
    control.getAttitudeGains(kpr, kir, kdr, kpp, kip, kdp);
    
    float yawKp, yawKi, yawKd, yawLPF;
    int yawLimit;
    bool yawEnabled, yawInverted;
    control.getYawParams(yawKp, yawKi, yawKd, yawLPF, yawLimit, yawEnabled, yawInverted);
    
    float altTarget, altKp, altKi, altKd, altLPF;
    int altLimit, throttle;
    control.getAltitudeParams(altTarget, altKp, altKi, altKd, altLPF, altLimit, throttle);
    
    float finGain, finLPF;
    int finMax, finDeadband;
    control.getFinParams(finGain, finLPF, finMax, finDeadband);
    
    int trim1, trim2;
    actuators.getESCTrims(trim1, trim2);
    
    telnet.printf("ctrl %s  throttle %dµs  throttlelimit ±%dµs\r\n",
                  ctrlEnabled ? "ON" : "OFF", throttle, altLimit);
    telnet.printf("alt target %.1fcm  Kp %.2f Ki %.2f Kd %.2f  LPF %.2f\r\n",
                  altTarget, altKp, altKi, altKd, altLPF);
    telnet.printf("ROLL  Kp %.2f Ki %.2f Kd %.2f | PITCH Kp %.2f Ki %.2f Kd %.2f\r\n",
                  kpr, kir, kdr, kpp, kip, kdp);
    telnet.printf("deadzone %.1f°  fingain %.1f  finLPF %.2f  finmax %d  finDB %d\r\n",
                  control.getDeadzone(), finGain, finLPF, finMax, finDeadband);
    telnet.printf("yaw %s  Kp %.3f Ki %.3f Kd %.3f  LPF %.3f  limit %d  inv %s\r\n",
                  yawEnabled ? "ON" : "OFF", yawKp, yawKi, yawKd, yawLPF, yawLimit,
                  yawInverted ? "ON" : "OFF");
    telnet.printf("ESC1 trim %d  ESC2 trim %d\r\n", trim1, trim2);
    telnet.printf("height %.1fcm  flow vx %.2f vy %.2f m/s\r\n",
                  msp.getHeight() * 100.0f, msp.getVelocityX(), msp.getVelocityY());
    
  } else if (name == "throttle") {
    int val = arg.toInt();
    if (arg.length() && val >= 1000 && val <= 2000) {
      control.setThrottle(val);
      telnet.printf("✔ throttle %d µs\r\n", val);
    } else {
      telnet.println("Enter throttle 1000-2000");
    }
    
  } else if (name == "throttlelimit" || name == "altlimit") {
    int v = arg.toInt();
    if (arg.length() && v >= 0 && v <= 700) {
      control.setAltitudeLimit(v);
      telnet.printf("✔ throttlelimit ±%d µs\r\n", v);
    } else {
      telnet.println("Enter throttlelimit 0-700");
    }
    
  } else if (name == "alt") {
    float cm = arg.toFloat();
    if (arg.length() && cm >= 5.0f && cm <= 500.0f) {
      control.setAltitudeTarget(cm);
      telnet.printf("✔ alt target %.1f cm\r\n", cm);
    } else {
      telnet.println("Enter alt 5-500 (cm)");
    }
    
  } else if (name == "altkp") {
    float v = arg.toFloat();
    if (arg.length() && v >= 0.0f && v <= 30.0f) {
      float ki, kd, target, lpf;
      int limit, throttle;
      control.getAltitudeParams(target, ki, ki, kd, lpf, limit, throttle);
      control.setAltitudeGains(v, ki, kd);
      telnet.printf("✔ altkp %.3f\r\n", v);
    } else {
      telnet.println("Enter altkp 0.0-30.0");
    }
    
  } else if (name == "altki") {
    float v = arg.toFloat();
    if (arg.length() && v >= 0.0f && v <= 3.0f) {
      float kp, kd, target, lpf;
      int limit, throttle;
      control.getAltitudeParams(target, kp, kp, kd, lpf, limit, throttle);
      control.setAltitudeGains(kp, v, kd);
      telnet.printf("✔ altki %.3f\r\n", v);
    } else {
      telnet.println("Enter altki 0.0-3.0");
    }
    
  } else if (name == "altkd") {
    float v = arg.toFloat();
    if (arg.length() && v >= 0.0f && v <= 80.0f) {
      float kp, ki, target, lpf;
      int limit, throttle;
      control.getAltitudeParams(target, kp, ki, ki, lpf, limit, throttle);
      control.setAltitudeGains(kp, ki, v);
      telnet.printf("✔ altkd %.3f\r\n", v);
    } else {
      telnet.println("Enter altkd 0.0-80.0");
    }
    
  } else if (name == "altlpf") {
    float v = arg.toFloat();
    if (arg.length() && v >= 0.0f && v <= 1.0f) {
      control.setAltitudeLPF(v);
      telnet.printf("✔ altlpf %.2f\r\n", v);
    } else {
      telnet.println("Enter altlpf 0.00-1.00");
    }
    
  } else if (name == "kp") {
    float v = arg.toFloat();
    if (arg.length() && v >= 0.0f && v <= 10.0f) {
      float ki, kd, kpp, kip, kdp;
      control.getAttitudeGains(v, ki, kd, kpp, kip, kdp);
      control.setAttitudeGains(v, ki, kd);
      telnet.printf("✔ Kp both axes = %.3f\r\n", v);
    } else {
      telnet.println("Enter Kp 0.0-10.0");
    }
    
  } else if (name == "ki") {
    float v = arg.toFloat();
    if (arg.length() && v >= 0.0f && v <= 5.0f) {
      float kp, kd, kpp, kip, kdp;
      control.getAttitudeGains(kp, v, kd, kpp, kip, kdp);
      control.setAttitudeGains(kp, v, kd);
      telnet.printf("✔ Ki both axes = %.3f\r\n", v);
    } else {
      telnet.println("Enter Ki 0.0-5.0");
    }
    
  } else if (name == "kd") {
    float v = arg.toFloat();
    if (arg.length() && v >= 0.0f && v <= 10.0f) {
      float kp, ki, kpp, kip, kdp;
      control.getAttitudeGains(kp, ki, v, kpp, kip, kdp);
      control.setAttitudeGains(kp, ki, v);
      telnet.printf("✔ Kd both axes = %.3f\r\n", v);
    } else {
      telnet.println("Enter Kd 0.0-10.0");
    }
    
  } else if (name == "kpr") {
    float v = arg.toFloat();
    if (arg.length() && v >= 0.0f && v <= 10.0f) {
      float ki, kd, kpp, kip, kdp;
      control.getAttitudeGains(v, ki, kd, kpp, kip, kdp);
      control.setRollGains(v, ki, kd);
      telnet.printf("✔ kpr %.3f\r\n", v);
    } else {
      telnet.println("Enter kpr 0.0-10.0");
    }
    
  } else if (name == "deadzone") {
    float v = arg.toFloat();
    if (arg.length() && v >= 0.0f && v <= 45.0f) {
      control.setDeadzone(v);
      telnet.printf("✔ deadzone %.2f deg\r\n", v);
    } else {
      telnet.println("Enter deadzone 0-45 (deg)");
    }
    
  } else if (name == "yaw") {
    if (arg == "on") {
      control.setYawEnabled(true);
      telnet.println("✔ yaw ON");
    } else if (arg == "off") {
      control.setYawEnabled(false);
      telnet.println("✖ yaw OFF");
    } else {
      telnet.println("yaw on|off");
    }
    
  } else if (name == "esc1") {
    int v = arg.toInt();
    if (arg.length() && v >= 0 && v <= 1000) {
      int trim1, trim2;
      actuators.getESCTrims(trim1, trim2);
      actuators.setESCTrims(v, trim2);
      telnet.printf("✔ ESC1 trim %d µs\r\n", v);
    } else {
      telnet.println("Enter esc1 0-1000");
    }
    
  } else if (name == "esc2") {
    int v = arg.toInt();
    if (arg.length() && v >= 0 && v <= 1000) {
      int trim1, trim2;
      actuators.getESCTrims(trim1, trim2);
      actuators.setESCTrims(trim1, v);
      telnet.printf("✔ ESC2 trim %d µs\r\n", v);
    } else {
      telnet.println("Enter esc2 0-1000");
    }
    
  } else if (name == "flow") {
    telnet.printf("flow qual %u  vx %.2f m/s  vy %.2f m/s  range %.1f cm (qual %u)\r\n",
                  msp.getFlowQuality(), msp.getVelocityX(), msp.getVelocityY(),
                  msp.getHeight() * 100.0f, msp.getRangeQuality());
    
  } else if (cmd.length()) {
    telnet.println(
      "Commands: start | stop | status | throttle <µs> | alt <cm> | "
      "kp <v> | ki <v> | kd <v> | yaw on|off | flow"
    );
  }
}
