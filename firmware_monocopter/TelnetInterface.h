#ifndef TELNET_INTERFACE_H
#define TELNET_INTERFACE_H

#include <WiFi.h>

class ControlSystem;
class Actuators;
class MSPProtocol;

class TelnetInterface {
public:
  TelnetInterface();
  void begin();
  void update(bool& ctrlEnabled, ControlSystem& control, 
             Actuators& actuators, MSPProtocol& msp);
  
  bool hasClient() const { return telnet && telnet.connected(); }
  bool shouldSendTelemetry() const { return telnet.peek() == -1; }
  void sendTelemetry(float roll, float pitch, float height, int throttle);
  
private:
  WiFiServer telnetServer;
  WiFiClient telnet;
  String cmd;
  
  void processCommand(const String& name, const String& arg,
                     bool& ctrlEnabled, ControlSystem& control,
                     Actuators& actuators, MSPProtocol& msp);
};

#endif
