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
  
  bool hasClient() { return telnet && telnet.connected(); }  // REMOVED const
  bool shouldSendTelemetry() { return telnet.peek() == -1; }  // REMOVED const
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
