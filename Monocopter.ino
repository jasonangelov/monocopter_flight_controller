/**********************************************************************
 *  Monocopter hover controller – ESP32 + Adafruit BNO055 (I²C)
 *  + Matek 3901-L0X (PMW3901 + VL53L0X) via UART MSP v2/v1
 *  
 *  Main control loop and initialization
 *********************************************************************/
#include <WiFi.h>
#include <ESPmDNS.h>
#include <ESP32Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Wire.h>
#include <math.h>

#include "Config.h"
#include "MSPProtocol.h"
#include "ControlSystem.h"
#include "TelnetInterface.h"
#include "Actuators.h"

/* ---------- Objects ---------- */
Adafruit_BNO055 bno = Adafruit_BNO055(55);
MSPProtocol msp;
ControlSystem control;
TelnetInterface telnetInterface;
Actuators actuators;

/* ---------- State ---------- */
bool ctrlEnabled = false;
uint32_t lastLoopUs = 0;

/* ---------- Setup ---------- */
void setup() {
  Serial.begin(115200);
  
  // Initialize actuators
  actuators.init();
  
  // WiFi setup
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 8000) {
    delay(100);
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("✔ WiFi connected | IP %s\n", WiFi.localIP().toString().c_str());
  } else {
    WiFi.mode(WIFI_AP);
    WiFi.softAP("MonocopterAP", "12345678");
    Serial.printf("⚠ AP Mode | IP %s\n", WiFi.softAPIP().toString().c_str());
  }
  
  MDNS.begin("monocopter");
  
  // Initialize telnet
  telnetInterface.begin();
  
  // Initialize IMU
  Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
  if (!bno.begin()) {
    Serial.println(F("❌ BNO055 not detected"));
    while (1) delay(10);
  }
  bno.setExtCrystalUse(true);
  
  // Initialize MSP communication
  msp.begin();
  
  // Initialize control system
  control.init();
  
  lastLoopUs = micros();
  Serial.println(F("Ready - Connect via telnet to monocopter.local:23"));
}

/* ---------- Main Loop ---------- */
void loop() {
  // Handle telnet commands
  telnetInterface.update(ctrlEnabled, control, actuators, msp);
  
  // Update MSP sensors
  msp.update();
  
  // Control loop at ~200 Hz
  uint32_t now = micros();
  float dt = (now - lastLoopUs) * 1e-6f;
  if (dt >= 0.005f) {
    lastLoopUs = now;
    
    if (ctrlEnabled) {
      // Get IMU data
      imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
      
      float roll = euler.y();
      float pitch = euler.z();
      float gyroZ = gyro.z() * 57.2957795f; // Convert to deg/s
      
      // Get sensor data
      float heightM = msp.getHeight();
      uint8_t rangeQuality = msp.getRangeQuality();
      uint32_t rangeAge = msp.getRangeAge();
      
      // Run control algorithms
      control.updateAttitude(roll, pitch, dt);
      control.updateYaw(gyroZ, dt);
      control.updateAltitude(heightM, rangeQuality, rangeAge, roll, pitch, dt);
      
      // Get control outputs
      float rollCmd = control.getRollCommand();
      float pitchCmd = control.getPitchCommand();
      int throttle = control.getThrottleCommand();
      int yawDiff = control.getYawDifferential();
      
      // Apply to actuators
      actuators.writeFins(rollCmd, pitchCmd);
      actuators.setMotors(throttle, yawDiff);
      
      // Telemetry output
      if (telnetInterface.hasClient() && telnetInterface.shouldSendTelemetry()) {
        telnetInterface.sendTelemetry(roll, pitch, heightM, throttle);
      }
    } else {
      actuators.motorsOff();
    }
  }
}