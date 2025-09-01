#ifndef CONFIG_H
#define CONFIG_H

/* ---------- WiFi Configuration ---------- */
// Use #define instead of const char* to avoid multiple definition errors
#define WIFI_SSID "YOUR_NETWORK"
#define WIFI_PASS "YOUR_PASSWORD"

/* ---------- Pin Definitions ---------- */
const int PIN_S1 = 14;   // Servo 1: +Roll
const int PIN_S2 = 27;   // Servo 2: -Roll  
const int PIN_S3 = 26;   // Servo 3: +Pitch
const int PIN_S4 = 25;   // Servo 4: -Pitch
const int PIN_ESC1 = 18; // ESC1 (CW)
const int PIN_ESC2 = 23; // ESC2 (CCW)

const int IMU_SDA_PIN = 16;
const int IMU_SCL_PIN = 17;

const int UART_RX = 33;  // ESP32 receives from sensor TX
const int UART_TX = 32;  // ESP32 transmits to sensor RX

/* ---------- PWM Constants ---------- */
const int SERVO_MIN   = 1100;
const int SERVO_MAX   = 1900;
const int SERVO_NEUT  = 1500;
const int SERVO_RANGE = 250;
const int ESC_OFF     = 1000;
const int ESC_HOVER   = 1200;

/* ---------- Control Constants ---------- */
const float DEG2RAD = 0.01745329252f;
const uint32_t ALT_VALID_MS = 300;  // LiDAR freshness window

#endif
