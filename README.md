# Monocopter Flight Controller

[![Compile Firmware](https://github.com/jasonangelov/monocopter_flight_controller/actions/workflows/compile.yml/badge.svg)](https://github.com/jasonangelov/monocopter_flight_controller/actions/workflows/compile.yml)


ESP32-based autonomous flight controller for a single-rotor drone with thrust vectoring control.

## Features
- 200Hz cascade PID control loop
- IMU-based attitude stabilization (BNO055)
- LiDAR altitude hold (VL53L0X)
- WiFi telemetry and tuning interface
- Differential thrust yaw control

## Work in progress 
- Optical flow positioning (PMW3901)

## Hardware
- ESP32-WROOM-32
- Adafruit BNO055 IMU
- Matek 3901-L0X Optical Flow
- 4x servo-controlled fins
- 2x brushless motors with ESCs

## Setup
1. Install Arduino IDE with ESP32 support
2. Install required libraries (ESP32Servo, Adafruit BNO055)
3. Configure WiFi credentials in Config.h
4. Upload to ESP32
5. Connect via telnet to monocopter.local:23

## Commands
- `start` - Begin flight
- `stop` - Emergency stop
- `status` - View all parameters
- `throttle <1000-2000>` - Set base throttle
- `alt <cm>` - Set altitude target

# Flight Videos

### Trial 1
[![](https://img.youtube.com/vi/riI-cIHnShg/hqdefault.jpg)](https://www.youtube.com/watch?v=riI-cIHnShg)

### Trial 2
[![](https://img.youtube.com/vi/cCDeujReH3Y/hqdefault.jpg)](https://youtu.be/cCDeujReH3Y)

### Trial 3
[![](https://img.youtube.com/vi/jJXlnwY6VRc/hqdefault.jpg)](https://www.youtube.com/watch?v=jJXlnwY6VRc)


## Authors
- Jason Angelov - UCLA Computer Science
- William Crowhurst - UCB Physics
