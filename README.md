# Monocopter Flight Controller

[![Compile Firmware](https://github.com/jasonangelov/monocopter_flight_controller/actions/workflows/compile.yml/badge.svg)](https://github.com/jasonangelov/monocopter_flight_controller/actions/workflows/compile.yml)


ESP32-based autonomous flight controller for a single-rotor drone with thrust vectoring control.

## 1) Introduction
### 1.1 What is a monocopter?
- One thrust column, **vectoring fins** steer the vehicle

### 1.2 Project goals & aspirations
- **Steady hover: ** Achieve stable hover with minimal drift.
- **Controllable movement:** Accurately follow live commands to control position, height, and velocity.
- **Autonomous landing:** Automatically land/return.
- **Robust recovery** after bumps/tilts

### 1.3 Core challenges (what makes this hard)
- **Yaw control:** When propellers spin, conservation of angular momentum causes the chassis to spin (yaw).
- **Gyroscopic precession:** External torques cause precession due if the chassis has none zero yaw angular momentum.
- **Coupled aerodynamics**: fin moves affect both lift and rotation.
- **Balanced weight:** Center of mass is colinear with chassis/propellers.
- **Control system**: Sensor fusion + PID control. Tuning PID values.
- **CAD design + Hardware**: Wire management, heat dissipation, power delivery.

---

## 2) Current Design (Latest Version: Coaxial, Counter-Rotating)
### 2.1 Engineering design
- **Airframe**: ducted, coaxial contra-rotating props for torque cancellation.
- **Actuation**: four fin servos for roll/pitch.
- **Yaw control** Differential thrust compensates for any yaw rotations.
- **Compute/Comms**: ESP32, Wi-Fi/Telnet for live telemetry & tuning

### 2.2 Hardware at a glance
- **MCU**: ESP32-WROOM-32  
- **Sensors**: IMU (BNO055) for attitude, LiDAR (VL53L0X) for altitude
- **Power**: clean 5 V servo rail, shared grounds.
- 4x servo-controlled fins
- 2x brushless motors with ESCs

> _Link to: BOM.md, wiring diagram image, CAD/STLs_

### 2.3 Software & control loop
- **200 Hz** cascade PID control loop:
  - IMU-based attitude stabilization
  - Differential thrust yaw control
  - Altitude hold using LiDAR
- WiFi telemetry and tuning interface

> _Link to: docs/controls.md (overview), docs/commands.md (CLI/Telnet)_

### 2.4 Current status (what it can do today)
-  Reliable lift, level attitude, altitude hold

### 2.5 Necessary future improvements
- **Reduced lateral drift:** Use optical flow sensor + gps to remove sideways drift. 

# Flight Videos

<table>
  <tr>
    <td align="center" width="33%">
      <a href="https://www.youtube.com/watch?v=riI-cIHnShg">
        <img src="https://img.youtube.com/vi/riI-cIHnShg/hqdefault.jpg" alt="Trial 1 thumbnail" width="280"/>
      </a><br/>
      <sub><b>Trial 1</b> — <i>Short caption here (e.g., indoor hover test)</i></sub>
    </td>
    <td align="center" width="33%">
      <a href="https://youtu.be/cCDeujReH3Y">
        <img src="https://img.youtube.com/vi/cCDeujReH3Y/hqdefault.jpg" alt="Trial 2 thumbnail" width="280"/>
      </a><br/>
      <sub><b>Trial 2</b> — <i>Short caption here (e.g., altitude hold + drift)</i></sub>
    </td>
    <td align="center" width="33%">
      <a href="https://www.youtube.com/watch?v=jJXlnwY6VRc">
        <img src="https://img.youtube.com/vi/jJXlnwY6VRc/hqdefault.jpg" alt="Trial 3 thumbnail" width="280"/>
      </a><br/>
      <sub><b>Trial 3</b> — <i>Short caption here (e.g., disturbance recovery)</i></sub>
    </td>
  </tr>
</table>


## CAD DESIGN: Current Version (V4)

<table>
  <tr>
    <td align="center" width="33%">
      <img src="pictures/V4/V4.png" alt="V4 overall assembly" width="280"/><br/>
      <sub><b>Overall Assembly (V4)</b> — ducted coaxial layout.</sub>
    </td>
    <td align="center" width="33%">
      <img src="pictures/V4/V4 Vanes.png" alt="V4 thrust-vectoring vanes" width="280"/><br/>
      <sub><b>Thrust-Vectoring Vanes</b> — four-fin control surfaces.</sub>
    </td>
    <td align="center" width="33%">
      <img src="pictures/V4/V4 Battery holder.png" alt="V4 battery holder" width="280"/><br/>
      <sub><b>Battery Holder</b> — centered for COM balance.</sub>
    </td>
  </tr>
</table>

---

## 3) Project History (What we tried, what failed, what we learned)

### 3.1 V1 — Single EDF + ESP32 (first prototype)
- **What we did**: IMU-only stabilization, single edf thrust, manual throttle.  
- **What failed**: torque-induced yaw due to single edf, poor weight distribution, wire management
- **Lesson**: Ensure center of mass is colinear with chassis/propellers, switch to wireless connection for efficient debugging/tuning


## V1 (Design & Early Tests)

<!-- Replace PATH_TO_V1_PRINT.png with your actual PNG path, e.g., pictures/V1/print.png -->
<table>
  <tr>
    <td align="center" width="33%">
      <a href="pictures/V1/v1_print.JPG">
        <img src="pictures/V1/v1_print.JPG" alt="V1 3D print" width="260"/>
      </a><br/>
      <sub><b>V1 Print</b> — <i>Early CAD/print layout</i></sub>
    </td>
    <td align="center" width="33%">
      <a href="https://youtube.com/shorts/o5gzwul9h58?feature=share">
        <img src="https://img.youtube.com/vi/o5gzwul9h58/hqdefault.jpg" alt="V1 design video thumbnail" width="260"/>
      </a><br/>
      <sub><b>V1 Design Video</b> — <i>Walkthrough of V1 concept</i></sub>
    </td>
    <td align="center" width="33%">
      <a href="https://youtube.com/shorts/oQ6uXIYUpS8?feature=share">
        <img src="https://img.youtube.com/vi/oQ6uXIYUpS8/hqdefault.jpg" alt="V1 flight video thumbnail" width="260"/>
      </a><br/>
      <sub><b>V1 Flight Clip</b> — <i>First hover attempts</i></sub>
    </td>
  </tr>
</table>




### 3.2 V2 — EDF + Pi Zero + 3D-printed gimbal (Reinforcement learning attempt)
- **What we did**: Designed and built a gimbal test rig; experimented with **reinforcement learning**.  
- **What failed**: Failed to compensate for both yaw spinning and attitude tilts, Poor training efficiency, RL training did not translate to real world environment, lack of RTOS on Raspberry Pi Zero + Python code.
- **Lesson**: Use two counter rotating propellers to eliminate yaw spin, use a traditional control system approach, better telemetry + debugging tools.


## V2 (Gimbal & String Rigs)
<table>
  <tr>
    <td align="center" width="50%">
      <a href="pictures/V2/v2_gimbal.JPG">
        <img src="pictures/V2/v2_gimbal.JPG" alt="V2 gimbal rig" width="320"/>
      </a><br/>
      <sub><b>V2 Gimbal Rig</b> — <i>Bench tuning on a hinge/gimbal mount.</i></sub>
    </td>
    <td align="center" width="50%">
      <a href="pictures/V2/v2_string.JPG">
        <img src="pictures/V2/v2_string.JPG" alt="V2 string test rig" width="320"/>
      </a><br/>
      <sub><b>V2 String Test</b> — <i>Free-hanging stability and recovery checks.</i></sub>
    </td>
  </tr>
</table>




### 3.3 V3 — Coaxial contra-rotating props (current)
- **What we did**: torque cancellation via coaxial pair, **differential thrust** for yaw, larger chassis diameter.
- **What worked**: Perfect yaw control, improved balance and weight distribution with larger chassis.
- **What’s next**: optical-flow to measure translational velocity and drift, gps for position control.

---

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


## Authors
- Jason Angelov - UCLA Computer Science
- William Crowhurst - UCB Physics
