<h2 align="center"><em><code>Rocket++</code></em></h2>
<p align="center">Flight code for the South Brunswick Aerospace Team - ARC 2025–2026</p>

## Overview
This repository contains the flight software for an autonomous airbrake control system used in our high-powered rocket. The goal is to dynamically predict apogee in real-time and deploy airbrakes at the optimal moment to hit a target altitude (228.6 meters or 750 feet) with high accuracy.

## Features
- Real-time altitude and acceleration tracking
- Predictive apogee simulation based on current flight conditions
- Autonomous airbrake deployment logic
- BME280 barometric pressure sensor integration
- MPU6050 accelerometer integration
- Servo-controlled airbrakes
- Clean data logging to serial

## Hardware Dependencies
- **BME280** barometric sensor (I2C)
- **MPU6050** accelerometer (I2C)
- **Servo motor** for airbrake deployment
- **Arduino-compatible board** (e.g. Uno, Nano, Teensy)

## Constants to Configure
Update these in the `.ino` file to match your rocket:

```cpp
#define PIN 1 // Servo control pin
#define CLOSED_ANGLE 0 // Servo angle for stowed brakes
#define DEPLOYED_ANGLE 90 // Servo angle for deployed brakes

const float mass = 1.0; // Rocket mass in kg
const float area_normal = 0.01; // Frontal area without brakes
const float Cd_normal = 0.75; // Drag coefficient (normal flight)

const float area_brake = 0.03; // Area with brakes deployed
const float Cd_brake = 1.5; // Drag coefficient with brakes deployed

#define TARGET_APOGEE_M 228.6 // Target apogee in meters
```

## Potential Updates
- Add real-time Kalman filtering for smoother velocity estimation
- Use flash memory logging
- Integrate deployment override switch
- Add fallback thresholds for manual deployment

## Usage
Simply upload the `.ino` to your Arduino-compatible board, make sure your I2C sensors are wired correctly, and power the servo with an appropriate external source.
