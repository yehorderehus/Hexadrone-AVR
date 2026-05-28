# 🤖 Hexadrone Standalone Safety Hub

[▶️ Video Demonstration](https://www.youtube.com/watch?v=5gf2d8lJZK4)

This project utilizes an `AVR128DB48` microcontroller to manage a 5-peripheral telemetry system for hexadrone operations.

## System Architecture

The project follows a modular design, separating hardware interactions from the intelligent logic layer.

```text
├── KiCad/               # Circuit Design
├── media/               # Project media & documentation assets
├── src/
│   ├── main.c           # System Orchestrator & Loop Control
│   ├── hal_avr.c/h      # Hardware Abstraction Layer (Low-level peripherals)
│   ├── lcd.c/h          # LCD Driver (I2C)
│   ├── sensors.c/h      # Intelligent Driver Layer (IMU, LiDAR, Ultrasonic)
│   └── telemetry.c/h    # Real-time status UI & Data Formatting
├── Scheme.pdf           # Electrical schematic
├── README.md            # Documentation
└── LICENSE              # Project license
```

## The 5-Peripheral Safety Matrix

The hub manages five distinct peripherals through varied communication protocols:

* **LiDAR (VL53L0X):** Precision range finding via I2C with median filtering to mitigate noise artifacts.
* **LCD (1602 + PCF8574):** Real-time telemetry dashboard sharing the I2C bus.
* **IMU (MPU-9250):** 9-axis motion tracking via SPI with button-triggered calibration.
* **Ultrasonic (HC-SR04):** Wide-angle proximity detection via dedicated hardware timer/capture.
* **Passive Buzzer:** Audio alert system driven by TCA0 PWM for proximity warnings.

## Telemetry Dashboard

The system provides a comprehensive, two-line status display to monitor navigation safety and drone attitude simultaneously:

**Line 1: Environmental Awareness**
`L:###cm U:###cm`
Displays the filtered distance to obstacles from both the LiDAR and Ultrasonic sensors.

**Line 2: Attitude & Orientation**
`G#.## P:+## R:+##`

* **G**: Calibrated accelerometer magnitude (in Gs).
* **P**: Fused Pitch angle (degrees).
* **R**: Fused Roll angle (degrees).

## Calibration Routine

* **Procedure**: Place the hexadrone on a level, stationary surface.
* **Trigger**: Press the hardware button.
* **Feedback**: The system provides audio cues (buzzer) to signal the start and successful completion of the IMU calibration, as well as a display message.
