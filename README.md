# 🤖 Hexadrone Standalone Safety Hub

This project utilizes an `AVR128DB48` microcontroller to manage a 5-peripheral safety and telemetry system.

## File Structure Overview

```text
├── src/
│   ├── main.c           # System Orchestrator & Loop Control
│   ├── hal_avr.c/h      # Hardware Abstraction Layer (The "Silicon" level)
│   ├── lcd.c/h          # LCD Driver
│   ├── sensors.c/h      # Intelligent Driver Layer (The "Brain" level)
│   └── telemetry.c/h    # Communication & UI Layer (The "Output" level)
├── Scheme.pdf           # Electrical Schematic
└── README.md            # Project Description
```

## The 5-Peripheral Safety Matrix

The hub manages five distinct peripherals through varied communication protocols:

    LiDAR (VL53L0X): High-precision laser ranging via I2C.

    LCD (1602 + PCF8574): Real-time status UI sharing the I2C bus.

    IMU (MPU-9250): 9-axis motion and orientation tracking via SPI.

    Ultrasonic (HC-SR04): Wide-angle proximity detection via GPIO/Timing.

    Passive Buzzer: Audio alert system driven by TCA0 (PWM).

## Display

```text
+----------------+
|LID:125cm   [OK]|
|ULT: 45cm P:-05 |
+----------------+
```
