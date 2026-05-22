#ifndef CONFIG_H
#define CONFIG_H

// --- Sensors Tuning ---

// Distance in cm to trigger the buzzer warning
#define DISTANCE_WARNING_THRESHOLD 30

// Ultrasonic range in cm
#define ULTRASONIC_MAX_RANGE 80

// LiDAR range in cm
#define LIDAR_MIN_RANGE 3
#define LIDAR_MAX_RANGE 150

// Invalid sensor distance value
#define INVALID_DISTANCE 0xFFFF

// --- I2C Device Addresses ---
#define VL53L0X_ADDR 0x29
#define LCD1602_ADDR 0x27

#endif // CONFIG_H
