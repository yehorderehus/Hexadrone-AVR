#include "sensors.h"
#include "hal_avr.h"
#include <util/atomic.h>
#include <math.h> // for sqrtf

// Assuming MPU6050 library is available
// Include appropriate headers for MPU6050
// #include "mpu6050.h"

// MPU6050 device and data structures (placeholders)
typedef struct {
    int16_t ax, ay, az;
} imu_raw_t;

// Assuming mpu_dev is initialized elsewhere or here
// static mpu6050_t mpu_dev; // Placeholder

#define ACCEL_SCALE 9.81f / 16384.0f // Assuming 2g range, adjust as needed

// Global sensor data
SensorData_t sensor_data;

// Initialize sensors
void Sensors_Init(void) {
    // Initialize ultrasonic sensor
    HAL_Ultrasonic_Init();

    // Initialize I2C for IMU
    twi0_init();

    // Initialize MPU6050 (placeholder)
    // mpu6050_init(&mpu_dev);
}

// Update sensor readings
void Sensors_Update(void) {
    // Trigger ultrasonic sensor
    HAL_Ultrasonic_Trigger();

    // Read ultrasonic distance
    sensor_data.ultrasonic_distance = Sensors_GetUltrasonicDistance();

    // Read IMU data
    imu_raw_t imu;
    // if (mpu6050_read_raw(&mpu_dev, &imu)) {
    //     sensor_data.accel_x = (float)imu.ax * ACCEL_SCALE;
    //     sensor_data.accel_y = (float)imu.ay * ACCEL_SCALE;
    //     sensor_data.accel_z = (float)imu.az * ACCEL_SCALE;
    //     sensor_data.accel_magnitude = sqrtf(sensor_data.accel_x * sensor_data.accel_x +
    //                                        sensor_data.accel_y * sensor_data.accel_y +
    //                                        sensor_data.accel_z * sensor_data.accel_z);
    // }

    // Placeholder values for now
    sensor_data.accel_x = 0.0f;
    sensor_data.accel_y = 0.0f;
    sensor_data.accel_z = 9.81f;
    sensor_data.accel_magnitude = 9.81f;
}

// Run safety checks and alert if necessary
void Sensors_RunSafetyCheck(void) {
    // Check ultrasonic distance
    if (sensor_data.ultrasonic_distance < 50) { // Example threshold: 50cm
        HAL_Buzzer_SetFreq(1000); // Set buzzer frequency for alert
    } else {
        HAL_Buzzer_SetFreq(0); // Turn off buzzer
    }

    // Add more safety checks as needed (e.g., IMU for free fall)
}

// Get ultrasonic distance safely
uint16_t Sensors_GetUltrasonicDistance(void) {
    uint16_t local_duration;
    uint8_t ready;

    // Safely grab the data using ATOMIC_BLOCK
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        ready = echo_ready;
        if (ready) {
            local_duration = echo_duration;
            echo_ready = 0; // Reset flag
        }
    }

    if (ready) {
        // Convert ticks to microseconds (assuming 12MHz clock, 1 tick = 1/12 us)
        uint32_t time_us = local_duration / 12;

        // Convert microseconds to centimeters (speed of sound ~343 m/s, round trip)
        uint16_t distance_cm = time_us / 58;
        return distance_cm;
    }

    return 0xFFFF; // Return large number if no new data
}
