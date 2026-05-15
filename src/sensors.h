#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>

// Sensor data structure
typedef struct {
    uint16_t ultrasonic_distance; // in cm
    float accel_x, accel_y, accel_z; // accelerometer values
    float accel_magnitude; // magnitude of acceleration
    // Add more sensor data as needed (LiDAR, IMU angles, etc.)
} SensorData_t;

// Global sensor data
extern SensorData_t sensor_data;

// Sensor module functions
void Sensors_Init(void);
void Sensors_Update(void);
void Sensors_RunSafetyCheck(void);
uint16_t Sensors_GetUltrasonicDistance(void);

#endif // SENSORS_H
