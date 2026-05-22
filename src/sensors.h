#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>

// Sensor data structure
// Holds the current processed values for the sensor subsystem.
typedef struct
{
    uint16_t ultrasonic_distance;    // ultrasonic distance in cm or INVALID_DISTANCE
    uint16_t lidar_distance;         // VL53L0X distance in cm or INVALID_DISTANCE

    float accel_magnitude;           // calibrated acceleration magnitude in G

    float roll;                      // fused roll angle in degrees
    float pitch;                     // fused pitch angle in degrees
    float yaw;                       // integrated yaw angle in degrees

    float filter_roll;               // internal roll state for complementary filter
    float filter_pitch;              // internal pitch state for complementary filter

    float roll_zero;                 // roll calibration offset
    float pitch_zero;                // pitch calibration offset

    float offset_x;                  // accelerometer bias offset in G
    float offset_y;                  // accelerometer bias offset in G
    float offset_z;                  // accelerometer bias offset in G

    float gyro_offset_x;             // gyro bias offset in dps
    float gyro_offset_y;             // gyro bias offset in dps
    float gyro_offset_z;             // gyro bias offset in dps

    uint8_t is_calibrating;          // nonzero while IMU calibration is running
} SensorData_t;

// Global sensor data
extern SensorData_t sensor_data;

// Sensor module functions
void Sensors_Init(void);
void Sensors_Update(void);
void Sensors_RunSafetyCheck(void);
void Sensors_CalibrateIMU(void);
uint16_t Sensors_GetUltrasonicDistance(void);

#endif // SENSORS_H
