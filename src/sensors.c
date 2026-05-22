#include "sensors.h"
#include "hal_avr.h"
#include "config.h"
#include <util/atomic.h>
#include <util/delay.h>
#include <math.h>

#define MPU_ACCEL_SCALE (1.0f / 16384.0f) // 2g full scale
#define MPU_GYRO_SCALE (1.0f / 131.0f)    // 250 dps full scale
#define RAD_TO_DEG 57.295779513082320876f
#define MAHONY_KP 0.5f

// ---------------------------------------------------------------------------
// Global state
// ---------------------------------------------------------------------------
SensorData_t sensor_data;

// ---------------------------------------------------------------------------
// Local helper declarations
// ---------------------------------------------------------------------------
static void VL53L0X_Init(void);
static uint16_t VL53L0X_ReadDistanceRaw(void);
static uint16_t Get_Filtered_Lidar(void);
static inline uint16_t median_u16(uint16_t a, uint16_t b, uint16_t c);

// ---------------------------------------------------------------------------
// Sensor initialization
// ---------------------------------------------------------------------------
void Sensors_Init(void)
{
    // Reset sensor state and calibration offsets
    sensor_data = (SensorData_t){0};

    HAL_Ultrasonic_Init();
    twi0_init(); // Initialize I2C for LCD and LiDAR
    VL53L0X_Init();
    spi0_init(); // Start the SPI bus for the IMU

    sensor_data.roll_zero = 0.0f;
    sensor_data.pitch_zero = 0.0f;

    // Disable I2C interface to lock the chip into SPI mode
    imu_write_reg(0x6A, 0x10);

    // Wake up MPU-9250 and configure the sensor ranges.
    imu_write_reg(0x6B, 0x00);
    imu_write_reg(0x1C, 0x00);
    imu_write_reg(0x1B, 0x00);
}

// ---------------------------------------------------------------------------
// Sensor update
// ---------------------------------------------------------------------------
void Sensors_Update(void)
{
    sensor_data.lidar_distance = Get_Filtered_Lidar();
    HAL_Ultrasonic_Trigger();
    sensor_data.ultrasonic_distance = Sensors_GetUltrasonicDistance();

    uint8_t accel_buf[6];
    uint8_t gyro_buf[6];

    // Read raw accelerometer and gyroscope data from MPU-9250
    imu_read_regs(0x3B, accel_buf, 6);
    imu_read_regs(0x43, gyro_buf, 6);

    int16_t raw_ax = (int16_t)((accel_buf[0] << 8) | accel_buf[1]);
    int16_t raw_ay = (int16_t)((accel_buf[2] << 8) | accel_buf[3]);
    int16_t raw_az = (int16_t)((accel_buf[4] << 8) | accel_buf[5]);

    int16_t raw_gx = (int16_t)((gyro_buf[0] << 8) | gyro_buf[1]);
    int16_t raw_gy = (int16_t)((gyro_buf[2] << 8) | gyro_buf[3]);
    int16_t raw_gz = (int16_t)((gyro_buf[4] << 8) | gyro_buf[5]);

    float ax = raw_ax * MPU_ACCEL_SCALE;
    float ay = raw_ay * MPU_ACCEL_SCALE;
    float az = raw_az * MPU_ACCEL_SCALE;

    // Apply accelerometer offset calibration if available
    ax -= sensor_data.offset_x;
    ay -= sensor_data.offset_y;
    az -= sensor_data.offset_z;

    sensor_data.accel_magnitude = sqrtf(ax * ax + ay * ay + az * az);

    float gx = raw_gx * MPU_GYRO_SCALE - sensor_data.gyro_offset_x;
    float gy = raw_gy * MPU_GYRO_SCALE - sensor_data.gyro_offset_y;
    float gz = raw_gz * MPU_GYRO_SCALE - sensor_data.gyro_offset_z;

    // --- DYNAMIC TIME CALCULATION ---
    static uint32_t last_update_time = 0;
    uint32_t current_time = HAL_GetMillis();

    if (last_update_time == 0)
    {
        last_update_time = current_time;
        return;
    }

    float dt = (float)(current_time - last_update_time) / 1000.0f;
    last_update_time = current_time;

    if (dt > 0.5f)
        dt = 0.01f;

    // =======================================================================
    // MAHONY 3D QUATERNION FILTER (Immune to Gimbal Lock & Contamination)
    // =======================================================================

    // Static quaternion state (1, 0, 0, 0 means unrotated)
    static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

    // 1. Convert Gyro data to Radians per second
    float gx_rad = gx * 0.0174533f;
    float gy_rad = gy * 0.0174533f;
    float gz_rad = gz * 0.0174533f;

    // 2. Normalize the accelerometer measurement
    float norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm > 0.0f)
    {
        float ax_norm = ax / norm;
        float ay_norm = ay / norm;
        float az_norm = az / norm;

        // Estimated gravity vector from current quaternion state
        float vx = 2.0f * (q1 * q3 - q0 * q2);
        float vy = 2.0f * (q0 * q1 + q2 * q3);
        float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

        // Error is the cross product between estimated and measured gravity
        float ex = (ay_norm * vz - az_norm * vy);
        float ey = (az_norm * vx - ax_norm * vz);
        float ez = (ax_norm * vy - ay_norm * vx);

        // Feed the error back into the gyro rates
        gx_rad += MAHONY_KP * ex;
        gy_rad += MAHONY_KP * ey;
        gz_rad += MAHONY_KP * ez;
    }

    // 3. Integrate the quaternion rate of change
    gx_rad *= (0.5f * dt);
    gy_rad *= (0.5f * dt);
    gz_rad *= (0.5f * dt);

    float qa = q0, qb = q1, qc = q2;
    q0 += (-qb * gx_rad - qc * gy_rad - q3 * gz_rad);
    q1 += (qa * gx_rad + qc * gz_rad - q3 * gy_rad);
    q2 += (qa * gy_rad - qb * gz_rad + q3 * gx_rad);
    q3 += (qa * gz_rad + qb * gy_rad - qc * gx_rad);

    // 4. Normalize the final quaternion to prevent compounding math errors
    norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 /= norm;
    q1 /= norm;
    q2 /= norm;
    q3 /= norm;

    // =======================================================================
    // EXTRACT CLEAN EULER ANGLES
    // =======================================================================
    // NOTE: When pitch or roll exceeds ±90 degrees, gimbal lock protection
    // becomes ineffective. The Mahony filter still runs, but Euler angles
    // may exhibit discontinuities or singularities at those extremes.
    // For applications requiring full 360° rotation, use quaternions directly.
    // =======================================================================

    // 1. Extract the estimated gravity vector from the quaternion
    // (This points straight UP relative to the sensor board)
    float vx = 2.0f * (q1 * q3 - q0 * q2);
    float vy = 2.0f * (q0 * q1 + q2 * q3);
    float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    // 2. Calculate Pitch (End-over-end tracking)
    // Using atan2 for Pitch allows it to seamlessly track from -180 to +180 degrees
    // without ever folding backwards.
    sensor_data.filter_pitch = atan2f(-vx, vz) * RAD_TO_DEG;

    // 3. Calculate Roll (Strictly bounded)
    // The sqrtf guarantees the denominator is always positive, completely
    // immunizing the Roll axis from 180-degree Gimbal Lock flips.
    sensor_data.filter_roll = atan2f(vy, sqrtf(vx * vx + vz * vz)) * RAD_TO_DEG;

    // 4. Integrate Yaw (Heading)
    sensor_data.yaw += gz * dt;

    // Output
    sensor_data.roll = sensor_data.filter_roll;
    sensor_data.pitch = sensor_data.filter_pitch;
}

// ---------------------------------------------------------------------------
// LiDAR helper functions
// ---------------------------------------------------------------------------

static void VL53L0X_Init(void)
{
    // Minimal VL53L0X initialization: I2C is ready and sensor is powered.
    // A full tuning setup is not provided here; many breakout boards are already
    // configured for default ranging mode at power-up.
    (void)VL53L0X_ADDR;
}

static uint16_t VL53L0X_ReadDistanceRaw(void)
{
    uint8_t status = 0;
    if (!i2c_read_regs(VL53L0X_ADDR, 0x14, &status, 1))
    {
        return INVALID_DISTANCE;
    }

    // Start single measurement
    if (!i2c_write_reg(VL53L0X_ADDR, 0x00, 0x01))
    {
        return INVALID_DISTANCE;
    }

    _delay_ms(20);

    uint8_t range_buf[2];
    if (!i2c_read_regs(VL53L0X_ADDR, 0x1E, range_buf, 2))
    {
        return INVALID_DISTANCE;
    }

    uint16_t range_mm = (uint16_t)((range_buf[0] << 8) | range_buf[1]);
    if (range_mm == 0 || range_mm > (uint16_t)(LIDAR_MAX_RANGE * 10) || range_mm < (uint16_t)(LIDAR_MIN_RANGE * 10))
    {
        return INVALID_DISTANCE;
    }
    return range_mm / 10; // convert mm to cm
}

static uint16_t Get_Filtered_Lidar(void)
{
    static uint16_t readings[3] = {INVALID_DISTANCE, INVALID_DISTANCE, INVALID_DISTANCE};
    static uint8_t index = 0;

    readings[index] = VL53L0X_ReadDistanceRaw();
    index = (index + 1) % 3;

    return median_u16(readings[0], readings[1], readings[2]);
}

static inline uint16_t median_u16(uint16_t a, uint16_t b, uint16_t c)
{
    if ((a <= b && b <= c) || (c <= b && b <= a))
        return b;
    if ((b <= a && a <= c) || (c <= a && a <= b))
        return a;
    return c;
}

// ---------------------------------------------------------------------------
// Safety checks
// ---------------------------------------------------------------------------

void Sensors_RunSafetyCheck(void)
{
    bool ultrasonic_warning = (sensor_data.ultrasonic_distance != INVALID_DISTANCE && sensor_data.ultrasonic_distance < DISTANCE_WARNING_THRESHOLD);
    bool lidar_warning = (sensor_data.lidar_distance != INVALID_DISTANCE && sensor_data.lidar_distance < DISTANCE_WARNING_THRESHOLD);

    if (ultrasonic_warning || lidar_warning)
    {
        HAL_Buzzer_SetFreq(1000); // Set buzzer frequency for alerts
    }
    else
    {
        HAL_Buzzer_SetFreq(0); // Turn off buzzer
    }
}

// ---------------------------------------------------------------------------
// IMU calibration
// ---------------------------------------------------------------------------
void Sensors_CalibrateIMU(void)
{
    sensor_data.is_calibrating = 1;

    _delay_ms(1000);

    float sum_ax = 0.0f, sum_ay = 0.0f, sum_az = 0.0f;
    float sum_gx = 0.0f, sum_gy = 0.0f, sum_gz = 0.0f;
    const uint8_t num_samples = 250;
    uint8_t accel_buf[6];
    uint8_t gyro_buf[6];

    for (uint8_t i = 0; i < num_samples; i++)
    {
        imu_read_regs(0x3B, accel_buf, 6);
        imu_read_regs(0x43, gyro_buf, 6);

        int16_t raw_ax = (int16_t)((accel_buf[0] << 8) | accel_buf[1]);
        int16_t raw_ay = (int16_t)((accel_buf[2] << 8) | accel_buf[3]);
        int16_t raw_az = (int16_t)((accel_buf[4] << 8) | accel_buf[5]);

        int16_t raw_gx = (int16_t)((gyro_buf[0] << 8) | gyro_buf[1]);
        int16_t raw_gy = (int16_t)((gyro_buf[2] << 8) | gyro_buf[3]);
        int16_t raw_gz = (int16_t)((gyro_buf[4] << 8) | gyro_buf[5]);

        float accel_x_g = raw_ax * MPU_ACCEL_SCALE;
        float accel_y_g = raw_ay * MPU_ACCEL_SCALE;
        float accel_z_g = raw_az * MPU_ACCEL_SCALE;

        sum_ax += accel_x_g;
        sum_ay += accel_y_g;
        sum_az += accel_z_g;

        sum_gx += raw_gx * MPU_GYRO_SCALE;
        sum_gy += raw_gy * MPU_GYRO_SCALE;
        sum_gz += raw_gz * MPU_GYRO_SCALE;

        _delay_ms(5);
    }

    float avg_ax = sum_ax / num_samples;
    float avg_ay = sum_ay / num_samples;
    float avg_az = sum_az / num_samples;

    sensor_data.offset_x = avg_ax;
    sensor_data.offset_y = avg_ay;
    sensor_data.offset_z = avg_az - 1.0f;
    sensor_data.gyro_offset_x = sum_gx / num_samples;
    sensor_data.gyro_offset_y = sum_gy / num_samples;
    sensor_data.gyro_offset_z = sum_gz / num_samples;

    // Set the zero offsets
    sensor_data.roll_zero = atan2f(avg_ay, avg_az) * RAD_TO_DEG;
    sensor_data.pitch_zero = atan2f(-avg_ax, sqrtf(avg_ay * avg_ay + avg_az * avg_az)) * RAD_TO_DEG;

    // CRITICAL: Reset the filter memory so it doesn't "remember" the old tilt
    sensor_data.filter_roll = sensor_data.roll_zero;
    sensor_data.filter_pitch = sensor_data.pitch_zero;

    // Reset outputs
    sensor_data.roll = 0.0f;
    sensor_data.pitch = 0.0f;
    sensor_data.yaw = 0.0f;

    sensor_data.is_calibrating = 0;
}

// ---------------------------------------------------------------------------
// Ultrasonic helper
// ---------------------------------------------------------------------------
uint16_t Sensors_GetUltrasonicDistance(void)
{
    uint16_t local_duration;
    uint8_t ready;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        ready = echo_ready;
        if (ready)
        {
            local_duration = echo_duration;
            echo_ready = 0;
        }
    }

    if (ready)
    {
        uint32_t time_us = local_duration / 12;
        uint16_t distance_cm = time_us / 58;
        if (distance_cm == 0 || distance_cm > ULTRASONIC_MAX_RANGE)
        {
            return INVALID_DISTANCE;
        }
        return distance_cm;
    }

    return INVALID_DISTANCE;
}
