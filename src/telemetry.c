#include "telemetry.h"
#include "hal_avr.h"
#include "lcd.h"
#include "sensors.h"
#include "config.h"
#include <stdbool.h>
#include <stdio.h>
#include <math.h>

// Initialize telemetry
void Telemetry_Init(void)
{
    LCD_Init();
    LCD_Clear();
}

// Send telemetry data
void Telemetry_Send(void)
{
    char buffer[17];

    // === Line 1: Distance sensors (LiDAR + Ultrasonic) ===
    bool lidar_ok = (sensor_data.lidar_distance != INVALID_DISTANCE);
    bool ultra_ok = (sensor_data.ultrasonic_distance != INVALID_DISTANCE);

    // Format distance line based on sensor validity
    if (!lidar_ok && !ultra_ok)
    {
        snprintf(buffer, sizeof(buffer), "L:---cm U:---cm");
    }
    else if (!lidar_ok)
    {
        snprintf(buffer, sizeof(buffer), "L:---cm U:%3ucm", sensor_data.ultrasonic_distance);
    }
    else if (!ultra_ok)
    {
        snprintf(buffer, sizeof(buffer), "L:%3ucm U:---cm", sensor_data.lidar_distance);
    }
    else
    {
        snprintf(buffer, sizeof(buffer), "L:%3ucm U:%3ucm",
                 sensor_data.lidar_distance, sensor_data.ultrasonic_distance);
    }
    LCD_Print(LCD_LINE1, buffer);

    // === Line 2: Accelerometer, Pitch, and Roll ===
    snprintf(buffer, sizeof(buffer), "G%.1f P%+03d R%+03d",
             sensor_data.accel_magnitude,
             (int)sensor_data.pitch,
             (int)sensor_data.roll);
    LCD_Print(LCD_LINE2, buffer);
}
