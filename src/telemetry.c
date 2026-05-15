#include "telemetry.h"
#include "sensors.h"
#include "hal_avr.h"
#include "lcd.h"
#include <stdio.h>

// Initialize telemetry (LCD)
void Telemetry_Init(void) {
    LCD_Init();
    LCD_Clear();
}

// Send telemetry data to LCD and UART
void Telemetry_Send(void) {
    char buffer[17];

    // Display ultrasonic distance on LCD line 1
    snprintf(buffer, sizeof(buffer), "ULT:%3ucm   [OK]", sensor_data.ultrasonic_distance);
    LCD_Print(LCD_LINE1, buffer);

    // Display accelerometer magnitude on LCD line 2 (placeholder for pitch)
    int8_t pitch_placeholder = (int8_t)(sensor_data.accel_magnitude * 10); // Example
    snprintf(buffer, sizeof(buffer), "ACC:%.2f P:%03d", sensor_data.accel_magnitude, pitch_placeholder);
    LCD_Print(LCD_LINE2, buffer);

    // Send data via UART (optional, for debugging)
    // uart_write_str("Sensor data sent\n");
}
