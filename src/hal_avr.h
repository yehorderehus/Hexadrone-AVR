#ifndef HAL_AVR_H
#define HAL_AVR_H

#include <avr/io.h>
#include <stdint.h>
#include <stdbool.h>

// Hardware Abstraction Layer for AVR microcontroller
// Provides low-level access to peripherals used by the application.

// --- Buzzer ----------------------------------------------------------------
void HAL_Buzzer_Init(void);
void HAL_Buzzer_SetFreq(uint16_t freq_val);

// --- I2C/TWI ---------------------------------------------------------------
void twi0_init(void);
bool twi0_wait_write(void);
bool twi0_wait_read_ready(void);
bool i2c_write_reg(uint8_t dev7, uint8_t reg, uint8_t value);
bool i2c_read_regs(uint8_t dev7, uint8_t start_reg, uint8_t *buf, uint8_t len);

// --- Ultrasonic sensor -----------------------------------------------------
void HAL_Ultrasonic_Init(void);
void HAL_Ultrasonic_Trigger(void);

// External variables used by the ultrasonic ISR
extern volatile uint16_t echo_duration;
extern volatile uint8_t echo_ready;

// --- Button input ----------------------------------------------------------
void HAL_Button_Init(void);
bool HAL_Button_GetAndClearEvent(void);

// --- SPI / IMU -------------------------------------------------------------
void spi0_init(void);
uint8_t spi0_transfer(uint8_t data);
void imu_write_reg(uint8_t reg, uint8_t data);
void imu_read_regs(uint8_t reg, uint8_t *buf, uint8_t len);

// --- System Time -----------------------------------------------------------
void HAL_Time_Init(void);
uint32_t HAL_GetMillis(void);

#endif // HAL_AVR_H
