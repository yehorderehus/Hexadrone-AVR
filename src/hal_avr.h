#ifndef HAL_AVR_H
#define HAL_AVR_H

#include <avr/io.h>
#include <stdint.h>
#include <stdbool.h>

// Hardware Abstraction Layer for AVR microcontroller
// Provides low-level functions for peripherals: Buzzer, I2C, UART, Ultrasonic sensor

// Buzzer functions
void HAL_Buzzer_Init(void);
void HAL_Buzzer_SetFreq(uint16_t freq_val);

// I2C/TWI functions
void twi0_init(void);
bool twi0_wait_write(void);
bool twi0_wait_read_ready(void);
bool i2c_write_reg(uint8_t dev7, uint8_t reg, uint8_t value);
bool i2c_read_regs(uint8_t dev7, uint8_t start_reg, uint8_t *buf, uint8_t len);

// UART functions
void uart_init(uint32_t f_cpu, uint32_t baud);
void uart_write_byte(uint8_t c);
void uart_write_str(char *s);

// Ultrasonic sensor functions
void HAL_Ultrasonic_Init(void);
void HAL_Ultrasonic_Trigger(void);

// External variables for ultrasonic ISR
extern volatile uint16_t echo_duration;
extern volatile uint8_t echo_ready;

#endif // HAL_AVR_H
