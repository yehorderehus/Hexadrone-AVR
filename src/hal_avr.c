#include "hal_avr.h"
#include <avr/interrupt.h>
#include <util/delay.h>

// Global variables for ultrasonic sensor
volatile uint16_t echo_duration;
volatile uint8_t echo_ready;

// Buzzer initialization using TCA0 timer for frequency generation
void HAL_Buzzer_Init(void) {
    PORTMUX.TCAROUTEA = PORTMUX_TCA0_PORTB_gc;
    PORTB.DIRSET = PIN0_bm;

    TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_FRQ_gc | TCA_SINGLE_CMP0EN_bm;
    TCA0.SINGLE.CMP0 = 0;
    TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV4_gc | TCA_SINGLE_ENABLE_bm;
}

// Set buzzer frequency by adjusting compare value
void HAL_Buzzer_SetFreq(uint16_t freq_val) {
    TCA0.SINGLE.CMP0 = freq_val;
}

// Initialize TWI0 for I2C communication at 400kHz
void twi0_init(void)
{
    // Baud calculation: 24e6/(2*400000) - (5 + (24e6 * 600e-9)/2)
    TWI0.MBAUD = 18;
    // Enable master mode
    TWI0.MCTRLA = TWI_ENABLE_bm;
    // Set bus state to idle
    TWI0.MSTATUS = TWI_BUSSTATE_IDLE_gc;
}

// Wait for write operation to complete, return true if ACK received
bool twi0_wait_write(void)
{
    while (1)
    {
        if (TWI0.MSTATUS & (TWI_WIF_bm | TWI_RIF_bm))
        {
            return ((TWI0.MSTATUS & TWI_RXACK_bm) == 0);
        }

        if (TWI0.MSTATUS & (TWI_BUSERR_bm | TWI_ARBLOST_bm))
        {
            return false;
        }
    }
}

// Wait for read data to be ready
bool twi0_wait_read_ready(void)
{
    while (1)
    {
        if (TWI0.MSTATUS & TWI_RIF_bm)
        {
            return true;
        }

        if (TWI0.MSTATUS & (TWI_BUSERR_bm | TWI_ARBLOST_bm))
        {
            return false;
        }
    }
}

// Write a single register via I2C
bool i2c_write_reg(uint8_t dev7, uint8_t reg, uint8_t value)
{
    TWI0.MADDR = (uint8_t)(dev7 << 1);   // Write address
    if (!twi0_wait_write())
    {
        TWI0.MCTRLB = TWI_MCMD_STOP_gc;
        return false;
    }

    TWI0.MDATA = reg;
    if (!twi0_wait_write())
    {
        TWI0.MCTRLB = TWI_MCMD_STOP_gc;
        return false;
    }

    TWI0.MDATA = value;
    if (!twi0_wait_write())
    {
        TWI0.MCTRLB = TWI_MCMD_STOP_gc;
        return false;
    }

    TWI0.MCTRLB = TWI_MCMD_STOP_gc;
    return true;
}

// Read multiple registers via I2C
bool i2c_read_regs(uint8_t dev7, uint8_t start_reg, uint8_t *buf, uint8_t len)
{
    // Write register address
    TWI0.MADDR = (uint8_t)(dev7 << 1);   // Write
    if (!twi0_wait_write())
    {
        TWI0.MCTRLB = TWI_MCMD_STOP_gc;
        return false;
    }

    TWI0.MDATA = start_reg;
    if (!twi0_wait_write())
    {
        TWI0.MCTRLB = TWI_MCMD_STOP_gc;
        return false;
    }

    // Repeated START + read
    TWI0.MADDR = (uint8_t)((dev7 << 1) | 0x01);
    if (!twi0_wait_write())
    {
        TWI0.MCTRLB = TWI_MCMD_STOP_gc;
        return false;
    }

    for (uint8_t i = 0; i < len; i++)
    {
        if (!twi0_wait_read_ready())
        {
            TWI0.MCTRLB = TWI_MCMD_STOP_gc;
            return false;
        }

        buf[i] = TWI0.MDATA;

        if (i == (len - 1))
        {
            // Last byte: NACK + STOP
            TWI0.MCTRLB = TWI_ACKACT_bm | TWI_MCMD_STOP_gc;
        }
        else
        {
            // Continue reception
            TWI0.MCTRLB = TWI_MCMD_RECVTRANS_gc;
        }
    }

    return true;
}

// Initialize UART on USART3
void uart_init(uint32_t f_cpu, uint32_t baud)
{
    PORTB.DIRSET = PIN0_bm;   // TX pin
    PORTB.DIRCLR = PIN1_bm;   // RX pin

    USART3.BAUD  = (uint16_t)((f_cpu * 4UL) / baud);
    USART3.CTRLB |= USART_TXEN_bm | USART_RXEN_bm;

    USART3.CTRLA = USART_RXCIE_bm;
}

// Write a single byte via UART
void uart_write_byte(uint8_t c)
{
    while(!(USART3.STATUS & USART_DREIF_bm)){}
    USART3.TXDATAL = c;
}

// Write a string via UART
void uart_write_str(char *s)
{
    while(*s) {
        while(!(USART3.STATUS & USART_DREIF_bm));
        uart_write_byte((uint8_t)*s++);
    }
}

// Initialize ultrasonic sensor pins and timer
void HAL_Ultrasonic_Init(void) {
    // Setup TRIG pin (PD0 as Output)
    PORTD.DIRSET = PIN0_bm;
    PORTD.OUTCLR = PIN0_bm; // Keep low initially

    // Setup ECHO pin (PD1 as Input)
    PORTD.DIRCLR = PIN1_bm;

    // Route PD1 to Event System
    EVSYS.CHANNEL0 = 0x49; // Hex code for PORTD PIN1 generator
    EVSYS.USERTCB0CAPT = EVSYS_USER_CHANNEL0_gc;

    // Setup TCB0 for Input Capture (Pulse Width Measurement)
    TCB0.CTRLA = TCB_CLKSEL_DIV2_gc; // 24MHz / 2 = 12MHz clock (1 tick = ~0.083 us)
    TCB0.CTRLB = TCB_CNTMODE_PW_gc;  // Pulse Width measurement mode
    TCB0.INTCTRL = TCB_CAPT_bm;      // Enable Capture Interrupt
    TCB0.EVCTRL = TCB_CAPTEI_bm;     // Enable Event Input
    TCB0.CTRLA |= TCB_ENABLE_bm;     // Start Timer
}

// Interrupt Service Routine for ultrasonic echo capture
ISR(TCB0_INT_vect) {
    echo_duration = TCB0.CCMP; // Read captured pulse length
    echo_ready = 1;            // Set data ready flag
    TCB0.INTFLAGS = TCB_CAPT_bm; // Clear interrupt flag
}

// Trigger ultrasonic sensor by sending 10us pulse on TRIG pin
void HAL_Ultrasonic_Trigger(void) {
    PORTD.OUTSET = PIN0_bm; // TRIG HIGH
    _delay_us(10);          // Wait 10us
    PORTD.OUTCLR = PIN0_bm; // TRIG LOW
}
