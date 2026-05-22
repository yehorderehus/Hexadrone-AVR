#include "hal_avr.h"
#include <avr/interrupt.h>
#include <util/delay.h>

// Hardware Abstraction Layer implementation for AVR peripherals.
// Sections include buzzer, I2C/TWI, ultrasonic, button, and SPI/IMU.

// Shared ultrasonic state
volatile uint16_t echo_duration;
volatile uint8_t echo_ready;

// ---------------------------------------------------------------------------
// Buzzer
// ---------------------------------------------------------------------------

void HAL_Buzzer_Init(void)
{
    PORTMUX.TCAROUTEA = PORTMUX_TCA0_PORTB_gc;
    PORTB.DIRSET = PIN0_bm;

    TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_FRQ_gc | TCA_SINGLE_CMP0EN_bm;
    TCA0.SINGLE.CMP0 = 0;
    TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV4_gc | TCA_SINGLE_ENABLE_bm;
}

// Set buzzer frequency (in real Hz) or mute it
void HAL_Buzzer_SetFreq(uint16_t freq_val)
{
    if (freq_val == 0)
    {
        // MUTE: Disable the timer output to the pin safely
        TCA0.SINGLE.CTRLB &= ~TCA_SINGLE_CMP0EN_bm;
        PORTB.OUTCLR = PIN0_bm; // Ensure pin stays LOW
    }
    else
    {
        // Convert desired Hz into timer hardware ticks
        // Math: (Clock / (2 * Prescaler * Freq)) - 1
        // (24,000,000 / (2 * 4 * freq_val)) - 1 = (3,000,000 / freq_val) - 1
        TCA0.SINGLE.CMP0 = (uint16_t)(3000000UL / freq_val) - 1;

        // UN-MUTE: Re-enable the timer output
        TCA0.SINGLE.CTRLB |= TCA_SINGLE_CMP0EN_bm;
    }
}

// ---------------------------------------------------------------------------
// I2C / TWI
// ---------------------------------------------------------------------------

// Initialize TWI0 for I2C communication at 100kHz
void twi0_init(void)
{
    // Enable internal pull-ups on I2C pins (PA2=SDA, PA3=SCL)
    PORTA.PIN2CTRL = PORT_PULLUPEN_bm;
    PORTA.PIN3CTRL = PORT_PULLUPEN_bm;

    // Baud calculation: 24MHz / (2 * 100kHz) - 5 = 115
    TWI0.MBAUD = 115;

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
    TWI0.MADDR = (uint8_t)(dev7 << 1); // Write address
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
    TWI0.MADDR = (uint8_t)(dev7 << 1); // Write
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

// ---------------------------------------------------------------------------
// Ultrasonic sensor
// ---------------------------------------------------------------------------

// Initialize ultrasonic sensor pins and timer
void HAL_Ultrasonic_Init(void)
{
    // Setup TRIG pin (PD0 as Output)
    PORTD.DIRSET = PIN0_bm;
    PORTD.OUTCLR = PIN0_bm; // Keep low initially

    // Setup ECHO pin (PD1 as Input)
    PORTD.DIRCLR = PIN1_bm;

    // Route PD1 to Event System
    EVSYS.CHANNEL2 = EVSYS_CHANNEL2_PORTD_PIN1_gc;

    // Tell TCB0 to listen to Channel
    EVSYS.USERTCB0CAPT = EVSYS_USER_CHANNEL2_gc;

    // Setup TCB0 for Input Capture (Pulse Width Measurement)
    TCB0.CTRLA = TCB_CLKSEL_DIV2_gc; // 24MHz / 2 = 12MHz clock (1 tick = ~0.083 us)
    TCB0.CTRLB = TCB_CNTMODE_PW_gc;  // Pulse Width measurement mode
    TCB0.INTCTRL = TCB_CAPT_bm;      // Enable Capture Interrupt
    TCB0.EVCTRL = TCB_CAPTEI_bm;     // Enable Event Input
    TCB0.CTRLA |= TCB_ENABLE_bm;     // Start Timer
}

// Interrupt Service Routine for ultrasonic echo capture
ISR(TCB0_INT_vect)
{
    echo_duration = TCB0.CCMP;   // Read captured pulse length
    echo_ready = 1;              // Set data ready flag
    TCB0.INTFLAGS = TCB_CAPT_bm; // Clear interrupt flag
}

// Trigger ultrasonic sensor by sending 10us pulse on TRIG pin
void HAL_Ultrasonic_Trigger(void)
{
    PORTD.OUTSET = PIN0_bm; // TRIG HIGH
    _delay_us(10);          // Wait 10us
    PORTD.OUTCLR = PIN0_bm; // TRIG LOW
}

// ---------------------------------------------------------------------------
// Button input
// ---------------------------------------------------------------------------

// Private button event flag
static volatile uint8_t button_event_flag = 0;

void HAL_Button_Init(void)
{
    PORTB.DIRCLR = PIN2_bm;
    PORTB.PIN2CTRL = PORT_PULLUPEN_bm | PORT_ISC_FALLING_gc;
}

// The "Getter" function for the main loop to use
bool HAL_Button_GetAndClearEvent(void)
{
    if (button_event_flag)
    {
        button_event_flag = 0; // Clear it so it only triggers once
        return true;
    }
    return false;
}

ISR(PORTB_PORT_vect)
{
    if (PORTB.INTFLAGS & PIN2_bm)
    {
        button_event_flag = 1; // Just record that the hardware did a thing
        PORTB.INTFLAGS = PIN2_bm;
    }
}

// ---------------------------------------------------------------------------
// SPI / IMU (MPU-9250)
// ---------------------------------------------------------------------------

void spi0_init(void)
{
    // 1. Set MOSI (PA4), SCK (PA6), and CS (PA7) as outputs
    PORTA.DIRSET = PIN4_bm | PIN6_bm | PIN7_bm;

    // 2. Set MISO (PA5) as input
    PORTA.DIRCLR = PIN5_bm;

    // 3. Keep Chip Select (CS) HIGH (inactive) by default
    PORTA.OUTSET = PIN7_bm;

    // 4. Enable SPI Master, Prescaler DIV4 (24MHz / 16 = 1.5MHz SPI clock)
    SPI0.CTRLA = SPI_MASTER_bm | SPI_ENABLE_bm | SPI_PRESC_DIV16_gc;
}

// Low-level SPI transfer (sends a byte, returns the received byte)
uint8_t spi0_transfer(uint8_t data)
{
    SPI0.DATA = data;
    while (!(SPI0.INTFLAGS & SPI_IF_bm))
        ; // Wait for transmission to complete
    return SPI0.DATA;
}

// Write to an MPU-9250 register over SPI
void imu_write_reg(uint8_t reg, uint8_t data)
{
    PORTA.OUTCLR = PIN7_bm;    // Pull CS LOW to start talking
    spi0_transfer(reg & 0x7F); // MSB must be 0 for WRITE
    spi0_transfer(data);       // Send the data
    PORTA.OUTSET = PIN7_bm;    // Pull CS HIGH to finish
}

// Read from MPU-9250 registers over SPI
void imu_read_regs(uint8_t reg, uint8_t *buf, uint8_t len)
{
    PORTA.OUTCLR = PIN7_bm;    // Pull CS LOW to start talking
    spi0_transfer(reg | 0x80); // MSB must be 1 for READ (The SPI "Gotcha")

    for (uint8_t i = 0; i < len; i++)
    {
        buf[i] = spi0_transfer(0xFF); // Send dummy byte (0xFF) to clock in the data
    }
    PORTA.OUTSET = PIN7_bm; // Pull CS HIGH to finish
}

// ---------------------------------------------------------------------------
// --- System Time (millis) --------------------------------------------------
// ---------------------------------------------------------------------------

volatile uint32_t system_millis = 0;

void HAL_Time_Init(void)
{
    // Configure TCB1 to generate an interrupt every 1 millisecond.
    // Clock is 24MHz. DIV2 prescaler = 12MHz.
    // 12,000,000 ticks per second / 1000 = 12,000 ticks per millisecond.

    TCB1.CTRLA = TCB_CLKSEL_DIV2_gc;
    TCB1.CTRLB = TCB_CNTMODE_INT_gc; // Periodic Interrupt Mode
    TCB1.CCMP = 12000;               // Top value for 1ms
    TCB1.INTCTRL = TCB_CAPT_bm;      // Enable interrupt
    TCB1.CTRLA |= TCB_ENABLE_bm;     // Start timer
}

ISR(TCB1_INT_vect)
{
    system_millis++;
    TCB1.INTFLAGS = TCB_CAPT_bm; // Clear the interrupt flag
}

uint32_t HAL_GetMillis(void)
{
    uint32_t ms;
    // Disable interrupts briefly to safely read the 32-bit volatile variable
    // because an 8-bit AVR takes multiple clock cycles to read 32 bits.
    cli();
    ms = system_millis;
    sei();
    return ms;
}
