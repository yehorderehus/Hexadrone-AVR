#define F_CPU 24000000UL // 24MHz
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// Project-specific headers
#include "hal_avr.h"
#include "sensors.h"
#include "telemetry.h"

void CLK_init(void) {
    // Set internal oscillator to 24MHz
    _PROTECTED_WRITE(CLKCTRL.OSCHFCTRLA, CLKCTRL_FRQSEL_24M_gc);
}

int main(void) {
    // 1. Core Hardware Setup
    CLK_init();
    uart_init(F_CPU, 115200); 
    PORTC.DIRSET = PIN0_bm;

    // 2. Peripheral Subsystem Initialization
    Sensors_Init();
    Telemetry_Init();

    // 3. Enable Global Interrupts
    sei();

    // 4. Main Operational Loop
    while (1) {
        PORTC.OUTTGL = PIN0_bm; // Heartbeat LED
        
        Sensors_Update();
        Sensors_RunSafetyCheck();
        Telemetry_Send();

        _delay_ms(100); // 10Hz loop pacing
    }
}
