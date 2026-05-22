#define F_CPU 24000000UL // 24MHz
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// Project-specific headers
#include "hal_avr.h"
#include "lcd.h"
#include "sensors.h"
#include "telemetry.h"

void CLK_init(void)
{
    // Set internal oscillator to 24MHz
    _PROTECTED_WRITE(CLKCTRL.OSCHFCTRLA, CLKCTRL_FRQSEL_24M_gc);
}

void Handle_CalibrationRequest(void)
{
    // Visual feedback
    LCD_Clear();
    LCD_Print(1, "CALIBRATING...");

    // Audio feedback
    HAL_Buzzer_SetFreq(800);
    _delay_ms(100);
    HAL_Buzzer_SetFreq(0);

    // Tell the sensor layer to do its math
    Sensors_CalibrateIMU();

    // Audio feedback
    HAL_Buzzer_SetFreq(1200);
    _delay_ms(80);
    HAL_Buzzer_SetFreq(0);

    // Visual clear
    LCD_Clear();
}

int main(void)
{
    // 1. Core Hardware Setup
    CLK_init();

    // 2. Peripheral Subsystem Initialization
    HAL_Time_Init();
    Sensors_Init();
    Telemetry_Init();
    HAL_Buzzer_Init();
    HAL_Button_Init();

    // 3. Enable Global Interrupts
    sei();

    // 4. Main Operational Loop
    while (1)
    {
        // 1. Ask the HAL if the hardware button was pressed
        if (HAL_Button_GetAndClearEvent())
        {
            Handle_CalibrationRequest();

            // --- DEBOUNCE FIX ---
            // 1. Clear the hardware interrupt flag to discard physical bounces
            PORTB.INTFLAGS = PIN2_bm;

            // 2. Clear the software flag just in case the ISR already caught one
            HAL_Button_GetAndClearEvent();
        }

        // 2. Normal 10Hz Routine
        Sensors_Update();
        Sensors_RunSafetyCheck();
        Telemetry_Send();

        _delay_ms(100);
    }
}
