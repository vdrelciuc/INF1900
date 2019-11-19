/// Functions for timers
/// \file Timer.cpp
/// \author Misha Krieger-Raynauld (1952515), Simon Gauvin (1951457), Vlad Drelciuc (1941366), Nicolas Charron (1960356)
/// \date 2019-02-20

#include "Timer.h"
#include "Config.h"
#include "Debug.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay_basic.h>

namespace
{
    constexpr uint16_t msDelayLoop2Multiplier =  F_CPU / 1000 / 4; // Coefficient (2000) for generating a 1 ms delay with _delay_loop_2
    constexpr uint16_t usDelayLoop2Multiplier =  F_CPU / 1000 / 1000 / 4; // Coefficient (2) for generating a 1 us delay with _delay_loop_2
} // namespace

namespace lib
{
    // Global variable for the timer
    volatile bool isTimerExpired;
    
    void initializeTimer()
    {   
        cli(); // Clear global interrupt flag to disable interrupts

        // Timer counter control register flags (3x8 bits)
        TCCR1A = 0; // Not used in this particular case
        TCCR1B |= (1 << CS12) | (1 << CS10); // Set clock select to clock divided by 1024 by setting CS1 bits to 101
        TCCR1B |= (1 << WGM12); // Set CTC mode with OCR1A as TOP source by setting WGM1 bits to 0100 (two LSBs on TCCR1A and two MSBs on TCCR1B)
        TCCR1C = 0; // Not used in this particular case

        // Enable interrupt when TCNT1 = OCR1A
        TIMSK1 |= (1 << OCIE1A);

        sei(); // Set global interrupt flag to enable interrupts
    }
        
    void startTimer(uint16_t duration)
    {
        cli(); // Clear global interrupt flag to disable interrupts
        
        isTimerExpired = false;

        // Reset counter
        TCNT1 = 0;
        
        // Set output compare register to the desired duration
        OCR1A = duration;
        
        sei(); // Set global interrupt flag to enable interrupts
    }

    void usSleep(uint16_t duration)
    {
        // No overflow : 131070 < 4294967295
        uint32_t delayLoop2Ticks = static_cast<uint32_t>(duration) * usDelayLoop2Multiplier;

        // Since UINT16_MAX is the maximum value we can pass to delay_loop_2,
        // we have to call _delay_loop_2 multiple times
        uint8_t nbIterations = delayLoop2Ticks / UINT16_MAX;
        for (uint8_t i = 0; i < nbIterations; i++)
        {
            _delay_loop_2(UINT16_MAX);
        }

        // And then sleep for the rest of the time
        uint16_t remainder = delayLoop2Ticks % UINT16_MAX;
        if (remainder > 0)
        {
            _delay_loop_2(remainder);
        }
    }

    void msSleep(uint16_t duration)
    {
        // No overflow : 131070000 < 4294967295
        uint32_t delayLoop2Ticks = static_cast<uint32_t>(duration) * msDelayLoop2Multiplier;

        // Since UINT16_MAX is the maximum value we can pass to delay_loop_2,
        // we have to call _delay_loop_2 multiple times
        uint16_t nbIterations = delayLoop2Ticks / UINT16_MAX; // No overflow: 2000 < 65535
        for (uint16_t i = 0; i < nbIterations; i++)
        {
            _delay_loop_2(UINT16_MAX);
        }

        // And then sleep for the rest of the time
        uint16_t remainder = delayLoop2Ticks % UINT16_MAX;
        if (remainder > 0)
        {
            _delay_loop_2(remainder);
        }
    }
} // namespace lib

/// Interrupt service routine for TCNT1 match with OCR1A
ISR(TIMER1_COMPA_vect)
{
    lib::isTimerExpired = true;
}