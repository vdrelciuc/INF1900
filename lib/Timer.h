/// Functions for timers
/// \file Timer.h
/// \author Misha Krieger-Raynauld (1952515), Simon Gauvin (1951457), Vlad Drelciuc (1941366), Nicolas Charron (1960356)
/// \date 2019-02-20

#ifndef TIMER_H
#define TIMER_H

#include <stdint.h>

namespace lib
{
    /// Global variable for the timer
    extern volatile bool isTimerExpired;
    
    /// Set TCNT0 settings for use as a timer with interrupts on OCR0A
    void initializeTimer();
    
    /// Reset and set timer with a given duration of up to 8.3 seconds
    /// \param duration Duration of the timer in clock cycles multiplied by prescaler (1024).
    ///                 Note: max value is 65535 (about 8.3 seconds)
    void startTimer(uint16_t duration);

    /// Sleep for a number of microseconds
    /// \param duration The duration for which to sleep, in milliseconds    
    void usSleep(uint16_t duration);

    /// Sleep for a number of milliseconds
    /// \param duration The duration for which to sleep, in milliseconds
    void msSleep(uint16_t duration);
} // namespace lib

#endif // TIMER_H