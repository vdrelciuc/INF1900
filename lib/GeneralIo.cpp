/// Functions for the various IO interactions
/// \file GeneralIo.h
/// \author Misha Krieger-Raynauld (1952515), Simon Gauvin (1951457), Vlad Drelciuc (1941366), Nicolas Charron (1960356)
/// \date 2019-02-20

#include "GeneralIo.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include "Adc.h"
#include "Config.h"
#include "Debug.h"
#include "Timer.h"
#include "Usart.h"

namespace
{
    // Static ADC to read the line tracker values
    lib::Adc adc;
} // namespace

namespace lib
{
    // Global variable for button interrupt
    volatile bool wasButtonPressed;

    void initializePins()
    {
        DDRB |= static_cast<uint8_t>(Led::Mask); // Output mode for onboard LED
        DDRB |= static_cast<uint8_t>(Piezo::Mask); // Output mode for piezo
        DDRB &= ~calibrationMask; // Input mode for calibration pin
        DDRC |= static_cast<uint8_t>(TrackerLed::Mask); // Output mode for tracker LEDs
        DDRC &= ~infraredRxMask; // Input mode for infrared photodetector
        DDRD &= ~buttonMask; // Input mode for interrupt button
        DDRD |= static_cast<uint8_t>(Motor::Mask); // Output mode for motors or IR transmitter
    }

    void initializePiezo()
    {
        // Set piezo ground pin to 0
        PORTB &= ~static_cast<uint8_t>(Piezo::Ground);

        // Timer counter control register flags (2x8 bits)
        TCCR0A |= (1 << WGM01); // Set CTC mode with OCR0A as TOP source by setting WGM0 bits to 010 (two LSBs on TCCR0A and MSB on TCCR0B)
        TCCR0B |= (1 << CS02); // Set clock select to clock divided by 256 by setting CS0 bits to 100 (prescaler 256)
    }
    
    void setLedColor(Led ledColor)
    {
        PORTB = (PORTB & ~static_cast<uint8_t>(Led::Mask)) | static_cast<uint8_t>(ledColor); // Set the LED color
    }

    void invertLedColor()
    {
        PORTB ^= static_cast<uint8_t>(Led::Mask);
    }

    void displayNumberOnTrackerLeds(uint8_t number)
    {
        // Turn off LEDs if number is 0
        if (number == 0)
        {
            PORTC &= ~static_cast<uint8_t>(TrackerLed::Mask);
            return;
        }

        // Decrement to index at 0 for tracker LEDs
        number--;

        // Clamp
        if (number > nbLineTrackerSensors - 1)
        {
            number = nbLineTrackerSensors - 1;
        }

        // Set the tracker LEDs
        PORTC = (PORTC & ~static_cast<uint8_t>(TrackerLed::Mask)) | static_cast<uint8_t>(TrackerLed::First) << (number);
    }

    void startPiezo(uint16_t noteId)
    {
        static constexpr uint8_t firstNoteIndexOffset = 45;
        static constexpr uint16_t frequencies[] = {110, // NOTE_A2
                                                   117, // NOTE_AS2
                                                   123, // NOTE_B2
                                                   131, // NOTE_C3
                                                   139, // NOTE_CS3
                                                   147, // NOTE_D3
                                                   156, // NOTE_DS3
                                                   165, // NOTE_E3
                                                   175, // NOTE_F3
                                                   185, // NOTE_FS3
                                                   196, // NOTE_G3
                                                   208, // NOTE_GS3
                                                   220, // NOTE_A3
                                                   233, // NOTE_AS3
                                                   247, // NOTE_B3
                                                   262, // NOTE_C4
                                                   277, // NOTE_CS4
                                                   294, // NOTE_D4
                                                   311, // NOTE_DS4
                                                   330, // NOTE_E4
                                                   349, // NOTE_F4
                                                   370, // NOTE_FS4
                                                   392, // NOTE_G4
                                                   415, // NOTE_GS4
                                                   440, // NOTE_A4
                                                   466, // NOTE_AS4
                                                   494, // NOTE_B4
                                                   523, // NOTE_C5
                                                   554, // NOTE_CS5
                                                   587, // NOTE_D5
                                                   622, // NOTE_DS5
                                                   659, // NOTE_E5 
                                                   698, // NOTE_F5
                                                   740, // NOTE_FS5
                                                   784, // NOTE_G5
                                                   831, // NOTE_GS5
                                                   880}; // NOTE_A5
        
        uint8_t noteIndex = noteId - firstNoteIndexOffset;

        // If noteIndex is invalid, do not continue
        if (noteIndex >= sizeof(frequencies) / sizeof(uint16_t))
        {
            return;
        }

        TCNT0 = 0; // Reset counter to avoid missing compare match if new OCR0A value is lower than current TCNT0 value
        TCCR0A |= (1 << COM0A0); // Set compare output mode to toggle OC0A on compare match by setting COM0A bits to 01
        
        static constexpr uint16_t counterIncrementsPerSec = F_CPU / 256; // 256 is prescaler
        // Set OCR0A to F_CPU / N / frequency /  2 - 1 to set note (Atmel documentation p.96)
        OCR0A = counterIncrementsPerSec / (frequencies[noteIndex] * 2) - 1;
    }
    
    void stopPiezo()
    {
        TCCR0A &= ~(1 << COM0A1) & ~(1 << COM0A0); // Set compare output mode to normal (do not toggle OC0A on compare match) by setting COM0A bits to 00
    }

    void playShortPiezoSound()
    {
        startPiezo(50);
        msSleep(100);
        stopPiezo();
    }

    bool isButtonPressed()
    {
        bool isFirstStatePressed;
        bool isSecondStatePressed;

        // Loop until valid button state is polled
        do
        {
            // Check button state twice to debounce
            isFirstStatePressed = PIND & buttonMask;
            
            static constexpr uint8_t msDebounceDelay = 10;
            msSleep(msDebounceDelay);
            
            isSecondStatePressed = PIND & buttonMask;
        } while (isFirstStatePressed != isSecondStatePressed);

        return isFirstStatePressed;
    }

    void waitForButtonPress()
    {
        while (isButtonPressed() == false)
        {
        }
    }

    uint8_t getButtonPressCount()
    {
        // Block until there is a first press
        waitForButtonPress();

        startTimer(secTimerMultiplier * 2);
        uint8_t counter = 1; // Register first press

        // Buffer to hold previous value to determine if button is currently descending
        bool currentIsButtonPressed = true; // Initially set to true as getButtonPressCount should be called after button non-debounced ISR
        bool previousIsButtonPressed = true; // Initially set to true as getButtonPressCount should be called after button non-debounced ISR

        while (isTimerExpired == false)
        {
            // Update previous button state and get current button state
            previousIsButtonPressed = currentIsButtonPressed;
            currentIsButtonPressed = isButtonPressed();

            // Check for descending button
            if (currentIsButtonPressed == true && previousIsButtonPressed == false)
            {
                if (counter < 9) // Max of 9 presses
                {
                    counter++;
                }
                else
                {
                    counter = 1;
                }
                startTimer(2 * secTimerMultiplier); // Reset timer on button press
            }
        }

        return counter;
    }

    void enableButtonInterrupts()
    {
        cli(); // Clear global interrupt flag to disable interrupts

        wasButtonPressed = false;

        EICRA |= (1 << ISC01) | (1 << ISC00); // Enable interrupt generation on rising edge by setting ISC0 bits to 11
        EIMSK |= (1 << INT0); // Enable external interrupts for INT0 button

        // Clear interrupt flag for INT0 because ISC bits were changed, which can trigger an interrupt
        EIFR |= (1 << INTF0);

        sei(); // Set global interrupt flag to enable interrupts
    }

    void disableButtonInterrupts()
    {
        cli(); // Clear global interrupt flag to disable interrupts

        wasButtonPressed = false;

        EIMSK &= ~(1 << INT0); // Disable external interrupts for INT0 button
        EICRA &= ~(1 << ISC01) & ~(1 << ISC00); // Disable interrupt generation on rising edge by setting ISC0 bits to 00

        // Clear interrupt flag for INT0 because ISC bits were changed, which can trigger an interrupt
        EIFR |= (1 << INTF0);

        sei(); // Set global interrupt flag to enable interrupts
    }

    uint8_t readLineTrackerValues()
    {
        // Values above which to consider that each sensor is seeing white
        static constexpr uint8_t highestBlackValues[nbLineTrackerSensors] = {150, 180, 150, 150, 150};
        
        uint8_t lineTrackerValues = 0;
        for (uint8_t i = 0; i < nbLineTrackerSensors; i++)
        {
            bool isOnBlack = adc.readAnalog8Bit(i) <= highestBlackValues[i];
            if (isOnBlack)
            {
                lineTrackerValues |= 1 << (nbLineTrackerSensors - 1 - i); // Set MSB first
                PORTC |= (static_cast<uint8_t>(TrackerLed::First) << i); // Turn on corresponding LED for sensor if on black
            }
            else
            {
                PORTC &= ~(static_cast<uint8_t>(TrackerLed::First) << i); // Turn off corresponding LED for sensor if on white            
            }            
        }
        return lineTrackerValues;
    }

    bool isLineTrackerSensorOnBlack(uint8_t lineTrackerValues, uint8_t lineTrackerSensorNumber)
    {
        return lineTrackerValues & 1 << (nbLineTrackerSensors - 1 - lineTrackerSensorNumber);
    }

    void printLineTrackerSensorValues()
    {
        for (uint8_t i = 0; i < nbLineTrackerSensors; ++i)
        {
            DEBUG_PRINT_NUMBER(adc.readAnalog8Bit(i));
            DEBUG_PRINT(' ');
        }
        DEBUG_PRINT('\n');
    }

    void startupSequence()
    {
        // First note
        setLedColor(Led::Green);
        startPiezo(50);
        msSleep(120);
        setLedColor(Led::Off);
        stopPiezo();
        msSleep(20);

        // Second note
        setLedColor(Led::Green);
        startPiezo(50);
        msSleep(120);
        setLedColor(Led::Off);
        stopPiezo();
        msSleep(20);

        // Third note
        setLedColor(Led::Red);
        startPiezo(62);
        msSleep(240);
        setLedColor(Led::Off);
        stopPiezo();
        msSleep(20);

        // Fourth note
        setLedColor(Led::Green);
        startPiezo(57);
        msSleep(120);
        setLedColor(Led::Off);
        stopPiezo();
    }
} // namespace lib

/// Interrupt service routine for INT0 interrupt (without debouncer) when enabled
ISR(INT0_vect)
{
    lib::wasButtonPressed = true;

    // Clear interrupt flag for INT0 to prevent bouncing
    EIFR |= (1 << INTF0);
}