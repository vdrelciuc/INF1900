/// Functions for the various IO interactions
/// \file GeneralIo.h
/// \author Misha Krieger-Raynauld (1952515), Simon Gauvin (1951457), Vlad Drelciuc (1941366), Nicolas Charron (1960356)
/// \date 2019-02-20

#ifndef GENERALIO_H
#define GENERALIO_H

#include <stdint.h>
#include "Config.h"

namespace lib
{   
    /// Global variable for button interrupt (without debouncer)
    extern volatile bool wasButtonPressed;

    // The number of line tracker sensors
    constexpr uint8_t nbLineTrackerSensors = 5;

    /// Initialize port modes (input/output) according to constants defined in Config.h
    void initializePins();
    
    /// Set TCNT0 settings for use as a wave generation source for piezo connected to OC0A
    void initializePiezo();

    /// Change the LED color by giving it a mask with which to enable or disable LED pins
    /// \param ledColor Mask with which to change LED pins state (see constants in Config.h)
    void setLedColor(Led ledColor);

    /// Invert the current LED color
    void invertLedColor();

    /// Use the line tracker LEDs to display a number (from 1 to 5). Providing 0 as a number will turn all tracker LEDs off.
    /// number The number to display with the LEDs (indexed from 1 and viewed from left to right when looking at the robot from behind).
    ///        Values above 5 will be clamped to 5, and zero will turn off LEDs.
    void displayNumberOnTrackerLeds(uint8_t number);

    /// Play a given note on the piezo
    /// \param noteId MIDI note ID to play
    void startPiezo(uint16_t noteId);

    /// Stop playing sound through the piezo
    void stopPiezo();
    
    /// Play a short sound through the piezo for debugging.
    /// Blocks for 100 milliseconds
    void playShortPiezoSound();

    /// Get the current state of the interrupt button with a software-controlled debouncer.
    /// Blocks for at least 10 ms, or until valid button state can be polled
    /// \return Whether the button is pressed
    bool isButtonPressed();

    /// Block execution until the button is pressed. Useful for debugging
    void waitForButtonPress();

    /// Get how many times the button was pressed (max of 9 presses).
    /// Blocks until the button is pressed and then for 2 more seconds every time the button is pressed
    /// Note: requires initializeTimer() to have been called beforehand
    /// \return How many times the button is pressed
    uint8_t getButtonPressCount();

    /// Read the logical values of the line tracker sensor readings. This function will also open the corresponding LEDs on the breadboard.
    /// \return The values of the line tracker as represented with bits.
    ///         The first (leftmost) tracker is represented by the MSB, and last (rightmost) by the LSB
    uint8_t readLineTrackerValues();

    /// Enable interrupts for interrupt button
    void enableButtonInterrupts();

    /// Disable interrupts for interrupt button
    void disableButtonInterrupts();

    /// Check if a given sensor is on black among the given line tracker sensor readings
    /// \param lineTrackerValues          The values of the line tracker readings to test
    /// \param lineTrackerSensorNumber    The sensor number to test, indexed from 0
    /// \return Whether the specified line tracker sensor value is on black
    bool isLineTrackerSensorOnBlack(uint8_t lineTrackerValues, uint8_t lineTrackerSensorNumber);

    /// Print every value of each line tracker sensor when in debug mode. This function does nothing otherwise.
    /// Note: in debug mode, requires initializeUsart() to have been called beforehand
    void printLineTrackerSensorValues();

    /// Play startup sequence using LED and piezo
    /// Note: requires initializePiezo() to have been called beforehand
    void startupSequence();
} // namespace lib

#endif // GENERALIO_H