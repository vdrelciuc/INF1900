/// Functions for controlling the motors
/// \file Motors.h
/// \author Misha Krieger-Raynauld (1952515), Simon Gauvin (1951457), Vlad Drelciuc (1941366), Nicolas Charron (1960356)
/// \date 2019-02-20

#ifndef MOTORS_H
#define MOTORS_H

#include <stdint.h>

namespace lib
{
    /// Speed and timings to use in section 1 when going forward for a set calibrated amount of time (set by calibrateMotorTimings)
    constexpr uint8_t calibratedTimingSpeed = 120;
    extern uint16_t msSection1StartOffsetDuration;
    extern uint16_t msSensorToCenterOfRotationDuration;
    extern uint16_t msBetweenPointsDuration;

    // Speed and timings for 90 degree rotations (set by calibrateMotorTimings)
    constexpr uint8_t calibratedRotationSpeed = 100;
    extern uint16_t msRotate90ClockwiseDuration;
    extern uint16_t msRotate90CounterclockwiseDuration;
    constexpr uint8_t msForceStopAfterRotate90Duration = 128;

    /// Set TCNT2 settings for use as a PWM source for motors connected to OC2A and OC2B
    void initializeMotors();

    /// Run semi-automated tests to calibrate the timings used for 90 degree rotations, slight rotations, and section 1 blind movement.
    /// Places the values as the first 12 bytes of external EEPROM.
    /// Note: needs to be placed perfectly straight with its middle line tracker sensor on the outside edge of a black line for each of the first 4 tests.
    ///       For the 5th and 6th tests, needs to be placed perpendicular to black lines separated by 8 and 5 inches respectively.
    ///       For the 7th test, needs to be placed behind a black line, after which there is 3 inches before the edge of another black line.
    /// Note: in debug mode, requires initializeUsart() to have been called beforehand
    void calibrateMotorTimings();

    /// Read the motor timings back from external EEPROM
    void readMotorTimings();

    /// Set PWM duty cycle for each motor to adjust their speed.
    /// It also hardcodes a nerf for the right motor, which, experimentally, has shown to be stronger than the left one
    /// \param leftDutyCycle  Duty cycle with which to set OCR2A (clamped between -255 and 255), negative values mean the wheel will go in reverse
    /// \param rightDutyCycle Duty cycle with which to set OCR2B (clamped between -255 and 255), negative values mean the wheel will go in reverse
    void setMotorSpeed(int16_t leftDutyCycle, int16_t rightDutyCycle);

    /// Make the robot go straight by giving its wheels the same speed. This differs from setMotorSpeed as it gives
    /// the motors a short max power pulse to kickstart them, in the hopes of getting a straighter line
    /// \param dutyCycle Duty cycle with which to set the motors (clamped between -255 and 255), negative values mean the wheels will go in reverse
    void goStraight(int16_t dutyCycle);

    /// Give the motors a short max power pulse to kickstart them for going forward.
    /// Blocks for a certain amount of time before returning with the motors still in their max speed
    void pulseMotorsForward();

    /// Give the motors a short max power pulse to kickstart them for going backward.
    /// Blocks for a certain amount of time before returning with the motors still in their max speed
    void pulseMotorsBackward();

    /// Give the motors a short max power pulse to kickstart them for rotating clockwise.
    /// Blocks for a certain amount of time before returning with the motors still in their max speed
    void pulseMotorsClockwise();

    /// Give the motors a short max power pulse to kickstart them for rotating counterclockwise.
    /// Blocks for a certain amount of time before returning with the motors still in their max speed
    void pulseMotorsCounterclockwise();

    /// Rotate 90 degrees clockwise while staying in place
    void rotate90Clockwise();

    /// Rotate 90 degrees clockwise while staying in place
    void rotate90Counterclockwise();

    /// Rotate by a small angle clockwise. Useful for realigning
    void rotateSlightlyClockwise();

    /// Rotate by a small angle counterclockwise. Useful for realigning
    void rotateSlightlyCounterclockwise();

    /// Set PWM duty cycle to the opposite of current speed for a small amount of time to decelerate motors faster and then turn them off
    /// \param msForcedDecelerationDuration  8-bit time in milliseconds for which to decelerate forcefully. Default value is 255 ms
    void forceStopMotors(uint8_t msForcedDecelerationDuration = 255);

    /// Get left motor PWM duty cycle
    /// \return Duty cycle (clamped between -255 and 255), negative values mean the wheel is going in reverse
    int16_t getLeftMotorSpeed();

    /// Get right motor PWM duty cycle
    /// \return Duty cycle (clamped between -255 and 255), negative values mean the wheel is going in reverse
    int16_t getRightMotorSpeed();

    /// Get the average of both motor PWM duty cycles
    /// \return The average speed of both motors
    int16_t getAverageMotorSpeed();
} // namespace lib

#endif // MOTORS_H