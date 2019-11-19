/// Functions for controlling the motors
/// \file Motors.cpp
/// \author Misha Krieger-Raynauld (1952515), Simon Gauvin (1951457), Vlad Drelciuc (1941366), Nicolas Charron (1960356)
/// \date 2019-02-20

#include "Motors.h"
#include <avr/io.h>
#include "Config.h"
#include "Debug.h"
#include "ExternalMemory.h"
#include "GeneralIo.h"
#include "Timer.h"

namespace
{
    // Number of calibration tests;
    constexpr uint8_t nbCalibrationTests = 7;

    // Timings for slight rotations (set by calibrateMotorTimings)
    uint16_t msRotateSlightlyClockwiseDuration;
    uint16_t msRotateSlightlyCounterclockwiseDuration;
    uint8_t msForceStopAfterRotateSlightlyDuration = 50;

    // Duration of max power pulse for pulse functions
    constexpr uint8_t msMaxPulseDuration = 60;

    constexpr int8_t rightMotorNerfNumerator = 24;
    constexpr int8_t rightMotorNerfDenominator = 25;
} // namespace

namespace lib
{
    // Global variable timings for 90 degree rotations (set by calibrateMotorRotationTimings)
    uint16_t msRotate90ClockwiseDuration;
    uint16_t msRotate90CounterclockwiseDuration;

    // Global variable timings for section 1 blind movement
    uint16_t msSection1StartOffsetDuration;
    uint16_t msSensorToCenterOfRotationDuration;
    uint16_t msBetweenPointsDuration;

    void initializeMotors()
    {        
        // Timer counter control register flags (2x8 bits)
        TCCR2A |= (1 << WGM20); // Set wave generation mode to phase-correct 8-bit (0xFF TOP value) PWM by setting WGM2 bits to 001 (two LSBs on TCCR2A and MSB on TCCR2B)
        TCCR2A |= (1 << COM2A1); // Set compare output mode to clear OC2A on compare match when up-counting and vice-versa by setting COM2A bits to 10
        TCCR2A |= (1 << COM2B1); // Set compare output mode to clear OC2B on compare match when up-counting and vice-versa by setting COM2B bits to 10
        TCCR2B |= (1 << CS22) | (1 << CS21); // Set clock select to clock divided by 256 by setting CS2 bits to 110
    }

    void calibrateMotorTimings()
    {
        DEBUG_PRINT("CALIBRATION STARTED\n");

        ExternalMemory memoryInterface;

        // Calculate calibration values
        for (uint8_t i = 0; i < nbCalibrationTests; i++)
        {
            // Wait for button press
            DEBUG_PRINT("\tAbout to calibrate ");
            switch (i)
            {
            case 0:
                DEBUG_PRINT("clockwise 90 degree rotation");
                break;
            case 1:
                DEBUG_PRINT("counterclockwise 90 degree rotation");
                break;
            case 2:
                DEBUG_PRINT("slight clockwise rotation");
                break;
            case 3:
                DEBUG_PRINT("slight counterclockwise rotation");
                break;
            case 4:
                DEBUG_PRINT("section 1 start offset");
                break;
            case 5:
                DEBUG_PRINT("sensor to center of rotation");
                break;
            case 6:
                DEBUG_PRINT("distance between points");
                break;
            }
            DEBUG_PRINT(" timing, please place robot. Waiting for button press\n");
            
            while (isButtonPressed() == false)
            {
                readLineTrackerValues(); // Read line tracker values to help with placement
            }

            // Timing counter for the durations
            uint16_t msTiming = 0;

            // Calibrate clockwise rotation
            if (i == 0 || i == 2)
            {
                pulseMotorsClockwise();
                setMotorSpeed(calibratedRotationSpeed, -calibratedRotationSpeed);
            }
            // Calibrate counterclockwise rotation
            else if (i == 1 || i == 3)
            {
                pulseMotorsCounterclockwise();
                setMotorSpeed(-calibratedRotationSpeed, calibratedRotationSpeed);
            }
            // Straight line calibration timings
            else
            {
                goStraight(calibratedTimingSpeed);
            }

            uint8_t lineTrackerValues;

            // If calibrating 90 degree rotations
            if (i < 2)
            {
                bool hasLeftInitialBlackLine = false;
                // Loop while sensors the middle sensor is on white or while the line tracker has not at least left its initial position on the black line
                while (((lineTrackerValues = readLineTrackerValues()) & 0b00100) == 0 || hasLeftInitialBlackLine == false)
                {
                    // Set hasLeftInitialBlackLine if all sensors are on white
                    if (lineTrackerValues == 0b00000)
                    {
                        hasLeftInitialBlackLine = true;
                    }

                    static constexpr uint8_t msTimerIncrements = 50;
                    msSleep(msTimerIncrements);
                    msTiming += msTimerIncrements;
                }
                forceStopMotors(msForceStopAfterRotate90Duration);
            }
            // If calibrating slight rotations
            else if (i < 4)
            {
                uint8_t lineTrackerMask = (i == 2) ? 0b10000 : 0b00001; // CW for first test (i == 2), CCW for second (i == 3)
                // Loop until the edge sensor is on the black line after initially placing the robot centered on the black line
                while (((lineTrackerValues = readLineTrackerValues()) & lineTrackerMask) == 0)
                {
                    static constexpr uint8_t msTimerIncrements = 30;
                    msSleep(msTimerIncrements);
                    msTiming += msTimerIncrements;
                }
                forceStopMotors(msForceStopAfterRotateSlightlyDuration);
            }
            // If calibrating section 1 timings for going in a hardcoded straight line
            else if (i < 6)
            {
                uint8_t mask;
                // If calibrating section 1 start offset
                if (i < 5)
                {
                    mask = 0b01110;
                }
                // If calibrating sensors to center of rotation
                else
                {
                    mask = 0b00100;
                }
                
                // Loop until the three middle sensors are on black or until the center sensor is on black, depending on the mask
                while (((lineTrackerValues = readLineTrackerValues()) & mask) != mask)
                {
                    static constexpr uint8_t msTimerIncrements = 30;
                    msSleep(msTimerIncrements);
                    msTiming += msTimerIncrements;
                }
                forceStopMotors();
            }
            // If calibrating distance between points in section 1 (3 inches)
            else
            {
                bool hasLeftInitialWhiteSection = false;
                bool hasLeftInitialBlackLine = false;
                // Loop until the three middle sensors are on black or while the line tracker has not at least left its initial position on the black line
                while (((lineTrackerValues = readLineTrackerValues()) & 0b01110) != 0b01110 || hasLeftInitialBlackLine == false)
                {
                    // Set hasLeftInitialWhiteSection if the three middle sensors are on black
                    if ((lineTrackerValues & 0b01110) == 0b01110)
                    {
                        hasLeftInitialWhiteSection = true;
                    }
                    // Set hasLeftInitialBlackLine if both edge sensors are on white and the line tracker has left the white section
                    else if (hasLeftInitialWhiteSection && (lineTrackerValues & 0b10001) == 0)
                    {
                        hasLeftInitialBlackLine = true;
                    }

                    static constexpr uint8_t msTimerIncrements = 30;
                    msSleep(msTimerIncrements);
                    msTiming += msTimerIncrements;
                }
                forceStopMotors();
            }
            
            // Store calibrated value in EEPROM
            memoryInterface.write(i * 2, reinterpret_cast<const uint8_t*>(&msTiming), sizeof(msTiming));
            DEBUG_PRINT("\tStoring ");
            switch (i)
            {
            case 0:
                DEBUG_PRINT("clockwise 90 degree rotation");
                break;
            case 1:
                DEBUG_PRINT("counterclockwise 90 degree rotation");
                break;
            case 2:
                DEBUG_PRINT("slight clockwise rotation");
                break;
            case 3:
                DEBUG_PRINT("slight counterclockwise rotation");
                break;
            case 4:
                DEBUG_PRINT("section 1 start offset");
                break;
            case 5:
                DEBUG_PRINT("sensor to center of rottion");
                break;
            case 6:
                DEBUG_PRINT("distance between points");
                break;
            }
            DEBUG_PRINT(" timing: ");
            DEBUG_PRINT_NUMBER(msTiming);
            DEBUG_PRINT('\n');
        }

        DEBUG_PRINT("CALIBRATION COMPLETE\n");
        displayNumberOnTrackerLeds(0); // Turn off tracker LEDs while waiting for command (eventually, in main)

        msSleep(5); // Sleep for 5 ms to read back from EEPROM properly
    }

    void readMotorTimings()
    {
        DEBUG_PRINT("READING CALIBRATION STARTED\n");

        // Read calibrated motor timings from EEPROM
        ExternalMemory memoryInterface;

        // Read calibration values
        for (uint8_t i = 0; i < nbCalibrationTests; i++)
        {
            uint16_t msTiming = 0;
            memoryInterface.read(i * 2, reinterpret_cast<uint8_t*>(&msTiming), sizeof(msTiming));

            DEBUG_PRINT("\tReading ");
            switch (i)
            {
            case 0:
                DEBUG_PRINT("clockwise 90 degree rotation");
                break;
            case 1:
                DEBUG_PRINT("counterclockwise 90 degree rotation");
                break;
            case 2:
                DEBUG_PRINT("slight clockwise rotation");
                break;
            case 3:
                DEBUG_PRINT("slight counterclockwise rotation");
                break;
            case 4:
                DEBUG_PRINT("section 1 start offset");
                break;
            case 5:
                DEBUG_PRINT("sensor to center of rotation");
                break;
            case 6:
                DEBUG_PRINT("distance between points");
                break;
            }
            DEBUG_PRINT(" timing: ");
            DEBUG_PRINT_NUMBER(msTiming);
            DEBUG_PRINT('\n');

            switch (i)
            {
            case 0: // Update 90 degree clockwise rotation timing
                msRotate90ClockwiseDuration = msTiming;
                break;
            case 1: // Update 90 degree counterclockwise rotation timing
                msRotate90CounterclockwiseDuration = msTiming;
                break;
            case 2: // Update slight clockwise rotation timing
                msRotateSlightlyClockwiseDuration = msTiming;
                break;
            case 3: // Update slight counterclockwise rotation timing
                msRotateSlightlyCounterclockwiseDuration = msTiming;
                break;
            case 4: // Update timing for going straight at the start (8 inches) of section 1
                msSection1StartOffsetDuration = msTiming;
                break;
            case 5: // Update timing for going straight to align center of rotation (5 inches)
                msSensorToCenterOfRotationDuration = msTiming;
                break;
            case 6: // Update timing for going between points (3 inches) in section 1
                msBetweenPointsDuration = msTiming;
                break;
            }
        }

        DEBUG_PRINT("READING CALIBRATION COMPLETE\n");
    }
    
    void setMotorSpeed(int16_t leftDutyCycle, int16_t rightDutyCycle)
    {
        // Clamp left wheel duty cycle values between -255 and 255
        if (leftDutyCycle < -UINT8_MAX)
        {
            leftDutyCycle = -UINT8_MAX;
        }
        else if (leftDutyCycle > UINT8_MAX)
        {
            leftDutyCycle = UINT8_MAX;
        }
        
        // Clamp right wheel duty cycle values between -255 and 255
        if (rightDutyCycle < -UINT8_MAX)
        {
            rightDutyCycle = -UINT8_MAX;
        }
        else if (rightDutyCycle > UINT8_MAX)
        {
            rightDutyCycle = UINT8_MAX;
        }

        // Reverse direction pin if duty cycle value is below 0 and invert if negative to set duty cycle to OCR2 registers
        if (leftDutyCycle < 0)
        {
            PORTD |= static_cast<uint8_t>(Motor::LeftMotorDirection);
            leftDutyCycle = -leftDutyCycle;
        }
        else
        {
            PORTD &= ~static_cast<uint8_t>(Motor::LeftMotorDirection);
        }
        
        if (rightDutyCycle < 0)
        {
            PORTD |= static_cast<uint8_t>(Motor::RightMotorDirection);
            rightDutyCycle = -rightDutyCycle;
        }
        else
        {
            PORTD &= ~static_cast<uint8_t>(Motor::RightMotorDirection);
        }

        // Nerf right motor (because it is more powerful)
        rightDutyCycle = rightDutyCycle * rightMotorNerfNumerator / rightMotorNerfDenominator;
        
        // Set duty cycles to OCR2 registers
        // Important! OC2B is left, OC2A is right (see Config.h)
        OCR2A = rightDutyCycle;
        OCR2B = leftDutyCycle;        
    }

    void goStraight(int16_t dutyCycle)
    {
        // Motor max power pulse to kickstart them
        if (dutyCycle >= 0)
        {
            pulseMotorsForward();
        }
        else
        {
            pulseMotorsBackward();
        }

        // Set motor speed 
        setMotorSpeed(dutyCycle, dutyCycle);    
    }

    void pulseMotorsForward()
    {
        setMotorSpeed(UINT8_MAX, UINT8_MAX);
        msSleep(msMaxPulseDuration);
    }

    void pulseMotorsBackward()
    {
        setMotorSpeed(-UINT8_MAX, -UINT8_MAX);
        msSleep(msMaxPulseDuration);
    }

    void pulseMotorsClockwise()
    {
        setMotorSpeed(UINT8_MAX, -UINT8_MAX);
        msSleep(msMaxPulseDuration);
    }

    void pulseMotorsCounterclockwise()
    {
        setMotorSpeed(-UINT8_MAX, UINT8_MAX);
        msSleep(msMaxPulseDuration);
    }

    void rotate90Clockwise()
    {
        pulseMotorsClockwise();
        setMotorSpeed(calibratedRotationSpeed, -calibratedRotationSpeed);

        msSleep(msRotate90ClockwiseDuration);
        forceStopMotors(msForceStopAfterRotate90Duration);
    }

    void rotate90Counterclockwise()
    {
        pulseMotorsCounterclockwise();
        setMotorSpeed(-calibratedRotationSpeed, calibratedRotationSpeed);

        msSleep(msRotate90CounterclockwiseDuration);
        forceStopMotors(msForceStopAfterRotate90Duration);
    }

    void rotateSlightlyClockwise()
    {
        pulseMotorsClockwise();
        setMotorSpeed(calibratedRotationSpeed, -calibratedRotationSpeed);

        msSleep(msRotateSlightlyClockwiseDuration);
        forceStopMotors(msForceStopAfterRotateSlightlyDuration);
    }

    void rotateSlightlyCounterclockwise()
    {
        pulseMotorsCounterclockwise();
        setMotorSpeed(-calibratedRotationSpeed, calibratedRotationSpeed);

        msSleep(msRotateSlightlyClockwiseDuration);
        forceStopMotors(msForceStopAfterRotateSlightlyDuration);
    }

    void forceStopMotors(uint8_t msForcedDecelerationDuration)
    {
        setMotorSpeed(-getLeftMotorSpeed(), -getRightMotorSpeed());
        msSleep(msForcedDecelerationDuration);
        setMotorSpeed(0, 0);
        msSleep(msForcedDecelerationDuration); // Let motors slow back down
    }

    int16_t getLeftMotorSpeed()
    {
        if (PORTD & static_cast<uint8_t>(Motor::LeftMotorDirection)) // Read back direction bit from PORTD buffer
        {
            return -static_cast<int16_t>(OCR2B);
        }
        return OCR2B;
    }
    
    int16_t getRightMotorSpeed()
    {
        if (PORTD & static_cast<uint8_t>(Motor::RightMotorDirection)) // Read back direction bit from PORTD buffer
        {
            return -static_cast<int16_t>(OCR2A);
        }
        return OCR2A;
    }

    int16_t getAverageMotorSpeed()
    {
        return (getLeftMotorSpeed() + getRightMotorSpeed()) / 2;
    }
} // namespace lib
        
    