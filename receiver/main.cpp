/// Program for the travelling robot which receives data from the transmitter and travels on the various sections of the challenge
/// \file receiver/main.cpp
/// \author Misha Krieger-Raynauld (1952515), Simon Gauvin (1951457), Vlad Drelciuc (1941366), Nicolas Charron (1960356)
/// \date 2019-03-22

#include "Adc.h"
#include "Debug.h"
#include "ExitConditions.h"
#include "ExternalMemory.h"
#include "Infrared.h"
#include "Motors.h"
#include "Section1.h"
#include "Section2.h"
#include "Section3.h"
#include "Section4.h"
#include "Timer.h"
#include "TrackingAlgos.h"
#include "Usart.h"

int main()
{
    // Initialize
    lib::initializePins();
    lib::initializePiezo();
    lib::initializeTimer();
    lib::initializeUsart();
    lib::initializeMotors();

    // Startup sequence
    lib::startupSequence();
    DEBUG_PRINT("STARTED RECEIVER\n");

    // Print line tracker sensor readings and calibrate motor timings
    lib::setLedColor(lib::Led::Red); // Make LED red while calibrating or reading calibration settings
    if (PINB & lib::calibrationMask) // Connect wire between VCC and port B8 (pin B7) to enable calibration, or disable by connecting to GND
    {
        // Output sensor readings until button is pressed for manual adjustment in debug mode
        #ifdef DEBUG
            DEBUG_PRINT("PRINTING LINE TRACKER VALUES\n");
            while (lib::isButtonPressed() == false)
            {
                lib::printLineTrackerSensorValues();
            }
        #endif

        // Automatic motor timings calibration
        lib::calibrateMotorTimings();
    }

    // Read motor calibration timings back from EEPROM
    lib::readMotorTimings();

    // Test motor calibration if calibration pin is still connected
    while (PINB & lib::calibrationMask)
    {
        lib::msSleep(3000);
        DEBUG_PRINT("TESTING MOTOR CALIBRATION\n");
        lib::rotate90Clockwise();
        lib::msSleep(500);
        lib::rotate90Counterclockwise();
        lib::msSleep(500);
        lib::rotateSlightlyClockwise();
        lib::msSleep(500);
        lib::rotateSlightlyCounterclockwise();
    }

    // Turn off LED after reading calibration settings
    lib::setLedColor(lib::Led::Off);

    // Get selected section number from command
    enum class Section : uint8_t
    {
        Section1,
        Section2,
        Section3,
        Section4,
        Count
    } section;
    while (true)
    {
        DEBUG_PRINT("WAITING FOR SECTION SELECTION\n");

        uint8_t digit = lib::receiveInfraredCommandDigit();

        // Test if received digit is a valid section number
        if (digit > 4)
        {
            DEBUG_PRINT("INVALID SECTION NUMBER\n");
        }
        else
        {
            DEBUG_PRINT("SELECTED SECTION: ");
            DEBUG_PRINT_NUMBER(digit);
            DEBUG_PRINT('\n');

            // Set current section to received digit
            section = static_cast<Section>(digit - 1);

            // Set the LED color for the corresponding section number for 3 seconds
            lib::displayNumberOnTrackerLeds(digit);
            lib::msSleep(3000);

            break; // Start following section (stop waiting for valid command)
        }
    }

    // Cycle through sections
    for (uint8_t i = 0; i < static_cast<uint8_t>(Section::Count); i++)
    {
        switch (section)
        {
        case Section::Section1:
            DEBUG_PRINT("STARTED SECTION 1\n");
            followSection1();
            section = Section::Section2;
            DEBUG_PRINT("FINISHED SECTION 1\n");
            break;
        case Section::Section2:
            DEBUG_PRINT("STARTED SECTION 2\n");
            followSection2();
            section = Section::Section3;
            DEBUG_PRINT("FINISHED SECTION 2\n");
            break;
        case Section::Section3:
            DEBUG_PRINT("STARTED SECTION 3\n");
            followSection3();
            section = Section::Section4;
            DEBUG_PRINT("FINISHED SECTION 3\n");
            break;
        case Section::Section4:
            DEBUG_PRINT("STARTED SECTION 4\n");
            followSection4();
            section = Section::Section1;
            DEBUG_PRINT("FINISHED SECTION 4\n");
            break;
        }

        // Follow corner by turning left if this is not the last section
        if (i < static_cast<uint8_t>(Section::Count) - 1)
        {
            followCorner(false);
        }
        // Force stop if this is the last section
        else
        {
            lib::forceStopMotors();
        }
        
    }

    DEBUG_PRINT("RECEIVED FINISHED\n");

    // Make high pitch sound for 2 seconds
    lib::startPiezo(72);
    lib::msSleep(2000);
    lib::stopPiezo();

    // Turn off tracker LEDs when finished
    lib::displayNumberOnTrackerLeds(0);
}
