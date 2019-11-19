/// Function defining the behavior of the robot when in section 3
/// \file Section3.cpp
/// \author Misha Krieger-Raynauld (1952515), Simon Gauvin (1951457), Vlad Drelciuc (1941366), Nicolas Charron (1960356)
/// \date 2019-03-22

#include "Section3.h"
#include "Debug.h"
#include "ExitConditions.h"
#include "Motors.h"
#include "Timer.h"
#include "TrackingAlgos.h"

namespace
{
    /// Detect the line number on which the robot was placed
    /// \return The line number, indexed from 1
    uint8_t detectLineNumber()
    {
        static constexpr uint8_t speed = 140;

        followLine(speed, 40, 70, 20, false, anyEdgeSensorOnBlack);
        followLine(speed, 40, 70, 20, false, bothEdgeSensorsOnWhite);

        // Get time between first and second branch
        uint16_t firstInterval = followLine(speed, 40, 70, 20, false, anyEdgeSensorOnBlack);

        // Read line tracker values
        uint8_t lineTrackerValues = lib::readLineTrackerValues();

        bool isRightBranchFirst = lib::isLineTrackerSensorOnBlack(lineTrackerValues, 4);

        followLine(speed, 40, 70, 10, false, bothEdgeSensorsOnWhite);
        
        uint16_t secondInterval = followLine(speed, 40, 70, 20, false, anyEdgeSensorOnBlack);

        // If the first interval is way smaller than the second interval,
        // we know that the course is a longer one (#1 or #3)
        uint16_t augmentedFirstInterval = firstInterval * 3 / 2;
        // If the first interval, even augmented, is still smaller than the second
        bool isLongCourse = augmentedFirstInterval < secondInterval;

        uint8_t courseNumber;
        // If the first single branch was on the right, we know that the course is either #3 or #4
        if (isRightBranchFirst)
        {
            // If the course was short, it is course #3
            if (isLongCourse)
            {
                courseNumber = 3;
            }
            // Otherwise, it is course #4
            else
            {       
                courseNumber = 4;
            }
        }
        else
        {
            // If the course was short, it is course #1
            if (isLongCourse)
            {
                courseNumber = 1;
            }
            // Otherwise, it is course #2
            else
            {
                courseNumber = 2;
            }   
        }

        lib::forceStopMotors();

        DEBUG_PRINT('\t');
        DEBUG_PRINT_NUMBER(firstInterval);
        DEBUG_PRINT(' ');
        DEBUG_PRINT_NUMBER(secondInterval);
        DEBUG_PRINT(' ');
        DEBUG_PRINT_NUMBER(augmentedFirstInterval);
        DEBUG_PRINT(' ');
        DEBUG_PRINT_NUMBER(courseNumber);
        DEBUG_PRINT(' ');
        DEBUG_PRINT_NUMBER(isLongCourse);
        DEBUG_PRINT(' ');
        DEBUG_PRINT_NUMBER(isRightBranchFirst);
        DEBUG_PRINT('\n');

        return courseNumber;
    }
} // namespace

void followSection3()
{
    // Follow line until T
    followLine(165, 40, 50, 20, false, threeMiddleSensorsOnBlack);
    lib::forceStopMotors();

    DEBUG_PRINT("WAITING FOR BUTTON PRESS\n");
    while (lib::isButtonPressed() == false)
    {
        lib::readLineTrackerValues();
    }
    DEBUG_PRINT("DETECTION STARTED\n");
    uint8_t lineNumber = detectLineNumber();
    DEBUG_PRINT("DETECTION COMPLETE\n");

    // Show line number with LEDs
    lib::displayNumberOnTrackerLeds(lineNumber);
    DEBUG_PRINT("WAITING FOR BUTTON PRESS\n");
    while (lib::isButtonPressed() == false)
    {
        // Wait but do not read the sensor values here to keep showing the line number with the LEDs
    }

    // Follow line until corner
    followLine(150, 10, 70, 20, false, allSensorsOnWhite);
}
