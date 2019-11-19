/// Function defining the behavior of the robot when in section 1
/// \file Section1.cpp
/// \author Misha Krieger-Raynauld (1952515), Simon Gauvin (1951457), Vlad Drelciuc (1941366), Nicolas Charron (1960356)
/// \date 2019-03-22

#include "Section1.h"
#include "Debug.h"
#include "ExitConditions.h"
#include "Infrared.h"
#include "Motors.h"
#include "Timer.h"
#include "TrackingAlgos.h"

namespace
{
    // Fast and slow speeds to use when uneven speed is desired for turning, or just for going slower in certain critical sections
    constexpr uint8_t fastSpeed = 150;
    constexpr uint8_t slowSpeed = 90;

    // Timing constant for zigzags
    constexpr uint16_t msZigTime = 300;
    constexpr uint16_t msZagTime = 240;

    /// Go forward and turn left in a short distance
    /// \return The values of the line tracker values if the function exited prematurely because it detected a point.
    ///         This is to be used to check if a point was detected and forward these values to reorientSensorsOnPoint for realignment
    uint8_t zig()
    {
        lib::pulseMotorsCounterclockwise();
        lib::setMotorSpeed(slowSpeed, fastSpeed);

        lib::startTimer(static_cast<uint32_t>(lib::secTimerMultiplier) * msZigTime / 1000); // Start timer for zig time
        // Loop until timer is expired or a point is detected
        uint8_t lineTrackerValues = lib::readLineTrackerValues();
        while (lib::isTimerExpired == false && !lineTrackerValues)
        {
            lineTrackerValues = lib::readLineTrackerValues();
        }

        lib::forceStopMotors(100);
        lib::msSleep(200);

        return lineTrackerValues;
    }
    
    /// Go forward and turn right in a short distance
    /// \return The values of the line tracker values if the function exited prematurely because it detected a point.
    ///         This is to be used to check if a point was detected and forward these values to reorientSensorsOnPoint for realignment
    uint8_t zag()
    {
        lib::pulseMotorsClockwise();
        lib::setMotorSpeed(fastSpeed, slowSpeed);
        
        lib::startTimer(static_cast<uint32_t>(lib::secTimerMultiplier) * msZagTime / 1000); // Start timer for zag time
        // Loop until timer is expired or a point is detected
        uint8_t lineTrackerValues = lib::readLineTrackerValues();
        while (lib::isTimerExpired == false && !lineTrackerValues)
        {
            lineTrackerValues = lib::readLineTrackerValues();
        }

        lib::forceStopMotors(100);
        lib::msSleep(200);

        return lineTrackerValues;
    }

    /// Move backward until perfectly perpendicular to and slightly behind S2
    void alignOnInitialSegmentPosition()
    {
        // Move backwards until a group of two edge sensors is on black
        lib::setMotorSpeed(-slowSpeed, -slowSpeed);
        uint8_t lineTrackerValues = lib::readLineTrackerValues();
        while (((lineTrackerValues & 0b11000) != 0b11000) && (lineTrackerValues & 0b00011) != 0b00011)
        {
            lineTrackerValues = lib::readLineTrackerValues();
        }
        lib::forceStopMotors(50);

        // Move back the sensors that are not on black
        if (lib::readLineTrackerValues() & 0b00001) // Rotate CCW while blocking a wheel
        {
            lib::setMotorSpeed(-slowSpeed, 0);
        }
        else if (lib::readLineTrackerValues() & 0b10000) // Rotate CW while blocking a wheel
        {
            lib::setMotorSpeed(0, -slowSpeed);
        }

        // Wait until all sensors are on black to stop
        while (bothEdgeSensorsOnBlack() == false)
        {
        }
        lib::forceStopMotors(50);

        // Move backwards until the robot is just behind the black line for proper timings
        lib::setMotorSpeed(-slowSpeed, -slowSpeed);
        while (anyEdgeSensorOnBlack())
        {
        }
        lib::forceStopMotors();

        // Turn tracker LEDs off as they won't be updated for a while
        lib::displayNumberOnTrackerLeds(0);
    }

    /// Advance a calibrated distance between 2 points
    void advanceByDistanceBetweenPoints()
    {
        lib::goStraight(lib::calibratedTimingSpeed);
        lib::msSleep(lib::msBetweenPointsDuration);
        lib::forceStopMotors();
    }

    /// Realign the robot by analyzing the given sensor values to reorient it properly towards the next point
    /// \param lineTrackerValues The line tracker sensor readings to analyze to reorient.
    ///                          These are explicitly provided rather than polled inside the function in order to
    ///                          prevent the possibility of the readings changing since the last zig/zag force stop,
    ///                          as the points make for a very narrow target to stop on
    void reorientSensorsOnPoint(uint8_t lineTrackerValues)
    {
        // If leftmost sensor is on black, turn counterclockwise twice
        if (lineTrackerValues & 0b10000)
        {
            DEBUG_PRINT("\tRealigning because first sensor was on point\n");
            lib::rotateSlightlyCounterclockwise();
            lib::rotateSlightlyCounterclockwise();
        }
        // If second from left sensor is on black, turn counterclockwise once
        else if (lineTrackerValues & 0b01000)
        {
            DEBUG_PRINT("\tRealigning because second sensor was on point\n");
            lib::rotateSlightlyCounterclockwise();
        }
        // If second from right sensor is on black, turn clockwise once
        else if (lineTrackerValues & 0b00010)
        {
            DEBUG_PRINT("\tRealigning because fourth sensor was on point\n");
            lib::rotateSlightlyClockwise();
        }
        // If rightmost sensor is on black, turn clockwise twice
        else if (lineTrackerValues & 0b00001)
        {
            DEBUG_PRINT("\tRealigning because fifth sensor was on point\n");
            lib::rotateSlightlyClockwise();
            lib::rotateSlightlyClockwise();
        }
        else
        {
            DEBUG_PRINT("\tNo realignment was necessary\n");
        }
        
    }

    /// Move forward slightly. Useful to avoid retriggering the sensors on the same point
    void advanceSlightlyPastPoint()
    {
        lib::displayNumberOnTrackerLeds(0); // Turn off tracker LEDs as robot will now be leaving point and sensors will be above white
        lib::goStraight(lib::calibratedTimingSpeed);
        lib::msSleep(lib::msBetweenPointsDuration / 3); // Advance by a fraction of the time between two points
        lib::forceStopMotors(100);
    }

    /// Advance to the next point a certain number of times and realign each time
    /// \param nbRepetitions            The number of points to advance by
    /// \param maxZigZagsBetweenPoints   The maximum number of zigzags to make between two points.
    ///                                 If the number of zigzags exceeds this number, this function will return
    void advanceToNextPoint(uint8_t nbRepetitions, uint8_t maxZigZagsBetweenPoints = 8)
    {
        for (uint8_t i = 0; i < nbRepetitions; i++)
        {
            uint8_t lineTrackerValues = 0;
            uint8_t zigZagCounter = 0;

            // Zigzag until maximum number of zigzags or point hit
            DEBUG_PRINT("\tZigzag counter:");
            while (zigZagCounter < maxZigZagsBetweenPoints)
            {
                lineTrackerValues = zag(); // Perform zag (which returns early on detection) and return sensor readings for analysis
                zigZagCounter++;
                if (lineTrackerValues != 0b00000)
                {
                    break; // Break if point was detected
                }
                else
                {
                    lineTrackerValues = zig(); // Perform zig (which returns early on detection) and return sensor readings for analysis
                    zigZagCounter++;
                    DEBUG_PRINT(" ");
                    DEBUG_PRINT_NUMBER(zigZagCounter);
                    DEBUG_PRINT('\n');
                    if (lineTrackerValues != 0b00000)
                    {
                        break; // Break if point was detected
                    }
                 }
            }
            DEBUG_PRINT('\n');

            // Realign sensor orientation
            DEBUG_PRINT("\tPoint found, realigning\n");
            reorientSensorsOnPoint(lineTrackerValues); // Provide given sensor values for analysis
            lib::readLineTrackerValues(); // Update tracker LEDs
            lib::msSleep(200); // Sleep a little after realigning

            // Move forward slightly to avoid retriggering next point if this is not the last point to advance by
            if (i < nbRepetitions - 1)
            {
                advanceSlightlyPastPoint();
            }
        }
    }

    /// Use calibrated timing to center the robot's center of rotation on the point by moving forward by a hardcoded amount
    void centerRobotOnPoint()
    {
        lib::goStraight(lib::calibratedTimingSpeed);
        lib::msSleep(lib::msSensorToCenterOfRotationDuration);
        lib::forceStopMotors();
    }
} // namespace

void followSection1()
{
    // Move forward until slightly past T
    followLine(fastSpeed, 20, 40, 10, false, threeMiddleSensorsOnWhite);
    lib::forceStopMotors();
    lib::displayNumberOnTrackerLeds(0); // Turn off tracker LEDs while waiting for command

    // Get selected point from command
    DEBUG_PRINT("WAITING FOR POINT SELECTION\n");

    uint8_t digit = lib::receiveInfraredCommandDigit();

    DEBUG_PRINT("SELECTED POINT: ");
    DEBUG_PRINT_NUMBER(digit);
    DEBUG_PRINT('\n');

    // Set point to received digit
    enum class Point : uint8_t
    {
        P1,
        P2,
        P3,
        P4,
        P5,
        P6,
        P7,
        P8,
        P9
    } point = static_cast<Point>(digit - 1);

    // Set the LED color for the corresponding point number (clamped) for 3 seconds
    lib::displayNumberOnTrackerLeds(digit);
    lib::msSleep(3000);

    // Realign the robot on the starting T by making sure the robot is perfectly perpendicular to S2
    DEBUG_PRINT("\tAligning with T\n");
    alignOnInitialSegmentPosition();
    lib::msSleep(200);

    // Go forward by a hardcoded amount to align with the point grid
    DEBUG_PRINT("\tGoing forward until aligned with line P7-P8-P9\n");
    lib::goStraight(lib::calibratedTimingSpeed);
    lib::msSleep(lib::msSection1StartOffsetDuration + 150); // +150 to travel a bit more
    lib::forceStopMotors();
    lib::msSleep(200);

    // Rotate 90 degrees counterclockwise to face the point grid
    DEBUG_PRINT("\tRotating 90 degrees counterclockwise\n");
    lib::rotate90Counterclockwise();
    lib::msSleep(200);
    
    // Go backward a little bit in case the point was slightly overshot
    lib::goStraight(-lib::calibratedTimingSpeed);
    lib::msSleep(lib::msBetweenPointsDuration / 3);
    lib::forceStopMotors();
    lib::msSleep(200); // Sleep a little after reversing

    // Rotate slightly left to compensate for the fact that advanceToNextPoint starts by zigzaging right
    lib::rotateSlightlyCounterclockwise();

    // Move until first point (P9) is encountered by the line tracker then realign sensors on it
    DEBUG_PRINT("\tGoing to start point (P9)\n");
    advanceToNextPoint(1, 10);
    advanceSlightlyPastPoint();
    
    DEBUG_PRINT("\tStarting path to ");
    switch (point)
    {
    case Point::P1:
        DEBUG_PRINT("P1\n");
        advanceToNextPoint(2);
        centerRobotOnPoint();
        lib::rotate90Clockwise();
        advanceToNextPoint(1, 6);
        centerRobotOnPoint();
        break;
    case Point::P2:
        DEBUG_PRINT("P2\n");
        advanceToNextPoint(1);
        centerRobotOnPoint();
        lib::rotate90Clockwise();
        advanceToNextPoint(1, 6);
        centerRobotOnPoint();
        break;
    case Point::P3:
        DEBUG_PRINT("P3\n");
        centerRobotOnPoint();
        lib::rotate90Clockwise();
        advanceToNextPoint(1, 6);
        centerRobotOnPoint();
        break;
    case Point::P4:
        DEBUG_PRINT("P4\n");
        advanceToNextPoint(2);
        centerRobotOnPoint();
        lib::rotate90Clockwise();
        advanceByDistanceBetweenPoints();
        break;
    case Point::P5:
        DEBUG_PRINT("P5\n");
        advanceToNextPoint(1);
        centerRobotOnPoint();
        lib::rotate90Clockwise();
        advanceByDistanceBetweenPoints();
        break;
    case Point::P6:
        DEBUG_PRINT("P6\n");
        centerRobotOnPoint();
        lib::rotate90Clockwise();
        advanceByDistanceBetweenPoints();
        break;
    case Point::P7:
        DEBUG_PRINT("P7\n");
        advanceToNextPoint(2);
        centerRobotOnPoint();
        lib::rotate90Clockwise();
        break;
    case Point::P8:
        DEBUG_PRINT("P8\n");
        advanceToNextPoint(1);
        centerRobotOnPoint();
        lib::rotate90Clockwise();
        break;
    case Point::P9:
        DEBUG_PRINT("P9\n");
        centerRobotOnPoint();
        lib::rotate90Clockwise();
        break;
    }
    DEBUG_PRINT("\tReached point\n");

    // Make low pitch sound for 3 seconds
    lib::startPiezo(45);
    lib::msSleep(3000);
    lib::stopPiezo();

    // Rotate 90 degrees clockwise
    DEBUG_PRINT("\tRotating 90 degrees clockwise\n");
    lib::rotate90Clockwise();

    // Make low pitch sound for 3 seconds
    lib::startPiezo(45);
    lib::msSleep(3000);
    lib::stopPiezo();

    // Move to be aligned with third column according to the selected point
    DEBUG_PRINT("\tMoving to be aligned with S3\n");
    uint8_t column = static_cast<uint8_t>(point) % 3; // Get column number, indexed from 0
    switch (column)
    {
    case 0:
        advanceByDistanceBetweenPoints();
        // Fallthrough
    case 1:
        advanceByDistanceBetweenPoints();
    case 2:
        break;
    }

    // Rotate counterclockwise to point up
    lib::rotate90Counterclockwise();

    // Move to be on P3 (first row) according to the selected point
    int8_t row = static_cast<uint8_t>(point) / 3; // Get row number, indexed from 0
    switch (row)
    {
    case 2:
        advanceByDistanceBetweenPoints();
        // Fallthrough
    case 1:
        // Advance by slightly less than a point
        lib::goStraight(lib::calibratedTimingSpeed);
        lib::msSleep(lib::msBetweenPointsDuration * 2 / 3);
        lib::forceStopMotors();
        // Fallthrough
    case 0:
        break;
    }

    DEBUG_PRINT("\tLeaving point grid\n");
    bool detectedBlackWhileTurning = false;
    // Do a sequence of actions to try to get back to S3, exiting if it is encountered earlier than expected
    for (uint8_t i = 0; i < 3; i++)
    {
        // Note: using if-else rather than switch to be able to break from loop

        // Move forward twice by distance between points to be parallel to S3
        if (i == 0)
        {
            lib::goStraight(lib::calibratedTimingSpeed);
            lib::startTimer(static_cast<uint32_t>(lib::secTimerMultiplier) * lib::msBetweenPointsDuration / 1200);
            while (lib::isTimerExpired == false && allSensorsOnWhite())
            {
            }
            lib::forceStopMotors();

            if (allSensorsOnWhite() == false)
            {
                break;
            }
        }
        // Rotate to be perpendicular to S3
        else if (i == 1)
        {
            lib::pulseMotorsClockwise();
            lib::setMotorSpeed(lib::calibratedRotationSpeed, -lib::calibratedRotationSpeed);
            
            lib::startTimer(static_cast<uint32_t>(lib::secTimerMultiplier) * lib::msRotate90ClockwiseDuration / 920);
            while (lib::isTimerExpired == false)
            {
                if (allSensorsOnWhite() == false)
                {
                    detectedBlackWhileTurning = true;
                }
            }
            lib::forceStopMotors(100);
        }
        // Go forward until S3 is detected and turn to realign
        else
        {
            // Go straight until black is hit, unless robot already saw black earlier while turning
            if (detectedBlackWhileTurning == false)
            {
                lib::goStraight(fastSpeed);
                // Start a timer for half a second in case the robot misses the line
                lib::startTimer(lib::secTimerMultiplier / 2);
                while (allSensorsOnWhite() && lib::isTimerExpired == false)
                {
                }
            }

            // Advance slightly before turning
            lib::goStraight(fastSpeed);
            lib::msSleep(lib::msSensorToCenterOfRotationDuration / 4);

            // Rotate counterclockwise to be realigned on S3
            followCorner(false);
        }
    }

    // Follow S3 until realigned
    DEBUG_PRINT("\tReached S3\n");
    followLine(fastSpeed, 100, 100, 10, true, middleSensorOnBlack);
    lib::forceStopMotors(100); // Force stop to prevent drifting

    // Follow S3 until corner C2 is reached
    followLine(fastSpeed, 100, 100, 10, false, allSensorsOnWhite);
}