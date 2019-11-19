/// Functions defining the behavior of the robot when following a path
/// \file TrackingAlgos.cpp
/// \author Misha Krieger-Raynauld (1952515), Simon Gauvin (1951457), Vlad Drelciuc (1941366), Nicolas Charron (1960356)
/// \date 2019-03-22

#include "TrackingAlgos.h"
#include "Debug.h"
#include "ExitConditions.h"
#include "Motors.h"
#include "Timer.h"

uint16_t followLine(int16_t speed, uint8_t initialTurnDifference, uint8_t maxTurnDifference, uint8_t msTurnCorrectionDelay,
                    bool useEdgeSensors, bool (*exitConditionFunction)(), bool canOnlyTurnRight)
{
    enum class State : uint8_t
    {
        Searching,
        OnLine,
        TemporaryLeftSpeedUp,
        TemporaryRightSpeedUp,
        AbruptTurnLeft,
        AbruptTurnRight
    } robotState = State::OnLine;

    lib::setMotorSpeed(speed, speed);

    // Timer for the time spent inside the function
    uint16_t timeFollowed = 0;

    // Counter for how much to increase the correction with time
    uint16_t correctionCounter = 0;

    bool lastSeenSideIsRight = true;

    while (true)
    {
        // Read line sensor
        uint8_t lineTrackerValues = lib::readLineTrackerValues();

        if (exitConditionFunction())
        {
            return timeFollowed;
        }

        // Emergency detection

        // If robot sees nothing
        if (allSensorsOnWhite())
        {
            robotState = State::Searching;
        }
        // If all the sensors except the leftmost one are on white
        else if (useEdgeSensors == true && lineTrackerValues == 0b10000)
        {
            robotState = State::AbruptTurnLeft;
        }
        // If all the sensors except the rightmost one are on white
        else if (useEdgeSensors == true && lineTrackerValues == 0b00001)
        {
            robotState = State::AbruptTurnRight;
        }

        // Remember which side was last seen
        if (lib::isLineTrackerSensorOnBlack(lineTrackerValues, 4))
        {
            lastSeenSideIsRight = true;
        }
        else if (lib::isLineTrackerSensorOnBlack(lineTrackerValues, 0))
        {
            lastSeenSideIsRight = false;
        }

        switch (robotState)
        {
        case State::Searching:
            // If robot is centered on line
            if (lib::isLineTrackerSensorOnBlack(lineTrackerValues, 0))
            {
                lib::setMotorSpeed(speed, speed);
                robotState = State::OnLine;
            }
            else if (lastSeenSideIsRight == true)
            {
                // Turn right
                lib::setMotorSpeed(speed, -speed);
            }
            else
            {
                lib::setMotorSpeed(-speed, speed);                
            }
            break;
        case State::OnLine:
            // Do nothing, unless sensor 2 is on white sensor 1 or 3 is detecting a line
            if (lib::isLineTrackerSensorOnBlack(lineTrackerValues, 2) == false)
            {
                if (canOnlyTurnRight == false &&
                    lib::isLineTrackerSensorOnBlack(lineTrackerValues, 1) &&
                    lib::isLineTrackerSensorOnBlack(lineTrackerValues, 3) == false) // Check that opposite side sensor is not
                                                                                    // also on for proper alignment with T's
                {
                    robotState = State::TemporaryLeftSpeedUp;
                }
                else if (lib::isLineTrackerSensorOnBlack(lineTrackerValues, 3) &&
                        lib::isLineTrackerSensorOnBlack(lineTrackerValues, 1) == false) // Check that opposite side sensor is not
                                                                                        // also on for proper alignment with T's
                {
                    robotState = State::TemporaryRightSpeedUp;
                }
            }
            break;
        case State::TemporaryLeftSpeedUp: // Fallthrough because similar logic
        case State::TemporaryRightSpeedUp:
            // If robot is centered on line
            if (lib::isLineTrackerSensorOnBlack(lineTrackerValues, 2))
            {
                correctionCounter = 0;
                lib::setMotorSpeed(speed, speed);
                robotState = State::OnLine;
            }
            else
            {
                // If correctionCounter (the amount of turn difference) + initial turn diff is bigger than max turn
                if (correctionCounter + initialTurnDifference < maxTurnDifference)
                {
                    correctionCounter++;
                }

                if (robotState == State::TemporaryLeftSpeedUp)
                {
                    int16_t rightMotorSpeed = lib::getRightMotorSpeed();
                    // Speed down left motor proportionate to the time spent in this state
                    lib::setMotorSpeed(rightMotorSpeed - correctionCounter - initialTurnDifference, speed);
                }
                else
                {
                    int16_t leftMotorSpeed = lib::getLeftMotorSpeed();
                    // Speed down right motor proportionate to the time spent in this state
                    lib::setMotorSpeed(speed, leftMotorSpeed - correctionCounter - initialTurnDifference);
                }
            }
            break;
        case State::AbruptTurnLeft:
            if (lib::isLineTrackerSensorOnBlack(lineTrackerValues, 2))
            {
                robotState = State::OnLine;
                lib::forceStopMotors();
                lib::setMotorSpeed(speed, speed);
            }
            else
            {
                lib::setMotorSpeed(0, speed);
            }
            break;
        case State::AbruptTurnRight:
            if (lib::isLineTrackerSensorOnBlack(lineTrackerValues, 2))
            {
                robotState = State::OnLine;
                lib::forceStopMotors();
                lib::setMotorSpeed(speed, speed);
            }
            else
            {
                lib::setMotorSpeed(speed, 0);
            }
            break;
        }
        // Delay by a certain amount of time to be able to count the time
        lib::msSleep(msTurnCorrectionDelay);
        timeFollowed += msTurnCorrectionDelay;
    }
    return timeFollowed;
}

void followRectangle()
{
    enum class State
    {
        InSquare,
        TemporaryLeftSpeedUp,
        TemporaryRightSpeedUp
    } robotState = State::InSquare;

    int16_t speed = 120;
    uint8_t initialTurnDifference = 20;
    uint8_t maxTurnDifference = 80;
    uint8_t turnCorrectionDelay = 20;

    lib::setMotorSpeed(speed, speed);

    // Counter for how much to increase the correction with time
    uint16_t correctionCounter = 0;

    while (true)
    {
        // Read line sensor
        uint8_t lineTrackerValues = lib::readLineTrackerValues();

        if (bothEdgeSensorsOnBlack())
        {
            return;
        }

        switch (robotState)
        {
        case State::InSquare:
            if (lib::isLineTrackerSensorOnBlack(lineTrackerValues, 0))
            {
                robotState = State::TemporaryRightSpeedUp;
            }
            else if (lib::isLineTrackerSensorOnBlack(lineTrackerValues, 4))
            {
                robotState = State::TemporaryLeftSpeedUp;                
            }
            break;
        case State::TemporaryLeftSpeedUp: // Fallthrough because similar logic
        case State::TemporaryRightSpeedUp:
            // If robot is centered on line
            if (bothEdgeSensorsOnWhite() == true)
            {
                correctionCounter = 0;
                lib::setMotorSpeed(speed, speed);
                robotState = State::InSquare;
            }
            else
            {
                // If timer (the amount of turn difference) + initial turn diff is bigger than max turn
                if (correctionCounter + initialTurnDifference < maxTurnDifference)
                {
                    correctionCounter++;
                }

                if (robotState == State::TemporaryLeftSpeedUp)
                {
                    int16_t rightMotorSpeed = lib::getRightMotorSpeed();
                    // Speed down left motor proportionate to the time spent in this state
                    lib::setMotorSpeed(rightMotorSpeed - correctionCounter - initialTurnDifference, speed);
                }
                else
                {
                    int16_t leftMotorSpeed = lib::getLeftMotorSpeed();
                    // Speed down right motor proportionate to the time spent in this state
                    lib::setMotorSpeed(speed, leftMotorSpeed - correctionCounter - initialTurnDifference);
                }
            }
            break;
        }
        lib::msSleep(turnCorrectionDelay);
    }
}

void followCorner(bool turnClockwise)
{
    // Go forward a little to overshoot corner
    lib::goStraight(lib::calibratedTimingSpeed);
    lib::msSleep(lib::msSensorToCenterOfRotationDuration / 2);
    lib::forceStopMotors();
    
    // Begin rotation
    static constexpr uint8_t rotationSpeed = 100;
    if (turnClockwise)
    {
        lib::pulseMotorsClockwise();
        lib::setMotorSpeed(rotationSpeed, -rotationSpeed);
    }
    else
    {
        lib::pulseMotorsCounterclockwise();
        lib::setMotorSpeed(-rotationSpeed, rotationSpeed);
    }

    // First rotate without checking middle sensor to avoid false positive on S1
    // segment when following the corner at the end of section 4

    // Wait for all sensors to be on white and that at least half a second has passed
    // before the eventual check for if the middle sensor is back on the black line
    lib::startTimer(lib::secTimerMultiplier / 2);
    while (lib::readLineTrackerValues() != 0 || lib::isTimerExpired == false)
    {
    }
    
    // Rotate until exactly the middle sensor is back on the black line
    waitUntilSensorDetectsLine(2, true);
    lib::forceStopMotors();
}

void waitUntilSensorDetectsLine(uint8_t sensorIndex, bool preferExactMatch)
{
    if (sensorIndex > 4)
    {
        DEBUG_PRINT("ERROR: INVALID SENSOR NUMBER\n");
        return;
    }

    uint8_t lineTrackerValues = lib::readLineTrackerValues();
    

    if (preferExactMatch == true)
    {
        // Remember if the given sensor was on black
        bool wasSensorOnBlack = false;
        
        while (lineTrackerValues != (1 << (lib::nbLineTrackerSensors - 1 - sensorIndex)))
        {
            // Detect if sensor is on black if it has never been
            if (wasSensorOnBlack == false)
            {
                wasSensorOnBlack = lib::isLineTrackerSensorOnBlack(lineTrackerValues, sensorIndex);
            }

            // If the desired sensor was seen and it is no longer seen, the function must exit
            if (wasSensorOnBlack && lib::isLineTrackerSensorOnBlack(lineTrackerValues, sensorIndex) == false)
            {
                break;
            }
            
            lineTrackerValues = lib::readLineTrackerValues();
        }
    }
    else
    {
        while (lib::isLineTrackerSensorOnBlack(lineTrackerValues, sensorIndex) == false)
        {
            lineTrackerValues = lib::readLineTrackerValues();
        }
    }
}
