/// Function defining the behavior of the robot when in section 2
/// \file Section2.cpp
/// \author Misha Krieger-Raynauld (1952515), Simon Gauvin (1951457), Vlad Drelciuc (1941366), Nicolas Charron (1960356)
/// \date 2019-03-22

#include "Section2.h"
#include "Debug.h"
#include "ExitConditions.h"
#include "Motors.h"
#include "Timer.h"
#include "TrackingAlgos.h"

void followSection2()
{
    static constexpr uint8_t fastSpeed = 150;
    static constexpr uint8_t slowSpeed = 100;

    // Follow straight line for two seconds
    lib::startTimer(lib::secTimerMultiplier * 2);
    followLine(fastSpeed, 100, 100, 10, true, timerExpired);

    // Straight line. Stop early to start curve in time
    followLine(fastSpeed, 10, 20, 75, false, threeMiddleSensorsOnWhite);

    // Turn left until aligned with first curve
    lib::setMotorSpeed(50, 100);
    waitUntilSensorDetectsLine(2);

    // Follow curve until corner at the end of the curve
    followLine(slowSpeed, 30, 70, 5, false, allSensorsOnWhite);

    // Follow corner at the end of the curve by turning right
    followCorner(true);

    // Follow line for 2 seconds
    lib::startTimer(lib::secTimerMultiplier * 2);
    followLine(fastSpeed, 100, 100, 10, true, timerExpired);

    // Follow straight line after 2 seconds correction above
    followLine(fastSpeed, 20, 23, 100, false, threeMiddleSensorsOnWhite);

    // Turn left to align with second line at the end of the line
    lib::setMotorSpeed(60, 120);
    waitUntilSensorDetectsLine(2);

    // Follow straight line for at least 2 seconds
    lib::startTimer(lib::secTimerMultiplier * 2);
    followLine(fastSpeed, 100, 100, 10, true, timerExpired);

    // Follow straight line until curve
    followLine(fastSpeed, 20, 23, 100, false, threeMiddleSensorsOnWhite);

    // Turn left until aligned with second curve
    lib::setMotorSpeed(0, slowSpeed);
    waitUntilSensorDetectsLine(2);
    lib::forceStopMotors(100);

    // Follow curve for 2 seconds to realign in order to avoid the next ExitCondition triggering prematurely
    lib::startTimer(lib::secTimerMultiplier * 2);    
    followLine(slowSpeed, 40, 50, 30, false, timerExpired, true);

    // Follow curve slowly until straight segment while preventing corrections to the left
    followLine(slowSpeed, 40, 50, 30, false, firstSensorOnBlack, true);

    // Follow line until corner
    followLine(fastSpeed, 40, 50, 10, false, allSensorsOnWhite);
}
