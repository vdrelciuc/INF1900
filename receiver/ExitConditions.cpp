/// Function to be used as function pointers by follow line
/// \file ExitConditions.cpp
/// \author Misha Krieger-Raynauld (1952515), Simon Gauvin (1951457), Vlad Drelciuc (1941366), Nicolas Charron (1960356)
/// \date 2019-03-22

#include "ExitConditions.h"
#include "Debug.h"
#include "Timer.h"

bool threeMiddleSensorsOnWhite()
{
    return (lib::readLineTrackerValues() & 0b01110) == 0;
}

bool threeMiddleSensorsOnBlack()
{
    return (lib::readLineTrackerValues() & 0b01110) == 0b01110;
}

bool threeLeftSensorsOnBlack()
{
    return (lib::readLineTrackerValues() & 0b11100) == 0b11100;
}

bool allSensorsOnWhite()
{
    return lib::readLineTrackerValues() == 0b00000;
}

bool anyEdgeSensorOnBlack()
{
    return lib::readLineTrackerValues() & 0b10001;
}

bool bothEdgeSensorsOnWhite()
{
    return (lib::readLineTrackerValues() & 0b10001) == 0;
}

bool leftSensorOnWhite()
{
    uint8_t lineTrackerValues = lib::readLineTrackerValues();
    return lib::isLineTrackerSensorOnBlack(lineTrackerValues, 0) == false;
}

bool rightSensorOnWhite()
{
    uint8_t lineTrackerValues = lib::readLineTrackerValues();
    return lib::isLineTrackerSensorOnBlack(lineTrackerValues, 4) == false;
}

bool bothEdgeSensorsOnBlack()
{
    return (lib::readLineTrackerValues() & 0b10001) == 0b10001;
}

bool anySensorOnBlack()
{
    return !allSensorsOnWhite();
}

bool middleSensorOnBlack()
{
    return lib::readLineTrackerValues() & 0b00100;
}

bool firstSensorOnBlack()
{
    return lib::readLineTrackerValues() & 0b10000;
}

bool secondSensorOnBlack()
{
    return lib::readLineTrackerValues() & 0b01000;
}

bool timerExpired()
{
    return lib::isTimerExpired;
}

bool buttonPressed()
{
    return lib::wasButtonPressed;
}

bool neverStop()
{
    return false;
}