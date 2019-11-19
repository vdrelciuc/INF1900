/// Function to be used as function pointers to check whether to exit followLine
/// \file ExitConditions.h
/// \author Misha Krieger-Raynauld (1952515), Simon Gauvin (1951457), Vlad Drelciuc (1941366), Nicolas Charron (1960356)
/// \date 2019-03-22

#ifndef EXITCONDITIONS_H
#define EXITCONDITIONS_H

#include "GeneralIo.h"

/// Read line tracker values and return true when three middle sensors are on white
bool threeMiddleSensorsOnWhite();

/// Read line tracker values and return true when three middle sensors are on black
bool threeMiddleSensorsOnBlack();

/// Read line tracker values and return true when three leftmost sensors are on black
bool threeLeftSensorsOnBlack();

/// Read line tracker values and return true when all sensors are on white
bool allSensorsOnWhite();

/// Read line tracker values and return true when any of the edge sensors is on black
bool anyEdgeSensorOnBlack();

/// Read line tracker values and return true when both edge sensors are on white
bool bothEdgeSensorsOnWhite();

/// Read line tracker values and return true when left sensor is on white
bool leftSensorOnWhite();

/// Read line tracker values and return true when right sensor is on white
bool rightSensorOnWhite();

/// Read line tracker values and return true when both edge sensors are on black
bool bothEdgeSensorsOnBlack();

/// Read line tracker values and return true when any of the sensors is on black
bool anySensorOnBlack();

/// Read line tracker values and return true when middle sensor is on black
bool middleSensorOnBlack();

/// Read line tracker values and return true when first sensor is on black
bool firstSensorOnBlack();

/// Read line tracker values and return true when second sensor is on black
bool secondSensorOnBlack();

/// Return true when timer is expired
bool timerExpired();

/// Return true if the ISR variable for the asynchronous button interrupt was set
/// Note: requires lib::enableButtonInterrupts() to have been called beforehand
bool buttonPressed();

/// Never return true (for testing)
bool neverStop();

#endif // EXITCONDITIONS_H
