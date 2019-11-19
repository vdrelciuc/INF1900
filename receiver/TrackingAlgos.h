/// Functions defining the behavior of the robot when following a path
/// \file TrackingAlgos.h
/// \author Misha Krieger-Raynauld (1952515), Simon Gauvin (1951457), Vlad Drelciuc (1941366), Nicolas Charron (1960356)
/// \date 2019-03-22

#ifndef TRACKINGALGOS_H
#define TRACKINGALGOS_H

#include <stdint.h>

/// Algorithm for following a line. This function will block and return when stopCondition returns true
/// \param speed                    The speed at which the robot must follow the line
/// \param initialTurnDifference    The initial value at which the appropriate wheel slows down when the robot must correct itself
/// \param maxTurnDifference        The maximum value at which the appropriate wheel slows down when the robot must correct itself
///                                 If this parameter is equal or lower than initialTurnDifference, no progressive turning will take place
/// \param turnCorrectionDelay      The delay between each read of the sensors. This will also affect the speed at which the progressive
///                                 turning increases.
/// \param useEdgeSensors           Use edges sensors for tracking the line. If this parameter is true, the robot will block the appropriate
///                                 wheel to quickly go back on track.
/// \param exitConditionFunction    A pointer to a function returning a bool. This function passed as param must contain the logic to
///                                 determine when the algorithm ends.
/// \param canOnlyTurnRight         Tells the function if the robot is only allowed to turn right. Useful for S2 second curve end detection.
/// \return                         The time in milliseconds the robot followed the line
uint16_t followLine(int16_t speed, uint8_t initialTurnDifference, uint8_t maxTurnDifference, uint8_t turnCorrectionDelay,
                    bool useEdgeSensors, bool (*exitConditionFunction)(), bool canOnlyTurnRight = false);

/// Algorithm for staying inside a rectangle.
/// Blocks until rectangle has been left by checking if both edge sensors are on black. Useful for section 4
void followRectangle();

/// Algorithm for taking a sharp corner
/// \param turnCounterclockwise The direction of the turn
void followCorner(bool turnClockwise);

/// Block the execution of the program until the desired sensor detects a line
/// \param sensorIndex      The index of the sensor which has to hit a line, indexed from 0
/// \param preferExactMatch If true, the function will wait for the match to be exact and prefer to exit then.
///                         However, if the sensor number did at some point detect black but the exact match
///                         never happened and the sensor number is now no longer on black, the function will
///                         return true at that moment. Warning: this may cause undesirable overshoot for edge sensors.
///                         Useful for following corners. Default value is false
void waitUntilSensorDetectsLine(uint8_t sensorIndex, bool preferExactMatch = false);

#endif // TRACKINGALGOS_H
