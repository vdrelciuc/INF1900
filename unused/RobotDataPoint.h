#ifndef ROBOTDATAPOINT_H
#define ROBOTDATAPOINT_H

/// Struct to contain information to store in the logger for the tracking algorithms
struct RobotDataPoint
{
    //int16_t leftMotorSpeed;
    //int16_t rightMotorSpeed;
    //uint8_t lineTrackerState;
    //uint8_t lineTrackerValues : 5;
    int8_t error;
    //int16_t propTerm;
    //int16_t derivTerm;
    int16_t pidSpeedDifference;
};

#endif // ROBOTDATAPOINT_H