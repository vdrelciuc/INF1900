/// Analyze logs to determine whether the robot is in a curve and return true in that case
bool isTurning()
{
    int16_t speedDifference = 0;
    for (auto end = robotLogs.end(), it = end - 20; it != end; ++it)
    {
        speedDifference += it->leftMotorSpeed - it->rightMotorSpeed;
    }
    DEBUG_PRINT("Speed difference: ");
    DEBUG_PRINT_NUMBER(speedDifference);
    DEBUG_PRINT('\n');

    return speedDifference > 30;
}