/// Get error between desired setpoint for following a line and the given line tracker sensor readings
/// \param lineTrackerValues 8-bit unsigned int whose bit represents the line tracker sensor readings
/// \return The error between the center and the current position for use with a PID controller.
///         Positive values mean the robot is off course towards the left, negative towards the right.
///         If the sensor values combination is unknown, -128 is returned
int8_t getLineError(uint8_t lineTrackerValues)
{
    // Sensor values | Error
    // 0 0 0 0 1     |  4
    // 0 0 0 1 1     |  3
    // 0 0 1 1 1     |  3
    // 0 0 0 1 0     |  2
    // 0 0 1 1 0     |  1
    // 0 0 1 0 1     |  1
    // 0 0 1 0 0     |  0
    // 0 1 1 1 0     |  0
    // 1 1 1 1 1     |  0
    // 0 1 1 1 1     |  0
    // 1 1 1 1 0     |  0
    // 1 1 1 0 1     |  0
    // 1 0 1 1 1     |  0
    // 0 1 1 0 0     | -1
    // 1 0 1 0 0     | -1
    // 0 1 0 0 0     | -2
    // 1 1 1 0 0     | -3
    // 1 1 0 0 0     | -3
    // 1 0 0 0 0     | -4

    switch (lineTrackerValues)
    {
    case 0b00001:
        return 4;
        break;
    case 0b00011:
    case 0b00111:
        return 3;
        break;
    case 0b00010:
        return 2;
        break;
    case 0b00110:
    case 0b00101:
        return 1;
        break;
    case 0b00100:
    case 0b01110:
    case 0b11111:
    case 0b01111:
    case 0b11110:
    case 0b11101:
    case 0b10111:
        return 0;
        break;
    case 0b01100:
    case 0b10100:
        return -1;
        break;
    case 0b01000:
        return -2;
        break;
    case 0b11100:
    case 0b11000:
        return -3;
        break;
    case 0b10000:
        return -4;
        break;
    default: // If no pattern was recognized, return -128 as a flag
        return -128;
        break;
    }
}

/// Get error between desired setpoint for following a rectangle and the given line tracker sensor readings
/// \param lineTrackerValues 8-bit unsigned int whose bit represents the line tracker sensor readings
/// \return The error between the rectangle alignment and the current position for use with a PID controller.
///         Positive values mean the robot is off course towards the left, negative towards the right.
///         If the sensor values combination is unknown, -128 is returned
int8_t getRectangleError(uint8_t lineTrackerValues)
{
    // Sensor values | Error
    // 0 1 0 0 0     |  3
    // 1 1 0 0 0     |  2
    // 1 0 0 0 0     |  1
    // 0 0 0 0 0     |  0
    // 0 0 1 0 0     |  0
    // 0 1 1 1 0     |  0
    // 1 1 1 1 0     |  0
    // 0 1 1 1 1     |  0
    // 1 1 1 1 1     |  0
    // 0 0 0 0 1     | -1
    // 0 0 0 1 1     | -2
    // 0 0 0 1 0     | -3

    switch (lineTrackerValues)
    {
    case 0b01000:
        return 3;
        break;
    case 0b11000:
        return 2;
        break;
    case 0b10000:
        return 1;
        break;
    case 0b00000:
    case 0b00100:
    case 0b01110:
    case 0b11110:
    case 0b01111:
    case 0b11111:
        return 0;
        break;
    case 0b00001:
        return -1;
        break;
    case 0b00011:
        return -2;
        break;
    case 0b00010:
        return -3;
        break;
    default: // If no pattern was recognized, return -128 as a flag
        return -128;
        break;
    }
}

// Follow with PID by providing an error calculation function and an exit condition function
void pidFollow(int8_t (*errorCalculationFunction)(uint8_t lineTrackerValues),
                bool (*exitConditionFunction)())
{
    // PID gain constants to tweak (Kp first, to make the robot follow the line, and then Kd for smoothing)
    // See first answer: https://robotics.stackexchange.com/questions/167/what-are-good-strategies-for-tuning-pid-loops
    // 90 240
    static constexpr int16_t Kp = 40; // Proportional gain constant (magnitude of change - should contribute bulk of the output change)
    static constexpr int16_t Ki = 0; // Integral gain constant (rate of change - removes residual steady-state error from proportional change).
                                        // Unused for this particular application, as the range of values swings wildly and we
                                        // do not need steady-state error correction for long-term precision for our use case
    static constexpr int16_t Kd = 120; // Derivative gain constant (stability of the system - slow down of rate of change and prevent overshoot)
    
    static constexpr uint16_t msSleepDuration = 20;

    static constexpr int16_t baseSpeed = 150;
    static constexpr int16_t curveSlowDownValue = 60;
    
    static constexpr uint8_t nbSlowDownCyclesOnError = 50;
    uint8_t slowDownCounter = 0;
    
    // Persistent PID variables
    int8_t previousError = 0;
    int16_t I = 0; // Integral term

    // Timer for logger
    static constexpr uint16_t msPerLog = 100;
    uint16_t logTimer = 0;

    // Reset logs
    robotLogs.clear();

    while (true)
    {
        // Read line tracker values
        uint8_t lineTrackerValues = lib::readLineTrackerValues();
        /*for (uint8_t i = 3; i < 8; i++)
        {
            DEBUG_PRINT(((lineTrackerValues << i) & 0x80) ? '1' : '0');
        }
        DEBUG_PRINT(" | ");*/
                
        // Exit function if the exitCondition function evaluates to true
        if (exitConditionFunction())
        {
            DEBUG_PRINT("EXIT CONDITION\n");
            return;
        }

        // Error
        int8_t error = errorCalculationFunction(lineTrackerValues);
        if (error == -128)
        {
            DEBUG_PRINT("FUCK\n");
            //lib::playShortPiezoSound();
            error = 0;
        }

        // Calculate PID
        int16_t P = error; // Proportional term
        I += error; // Add to integral term
        int16_t D = error - previousError; // Derivative term (change in error)
        int16_t speedDifference = Kp * P + Ki * I + Kd * D; // PID control variable (sum of all terms multiplied by their gain)
        previousError = error; // Set previous error to current error for next loop

        // Print PID values
        /*DEBUG_PRINT("E: ");
        DEBUG_PRINT_NUMBER(error);
        DEBUG_PRINT(" | P: ");
        DEBUG_PRINT_NUMBER(P);
        DEBUG_PRINT(" | I: ");
        DEBUG_PRINT_NUMBER(I);
        DEBUG_PRINT('\n');
        DEBUG_PRINT(" | D: ");
        DEBUG_PRINT_NUMBER(D);
        DEBUG_PRINT(" | Value: ");
        DEBUG_PRINT_NUMBER(speedDifference);*/
        
        // Adjust motor speed ratio between the two wheels
        int16_t leftMotorSpeed = baseSpeed + speedDifference;
        int16_t rightMotorSpeed = baseSpeed - speedDifference;

        // Calculate average error
        int16_t errorSum = 0;
        int32_t pidSum = 0;
        static constexpr uint8_t nbLogsToAnalyze = 40;
        for (const auto& log : robotLogs)
        {
            errorSum += log.error * log.error;
            pidSum += log.pidSpeedDifference;
        }

        DEBUG_PRINT_NUMBER(errorSum);
        DEBUG_PRINT(' ');
        DEBUG_PRINT_NUMBER(pidSum);
        DEBUG_PRINT('\n');

        // Reduce speed when correcting for error (for slowdown in curves)
        // by restarting a counter for which to slow down
        static constexpr uint8_t magicValue = 35;
        if (errorSum > magicValue || errorSum < -magicValue)
        {
            slowDownCounter = nbSlowDownCyclesOnError;
        }
        
        // If the slow down counter is still counting down, slow down the robot
        if (slowDownCounter > 0)
        {
            // Bring left motor speed to zero
            if (leftMotorSpeed >= 0)
            {
                if (leftMotorSpeed - curveSlowDownValue < 0)
                {
                    leftMotorSpeed = 0;
                }
                else
                {
                    leftMotorSpeed -= curveSlowDownValue;
                }
            }
            else
            {
                if (leftMotorSpeed + curveSlowDownValue > 0)
                {
                    leftMotorSpeed = 0;
                }
                else
                {
                    leftMotorSpeed += curveSlowDownValue;   
                }
            }

            // Bring right motor speed to zero
            if (rightMotorSpeed >= 0)
            {
                if (rightMotorSpeed - rightMotorSpeed < 0)
                {
                    rightMotorSpeed = 0;
                }
                else
                {
                    rightMotorSpeed -= curveSlowDownValue;
                }
                
            }
            else
            {
                if (rightMotorSpeed + rightMotorSpeed > 0)
                {
                    rightMotorSpeed = 0;
                }
                else
                {
                    rightMotorSpeed += curveSlowDownValue;
                }
            }

            // Decrement slow down counter
            slowDownCounter--;

            lib::startPiezo(45);
        }
        else
        {
            lib::stopPiezo();
        }       

        // Set motor speed to calculated values
        lib::setMotorSpeed(leftMotorSpeed, rightMotorSpeed);

        // Print motor speeds
        /*DEBUG_PRINT(" | ");
        DEBUG_PRINT_NUMBER(leftMotorSpeed);
        DEBUG_PRINT(' ');
        DEBUG_PRINT_NUMBER(rightMotorSpeed);
        DEBUG_PRINT('\n');*/

        robotLogs.add({
            //lineTrackerValues,
            error,
            //P,
            //D,
            speedDifference
        });
        // Log current loop actions
        if (logTimer >= msPerLog)
        {
            logTimer -= msPerLog;
        }

        // Delay between loops
        lib::msSleep(msSleepDuration);

        // Increment log timer
        logTimer += msSleepDuration;
    }
    // Useful source: https://www.instructables.com/id/Line-Follower-Robot-PID-Control-Android-Setup/
}

void pidFollowLine(bool (*exitConditionFunction)())
{
    pidFollow(getLineError, exitConditionFunction);
}

void pidFollowRectangle()
{
    pidFollow(getRectangleError, bothEdgeSensorsOnBlack);
}