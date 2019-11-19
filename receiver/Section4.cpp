/// Function defining the behavior of the robot when in section 4
/// \file Section4.cpp
/// \author Misha Krieger-Raynauld (1952515), Simon Gauvin (1951457), Vlad Drelciuc (1941366), Nicolas Charron (1960356)
/// \date 2019-03-22

#include "Section4.h"
#include "Debug.h"
#include "Motors.h"
#include "ExitConditions.h"
#include "Timer.h"
#include "TrackingAlgos.h"

namespace
{
    /// Play the sound to make when entering or leaving a rectangle
    void playTwoConsecutiveShortSounds()
    {
        lib::startPiezo(72);
        lib::msSleep(50);
        lib::stopPiezo();
        lib::msSleep(20);
        lib::startPiezo(72);
        lib::msSleep(50);
        lib::stopPiezo();
    }
} // namespace

void followSection4()
{
    static constexpr uint8_t speed = 120;

    // Same behavior for the three rectangles
    static constexpr uint8_t nbRectangles = 3;
    for (uint8_t i = 0; i < nbRectangles; i++)
    {
        DEBUG_PRINT("\tEntering rectangle\n");

        // Follow line before rectangle
        followLine(speed, 20, 70, 20, false, threeMiddleSensorsOnBlack);

        // Follow rectangle perpendicular edge
        followLine(speed, 20, 70, 20, false, threeMiddleSensorsOnWhite);

        // Follow inside rectangle, playing sounds on entry and exit
        playTwoConsecutiveShortSounds();
        followRectangle();
        playTwoConsecutiveShortSounds();

        // Follow rectangle perpendicular edge
        followLine(speed, 20, 70, 20, false, bothEdgeSensorsOnWhite);

        DEBUG_PRINT("\tLeaving rectangle\n");
    }

    // Follow line until corner
    followLine(speed, 20, 70, 20, false, allSensorsOnWhite);
}