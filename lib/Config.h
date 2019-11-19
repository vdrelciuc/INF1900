/// Configuration constants and connections
/// \file Config.h
/// \author Misha Krieger-Raynauld (1952515), Simon Gauvin (1951457), Vlad Drelciuc (1941366), Nicolas Charron (1960356)
/// \date 2019-02-20

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                              //
//                                               PORT CONNECTIONS                                               //
//                         (note: port numbers start at 0 everywhere except in this diagram)                    //
//                                                                                                              //
//                                                    PORT A                                                    //
//                                                  +--------+                                                  //
//                     Line tracker ADC (1 - blue)  |1      2|  Line tracker ADC (2 - yellow)                   //
//                   Line tracker ADC (3 - orange)  |3      4|  Line tracker ADC (4 - green)                    //
//                    Line tracker ADC (5 - white)  |5      6|                                                  //
//                                                  |7      8|                                                  //
//                                Line tracker GND  |GND  VCC|  Line tracker VCC                                //
//                                                  +--------+                                                  //
//                                                                                                              //
//                                                    PORT B                                                    //
//                                                  +--------+                                                  //
//                                 Onboard LED (+)  |1      2|  Onboard LED (-)                                 //
//                                  Piezo red wire  |3      4|  Piezo black wire (OCOA)                         //
//                  [Must be free for programming]  |5      6|  [Must be free for programming]                  //
//                  [Must be free for programming]  |7      8|  [Must be free for programming] / calibration    //
//                         GND for calibration wire |GND  VCC|  VCC for calibration wire                        //
//                                                  +--------+                                                  //
//                                                                                                              //
//                                                    PORT C                                                    //
//                                                  +--------+                                                  //
//           External memory I2C (no wire - MemEN)  |1      2|  External memory I2C (no wire - MemEN)           //
//                              Line tracker LED 1  |3      4|  Line tracker LED 2                              //
//                              Line tracker LED 3  |5      6|  Line tracker LED 4                              //
//                              Line tracker LED 5  |7      8|  IR photodetector data                           //
//                           Line tracker LEDs GND  |GND  VCC|  IR photodetector VCC                            //
//                                                  +--------+                                                  //
//                                                                                                              //
//                                                    PORT D                                                    //
//                                                  +--------+                                                  //
//               USART RX RS-232 (no wire - DbgEN)  |1      2|  USART TX RS-232 (no wire - DbgEN)               //
//              Interrupt button (no wire - IntEN)  |3      4|                                                  //
//                            Left motor direction  |5      6|  Right motor direction                           //
//            Left motor enable (OC2B) / IR TX (-)  |7      8|  Right motor enable (OC2A) / IR TX (+) (OC2A)    //
//                                                  |GND  VCC|                                                  //
//                                                  +--------+                                                  //
//                                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                              //
//                                                EEPROM CONTENTS                                               //
//                                                                                                              //
// Bytes 0-1:   16-bit unsigned integer for msRotate90ClockwiseDuration                                         //
// Bytes 2-3:   16-bit unsigned integer for msRotate90CounterclockwiseDuration                                  //
// Bytes 4-5:   16-bit unsigned integer for msRotateSlightlyClockwiseDuration                                   //
// Bytes 6-7:   16-bit unsigned integer for msRotateSlightlyCounterclockwiseDuration                            //
// Bytes 8-9:   16-bit unsigned integer for msSection1StartOffsetDuration                                       //
// Bytes 10-11: 16-bit unsigned integer for msSensorToCenterOfRotationDuration                                  //
// Bytes 12-13: 16-bit unsigned integer for msBetweenPointsDuration                                             //
//                                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

#define F_CPU 8'000'000

namespace lib
{
    /// Coefficient (7813) for generating a 1 second duration with startTimer() (1024 is the prescaler for the timer)
    constexpr uint16_t secTimerMultiplier = F_CPU / 1024;
    
    /// Onboard LED masks on B0 and B1
    enum class Led : uint8_t
    {
        Off = 0b0000,
        Red = 0b0001,
        Green = 0b0010,
        Mask = 0b0011
    };
    
    /// Piezo masks on B2 and B3
    enum class Piezo : uint8_t
    {
        Ground = 0b0100,
        Pwm = 0b1000,
        Mask = 0b1100
    };

    /// Calibration jumper mask on B7
    constexpr uint8_t calibrationMask = 0b1000'0000;

    /// Line tracker LED masks on C2-C3-C4-C5-C6
    enum class TrackerLed : uint8_t
    {
        First = 0b0000'0100,
        Second = 0b0000'1000,
        Third = 0b0001'0000,
        Fourth = 0b0010'0000,
        Fifth = 0b0100'0000,
        Mask = 0b0111'1100
    };

    /// Infrared receiver mask on C7
    constexpr uint8_t infraredRxMask = 0b1000'0000;

    /// Interrupt button mask on D2
    constexpr uint8_t buttonMask = 0b0100;
    
    /// Motor masks on D4-D5-D6-D7
    enum class Motor : uint8_t
    {
        LeftMotorEnable = 0b0100'0000,
        LeftMotorDirection = 0b0001'0000,
        RightMotorEnable = 0b01000'0000,
        RightMotorDirection = 0b0010'0000,
        Mask = 0b1111'0000
    };

    /// Infrared transmitter masks on D6 and D7 (note: cannot be used with motors)
    enum class InfraredTx : uint8_t
    {
        Ground = 0b0100'0000,
        Pwm = 0b1000'0000,
        Mask = 0b1100'0000
    };
} // namespace lib

#endif // CONFIG_H