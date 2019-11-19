/// Functions for infrared date transfer using the SIRC protocol
/// \file Infrared.h
/// \author Misha Krieger-Raynauld (1952515), Simon Gauvin (1951457), Vlad Drelciuc (1941366), Nicolas Charron (1960356)
/// \date 2019-03-22

#ifndef INFRARED_H
#define INFRARED_H

#include <avr/io.h>

namespace lib
{   
    /// Set TCNT2 settings for use as a wave generation source for infrared transmission
    void initializeInfraredTx();

    /// Send command via infrared with 12-bit SIRC protocol
    /// \param command  The command to send via infrared
    /// \param address  The address of the command
    void sendInfraredCommand(uint8_t command, uint8_t address);

    /// Use the infrared receiver to receive data from the transmitter robot,
    /// or fall back to the button in case of failure (address will be 0 in that case).
    /// Blocks until data is received, either through the infrared photodetector or the interrupt button
    /// Note: in debug mode, requires initializeUsart() to have been called beforehand
    /// \return The digit number sent through the SIRC protocol or the number of times the button was pressed instead
    uint16_t receiveInfraredCommand();

    /// Extract command digit from IR or button command.
    /// Uses receiveInfraredCommand() to obtain the command, either through IR reception or the fallback button presses
    /// Note: in debug mode, requires initializeUsart() to have been called beforehand
    /// \return The digit of the received command
    uint8_t receiveInfraredCommandDigit();
} // namespace lib

#endif // INFRARED_H
