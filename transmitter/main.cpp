/// Program for the infrared transmitter robot which sends data using the SIRC protocol based on button presses
/// \file transmitter/main.cpp
/// \author Misha Krieger-Raynauld (1952515), Simon Gauvin (1951457), Vlad Drelciuc (1941366), Nicolas Charron (1960356)
/// \date 2019-03-22

#include "Debug.h"
#include "GeneralIo.h"
#include "Infrared.h"
#include "Timer.h"
#include "Usart.h"

int main()
{
    // Initialize
    lib::initializePins();
    lib::initializePiezo();
    lib::initializeTimer();
    lib::initializeUsart();
    lib::initializeInfraredTx();
    lib::startupSequence();

    DEBUG_PRINT("STARTED TRANSMITTER\n");

    while (true)
    {
        DEBUG_PRINT("WAITING FOR BUTTON PRESSES\n");
        uint8_t pressCount = lib::getButtonPressCount();
        
        static constexpr uint8_t tvDeviceAddress = 1;
        lib::sendInfraredCommand(pressCount, tvDeviceAddress);
        DEBUG_PRINT("DATA SENT: Command: ");
        DEBUG_PRINT_NUMBER(pressCount);
        DEBUG_PRINT(" | Address: ");
        DEBUG_PRINT_NUMBER(tvDeviceAddress);
        DEBUG_PRINT('\n');
    }
}
