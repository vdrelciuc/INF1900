/// Functions for infrared date transfer using the SIRC protocol (https://www.sbprojects.net/knowledge/ir/sirc.php)
/// \file Infrared.cpp
/// \author Misha Krieger-Raynauld (1952515), Simon Gauvin (1951457), Vlad Drelciuc (1941366), Nicolas Charron (1960356)
/// \date 2019-03-22

#include "Infrared.h"
#include "Config.h"
#include "Debug.h"
#include "GeneralIo.h"
#include "Timer.h"

namespace
{
    using namespace lib;

    constexpr uint16_t usShortSleep = 564; // 600 us - estimated execution delay
    constexpr uint16_t usLongSleep = 1128; // 1200 us - estimated execution delay
    constexpr uint16_t usHeaderSleep = 2256; // 2400 us - estimated execution delay

    /// Start flashing the infrared LED at 38 kHz
    void startInfraredTxPwm()
    {
        TCCR2A |= (1 << COM2A0); // Set compare output mode to toggle OC2A on compare match by setting COM2A bits to 01
    }

    /// Stop flashing the infrared transmitter
    void stopInfraredTxPwm()
    {
        TCCR2A &= ~(1 << COM2A0); // Set compare output mode to normal (do not toggle OC2A on compare match) by setting COM2A bits to 00
    }

    /// Send the header for SIRC protocol
    void sendInfraredTxHeader()
    {
        startInfraredTxPwm();
        usSleep(usHeaderSleep);
        stopInfraredTxPwm();
        usSleep(usShortSleep);
    }

    /// Send bit using infrared transmitter. If bit is a '1', pulse 38 kHz for 1.2 ms.
    /// If bit is a '0', pulse 38 kHz for 0.6 ms. In all cases, pause for 0.6 ms at the end
    /// \param bit Bit to send, either 0 (false) or 1 (true)
    void sendInfraredBit(bool bit)
    {
        startInfraredTxPwm();
        if (bit == 1)
        {
            usSleep(usLongSleep);
        }
        else
        {
            usSleep(usShortSleep);
        }
        stopInfraredTxPwm();
        usSleep(usShortSleep);
    }
} // namespace

namespace lib
{
    void initializeInfraredTx()
    {
        // Set infrared transmitter ground pin to 0
        PORTD &= ~static_cast<uint8_t>(InfraredTx::Ground);

        // Timer counter control register flags (2x8 bits)
        TCCR2A |= (1 << WGM21); // Set wave generation mode to CTC 8-bit (OCR2A TOP value) by setting WGM2 bits to 010
        TCCR2B |= (1 << CS20); // Set clock select to clock divided by 1 by setting CS2 bits to 001

        // Value for OCR2A for SIRC protocol (F_CPU / N / frequency /  2 - 1) (Atmel documentation p.96)
        static constexpr uint16_t sircFrequency = 38000; // 38 kHz
        static constexpr uint8_t ocr2aCompareValue = F_CPU / sircFrequency / 2 - 1;
        OCR2A = ocr2aCompareValue;
        TCNT2 = 0; // Set to 0 to avoid missing compare match
    }

    void sendInfraredCommand(uint8_t command, uint8_t address)
    {
        static constexpr uint8_t nbBitsInCommand = 7;
        static constexpr uint8_t nbBitsInAddress = 5;

        // Send three times for redundancy, with 45 ms in between each call
        for (uint8_t i = 0; i < 3; i++)
        {
            // 2.4 ms pulse + 0.6 ms off header
            sendInfraredTxHeader();

            // Send command, starting with LSB
            for (uint8_t i = 0; i < nbBitsInCommand; i++)
            {
                sendInfraredBit((command >> i) & 0b0001);
            }

            // Send address, starting with LSB
            for (uint8_t i = 0; i < nbBitsInAddress; i++)
            {
                sendInfraredBit((address >> i) & 0b0001);
            }

            msSleep(45);
        }
    }

    uint16_t receiveInfraredCommand()
    {
        // Enable button interrupts
        enableButtonInterrupts();

        static constexpr uint8_t nbBitsPerCommand = 12;
        static constexpr uint16_t nbHighTicksInHeaderPulse = 800; // Determined experimentally
        static constexpr uint16_t nbHighTicksInHighPulse = 450; // Determined experimentally

        // Debug mode logging
        #ifdef DEBUG
            uint16_t log[nbBitsPerCommand + 1] = {};
            uint16_t pulseCounter = 0; // For counting the number of pulses to check if the environment is emitting IR
        #endif
        
        bool wasSircHeaderDetected = false;
        bool currentIsHigh = false;
        bool previousIsHigh = false;
        uint16_t returnData = 0;
        uint8_t bitCounter = 0;
        uint16_t pulseLengthCounter = 0;
        while (true)
        {
            // Fall back to onboard interrupt button if button press was detected
            if (wasButtonPressed)
            {
                returnData = getButtonPressCount();
                break; // Exit loop as command was received through fallback button mode
            }

            // Read from IR photodetector while there are still bits to read
            if (bitCounter < nbBitsPerCommand)
            {
                previousIsHigh = currentIsHigh;
                currentIsHigh = !(PINC & infraredRxMask);
                
                // Increment counter if signal is high
                if (currentIsHigh)
                {
                    pulseLengthCounter++;
                }
                // Process bit on falling edge
                else if (currentIsHigh == false && previousIsHigh == true)
                {
                    #ifdef DEBUG
                        pulseCounter++;
                    #endif

                    // Check for header and start/restart if detected (allows for a second header to be sent to restart the command data)
                    if (pulseLengthCounter >= nbHighTicksInHeaderPulse) // If in header high pulse tick count range
                    {
                        wasSircHeaderDetected = true;
                        #ifdef DEBUG
                            log[0] = pulseLengthCounter;
                        #endif

                        // Reset variables if second header to reset
                        returnData = 0;
                        bitCounter = 0;
                    }
                    // Get bits if header had already been detected
                    else if (wasSircHeaderDetected)
                    {
                        if (pulseLengthCounter >= nbHighTicksInHighPulse) // High bit
                        {
                            returnData |= (1 << bitCounter);
                        }
                        else // Low bit
                        {
                            returnData &= ~(1 << bitCounter);
                        }
                        #ifdef DEBUG
                            // +1 to account for SIRC header
                            log[bitCounter + 1] = pulseLengthCounter;
                        #endif
                        bitCounter++;
                    }

                    pulseLengthCounter = 0; // Reset pulse length counter
                }
            }
            // Return value being read if the proper number of bits have been read
            else
            {
                // Print debug mode logging (includes header)
                #ifdef DEBUG
                    DEBUG_PRINT("IR COMMAND LOG:\n");
                    for (uint8_t i = 0 ; i < nbBitsPerCommand + 1; i++)
                    {
                        DEBUG_PRINT('\t');
                        if (i < 10)
                        {
                            DEBUG_PRINT(' '); // For digit-independent even spacing
                        }
                        DEBUG_PRINT_NUMBER(i);
                        DEBUG_PRINT(": ");
                        if (log[i] > nbHighTicksInHeaderPulse)
                        {
                            DEBUG_PRINT("\u001b[35m"); // Colored output for values that represent header bit
                        }
                        else if (log[i] > nbHighTicksInHighPulse)
                        {
                            DEBUG_PRINT("\u001b[36m"); // Colored output for values that represent high bit
                        }
                        DEBUG_PRINT_NUMBER(log[i]);
                        DEBUG_PRINT("\u001b[0m");
                        DEBUG_PRINT('\n');
                    }
                #endif

                break; // Exit loop after a successful IR command reception
            }
        }

        // Output received command
        #ifdef DEBUG // In ifdef to avoid unused variable warning when compiling in release mode
            DEBUG_PRINT("DATA RECEIVED: Command: ");
            DEBUG_PRINT_NUMBER(returnData & 0b0111'1111); // Extract 7 LSBs (command) from data

            DEBUG_PRINT(" | Address: ");
            static constexpr uint8_t nbCommandBits = 7;
            DEBUG_PRINT_NUMBER(returnData >> nbCommandBits); // Extract MSB (address) from data
            DEBUG_PRINT('\n');

            // Warning for extra IR signals detected
            if (pulseCounter > nbBitsPerCommand + 1)
            {
                DEBUG_PRINT("WARNING: DETECTED TOO MANY PULSES: ");
                DEBUG_PRINT_NUMBER(pulseCounter);
                DEBUG_PRINT('\n');
            }            
        #endif

        // Disable button interrupts and return the received IR command
        disableButtonInterrupts();

        return returnData;
    }

    uint8_t receiveInfraredCommandDigit()
    {
        uint16_t data = receiveInfraredCommand();
        uint8_t digit = data & 0b0111'1111; // Extract 7 LSBs (command) from data
        return digit;
    }
} // namespace lib
