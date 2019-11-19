/// Functions for USART transmission
/// \file Usart.cpp
/// \author Misha Krieger-Raynauld (1952515), Simon Gauvin (1951457), Vlad Drelciuc (1941366), Nicolas Charron (1960356)
/// \date 2019-02-20

#include "Usart.h"
#include <avr/io.h>
#include <stdlib.h>
#include <string.h>

namespace lib
{
    void initializeUsart()
    {        
        // USART baud rate register flags
        UBRR0H = 0;
        UBRR0L = 0xCF; // Set to 2400 bauds (207 with U2X0 set to 0 (default))
        
        // USART control and status register flags
        UCSR0A = 0;
        UCSR0B |= (1 << RXEN0) | (1 << TXEN0); // Enable receiver and transmitter for USART0 by setting RXEN0 and TXEN0 bits to 1
        UCSR0C &= ~(1 << UPM01) & ~(1 << UPM00); // Set parity mode to disabled by setting UPM0 bits to 00
        UCSR0C &= ~(1 << USBS0); // Set stop bit select mode to 1-bit by setting USBS0 bit to 0
        UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00); // Set character size to 8 bits by setting UCSZ0 bits to 11
    }
    
    void usartTransmit(uint8_t data)
    {
        // Wait for empty transmit buffer in USART0
        while ((UCSR0A & (1 << UDRE0)) == false)
        {
        }
        
        // Put data into buffer, send the data
        UDR0 = data;
    }
    
    void usartTransmit(const char* string)
    {       
        for (uint16_t i = 0, stringLength = strlen(string); i < stringLength; i++)
        {
            usartTransmit(string[i]);
        }
    }

    void usartTransmitNumber(int32_t number)
    {
        static constexpr uint8_t maxStringLength = 18; // 16 bits + 1 + '\0' (see ltoa doc)
        char str[maxStringLength];
        ltoa(number, str, 10); // Convert long to string
        uint8_t length = strlen(str);
        for (uint8_t i = 0; i < length; i++)
        {
            usartTransmit(str[i]);
        }
    }

    uint8_t usartReceive()
    {
        // Wait for data to be received
        while ((UCSR0A & (1 << RXC0)) == false)
        {
        }

        // Get and return received data from buffer
        return UDR0;
    }
} // namespace lib