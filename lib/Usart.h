/// Functions for USART transmission
/// \file Usart.h
/// \author Misha Krieger-Raynauld (1952515), Simon Gauvin (1951457), Vlad Drelciuc (1941366), Nicolas Charron (1960356)
/// \date 2019-02-20

#ifndef USART_H
#define USART_H

#include <stdint.h>

namespace lib
{
    /// Initialize USART0 for RS232 asynchronous communication
    void initializeUsart();
    
    /// Transmit data through USART. Refer to Atmel documentation, p.174
    /// \param data 8-bit data to transmit through USART
    void usartTransmit(uint8_t data);
    
    /// Transmit string data through USART
    /// \param string C-style string data to transmit through USART
    void usartTransmit(const char* string);

    /// Transmit number converted to string through USART
    /// \param number 32-bit signed int to convert to C-style string to transmit through USART
    void usartTransmitNumber(int32_t number);

    /// Receive data through USART. Refer to Atmel documentation, p.177
    /// \return 8-bit data received through USART
    uint8_t usartReceive();
} // namespace lib

#endif // USART_H