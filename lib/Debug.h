/// Macros for USART logging when in debug mode
/// \file Debug.h
/// \author Misha Krieger-Raynauld (1952515), Simon Gauvin (1951457), Vlad Drelciuc (1941366), Nicolas Charron (1960356)
/// \date 2019-02-20

#ifndef DEBUG_H
#define DEBUG_H

#ifdef DEBUG
    #include "Usart.h"
    
    /// Defines DEBUG_PRINT(a) macro to be replaced by usartTransmit(a)
    /// Note: requires initializeUsart() to have been called beforehand
    #define DEBUG_PRINT(a) lib::usartTransmit(a)

    /// Defines DEBUG_PRINT_NUMBER(a) macro to be replaced by usartTransmitNumber(a)
    /// Note: requires initializeUsart() to have been called beforehand
    #define DEBUG_PRINT_NUMBER(a) lib::usartTransmitNumber(a)
#else
    /// Defines DEBUG_PRINT(a) macro to be replaced by a no-op, which will be elided by the compiler
    #define DEBUG_PRINT(a) ((void)0) // no-op
    #define DEBUG_PRINT_NUMBER(a) ((void)0) // no-op
#endif

#endif // DEBUG_H
