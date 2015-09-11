/*This file has been prepared for Doxygen automatic documentation generation.*/
//! \file *********************************************************************
//!
//! \brief This file contains the function declarations
//!
//! - Compiler:           IAR EWAVR and GNU GCC for AVR
//! - Supported devices:  ATTiny861,ATTiny461,ATTiny261
//!
//! \author               Atmel Corporation: http://www.atmel.com \n
//!                       Support and FAQ: http://support.atmel.no/
//!
//! ***************************************************************************

/* Copyright (c) 2007, Atmel Corporation All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of ATMEL may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY AND
 * SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
#ifndef __TINYX61_MACROS_H__
#define __TINYX61_MACROS_H__

//_____  I N C L U D E S ___________________________________________________

//! @defgroup global_config Application configuration
//! @{
#include "compiler.h"                    // Compiler definitions
#ifdef __GNUC__
   #include <avr/io.h>                    // Use AVR-GCC library
   #include <avr/power.h>
#elif __ICCAVR__
   #include <ioavr.h>                     // Use IAR-AVR library
#else
   #error Current COMPILER not supported
#endif
//! @}

//_____ M A C R O S ________________________________________________________

#if defined(__ICCAVR__)
//! Save the current interrupt state.
#define SAVE_INTERRUPT()            __save_interrupt();

//! Restore the interrupt state.
#define RESTORE_INTERRUPT(state)    __restore_interrupt(state);

//! Disable global interrupt flag.
#define DISABLE_INTERRUPT()         __disable_interrupt();

#elif defined(__GNUC__)
//! Save the current interrupt state.
#define SAVE_INTERRUPT()            SREG

//! Restore the interrupt state.
#define RESTORE_INTERRUPT(state)    (SREG = (state))

//! Disable global interrupt flag.
#define DISABLE_INTERRUPT()         cli()

#else
//! Save the current interrupt state.
#define SAVE_INTERRUPT()            SREG

//! Restore the interrupt state.
#define RESTORE_INTERRUPT(state)    (SREG = (state))

//! Disable global interrupt flag.
#define DISABLE_INTERRUPT()         (SREG &= (~GLOBAL_INTERRUPT_BIT_MASK))
#endif


/*! \brief Write 16 bit value to TCNT0.
 *
 *  Writes a 16 bit value to TCNT0 (TCNT0L/H).
 *
 *  \param value New 16 bit TCNT0 value.
 */
#define TC0_WRITE_TCNT0(value) \
  { \
    TCNT0H = (U8)((value) >> 8); \
    TCNT0L = (U8)(value); \
  }


/*! \brief Read 16 bit value from TCNT0.
 *
 *  Reads the 16 bit value of TCNT0 (TCNT0L/H).
 *
 *  \param destinationVariable Destination variable.
 */
#define TC0_READ_TCNT0(destinationVariable) \
  { \
    U8 tempL; \
    tempL = TCNT0L; \
    (destinationVariable) = ((TCNT0H << 8) | tempL); \
  }


/*! \brief Atomic 16 bit write to TCNT0.
 *
 *  Writes a 16 bit value to TCNT0 (TCNT0L/H) in one atomic operation.
 *
 *  \param value New 16 bit TCNT0 value.
 */
#define TC0_WRITE_TCNT0_INT_SAFE(value) \
  { \
    U8 iFlagTemp; \
    iFlagTemp = SAVE_INTERRUPT(); \
    DISABLE_INTERRUPT(); \
    TCNT0H = (U8)((value) >> 8); \
    TCNT0L = (U8)(value); \
    RESTORE_INTERRUPT(iFlagTemp); \
  }


/*! \brief Atomic 16 bit read from TCNT0.
 *
 *  Reads the 16 bit value of TCNT0 (TCNT0L/H) in one atomic operation.
 *
 *  \param destinationVariable Destination variable.
 */
#define TC0_READ_TCNT0_INT_SAFE(destinationVariable) \
  { \
    U8 iFlagTemp; \
    U8 tempL; \
    iFlagTemp = SAVE_INTERRUPT(); \
    DISABLE_INTERRUPT(); \
    tempL = TCNT0L; \
    (destinationVariable) = ((TCNT0H << 8) | tempL); \
    RESTORE_INTERRUPT(iFlagTemp); \
  }


/*! \brief Write 16 bit value to OCR0A/B.
 *
 *  Writes a 16 bit value to OCR0A/B.
 *
 *  \param value New 16 bit OCR0A/B value.
 */
#define TC0_WRITE_16_BIT_OCR0AB(value) \
  { \
    OCR0B = (U8)((value) >> 8); \
    OCR0A = (U8)(value);\
  }


/*! \brief Read 16 bit value from OCR0A/B.
 *
 *  Reads the 16 bit OCR0A/B value.
 *
 *  \param destinationVariable Destination variable.
 */
#define TC0_READ_16_BIT_OCR0AB(destinationVariable) \
  { \
    U8 tempL = OCR0A; \
    (destinationVariable) = ((U16)OCR0B << 8) | tempL; \
  }


/*! \brief Atomic 16 bit read from OCRA/B.
 *
 *  Reads the 16 bit OCR0A/B value in one atomic operation.
 *
 *  \param destinationVariable Destination variable.
 */
#define TC0_READ_16_BIT_OCR0AB_INT_SAFE(destinationVariable) \
  { \
    U8 iFlagTemp; \
    iFlagTemp = SAVE_INTERRUPT(); \
    DISABLE_INTERRUPT(); \
    U8 tempL = OCR0A; \
    (destinationVariable) = ((U16)OCR0B << 8) | tempL; \
    RESTORE_INTERRUPT(iFlagTemp); \
  }


/*! \brief Write 16 bit value to OCR0A/B.
 *
 *  Writes a 16 bit value to OCR0A/B.
 *
 *  \param value New 16 bit OCR0A/B value.
 */
#define TC0_WRITE_16_BIT_OCR0AB_INT_SAFE(value) \
  { \
    U8 iFlagTemp; \
    iFlagTemp = SAVE_INTERRUPT(); \
    DISABLE_INTERRUPT(); \
    OCR0B = (U8)((value) >> 8); \
    OCR0A = (U8)(value);\
    RESTORE_INTERRUPT(iFlagTemp); \
  }


/*! \brief Write 10 bit value to a Timer/Counter1 register.
 *
 *  Writes a 10 bit value to any 10 bit Timer/Counter1 register.
 *
 *  \param destinationRegister Destination register.
 *  \param value Register value.
 */
#define TC1_WRITE_10_BIT_REGISTER(destinationRegister, value) \
  { \
    TC1H = ((value) >> 8); \
    (destinationRegister) = (U8)(value); \
  }


/*! \brief Read 10 bit value from a Timer/Counter1 register.
 *
 *  Reads a 10 bit value from any 10 bit Timer/counter1 register.
 *
 *  \param sourceRegister Source register
 *  \param destinationVariable Destination variable.
 */
#define TC1_READ_10_BIT_REGISTER(sourceRegister, destinationVariable) \
  { \
    U8 tempL; \
    tempL = (sourceRegister); \
    (destinationVariable) = ( ((U16)TC1H << 8) | tempL); \
  }


/*! \brief Atomic 10 bit write to a Timer/Counter1 register.
 *
 *  Writes a 10 bit value to any 10 bit Timer/Counter1 register in one atomic.
 *  operation.
 *
 *  \param destinationRegister Destination register.
 *  \param value Register value.
 */
#define TC1_WRITE_10_BIT_REGISTER_INT_SAFE(destinationRegister, value) \
  { \
  U8 iFlagTemp; \
  iFlagTemp = SAVE_INTERRUPT(); \
  DISABLE_INTERRUPT(); \
  TC1H = ((value) >> 8); \
  (destinationRegister) = (U8)(value); \
  RESTORE_INTERRUPT(iFlagTemp); \
  }


/*! \brief Atomic 10 bit read from a Timer/Counter1 register.
 *
 *  Reads a 10 bit value from any 10 bit Timer/counter1 register in one atomic
 *  operation.
 *
 *  \param sourceRegister Source register
 *  \param destinationVariable Destination variable
 */
#define TC1_READ_10_BIT_REGISTER_INT_SAFE(sourceRegister, destinationVariable) \
  { \
    U8 iFlagTemp; \
    U8 tempL; \
    iFlagTemp = SAVE_INTERRUPT(); \
    tempL = (sourceRegister); \
    (destinationVariable) = ( ((U16)TC1H << 8) | tempL); \
    RESTORE_INTERRUPT(iFlagTemp); \
  }


/*! \brief Set same output compare value for all output channels of Timer/counter1.
 *
 *  This macro sets the same output compare value to all three output channels
 *  of Timer/counter1. This does not apply to PWM6 mode, where all channels
 *  controlled by a single compare register.
 *
 *  \param compareValue New output compare value.
 */
#define TC1_SET_ALL_COMPARE_VALUES(compareValue) \
  { \
      U16 tempValue = compareValue; \
      TC1H = ((U8)((tempValue) >> 8)); \
      OCR1A = ((U8)tempValue); \
      OCR1B = ((U8)tempValue); \
      OCR1D = ((U8)tempValue); \
  }

/*! \brief Clear prescaler.
 *
 *  This function reset the internal CPU core clock prescaler
 *
 *  \param none
 */
#ifdef  __GNUC__
   #define Clear_prescaler()                       (clock_prescale_set(0))
#else
   #define Clear_prescaler()                       (Set_cpu_prescaler(0))
#endif

/*! \brief Set prescaler.
 *
 *  This function set the internal CPU core clock prescaler
 *
 *  \param x Prescaler Value.
 */
#ifdef  __GNUC__
   #define Set_cpu_prescaler(x)                        (clock_prescale_set(x))
#else
   extern void Set_cpu_prescaler(U8 x);
#endif

//_____ D E F I N I T I O N S ______________________________________________

//! Bit mask for the global interrupt flag in SREG.
#define GLOBAL_INTERRUPT_BIT_MASK     0x80

//! Macro that clears all Timer/counter0 interrupt flags.
#define CLEAR_ALL_TIMER1_INT_FLAGS    (TIFR = TIFR | 0xE4)

//! Macro that disables all Timer/Counter1 interrupts.
#define DISABLE_ALL_TIMER1_INTS       (TIMSK &= 0x1B)

//! Macro that enables Timer/Counter1 interrupt where zero crossings are detected.
#define SET_TIMER1_INT_ZC_DETECTION   (TIMSK = (1 << TOIE1))

//! Macro that clears all Timer/Counter0 interrupt flags.
#define CLEAR_ALL_TIMER0_INT_FLAGS    (TIFR = TIFR | 0x1B)

//! Macro that disables all Timer/Counter0 interrupts.
#define DISABLE_ALL_TIMER0_INTS       (TIMSK &= 0xE4)

//! Macro that enable Timer/Counter0 Compare A interrupt.
#define SET_TIMER0_COMPA_INT          (TIMSK = (1 << OCIE0A))

//! The ADC resolution.
#define ADC_RESOLUTION                1024

//! ADC clock prescaled by 8 value.
#define ADC_PRESCALER_8               ((0 << ADPS2) | (1 << ADPS1) | (1 << ADPS0))

//! ADC clock prescaled by 8 value.
#define ADC_PRESCALER_16              ((1 << ADPS2) | (0 << ADPS1) | (0 << ADPS0))

//! ADC trigger source selection to set ADC in free-running mode.
#define ADC_TS_FREERUNNING            ((0 << ADTS2) | (0 << ADTS1) | (0 << ADTS0))

#endif
