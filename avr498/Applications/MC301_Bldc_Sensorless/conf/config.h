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

#ifndef _CONFIG_H_
#define _CONFIG_H_


//_____ I N C L U D E S ____________________________________________________

#include "TinyX61_macros.h"

//_____ M A C R O S ________________________________________________________

//_____ D E F I N I T I O N S ______________________________________________

//! CPU core frequency in kHz
#define FOSC          8000


//! Driver stage pin mapping.
#define UL    PB0       //! UL
#define UH    PB1       //! UH
#define VL    PB2       //! VL
#define VH    PB3       //! VL
#define WL    PB4       //! WL
#define WH    PB5       //! WH


//! ADC multiplexer selection for channel U sampling.
#define ADC_MUX_U                ADMUX_PA1

//! ADC multiplexer selection for channel V sampling.
#define ADC_MUX_V                ADMUX_PA4

//! ADC multiplexer selection for channel W sampling.
#define ADC_MUX_W                ADMUX_PA5

//! ADC multiplexer selection for channel Speed Reference sampling.
#define ADC_MUX_SPEED_REF 	 	   ADMUX_PA2

//! ADC reference channel selection.
#define ADC_REF_CHANNEL          ((0 << REFS1) | (1 << REFS0))

//! Direction of rotation. Set to either CW or CCW for
#define DIRECTION_OF_ROTATION     CW
   
#endif // _CONFIG_H_

