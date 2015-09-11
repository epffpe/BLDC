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
 
#ifndef _BLDC_H_
#define _BLDC_H_

//_____  I N C L U D E S ___________________________________________________
#include "config.h"

//_____ M A C R O S ________________________________________________________

//_____ D E F I N I T I O N S ______________________________________________

//! Zero crossing polarity flag value for falling zero crossing.
#define EDGE_FALLING                  1

//! Zero crossing polarity flag value for rinsing zero crossing.
#define EDGE_RISING                   0

//! Clockwise rotation flag. Used only in macros.
#define CW                            0

//! Counterclockwise rotation flag. Used only in macros.
#define CCW                           1

//! Drive pattern for commutation step 1, CCW rotation.
#define DRIVE_PATTERN_STEP1_CCW      ((1 << UL) | (1 << VH))

//! Drive pattern for commutation step 2, CCW rotation.
#define DRIVE_PATTERN_STEP2_CCW      ((1 << UL) | (1 << WH))

//! Drive pattern for commutation step 3, CCW rotation.
#define DRIVE_PATTERN_STEP3_CCW      ((1 << VL) | (1 << WH))

//! Drive pattern for commutation step 4, CCW rotation.
#define DRIVE_PATTERN_STEP4_CCW      ((1 << VL) | (1 << UH))

//! Drive pattern for commutation step 5, CCW rotation.
#define DRIVE_PATTERN_STEP5_CCW      ((1 << WL) | (1 << UH))

//! Drive pattern for commutation step 6, CCW rotation.
#define DRIVE_PATTERN_STEP6_CCW      ((1 << WL) | (1 << VH))


//! Drive pattern for commutation step 1, CW rotation.
#define DRIVE_PATTERN_STEP1_CW      ((1 << VH) | (1 << WL))

//! Drive pattern for commutation step 2, CW rotation.
#define DRIVE_PATTERN_STEP2_CW      ((1 << UH) | (1 << WL))

//! Drive pattern for commutation step 3, CW rotation.
#define DRIVE_PATTERN_STEP3_CW      ((1 << UH) | (1 << VL))

//! Drive pattern for commutation step 4, CW rotation.
#define DRIVE_PATTERN_STEP4_CW      ((1 << WH) | (1 << VL))

//! Drive pattern for commutation step 5, CW rotation.
#define DRIVE_PATTERN_STEP5_CW      ((1 << WH) | (1 << UL))

//! Drive pattern for commutation step 6, CW rotation.
#define DRIVE_PATTERN_STEP6_CW      ((1 << VH) | (1 << UL))

//! Top value for the PWM timer.
#define PWM_TOP_VALUE               255

//! The minimum allowed PWM compare value.
#define MIN_PWM_COMPARE_VALUE       160

//! The maximum allowed PWM compare value.
#define MAX_PWM_COMPARE_VALUE       PWM_TOP_VALUE

//! PWM compare value used during startup.
#define STARTUP_PWM_COMPARE_VALUE   140


//ADC MUX setting to select PA7.
#define ADMUX_PA7   ((1 << MUX2) | (1 << MUX1) | (1 << MUX0))
#define ADMUX_PA1   ((0 << MUX2) | (0 << MUX1) | (1 << MUX0))
#define ADMUX_PA2   ((0 << MUX2) | (1 << MUX1) | (0 << MUX0))
#define ADMUX_PA4   ((0 << MUX2) | (1 << MUX1) | (1 << MUX0))
#define ADMUX_PA5   ((1 << MUX2) | (0 << MUX1) | (0 << MUX0))

//! ADC result alignment for BEMF measurement.
#define ADC_RES_ALIGNMENT_BEMF          (0 << ADLAR)

//! ADC result alignment for speed reference measurement.
#define ADC_RES_ALIGNMENT_SPEED_REF     (0 << ADLAR)

//! ADC result alignment for CURRENT measurement.
#define ADC_RES_ALIGNMENT_CURRENT       (0 << ADLAR)

//! ADC result alignment for reference voltage measurement.
#define ADC_RES_ALIGNMENT_REF_VOLTAGE   (0 << ADLAR)

//! ADMUX register value for channel U sampling.
#define ADMUX_U             (ADC_REF_CHANNEL | ADC_RES_ALIGNMENT_BEMF | ADC_MUX_U)

//! ADMUX register value for channel V sampling.
#define ADMUX_V             (ADC_REF_CHANNEL | ADC_RES_ALIGNMENT_BEMF | ADC_MUX_V)

//! ADMUX register value for channel W sampling.
#define ADMUX_W             (ADC_REF_CHANNEL | ADC_RES_ALIGNMENT_BEMF | ADC_MUX_W)

//! ADMUX register value for reference voltage sampling.
#define ADMUX_REF_VOLTAGE   (ADC_REF_CHANNEL | ADC_RES_ALIGNMENT_REF_VOLTAGE | ADC_MUX_SPEED_REF)

//! ADMUX register value for speed reference sampling.
#define ADMUX_SPEED_REF     (ADC_REF_CHANNEL | ADC_RES_ALIGNMENT_SPEED_REF | ADC_MUX_SPEED_REF)

//! ADC prescaler used.
#define ADC_PRESCALER       ADC_PRESCALER_8

//! ADC trigger source: TC1 Overflow
#define ADC_TRIGGER_SOURCE  ((1 << ADTS2) | (1 << ADTS1) | (0 << ADTS0))

//! Zero-cross threshold.
#define ADC_ZC_THRESHOLD    371

//! Macro that enable Timer/Counter0 interrupt responsible for commutation.
#define SET_TIMER0_INT_COMMUTATION    SET_TIMER0_COMPA_INT

#endif //_BLDC_H_
