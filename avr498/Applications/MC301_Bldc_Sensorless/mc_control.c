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

//_____  I N C L U D E S ___________________________________________________

#include "mc_control.h"
#include "mc_drv.h"
//_____ M A C R O S ________________________________________________________

//_____ D E F I N I T I O N S ______________________________________________

//_____ D E C L A R A T I O N S ____________________________________________

//Table of current measurement patterns for forward driving.
#ifdef __ICCAVR__
extern code U8
#elif __GNUC__
extern U8
#endif
//CW
  ADMUXTableForward[6];

//Table of current measurement patterns for reverse driving.
#ifdef __ICCAVR__
extern code U8
#elif __GNUC__
extern U8
#endif
//CCW
  ADMUXTableReverse[6];

//Table of output patterns for forward driving.
#ifdef __ICCAVR__
extern code U8
#elif __GNUC__
extern U8
#endif
//CW
 commTableForward[6];


//Table of output patterns for reverse driving.
#ifdef __ICCAVR__
extern code U8
#elif __GNUC__
extern U8
#endif
//CCW
 commTableReverse[6];

//Edge Level ZC for forward driving.
#ifdef __ICCAVR__
extern code U8
#elif __GNUC__
extern U8
#endif
//CW
 zcTableForward[6];


//Edge Level ZC for reverse driving.
#ifdef __ICCAVR__
extern code U8
#elif __GNUC__
extern U8
#endif
//CCW
 zcTableReverse[6];
 
extern volatile U8 nextCommutationStep;
extern volatile U16 speedReferenceADC;
extern volatile U8 speedUpdated;
extern volatile U8 zcPolarity;


//! Start sequence number of commutations
#define STARTUP_NUM_COMMUTATIONS  29

#ifdef __ICCAVR__
code U32
#elif __GNUC__
U32
#endif
start_delay[STARTUP_NUM_COMMUTATIONS+1] = 
{2000,1900,1800,1600,1400,1200,1000,800,600,550,500,480,460,440,420,400,380,360,355,350,345,340,335,330,325,320,315,310,305,300};

//! This function generates a delay used during startup
//!
void delay_us(volatile long delay)
{
  for (volatile long i=0;i<delay;i++);  
}

//! This function set duty cycle.
//!
void mc_set_duty_cycle(U16 duty)
{
  TC1_WRITE_10_BIT_REGISTER(OCR1A, duty);
}

/*! \brief Executes the motor startup sequence.
 *
 *  This function locks the motor into a known position and fires off a
 *  commutation sequence controlled by the Timer/counter1 overflow interrupt.
 */
void mc_start_motor()
{
  U8 i = 0;
  U8 j = 0;
  U8 end_of_start = 0;
  mc_set_duty_cycle(STARTUP_PWM_COMPARE_VALUE) ;
  
  nextCommutationStep = 0;

  while(end_of_start!=1)
  {
    switch(nextCommutationStep)
    {
      case 0:       
#if (DIRECTION_OF_ROTATION == CCW) 
        TCCR1E = commTableReverse[0];
#else
        TCCR1E = commTableForward[0];
#endif        
        if (i < 2) i += 1; 
        if ((i == 2)&&(j<STARTUP_NUM_COMMUTATIONS)) { 
           j += 1;i=0;
        }
       if ((i==2)&&(j==STARTUP_NUM_COMMUTATIONS)){
          end_of_start = 1;
        } 
        delay_us(start_delay[j]);  
        nextCommutationStep = 1;
        break;
      case 1: 
#if (DIRECTION_OF_ROTATION == CCW)      
        TCCR1E = commTableReverse[1];
#else
        TCCR1E = commTableForward[1];
#endif
        delay_us(start_delay[j]);  
        nextCommutationStep = 2;            
        break;
      case 2:   
#if (DIRECTION_OF_ROTATION == CCW)      
        TCCR1E = commTableReverse[2];
#else
        TCCR1E = commTableForward[2];
#endif        
        delay_us(start_delay[j]);  
        nextCommutationStep = 3;
        break;
      case 3:
#if (DIRECTION_OF_ROTATION == CCW)      
        TCCR1E = commTableReverse[3];
#else
        TCCR1E = commTableForward[3];
#endif        
        delay_us(start_delay[j]);  
        nextCommutationStep = 4;           
        break;
      case 4:       
#if (DIRECTION_OF_ROTATION == CCW)      
        TCCR1E = commTableReverse[4];
#else
        TCCR1E = commTableForward[4];
#endif        
        delay_us(start_delay[j]);  
        nextCommutationStep = 5;          
        break;
      case 5:  
#if (DIRECTION_OF_ROTATION == CCW)      
        TCCR1E = commTableReverse[5];
#else
        TCCR1E = commTableForward[5];
#endif        
        delay_us(start_delay[j]);  
        nextCommutationStep = 0;            
        break;
    default :
      nextCommutationStep = nextCommutationStep;
      break;
    }
  }

  //nextCommutationStep++;
  nextCommutationStep = 0;
  // Use LSB of nextCommutationStep to determine zero crossing polarity.  
#if (DIRECTION_OF_ROTATION == CCW)  
  zcPolarity = zcTableReverse[nextCommutationStep];
  ADMUX = ADMUXTableReverse[nextCommutationStep];
#else
  zcPolarity = zcTableForward[nextCommutationStep];
  ADMUX = ADMUXTableForward[nextCommutationStep];
#endif 
  // Switch to sensorless commutation.
  TC0_WRITE_TCNT0(0);

  // Enable Timer 1 Interrupt
  SET_TIMER1_INT_ZC_DETECTION;

}

//! This function launches speed control or no regulation
//!
void mc_regulation_loop()
{
  // Only update duty cycle if a new speed reference measurement has been made. (Done right after speed measurement is ready)
  if (speedUpdated)
  {
    speedUpdated = FALSE;
    // Calculate duty cycle from speed reference value.
    mc_set_duty_cycle(MIN_PWM_COMPARE_VALUE+(speedReferenceADC>>3));
  }
}


