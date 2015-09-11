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

#include "config.h"
#include "mc_drv.h"

//_____ M A C R O S ________________________________________________________

//_____ D E F I N I T I O N S ______________________________________________

//_____ D E C L A R A T I O N S ____________________________________________

#ifdef __ICCAVR__
code U8
#elif __GNUC__
U8
#endif
//CW
  ADMUXTableForward[6] = 
  {
  	ADMUX_V,
  	ADMUX_U,
  	ADMUX_W,
  	ADMUX_V,
  	ADMUX_U,
  	ADMUX_W
  };


#ifdef __ICCAVR__
code U8
#elif __GNUC__
U8
#endif
//CCW
  ADMUXTableReverse[6] = 
  {
  	ADMUX_V,
  	ADMUX_W,
  	ADMUX_U,
  	ADMUX_V,
  	ADMUX_W,
  	ADMUX_U
  };

//Table of output patterns for forward driving.
#ifdef __ICCAVR__
code U8
#elif __GNUC__
U8
#endif
//CW
 commTableForward[6] = {
  DRIVE_PATTERN_STEP2_CW,
  DRIVE_PATTERN_STEP1_CW,
  DRIVE_PATTERN_STEP6_CW,
  DRIVE_PATTERN_STEP5_CW,
  DRIVE_PATTERN_STEP4_CW,
  DRIVE_PATTERN_STEP3_CW
};


//Table of output patterns for reverse driving.
#ifdef __ICCAVR__
code U8
#elif __GNUC__
U8
#endif
//CCW
 commTableReverse[6] = {
  DRIVE_PATTERN_STEP5_CCW,
  DRIVE_PATTERN_STEP4_CCW,
  DRIVE_PATTERN_STEP3_CCW,
  DRIVE_PATTERN_STEP2_CCW,
  DRIVE_PATTERN_STEP1_CCW,
  DRIVE_PATTERN_STEP6_CCW
};

//Edge Level ZC for forward driving.
#ifdef __ICCAVR__
code U8
#elif __GNUC__
U8
#endif
//CW
 zcTableForward[6] = {
  EDGE_RISING,
  EDGE_FALLING,
  EDGE_RISING,
  EDGE_FALLING,
  EDGE_RISING,
  EDGE_FALLING 
};


//Edge Level ZC for reverse driving.
#ifdef __ICCAVR__
code U8
#elif __GNUC__
U8
#endif
//CCW
 zcTableReverse[6] = {
  EDGE_FALLING,
  EDGE_RISING,
  EDGE_FALLING,
  EDGE_RISING,
  EDGE_FALLING,
  EDGE_RISING
};
/*! \brief The power stage enable signals that will be output to the motor drivers
 *  at next commutation.
 *
 *  This variable holds the pattern of enable signals that will be output to the
 *  power stage at next commutation. It is stored in register R13 for quick access.
 */
volatile U8 nextDrivePattern;

/*! \brief Polarity of the expected zero crossing.
 *
 *  The polarity of the expected zero crossing.
 *  Could be eiter \ref EDGE_FALLING or \ref EDGE_RISING.
 */
volatile U8 zcPolarity ;

/*! \brief The commutation step that starts at next commutation.
 *
 *  The commutation step that starts at next commutation. This is used to keep
 *  track on where in the commutation cycle we are. Stored in register R11 for
 *  quick access
 */
volatile U8 nextCommutationStep ;

//! ADC reading of external analog speed reference.
volatile U16 speedReferenceADC;

//! Flag that specifies whether a new external speed reference and a motor speed measurement is available.
volatile U8 speedUpdated = FALSE;

void Timer0Init(void)
{
  // Set up Timer/counter0 for commutation timing, prescaler = 8.
  TCCR0B = ((0<<CS02)|(1<<CS01) |(0<<CS00)); 
}

void PLLInit(void)
{
  //Enable fast peripheral clock (64MHz for Timer1).
  PLLCSR = (1 << PCKE);
}


void PWMInit(void)
{
 
  //Clear on up-counting.
  TCCR1A = (1 << COM1A1) | (0 << COM1A0) | (1 << PWM1A);

  //Set WGM to PWM6, dual slope mode.
  TCCR1D = (1 << WGM11) | (1 << WGM10);

  //Set top value.
  TC1_WRITE_10_BIT_REGISTER(OCR1C, PWM_TOP_VALUE);

  //Run timer at full speed.
  TCCR1B = (1 << CS10);
  
}


void ADCInit(void)
{
  ADMUX = ADMUX_REF_VOLTAGE ;
  
  ADCSRA = (1 << ADEN) | (1 << ADSC) | (1 << ADIF) | (ADC_PRESCALER_16);
  while (ADCSRA & (1 << ADSC))
  {

  }
  speedReferenceADC = ADC;

  // Initialize the ADC for autotriggered operation on PWM timer overflow.
  ADCSRA = (1 << ADEN) | (0 << ADSC) | (1 << ADATE) | (1 << ADIF) | (0 << ADIE) | ADC_PRESCALER_16;
  ADCSRB = ADC_TRIGGER_SOURCE;
}


void PortsInit(void)
{
  //Set PWM pins as output. (PWM output is still controlled through TCCR1E register.)
  DDRB = (1 << PB0) | (1 << PB1) | (1 << PB2) | (1 << PB3) | (1 << PB4) | (1 << PB5)| (1 << PB6) ;

  // Disable digital input buffers on ADC channels.
  DIDR0 = (1 << ADC1D) | (1 << ADC2D) | (1 << ADC3D) | (1 << ADC4D) ;
}

//! This function initializes the hardware/software resources required for motor driver
//!
void mc_init_HW(void)
{
  Clear_prescaler();
  PLLInit();
  PWMInit();
  ADCInit();

  // Initialize timer0 for commutation timing
  Timer0Init();
  
  // Initialize alternate function
  PortsInit();

}

#ifndef __GNUC__
   #pragma optimize=none 
   void Set_cpu_prescaler(U8 x)
   {
      U8 save_int=SREG&0x80;
      Disable_interrupt();
      CLKPR=(1<<CLKPCE);
      CLKPR=x;
      if(save_int) { Enable_interrupt(); }
   }
#endif

/*! \brief Timer/counter0 bottom overflow. Used for zero-cross detection.
 *
 *  This interrupt service routine is called every time the up/down counting
 *  PWM counter reaches bottom. An ADC reading on the active channel is
 *  automatically triggered at the same time as this interrupt is triggered.
 *  This is used to detect a zero crossing.
 *
 *  In the event of a zero crossing, the time since last commutation is stored
 *  and Timer/counter1 compare A is set up to trigger at the next commutation
 *  instant.
 */
#ifdef __GNUC__
  ISR(TIMER1_OVF_vect)
#else
#pragma vector = TIM1_OVF_vect
__interrupt void MotorPWMBottom()
#endif
{

  U16 temp;

  CLEAR_ALL_TIMER1_INT_FLAGS;


  ADCSRA = (1 << ADEN) | (1 << ADSC) | (1 << ADIF) | (ADC_PRESCALER_16);
  while (ADCSRA & (1 << ADSC))
  {
  }
  temp= ADC;

  if (((zcPolarity == EDGE_RISING) && (temp > ADC_ZC_THRESHOLD)) || ((zcPolarity == EDGE_FALLING) && (temp < ADC_ZC_THRESHOLD)))
  {
    U16 timeSinceCommutation;

    // Find time since last commutation
    TC0_READ_TCNT0(timeSinceCommutation);
	
    // Reset Timer before setting the next Commutation Period
    TC0_WRITE_TCNT0(0);

    OCR0A = timeSinceCommutation;

    speedUpdated = TRUE;

    SET_TIMER0_INT_COMMUTATION;
    CLEAR_ALL_TIMER0_INT_FLAGS;

    // Disable Timer/Counter1 overflow ISR.
    DISABLE_ALL_TIMER1_INTS;

    // Read speed reference.

    // Make sure that a sample is not in progress.
    while (ADCSRA & (1 << ADSC))
    {

    }
    // Change channel
    ADMUX = ADMUX_SPEED_REF;

    // Start conversion manually.
    ADCSRA |= (1 << ADSC);

    // Wait for conversion to complete.
    while((ADCSRA & (1 << ADSC)))
    {

    }
    speedReferenceADC = ADC;

    ADCSRA |= (1 << ADATE) | (0 << ADIE) | ADC_PRESCALER;
  }

}


/*! \brief Commutates and prepares for new zero-cross detection.
 *
 *  This interrupt service routine is triggered exactly when a commutation
 *  is scheduled. The commutation is performed instantly and Timer/counter0
 *  is reset to measure the delay between commutation and zero-cross detection.
 *
 *  Commutation causes large transients on all phases for a short while that could
 *  cause false zero-cross detections. A zero cross detection hold-off period is
 *  therefore used to avoid any false readings. This is performed by using Timer/counter1
 *  compare B. The compare is set to happen after the specified hold-off period.
 *  Timer/counter1 compare B interrupt handler then enables the zero-cross detection.
 */
#ifdef __GNUC__
  ISR(TIMER0_COMPA_vect)
#else
#pragma vector = TIM0_COMPA_vect
__interrupt void Commutate()
#endif
{
  CLEAR_ALL_TIMER0_INT_FLAGS;

#if (DIRECTION_OF_ROTATION == CCW)
  // Commutate and clear commutation timer.
  TCCR1E = commTableReverse[nextCommutationStep];
#else
  // Commutate and clear commutation timer.
  TCCR1E = commTableForward[nextCommutationStep];
#endif


  TC0_WRITE_TCNT0(0);

#if (DIRECTION_OF_ROTATION == CCW)  
  zcPolarity = zcTableReverse[nextCommutationStep];
  ADMUX = ADMUXTableReverse[nextCommutationStep];
#else
  zcPolarity = zcTableForward[nextCommutationStep];
  ADMUX = ADMUXTableForward[nextCommutationStep];
#endif

  nextCommutationStep ++;

  if (nextCommutationStep > 5)
  {
    nextCommutationStep = 0;
  }

  DISABLE_ALL_TIMER0_INTS;
  SET_TIMER1_INT_ZC_DETECTION;

}
