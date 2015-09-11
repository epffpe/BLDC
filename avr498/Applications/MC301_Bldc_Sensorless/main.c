/*This file has been prepared for Doxygen automatic documentation generation.*/
//! \file *********************************************************************
//!
//! \brief Main for Motor application.
//!
//! - Compiler:           IAR EWAVR and GNU GCC for AVR
//! - Supported devices:  ATTiny861,ATTiny461,ATTiny261
//!
//! \author               Atmel Corporation: http://www.atmel.com \n
//!                       Support and FAQ: http://support.atmel.no/
//!
//! ***************************************************************************
//!
//! @mainpage ATTiny261/461/861 Sensorless application
//!
//! @section intro License
//! Use of this program is subject to Atmel's End User License Agreement.
//!
//! Please read file  \ref lic_page for copyright notice.
//!
//! @section install Description
//! This embedded application source code illustrates how to implement a sensorless application
//! over the ATTiny261/461/861 controller.
//!
//! @section sample About the sample application
//! By default the sample code is delivered configured for MC301
//!
//! @section src_code About the source code
//! This source code is usable with the following compilers:
//! - IAR Embedded Workbench (5.11A and higher)
//! - AVRGCC (WinAVR 20080411 and higher).
//!
//! Support for other compilers may required modifications or attention for:
//! - compiler.h file 
//! - special registers declaration file
//! - interrupt subroutines declarations
//!
//! @section arch Architecture
//! The application entry point is located is the main.c file.
//! The main function first performs the initialization of hardware ressources and then runs it in an infinite loop.
//! No real time schedule is performed, when tick ends, the regulation loop is called.
//!
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
#include "config.h"

//_____ M A C R O S ________________________________________________________

//_____ D E F I N I T I O N S ______________________________________________

int main(void)
{ 

  // Init hardware
  mc_init_HW();
    
  // Run Ramp Up sequence
  mc_start_motor();

  // Enable interrupts. The rest will now be handled by the ADC interrupts.
  Enable_interrupt();
   
  for (;;) {
   mc_regulation_loop();
  }

  return 0;
}

