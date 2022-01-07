//#############################################################################
//
// FILE:   nmi_ex1_cpu1handling.c
//
// TITLE:  NMI Handling between CPU cores
//
//! \addtogroup driver_dual_example_list
//! <h1> NMI handling</h1>
//!
//! This example demonstrates how to handle an NMI.
//!
//! The watchdog of CPU 2 is configured to reset the core once the watchdog 
//! overflows and in the CPU 1 the NMI is triggered.
//! The NMI status is read and is verified to be due to CPU2 Watchdog reset.
//! \b  Watch \b Variables
//!  - \e pass Indicates that the CPU2 has been reset by its watchdog and an 
//!       -NMI was triggered at CPU1
//!
//
//#############################################################################
// $TI Release: F2837xD Support Library v3.10.00.00 $
// $Release Date: Tue May 26 17:13:46 IST 2020 $
// $Copyright:
// Copyright (C) 2013-2020 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//#############################################################################

//
// Included Files
//
#include <stdint.h>
#include <stdbool.h>

#include "sysctl.h"
#include "driverlib.h"
#include "device.h"

//
// Globals
//
volatile uint32_t nmistatus = 0,nmiflagstatus =0 ,nmisdflagstatus =0,nmiwdcnt =0;
volatile uint16_t nmi_isr_called = 0,pass = 0;

//
// Function Prototypes
//
interrupt void nmi_isr(void);

void main(void)
{
    //
    // Initialize device clock and peripherals
    //
    Device_init();

    //
    // Initialize PIE and clear PIE registers. Disables CPU interrupts
    //
    Interrupt_initModule();

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR)
    //
    Interrupt_initVectorTable();
    SysCtl_clearAllNMIFlags();
    Interrupt_register(INT_NMI, &nmi_isr);

    //
    //Enabling the NMI global interrupt
    //
    EALLOW;
    HWREGH(NMI_BASE + NMI_O_CFG) |= NMI_CFG_NMIE;
    EDIS;

    Interrupt_enable(INT_NMI);

    //
    // Enable Global Interrupt (INTM) and Real Time interrupt (DBGM)
    //
    EINT;
    ERTM;

    while(nmi_isr_called != 1)
    {
        pass=0;
    }

    if(SysCtl_isNMIShadowFlagSet(SYSCTL_NMI_CPU2WDRSN)==true)
    pass = 1;

    SysCtl_clearAllNMIFlags();
    if(pass == 1)
    {
        Example_setResultPass();
    }
    else
    {
        Example_setResultFail();
    }

    Example_done();
}

interrupt void nmi_isr(void)
{
    nmi_isr_called = 1;
    nmiflagstatus = SysCtl_getNMIFlagStatus();
    nmisdflagstatus = SysCtl_getNMIShadowFlagStatus();
}

//
// End of File
//
