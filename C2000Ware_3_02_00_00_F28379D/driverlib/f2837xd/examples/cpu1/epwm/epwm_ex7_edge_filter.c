//#############################################################################
//
// FILE:   epwm_ex7_edge_filter.c
//
// TITLE:  ePWM Using Digital Compare Submodule with
//         Edge Filter.
//
//! \addtogroup driver_example_list
//! <h1>ePWM Digital Compare Edge Filter</h1>
//!
//! This example configures ePWM1 as follows
//!  - ePWM1 with DCBEVT2 forcing the ePWM output LOW as a CBC source
//!  - GPIO25 is used as the input to the INPUT XBAR INPUT1
//!  - INPUT1 (from INPUT XBAR) is used as the source for DCBEVT2
//!  - GPIO25 is set to output and toggled in the main loop to trip the PWM
//!  - The DCBEVT2 is the source for DCFILT
//!  - The DCFILT will count edges of the DCBEVT2 and generate a signal to
//!    to trip the ePWM on the 4th edge of DCBEVT2
//!
//! \b External \b Connections \n
//! - GPIO0 EPWM1A
//! - GPIO1 EPWM1B
//! - GPIO25 TRIPIN1 (Output Pin, toggled through software)
//!
//! \b Watch \b Variables \n
//! - None.
//
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
#include "driverlib.h"
#include "device.h"


//
// Function Prototypes
//
void initEPWM1(void);
void initTZGPIO(void);
void initEPWMGPIO(void);
__interrupt void epwm1ISR(void);

void main(void)
{

    //
    // Initialize device clock and peripherals
    //
    Device_init();

    //
    // Disable pin locks and enable internal pull-ups.
    //
    Device_initGPIO();

    //
    // Initialize PIE and clear PIE registers. Disables CPU interrupts.
    //
    Interrupt_initModule();

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    Interrupt_initVectorTable();

    //
    // Interrupts that are used in this example are re-mapped to ISR functions
    // found within this file.
    //
    Interrupt_register(INT_EPWM1, &epwm1ISR);

    //
    // Configure ePWM1, and TZ GPIOs
    //
    initEPWMGPIO();
    initTZGPIO();

    //
    // Disable sync(Freeze clock to PWM as well). GTBCLKSYNC is applicable
    // only for multiple core devices. Uncomment the below statement if
    // applicable.
    //
    // SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_GTBCLKSYNC);
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    //
    // Initialize ePWM1
    //
    initEPWM1();

    //
    // Enable sync and clock to PWM
    //
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    //
    // Enable interrupts required for this example
    //
    Interrupt_enable(INT_EPWM1);

    //
    // Enable Global Interrupt (INTM) and real time interrupt (DBGM)
    //
    EINT;
    ERTM;

    //
    // Delay to let ePWM do complete uninterrupted cycles
    // This allows the user to see the PWM waveform uninterrupted
    // on the scope.
    //
    SysCtl_delay(600000U);

    //
    // IDLE loop. Just sit and loop forever (optional):
    //
    for(;;)
    {
        //
        // Toggle the GPIO multiple times, the trip occurs
        // on the 4th edge
        //
        while(HWREGH(EPWM1_BASE + EPWM_O_TBCTR) < 4000);
        GPIO_writePin(25, 0);
        while(HWREGH(EPWM1_BASE + EPWM_O_TBCTR) < 5000);
        GPIO_writePin(25, 1);
        while(HWREGH(EPWM1_BASE + EPWM_O_TBCTR) < 6000);
        GPIO_writePin(25, 0);
        while(HWREGH(EPWM1_BASE + EPWM_O_TBCTR) < 7000);
        GPIO_writePin(25, 1);
        while(HWREGH(EPWM1_BASE + EPWM_O_TBCTR) < 8000);
        GPIO_writePin(25, 0);
        while(HWREGH(EPWM1_BASE + EPWM_O_TBCTR) < 9000);
        GPIO_writePin(25, 1);
        while(HWREGH(EPWM1_BASE + EPWM_O_TBCTR) < 10000);
        GPIO_writePin(25, 0);
        while(HWREGH(EPWM1_BASE + EPWM_O_TBCTR) < 11000);
        GPIO_writePin(25, 1);
        while(HWREGH(EPWM1_BASE + EPWM_O_TBCTR) > 3000);
        GPIO_writePin(25, 1);
        NOP;
    }
}

interrupt void epwm1ISR(void){

    EPWM_clearTripZoneFlag(EPWM1_BASE, EPWM_TZ_FLAG_CBC | EPWM_TZ_FLAG_DCBEVT2 | EPWM_TZ_INTERRUPT);
    EPWM_clearCycleByCycleTripZoneFlag(EPWM1_BASE, EPWM_TZ_CBC_FLAG_DCBEVT2);

    //
    // Clear INT flag for this timer
    //
    EPWM_clearEventTriggerInterruptFlag(EPWM1_BASE);

    //
    // Acknowledge interrupt group
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP3);

}

//
// initEPWM1 - Configure ePWM1
//
void initEPWM1()
{
    //
    // Enable DCBEVT2 as CBC sources
    //
    EPWM_enableTripZoneSignals(EPWM1_BASE, EPWM_TZ_SIGNAL_DCBEVT2);

    //
    // CBC will clear at CTR=0
    //
    EPWM_selectCycleByCycleTripZoneClearEvent(EPWM1_BASE, EPWM_TZ_CBC_PULSE_CLR_CNTR_ZERO);

    //
    // Select TRIPIN1 (INPUT XBAR INPUT1) as the source for DCBH
    //
    EPWM_selectDigitalCompareTripInput(
            EPWM1_BASE, EPWM_DC_TRIP_TRIPIN1, EPWM_DC_TYPE_DCBH);
    //
    // DCBEVT2 is generated when DCBH is low
    //
    EPWM_setTripZoneDigitalCompareEventCondition(
            EPWM1_BASE, EPWM_TZ_DC_OUTPUT_B2, EPWM_TZ_EVENT_DCXH_LOW);

    //
    // DCBEVT2 as input for DCFILT
    //
    EPWM_setDigitalCompareFilterInput(EPWM1_BASE, EPWM_DC_WINDOW_SOURCE_DCBEVT2);

    //
    // Edge filter will count rising edges
    //
    EPWM_setDigitalCompareEdgeFilterMode(EPWM1_BASE, EPWM_DC_EDGEFILT_MODE_RISING);

    //
    // Edge filter will generate TBCLK wide signal at the 4th edge
    //
    EPWM_setDigitalCompareEdgeFilterEdgeCount(EPWM1_BASE, EPWM_DC_EDGEFILT_EDGECNT_4);

    //
    // Edge filter will reset at CTR=0
    //
    EPWM_setValleyTriggerSource(EPWM1_BASE, EPWM_VALLEY_TRIGGER_EVENT_CNTR_ZERO);

    //
    // Edge filter enabled
    //
    EPWM_enableValleyCapture(EPWM1_BASE);
    EPWM_enableDigitalCompareEdgeFilter(EPWM1_BASE);

    //
    // DCBEVT2 is the DCFILT input
    //
    EPWM_setDigitalCompareEventSource(
            EPWM1_BASE, EPWM_DC_MODULE_B, EPWM_DC_EVENT_2, EPWM_DC_EVENT_SOURCE_FILT_SIGNAL);

    //
    // DCBEVT2 is asynchronous
    //
    EPWM_setDigitalCompareEventSyncMode(
            EPWM1_BASE, EPWM_DC_MODULE_B, EPWM_DC_EVENT_2, EPWM_DC_EVENT_INPUT_NOT_SYNCED);

    //
    // Action on CBC TZ A
    //
    EPWM_setTripZoneAction(EPWM1_BASE,
                           EPWM_TZ_ACTION_EVENT_TZA,
                           EPWM_TZ_ACTION_DISABLE);
    //
    // Action on CBC TZ B
    //
    EPWM_setTripZoneAction(EPWM1_BASE,
                           EPWM_TZ_ACTION_EVENT_TZB,
                           EPWM_TZ_ACTION_LOW);


    //
    // Set-up TBCLK
    //
    EPWM_setTimeBasePeriod(EPWM1_BASE, 12000U);
    EPWM_setPhaseShift(EPWM1_BASE, 0U);
    EPWM_setTimeBaseCounter(EPWM1_BASE, 0U);
    EPWM_setTimeBaseCounterMode(EPWM1_BASE, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_disablePhaseShiftLoad(EPWM1_BASE);

    //
    // Set ePWM clock pre-scaler
    //
    EPWM_setClockPrescaler(EPWM1_BASE,
                           EPWM_CLOCK_DIVIDER_4,
                           EPWM_HSCLOCK_DIVIDER_4);

    //
    // Set up shadowing
    //
    EPWM_setCounterCompareShadowLoadMode(EPWM1_BASE,
                                         EPWM_COUNTER_COMPARE_A,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);

    //
    // Set-up compare
    //
    EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, 2000U);

    //
    // Set actions
    //
    EPWM_setActionQualifierAction(EPWM1_BASE,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM1_BASE,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    EPWM_setActionQualifierAction(EPWM1_BASE,
                                  EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM1_BASE,
                                  EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);

    //
    // Interrupt where we will change the Compare Values
    // Select INT on Time base counter zero event,
    // Enable INT, generate INT on 3rd event
    //
    EPWM_setInterruptSource(EPWM1_BASE, EPWM_INT_TBCTR_ZERO);
    EPWM_enableInterrupt(EPWM1_BASE);
    EPWM_setInterruptEventCount(EPWM1_BASE, 3U);
}

//
// initTZGPIO - Configure TZ GPIO
//
void initTZGPIO(void)
{
    //
    // Set GPIO 25 as as Asynchronous input with pull up enabled
    //
    GPIO_setPadConfig(25, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(GPIO_25_GPIO25);
    GPIO_setDirectionMode(25, GPIO_DIR_MODE_OUT);
    GPIO_writePin(25, 1);

    //
    // Set GPIO 25 as TZ1 input
    //
    XBAR_setInputPin(XBAR_INPUT1, 25);

}

//
// initEPWMGPIO - Configure ePWM GPIO
//
void initEPWMGPIO(void)
{
    //
    // Disable pull up on GPIO 0 configure them as PWM1A
    //
    GPIO_setPadConfig(0, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(GPIO_0_EPWM1A);

    //
    // Disable pull up on GPIO 1 configure them as PWM1B
    //
    GPIO_setPadConfig(1, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(GPIO_1_EPWM1B);

}
