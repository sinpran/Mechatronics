//#############################################################################
// FILE:   labstarter_main.c
//
// TITLE:  Lab Starter
//#############################################################################

// Included Files
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"
#include "f28379dSerial.h"
#include "LEDPatterns.h"
#include "song.h"
#include "dsp.h"
#include "fpu32/fpu_rfft.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398


// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI_isr(void);
__interrupt void ADCD_ISR(void);

float readEncRight(void);
float readEncLeft(void);

void setEPWM2A(float controleffort);
void setEPWM2B(float controleffort);
void init_eQEPs(void);

int xy_control(float *vref_forxy, float *turn_forxy,float turn_thres, float x_pos,float y_pos,float x_desired,float y_desired,
                float thetaabs,float target_radius,float target_radius_near);
float my_atanf(float dy, float dx);

void serialRXA(serial_t *s, char data);

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls = 0;
uint32_t numRXA = 0;
uint16_t UARTPrint = 0;
uint16_t LEDdisplaynum = 0;
int16_t adcd0result = 0;
int16_t adcd1result = 0;
float var1 = 0;
float var2 = 0;
int32_t count = 0;

float LeftWheel = 0;
float RightWheel = 0;
float RightWheelDistance = 0;
float LeftWheelDistance = 0;

float LeftDistance = 0;
float LeftDesired = 0;
float LeftReturn = 0;

float uleft = 5.0;
float uright = 5.0;
float VLeftK = 0;
float VRightK = 0;
float Vref = 0.1;

float RightDist_K_1 = 0;
float LeftDist_K_1 = 0;

float Kp = 3;
float Kp_left_wall = -1;
float Kp_front = -1;
float Ki = 25;

float e_distance = 0;
float e_front = 0;
float e_left = 0;
float e_right = 0;
float e_left_K_1 = 0;
float e_right_K_1 = 0;

float I_left = 0;
float I_right = 0;
float I_left_K_1 = 0;
float I_right_K_1 = 0;

float turn = 0;
float e_turn = 0;
float e_turn_right = 0;
float e_turn_left = 0;
float Kp_turn = 3;

int16_t avoidstate = 0;
int16_t nextmove = 0;
float FrontDistance =0;
float FrontThresh = 0;
float FrontReturn = 0;

float Wr = 0.61;
float Rwh = 0.10625;
float theta_dot_right = 0;
float theta_dot_left = 0;
float Xr = 0;
float X_dot_r = 0;
float Y_dot_r = 0;
float X_K_1 = 0;
float Y_K_1 = 0;
float X_dot_K_1 = 0;
float Y_dot_K_1 = 0;
float Yr = 0;
float phi_r =0;
float theta_dot_avg = 0;
int16_t followCount = 0;


//This function sets DACA to the voltage between 0V and 3V passed to this function.
//If outside 0V to 3V the output is saturated at 0V to 3V
//Example code
//float myu = 2.25;
//setDACA(myu); // DACA will now output 2.25 Volts
void setDACA(float dacouta0) {
 if (dacouta0 > 3.0) dacouta0 = 3.0;
 if (dacouta0 < 0.0) dacouta0 = 0.0;
 DacaRegs.DACVALS.bit.DACVALS = dacouta0*4095.0/3.0; // perform scaling of 0-3 to 0-4095
}
void setDACB(float dacouta1) {
 if (dacouta1 > 3.0) dacouta1 = 3.0;
 if (dacouta1 < 0.0) dacouta1 = 0.0;
 DacbRegs.DACVALS.bit.DACVALS = dacouta1*4095.0/3.0; // perform scaling of 0-3 to 0-4095
}


void main(void)
{
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xD_SysCtrl.c file.
    InitSysCtrl();

    InitGpio();

    // Blue LED on LuanchPad
    GPIO_SetupPinMux(31, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO31 = 1;

    // Red LED on LaunchPad
    GPIO_SetupPinMux(34, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(34, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO34 = 1;

    // LED1 and PWM Pin
    GPIO_SetupPinMux(22, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(22, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;

    // LED2
    GPIO_SetupPinMux(94, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(94, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1;

    // LED3
    GPIO_SetupPinMux(95, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(95, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO95 = 1;

    // LED4
    GPIO_SetupPinMux(97, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(97, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO97 = 1;

    // LED5
    GPIO_SetupPinMux(111, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(111, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO111 = 1;

    // LED6
    GPIO_SetupPinMux(130, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(130, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO130 = 1;

    // LED7
    GPIO_SetupPinMux(131, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(131, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO131 = 1;

    // LED8
    GPIO_SetupPinMux(25, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(25, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;

    // LED9
    GPIO_SetupPinMux(26, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(26, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;

    // LED10
    GPIO_SetupPinMux(27, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(27, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;

    // LED11
    GPIO_SetupPinMux(60, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(60, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1;

    // LED12
    GPIO_SetupPinMux(61, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(61, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;

    // LED13
    GPIO_SetupPinMux(157, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(157, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO157 = 1;

    // LED14
    GPIO_SetupPinMux(158, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(158, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO158 = 1;

    // LED15
    GPIO_SetupPinMux(159, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(159, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO159 = 1;

    // LED16
    GPIO_SetupPinMux(160, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(160, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPFCLEAR.bit.GPIO160 = 1;

    //WIZNET Reset
    GPIO_SetupPinMux(0, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(0, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO0 = 1;

    //ESP8266 Reset
    GPIO_SetupPinMux(1, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(1, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO1 = 1;

    //SPIRAM  CS  Chip Select
    GPIO_SetupPinMux(19, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(19, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;

    //DRV8874 #1 DIR  Direction
    GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO29 = 1;

    //DRV8874 #2 DIR  Direction
    GPIO_SetupPinMux(32, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(32, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO32 = 1;

    //DAN28027  CS  Chip Select
    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO9 = 1;

    //MPU9250  CS  Chip Select
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;

    //WIZNET  CS  Chip Select
    GPIO_SetupPinMux(125, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(125, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDSET.bit.GPIO125 = 1;

    //PushButton 1
    GPIO_SetupPinMux(4, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(4, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 2
    GPIO_SetupPinMux(5, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(5, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 3
    GPIO_SetupPinMux(6, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(6, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 4
    GPIO_SetupPinMux(7, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(7, GPIO_INPUT, GPIO_PULLUP);

    //Joy Stick Pushbutton
    GPIO_SetupPinMux(8, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(8, GPIO_INPUT, GPIO_PULLUP);

    //Setting up GPIO52
    GPIO_SetupPinMux(52, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(52, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO52 = 1;

    GPIO_SetupPinMux(2, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinMux(3, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinMux(14, GPIO_MUX_CPU1,1);
    GPIO_SetupPinMux(15, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinMux(16, GPIO_MUX_CPU1, 5);

    EALLOW; // Below are protected registers
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1; // For EPWM2A
    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 1; // For EPWM2B
    GpioCtrlRegs.GPAPUD.bit.GPIO14 = 1; // For EPWM8A
    GpioCtrlRegs.GPAPUD.bit.GPIO15 = 1; // For EPWM8B
    GpioCtrlRegs.GPAPUD.bit.GPIO16 = 1; // For EPWM9A
    GpioCtrlRegs.GPAPUD.bit.GPIO22 = 1; // For EPWM12A
    EDIS;

    //set EPwm2Regs for the Motors
    EPwm2Regs.TBCTL.bit.CTRMODE = 0;
    EPwm2Regs.TBCTL.bit.FREE_SOFT = 2;
    EPwm2Regs.TBCTL.bit.PHSEN = 0;
    EPwm2Regs.TBCTL.bit.CLKDIV = 0;
    //TBCTR
    EPwm2Regs.TBCTR = 0;
    //CMPA
    EPwm2Regs.CMPA.bit.CMPA = 0;
    EPwm2Regs.CMPB.bit.CMPB = 0;
    //AQCTLA
    EPwm2Regs.AQCTLA.bit.ZRO = 2;
    EPwm2Regs.AQCTLA.bit.CAU = 1;
    EPwm2Regs.AQCTLB.bit.CBU = 1;
    EPwm2Regs.AQCTLB.bit.ZRO = 2;
    EPwm2Regs.TBPRD = 2500;
    //TBPHS
    EPwm2Regs.TBPHS.bit.TBPHS =0;

    // Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    DINT;

    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the F2837xD_PieCtrl.c file.
    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2837xD_DefaultIsr.c.
    // This function is found in F2837xD_PieVect.c.
    InitPieVectTable();

    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this project
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    PieVectTable.TIMER1_INT = &cpu_timer1_isr;
    PieVectTable.TIMER2_INT = &cpu_timer2_isr;
    PieVectTable.SCIA_RX_INT = &RXAINT_recv_ready;
    PieVectTable.SCIC_RX_INT = &RXCINT_recv_ready;
    PieVectTable.SCID_RX_INT = &RXDINT_recv_ready;
    PieVectTable.SCIA_TX_INT = &TXAINT_data_sent;
    PieVectTable.SCIC_TX_INT = &TXCINT_data_sent;
    PieVectTable.SCID_TX_INT = &TXDINT_data_sent;

    PieVectTable.EMIF_ERROR_INT = &SWI_isr;
    //enabling interrupts for ADCD, ADCA, and ADCB
    PieVectTable.ADCD1_INT = &ADCD_ISR; // will be using ADCD for this project
    EDIS;    // This is needed to disable write to EALLOW protected registers


    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every second:
    // 200MHz CPU Freq, 1 second Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, 200, 10000);
    ConfigCpuTimer(&CpuTimer1, 200, 20000);
    ConfigCpuTimer(&CpuTimer2, 200, 40000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    init_serial(&SerialA,115200,serialRXA);
//    init_serial(&SerialC,115200,serialRXC);
//    init_serial(&SerialD,115200,serialRXD);
    init_eQEPs();


    EALLOW;
    EPwm5Regs.ETSEL.bit.SOCAEN = 0; // Disable SOC on A group
    EPwm5Regs.TBCTL.bit.CTRMODE = 3; // freeze counter
    EPwm5Regs.ETSEL.bit.SOCASEL = 2; // Select Event when counter equal to PRD
    EPwm5Regs.ETPS.bit.SOCAPRD = 1; // Generate pulse on 1st event (“pulse” is the same as “trigger”)
    EPwm5Regs.TBCTR = 0x0; // Clear counter
    //SE423, Lab 4 Page 3 of 11
    EPwm5Regs.TBPHS.bit.TBPHS = 0x0000; // Phase is 0
    EPwm5Regs.TBCTL.bit.PHSEN = 0; // Disable phase loading
    EPwm5Regs.TBCTL.bit.CLKDIV = 2; // divide by 1 50Mhz Clock
    EPwm5Regs.TBPRD = 50000; // Set Period to 1ms sample. Input clock is 50MHz.
    // Notice here that we are not setting CMPA or CMPB because we are not using the PWM signal
    EPwm5Regs.ETSEL.bit.SOCAEN = 1; //enable SOCA
    EPwm5Regs.TBCTL.bit.CTRMODE = 0; //unfreeze, and enter up count mode
    EDIS;

    EALLOW;
    //write configurations for all ADCs ADCA, ADCB, ADCC, ADCD
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdccRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcdRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    AdcSetMode(ADC_ADCC, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    AdcSetMode(ADC_ADCD, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    //Set pulse positions to late
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcdRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    //power up the ADCs
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcdRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    //delay for 1ms to allow ADC time to power up
    DELAY_US(1000);
   // SE423, Lab 4 Page 4 of 11
    //Select the channels to convert and end of conversion flag
    //Many statements commented out, To be used when using ADCA or ADCB
    //ADCA
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 2; //SOC0 will convert Channel you choose Does not have to be A0
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 13;// EPWM5 ADCSOCA or another trigger you choose will trigger SOC0
    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 3; //SOC1 will convert Channel you choose Does not have to be A1
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 13;// EPWM5 ADCSOCA or another trigger you choose will trigger SOC1
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 1; //set to last SOC that is converted and it will set INT1 flag ADCA1
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1; //enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
    //ADCB
    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 4; //SOC0 will convert Channel you choose Does not have to be B0
    AdcbRegs.ADCSOC0CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 13;// EPWM5 ADCSOCA or another trigger you choose will trigger SOC0
    //AdcbRegs.ADCSOC1CTL.bit.CHSEL = ???; //SOC1 will convert Channel you choose Does not have to be B1
    //AdcbRegs.ADCSOC1CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    //AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = ???;// EPWM5 ADCSOCA or another trigger you choose will trigger SOC1
    //AdcbRegs.ADCSOC2CTL.bit.CHSEL = ???; //SOC2 will convert Channel you choose Does not have to be B2
    //AdcbRegs.ADCSOC2CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    //AdcbRegs.ADCSOC2CTL.bit.TRIGSEL = ???;// EPWM5 ADCSOCA or another trigger you choose will trigger SOC2
    //AdcbRegs.ADCSOC3CTL.bit.CHSEL = ???; //SOC3 will convert Channel you choose Does not have to be B3
    //AdcbRegs.ADCSOC3CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    //AdcbRegs.ADCSOC3CTL.bit.TRIGSEL = ???;// EPWM5 ADCSOCA or another trigger you choose will trigger SOC3
    AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 0; //set to last SOC that is converted and it will set INT1 flag ADCB1
    AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1; //enable INT1 flag
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
    //ADCD
    AdcdRegs.ADCSOC0CTL.bit.CHSEL = 0; // set SOC0 to convert pin D0
    AdcdRegs.ADCSOC0CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcdRegs.ADCSOC0CTL.bit.TRIGSEL = 13; // EPWM5 ADCSOCA will trigger SOC0
    AdcdRegs.ADCSOC1CTL.bit.CHSEL = 1; //set SOC1 to convert pin D1
    AdcdRegs.ADCSOC1CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcdRegs.ADCSOC1CTL.bit.TRIGSEL = 13; // EPWM5 ADCSOCA will trigger SOC1
    //AdcdRegs.ADCSOC2CTL.bit.CHSEL = ???; //set SOC2 to convert pin D2
    //AdcdRegs.ADCSOC2CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    //AdcdRegs.ADCSOC2CTL.bit.TRIGSEL = ???; // EPWM5 ADCSOCA will trigger SOC2
    //AdcdRegs.ADCSOC3CTL.bit.CHSEL = ???; //set SOC3 to convert pin D3
    //AdcdRegs.ADCSOC3CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    //AdcdRegs.ADCSOC3CTL.bit.TRIGSEL = ???; // EPWM5 ADCSOCA will trigger SOC3
    AdcdRegs.ADCINTSEL1N2.bit.INT1SEL = 1; //set to SOC1, the last converted, and it will set INT1 flag ADCD1
    AdcdRegs.ADCINTSEL1N2.bit.INT1E = 1; //enable INT1 flag
    AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
    EDIS;

    // Enable DACA and DACB outputs
    EALLOW;
    DacaRegs.DACOUTEN.bit.DACOUTEN = 1;//enable dacA output-->uses ADCINA0
    DacaRegs.DACCTL.bit.LOADMODE = 0;//load on next sysclk
    DacaRegs.DACCTL.bit.DACREFSEL = 1;//use ADC VREF as reference voltage
    DacbRegs.DACOUTEN.bit.DACOUTEN = 1;//enable dacB output-->uses ADCINA1
    DacbRegs.DACCTL.bit.LOADMODE = 0;//load on next sysclk
    DacbRegs.DACCTL.bit.DACREFSEL = 1;//use ADC VREF as reference voltage
    EDIS;

    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.  
    IER |= M_INT1;
    IER |= M_INT8;  // SCIC SCID
    IER |= M_INT9;  // SCIA
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    // Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;
    //enable pie interrupt 1.6
    PieCtrlRegs.PIEIER1.bit.INTx6 = 1;
    //pie interrupt 1.1
    //PieCtrlRegs.PIEIER1.bit.INTx1 = 1;
    //pie interrupt 1.2
    //PieCtrlRegs.PIEIER1.bit.INTx2 = 1;
    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    
    // IDLE loop. Just sit and loop forever (optional):
    while(1)
    {
        if (UARTPrint == 1 ) {
               //serial_printf(&SerialA,"ADCD0: %.3f ADCD1: %.3f\r\n",var1,var2);
                //serial_printf(&SerialA,"RightWheel: %.3f LeftWheel: %.3f RightDistance: %.3f LeftDistance: %.3f VRight: %.3f Vleft: %.3f\r\n",RightWheel,LeftWheel, RightWheelDistance, LeftWheelDistance, VRightK, VLeftK);
                //serial_printf(&SerialA,"eturn: %.3f edistance: %.3f eturnright: %.3f eturnleft: %.3f\r\n",e_turn,e_distance, e_turn_left, e_turn_right);
               //serial_printf(&SerialA,"frontdistance: %.3f ADCD1: %.3f leftdistance: %.3f ADCD0: %.3f AvoidState: %d\r\n",FrontDistance,var2, LeftDistance, var1, avoidstate);
               serial_printf(&SerialA,"Xr: %.3f Yr: %.3f Phi: %.3f frontdistance: %.3f ADCD1: %.3f leftdistance: %.3f ADCD0: %.3f AvoidState: %d NextMove: %d followCount: %d\r\n",Xr, Yr, phi_r, FrontDistance,var2, LeftDistance, var1, avoidstate, nextmove, followCount);
            UARTPrint = 0;
        }
    }
}


// SWI_isr,  Using this interrupt as a Software started interrupt
__interrupt void SWI_isr(void) {

    // These three lines of code allow SWI_isr, to be interrupted by other interrupt functions
    // making it lower priority than all other Hardware interrupts.
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
    asm("       NOP");                    // Wait one cycle
    EINT;                                 // Clear INTM to enable interrupts



    // Insert SWI ISR Code here.......


    numSWIcalls++;
    
    DINT;

}

// cpu_timer0_isr - CPU Timer0 ISR
__interrupt void cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;

    numTimer0calls++;

//    if ((numTimer0calls%50) == 0) {
//        PieCtrlRegs.PIEIFR12.bit.INTx9 = 1;  // Manually cause the interrupt for the SWI
//    }

    if ((numTimer0calls%250) == 0) {
        displayLEDletter(LEDdisplaynum);
        LEDdisplaynum++;
        if (LEDdisplaynum == 0xFFFF) {  // prevent roll over exception
            LEDdisplaynum = 0;
        }
    }

    // Blink LaunchPad Red LED
    GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;

    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void)
{


    CpuTimer1.InterruptCount++;
}

// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void)
{


    // Blink LaunchPad Blue LED
    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;

    CpuTimer2.InterruptCount++;

    if ((CpuTimer2.InterruptCount % 50) == 0) {
        UARTPrint = 1;
    }
}


// This function is called each time a char is recieved over UARTA.
void serialRXA(serial_t *s, char data) {
    numRXA ++;
    if (data == 'q') {
        turn = turn + 0.05;
    }
    else if (data == 'r') {
        turn = turn - 0.05;
    }
    else if (data == '3') {
        Vref = Vref + 0.1;
    }
    else if (data == 's') {
        Vref = Vref - 0.1;
    }
    else {
        turn = 0;
        Vref = 0;
    }
}


//xk is the current ADC reading, xk_1 is the ADC reading one millisecond ago, xk_2 two milliseconds ago, etc
//float xk = 0;
//float xk_1 = 0;
//float xk_2 = 0;
//float xk_3 = 0;
//float xk_4 = 0;
//adcd1 pie interrupt
__interrupt void ADCD_ISR(void) {


    adcd0result = AdcdResultRegs.ADCRESULT0;
    adcd1result = AdcdResultRegs.ADCRESULT1;

    // Here covert ADCIND0, ADCIND1 to volts
    //Reads the ADCD results and converts to voltage
    var1 = adcd0result/4095.0*3.0; //left IR Sensor
    var2 = adcd1result/4095.0*3.0; //front IR Sensor

    //Reading wheel angles and correcting signs
    RightWheel = readEncRight();
    LeftWheel = -readEncLeft();

    //Using a Factor of 1/9.5 to convert from radians to ft
    RightWheelDistance = RightWheel/9.5;
    LeftWheelDistance = LeftWheel/9.5;

    //setting desired thresholds for the Left and Front IR Sensors
    LeftDistance = 3.0 - var1;
    LeftDesired = 2.0;
    LeftReturn = 2.5;
    FrontDistance = 3.0 - var2;
    FrontThresh = 2.0;
    FrontReturn = 2.5;

    //Calculating Velocities for each wheel
    VRightK = (RightWheelDistance - RightDist_K_1)/0.004;
    VLeftK = (LeftWheelDistance - LeftDist_K_1)/0.004;

    //calculating errors
    e_left = Vref - VLeftK;
    e_right = Vref - VRightK;
    e_distance = LeftDesired - LeftDistance;

    //calculating moment of inertia
    I_left = I_left_K_1 + .004*((e_left + e_left_K_1)/2);
    I_right = I_right_K_1 + .004*((e_right + e_right_K_1)/2);

    //calculating uleft and uright using new error calculations
    uleft = Kp*e_turn_left + Ki*I_left;
    uright = Kp*e_turn_right + Ki*I_right;

    //state machine to switch from different avoid states
    if (avoidstate == 0){
        //state machine to move from one position to the next using xy control
        if(nextmove == 0){
            xy_control(&Vref, &turn, 2, Xr, Yr, 0, 3, phi_r, 0.5, 0.75);
            if(xy_control(&Vref, &turn, 2, Xr, Yr, 0, 3, phi_r, 0.5, 0.75) == 1){
                nextmove = 1;
            }
        }
        else if (nextmove == 1){
            xy_control(&Vref, &turn, 2, Xr, Yr, 0, 4, phi_r, 0.5, 0.75);
            if(xy_control(&Vref, &turn, 2, Xr, Yr, 0, 4, phi_r, 0.5, 0.75) == 1){
                nextmove = 2;
            }
        }
        else if (nextmove == 2){
            xy_control(&Vref, &turn, 2, Xr, Yr, 2, 4, phi_r, 0.5, 0.75);
        }


        if(LeftDistance < LeftDesired){
            avoidstate = 1;
        }
        else if(FrontDistance < FrontThresh){
            avoidstate = 2;
        }
    }
    else if(avoidstate == 1){
        //controls the segbot to left wall follow if the left IR sensor senses an obstacle
        turn = e_distance * Kp_left_wall;
        Vref = 0.25;
        if(LeftDistance >= LeftDesired && FrontDistance > FrontReturn){
            //delay of 1 second before switching to xy control state
            followCount++;
            if(LeftDistance >= LeftDesired && FrontDistance > FrontReturn && followCount >= 250){
                avoidstate =0;
                followCount = 0;
            }
        }
        else if(FrontDistance < FrontThresh){
            avoidstate = 2;
        }
    }
    else if(avoidstate == 2){
        //controls the segbot to turn away to the right if sensing an obstacle directly in front
        turn = Kp_front * (3 - FrontDistance);
        Vref = 0.25;
        if(LeftDistance >= LeftDesired && FrontDistance > FrontReturn){
            //delay of 1 second before switching to xy control state
            followCount++;
            if(LeftDistance >= LeftDesired && FrontDistance > FrontReturn && followCount >= 250){
                avoidstate =0;
                followCount = 0;
            }
        }
        else if(LeftDistance < LeftDesired){
            avoidstate = 1;
        }
    }
    e_turn = turn + (VLeftK - VRightK);
    e_turn_left = Vref -VLeftK - (Kp_turn*e_turn);
    e_turn_right = Vref -VRightK + (Kp_turn*e_turn);

    //saturating between -10 and 10
    if(fabs(uright) > 10){
        I_right = I_right_K_1;
    }
    if(fabs(uleft) > 10){
        I_left = I_left_K_1;
    }

    setEPWM2B(-uleft);
    setEPWM2A(uright);

    //updating previous variables
    I_left_K_1 = I_left;
    I_right_K_1 = I_right;

    e_left_K_1 = e_turn_left;
    e_right_K_1 = e_turn_right;

    RightDist_K_1 = RightWheelDistance;
    LeftDist_K_1 = LeftWheelDistance;

    //calculating robot pose to be used in xy control
    //allows us to track the position of the robot and see whether it is going to the desired positions or not
    theta_dot_right = VRightK*9.5;
    theta_dot_left = VLeftK*9.5;
    phi_r = (Rwh/Wr)*(RightWheel-LeftWheel);

    theta_dot_avg = .5*(theta_dot_right + theta_dot_left);

    X_dot_r = Rwh * theta_dot_avg*cos(phi_r);
    Y_dot_r = Rwh * theta_dot_avg*sin(phi_r);

    //integrating Xdot and Ydot to get x and y coordinates
    Xr = X_K_1 + 0.004*(X_dot_r + X_dot_K_1)/2;
    Yr = Y_K_1 + 0.004*(Y_dot_r + Y_dot_K_1)/2;

    //updating previous variables
    X_K_1 = Xr;
    Y_K_1 = Yr;
    X_dot_K_1 = X_dot_r;
    Y_dot_K_1 = Y_dot_r;

    if(count % 10 == 0){
        UARTPrint = 1;
    }
    count++;

    AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

void init_eQEPs(void) {
    // setup eQEP1 pins for input
    EALLOW;
    //Disable internal pull-up for the selected output pins for reduced power consumption
    GpioCtrlRegs.GPAPUD.bit.GPIO20 = 1; // Disable pull-up on GPIO20 (EQEP1A)
    GpioCtrlRegs.GPAPUD.bit.GPIO21 = 1; // Disable pull-up on GPIO21 (EQEP1B)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO20 = 2; // Qual every 6 samples
    GpioCtrlRegs.GPAQSEL2.bit.GPIO21 = 2; // Qual every 6 samples
    EDIS;
    // This specifies which of the possible GPIO pins will be EQEP1 functional pins.
    // Comment out other unwanted lines.
    GPIO_SetupPinMux(20, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinMux(21, GPIO_MUX_CPU1, 1);
    EQep1Regs.QEPCTL.bit.QPEN = 0; // make sure eqep in reset
    EQep1Regs.QDECCTL.bit.QSRC = 0; // Quadrature count mode
    EQep1Regs.QPOSCTL.all = 0x0; // Disable eQep Position Compare
    EQep1Regs.QCAPCTL.all = 0x0; // Disable eQep Capture
    EQep1Regs.QEINT.all = 0x0; // Disable all eQep interrupts
    EQep1Regs.QPOSMAX = 0xFFFFFFFF; // use full range of the 32 bit count
    EQep1Regs.QEPCTL.bit.FREE_SOFT = 2; // EQep uneffected by emulation suspend in Code Composer
    EQep1Regs.QPOSCNT = 0;
    EQep1Regs.QEPCTL.bit.QPEN = 1; // Enable EQep

    // setup QEP2 pins for input
    EALLOW;
    //Disable internal pull-up for the selected output pinsfor reduced power consumption
    GpioCtrlRegs.GPBPUD.bit.GPIO54 = 1; // Disable pull-up on GPIO54 (EQEP2A)
    GpioCtrlRegs.GPBPUD.bit.GPIO55 = 1; // Disable pull-up on GPIO55 (EQEP2B)
    GpioCtrlRegs.GPBQSEL2.bit.GPIO54 = 2; // Qual every 6 samples
    GpioCtrlRegs.GPBQSEL2.bit.GPIO55 = 2; // Qual every 6 samples
    EDIS;
    GPIO_SetupPinMux(54, GPIO_MUX_CPU1, 5); // set GPIO54 and eQep2A
    GPIO_SetupPinMux(55, GPIO_MUX_CPU1, 5); // set GPIO54 and eQep2B
    EQep2Regs.QEPCTL.bit.QPEN = 0; // make sure qep reset
    EQep2Regs.QDECCTL.bit.QSRC = 0; // Quadrature count mode
    EQep2Regs.QPOSCTL.all = 0x0; // Disable eQep Position Compare
    EQep2Regs.QCAPCTL.all = 0x0; // Disable eQep Capture
    EQep2Regs.QEINT.all = 0x0; // Disable all eQep interrupts
    EQep2Regs.QPOSMAX = 0xFFFFFFFF; // use full range of the 32 bit count.
    EQep2Regs.QEPCTL.bit.FREE_SOFT = 2; // EQep uneffected by emulation suspend
    EQep2Regs.QPOSCNT = 0;
    EQep2Regs.QEPCTL.bit.QPEN = 1; // Enable EQep
}

float readEncLeft(void) {
    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU; //4294967295U
    raw = EQep1Regs.QPOSCNT;
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue; // I don't think this is needed and never true
    // 5 North South magnet poles in the encoder disk so 5 square waves per one revolution of the
    // DC motor's back shaft. Then Quadrature Decoder mode multiplies this by 4 so 20 counts per one rev
    // of the DC motor's back shaft. Then the gear motor's gear ratio is 30:1.
    return (raw*(TWOPI/600));
}
float readEncRight(void) {
    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU; //4294967295U -1 32bit signed int
    raw = EQep2Regs.QPOSCNT;
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue; // I don't think this is needed and never true
    // 5 North South magnet poles in the encoder disk so 5 square waves per one revolution of the
    // DC motor's back shaft. Then Quadrature Decoder mode multiplies this by 4 so 20 counts per one rev
    // of the DC motor's back shaft. Then the gear motor's gear ratio is 30:1.
    return (raw*(TWOPI/600));
}

void setEPWM2A(float controleffort){
    //saturates between 10 and -10
    if(controleffort > 10){
        controleffort = 10;
    }
    if(controleffort < -10){
        controleffort = -10;
    }

    //equation to update CMPA values so that any number can be entered
    EPwm2Regs.CMPA.bit.CMPA = (int)((controleffort)*(2500.0/20.0) + 1250);

}

void setEPWM2B(float controleffort){
    //saturates between 10 and -10
    if(controleffort > 10){
        controleffort = 10;
    }
    if(controleffort < -10){
        controleffort = -10;
    }
    //equation to update CMPB values so that any number can be entered
    EPwm2Regs.CMPB.bit.CMPB = (int)((controleffort)*(2500.0/20.0) + 1250);

}

//returns an angle to be used in xy control
float my_atanf(float dy, float dx)
{
    float ang;

    if (fabsf(dy) <= 0.001F) {
        if (dx >= 0.0F) {
            ang = 0.0F;
        } else {
            ang = PI;
        }
    } else if (fabsf(dx) <= 0.001F) {
        if (dy > 0.0F) {
            ang = HALFPI;
        } else {
            ang = -HALFPI;
        }
    } else {
        ang = atan2f(dy,dx);
    }
    return ang;
}

//function to move the segbot from position to the next
int xy_control(float *vref_forxy, float *turn_forxy,float turn_thres, float x_pos,float y_pos,float x_desired,float y_desired,
                float thetaabs,float target_radius,float target_radius_near)
{
    float dx,dy,alpha;
    float dist = 0.0F;
    float dir;
    float theta;
    int target_near = false;
    float turnerror = 0;

        // calculate theta (current heading) between -PI and PI
    if (thetaabs > PI) {
        theta = thetaabs - 2.0*PI*floorf((thetaabs+PI)/(2.0*PI));
    } else if (thetaabs < -PI) {
        theta = thetaabs - 2.0*PI*ceilf((thetaabs-PI)/(2.0*PI));
    } else {
        theta = thetaabs;
    }

    dx = x_desired - x_pos;
    dy = y_desired - y_pos;
    dist = sqrtf( dx*dx + dy*dy );
    dir = 1.0F;

    // calculate alpha (trajectory angle) between -PI and PI
    alpha = my_atanf(dy,dx);

    // calculate turn error
    //  turnerror = theta - alpha;  old way using left hand coordinate system.
    turnerror = alpha - theta;

    // check for shortest path
    if (fabsf(turnerror + 2.0*PI) < fabsf(turnerror)) turnerror += 2.0*PI;
    else if (fabsf(turnerror - 2.0*PI) < fabsf(turnerror)) turnerror -= 2.0*PI;

    if (dist < target_radius_near) {
        target_near = true;
        // Arrived to the target's (X,Y)
        if (dist < target_radius) {
            dir = 0.0F;
            turnerror = 0.0F;
        } else {
            // if we overshot target, we must change direction. This can cause the robot to bounce back and forth when
            // remaining at a point.
            if (fabsf(turnerror) > HALFPI) {
                dir = -dir;
            }
            turnerror = 0;
        }
    } else {
        target_near = false;
    }

    // vref is 0.3 tile/sec; but slower when close to target.
    *vref_forxy = dir*fmin(dist, .3);

    if (fabsf(*vref_forxy) > 0.0) {
        // if robot 1 tile away from target use a scaled KP value.
        *turn_forxy = (*vref_forxy*1)*turnerror;
    } else {
        // normally use a Kp gain of 2 (changed to 1 for this project)
        *turn_forxy = 1*turnerror;
    }

    // This helps with unbalanced wheel slipping.  If turn value greater than
    // turn_thres then just spin in place
    if (fabsf(*turn_forxy) > turn_thres) {
        *vref_forxy = 0;
    }
    return(target_near);
}

