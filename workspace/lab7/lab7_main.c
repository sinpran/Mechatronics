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
__interrupt void SPIB_isr(void);
__interrupt void ADCA_ISR(void);
void setupSPIB(void);

void serialRXA(serial_t *s, char data);
float readEncRight(void);
float readEncLeft(void);

void setEPWM2A(float controleffort);
void setEPWM2B(float controleffort);

void init_eQEPs(void);

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numTimer1calls = 0;
uint32_t numSWIcalls = 0;
uint32_t numRXA = 0;
uint16_t UARTPrint = 0;
uint16_t LEDdisplaynum = 0;
uint16_t rcservo = 0;
uint16_t rcservo_flag = 0;
uint16_t value = 0;
uint16_t ADC1reading;
uint16_t ADC2reading;
uint16_t ADC1;
uint16_t ADC2;
float ADC1_reading;
float ADC2_reading;
uint16_t ADC;

int16_t gyroX = 0;
int16_t gyroY = 0;
int16_t gyroZ = 0;
float gyroXG = 0;
float gyroYG = 0;
float gyroZG = 0;
int16_t accelX = 0;
int16_t accelY = 0;
int16_t accelZ = 0;
float accelXG = 0;
float accelYG = 0;
float accelZG = 0;

float LeftWheel = 0;
float RightWheel = 0;
float LeftWheel_1 = 0;
float RightWheel_1 = 0;
float RightDistance = 0;
float LeftDistance = 0;

float uleft = 5.0;
float uright = 5.0;
float VLeftK = 0;
float VLeftK_1 = 0;
float VRightK_1 = 0;
float VRightK = 0;
float Vref = 1;

float RightDist_K_1 = 0;
float LeftDist_K_1 = 0;

float Kp = 3;
float Ki = 20;

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

float var1 = 0;
float var2 = 0;
int32_t count = 0;

int16_t adca2result = 0;
int16_t adca3result = 0;

int16_t adcb0result = 0;

float xk[101] = {0};
float xk2[101] = {0};
//yk is the filtered value
float yk = 0;
float yk2 = 0;

float left_motor_angle = 0;
float right_motor_angle = 0;
uint32_t SPIB_count = 0;

// Needed global Variables
float accelx_offset = 0;
float accely_offset = 0;
float accelz_offset = 0;
float gyrox_offset = 0;
float gyroy_offset = 0;
float gyroz_offset = 0;
float accelzBalancePoint = -.76;
int16 IMU_data[9];
uint16_t temp=0;
int16_t doneCal = 0;
float tilt_value = 0;
float tilt_array[4] = {0, 0, 0, 0};
float gyro_value = 0;
float gyro_array[4] = {0, 0, 0, 0};
float LeftWheelArray[4] = {0,0,0,0};
float RightWheelArray[4] = {0,0,0,0};
// Kalman Filter vars
float T = 0.001; //sample rate, 1ms
float Q = 0.01; // made global to enable changing in runtime
float R = 25000;//50000;
float kalman_tilt = 0;
float kalman_P = 22.365;
int16_t SpibNumCalls = -1;
float pred_P = 0;
float kalman_K = 0;
int32_t timecount = 0;
int16_t calibration_state = 0;
int32_t calibration_count = 0;

//exercise 3 variables
float K1 = -60;
float K2 = -4.5;
float K3 = -1.1;
float Ubal = 0;
float vel_right=0;
float vel_left=0;
float vel_right_1=0;
float vel_left_1=0;

//exercise 4 variables
float WhlDiff = 0;
float WhlDiff_1 = 0;
float vel_WhlDiff = 0;
float vel_WhlDiff_1 = 0;
float turnref =0;
float turnref_1 =0;
float errDiff = 0;
float errDiff_1 = 0;
float intDiff = 0;
float intDiff_1 = 0;
float Kd = 0.08;
float FwdBackOffset = 0;
float turnrate =0;
float turnrate_1 =0;

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
    PieVectTable.SPIB_RX_INT = &SPIB_isr;

    PieVectTable.ADCA1_INT = &ADCA_ISR;
    PieVectTable.EMIF_ERROR_INT = &SWI_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers


    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every second:
    // 200MHz CPU Freq, 1 second Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, 200, 1000);
    ConfigCpuTimer(&CpuTimer1, 200, 4000);
    ConfigCpuTimer(&CpuTimer2, 200, 40000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    init_serial(&SerialA,115200,serialRXA);
//    init_serial(&SerialC,115200,serialRXC);
//    init_serial(&SerialD,115200,serialRXD);
    setupSPIB();
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
    EPwm5Regs.TBCTL.bit.CLKDIV = 0; // divide by 1 50Mhz Clock
    EPwm5Regs.TBPRD = 50000; // Set Period to 1ms sample. Input clock is 50MHz.
    // Notice here that we are not setting CMPA or CMPB because we are not using the PWM signal
    EPwm5Regs.ETSEL.bit.SOCAEN = 1; //enable SOCA
    EPwm5Regs.TBCTL.bit.CTRMODE = 0; //unfreeze, and enter up count mode
    EDIS;

    EALLOW;
    //write configurations for all ADCs ADCA, ADCB, ADCC, ADCD
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    //Set pulse positions to late
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    //power up the ADCs
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
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
    EDIS;

//    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0); // Set as GPIO9 and used as DAN28027 SS
//    GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL); // Make GPIO9 an Output Pin
//    GpioDataRegs.GPASET.bit.GPIO9 = 1; //Initially Set GPIO9/SS High so DAN28027 is not selected
//    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0); // Set as GPIO66 and used as MPU-9250 SS
//    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL); // Make GPIO66 an Output Pin
//    GpioDataRegs.GPCSET.bit.GPIO66 = 1; //Initially Set GPIO66/SS High so MPU-9250 is not selected
//    GPIO_SetupPinMux(63, GPIO_MUX_CPU1, 15); //Set GPIO63 pin to SPISIMOB
//    GPIO_SetupPinMux(64, GPIO_MUX_CPU1, 15); //Set GPIO64 pin to SPISOMIB
//    GPIO_SetupPinMux(65, GPIO_MUX_CPU1, 15); //Set GPIO65 pin to SPICLKB
    EALLOW;
    GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0; // Enable Pull-ups on SPI PINs Recommended by TI for SPI Pins
    GpioCtrlRegs.GPCPUD.bit.GPIO64 = 0;
    GpioCtrlRegs.GPCPUD.bit.GPIO65 = 0;
    GpioCtrlRegs.GPBQSEL2.bit.GPIO63 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    GpioCtrlRegs.GPCQSEL1.bit.GPIO64 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    GpioCtrlRegs.GPCQSEL1.bit.GPIO65 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    EDIS;
    // ---------------------------------------------------------------------------
//    SpibRegs.SPICCR.bit.SPISWRESET = 0; // Put SPI in Reset
//    SpibRegs.SPICTL.bit.CLK_PHASE = 1; //This happens to be the mode for both the DAN28027 and
//    SpibRegs.SPICCR.bit.CLKPOLARITY = 0; //The MPU-9250, Mode 01.
//    SpibRegs.SPICTL.bit.MASTER_SLAVE = 1; // Set to SPI Master
//    SpibRegs.SPICCR.bit.SPICHAR = 0xF; // Set to transmit and receive 16 bits each write to SPITXBUF
//    SpibRegs.SPICTL.bit.TALK = 1; // Enable transmission
//    SpibRegs.SPIPRI.bit.FREE = 1; // Free run, continue SPI operation
//    SpibRegs.SPICTL.bit.SPIINTENA = 0; // Disables the SPI interrupt
//    SpibRegs.SPIBRR.bit.SPI_BIT_RATE = 49; // Set SCLK bit rate to 1 MHz so 1us period. SPI base clock is
//     // 50MHZ. And this setting divides that base clock to create SCLK’s period
//    SpibRegs.SPISTS.all = 0x0000; // Clear status flags just in case they are set for some reasonSE423 4 Lab #5
//    SpibRegs.SPIFFTX.bit.SPIRST = 1;// Pull SPI FIFO out of reset, SPI FIFO can resume transmit or receive.
//    SpibRegs.SPIFFTX.bit.SPIFFENA = 1; // Enable SPI FIFO enhancements
//    SpibRegs.SPIFFTX.bit.TXFIFO = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
//    SpibRegs.SPIFFTX.bit.TXFFINTCLR = 1; // Write 1 to clear SPIFFTX[TXFFINT] flag just in case it is set
//    SpibRegs.SPIFFRX.bit.RXFIFORESET = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
//    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Write 1 to clear SPIFFRX[RXFFOVF] just in case it is set
//    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1; // Write 1 to clear SPIFFRX[RXFFINT] flag just in case it is set
//    SpibRegs.SPIFFRX.bit.RXFFIENA = 1; // Enable the RX FIFO Interrupt. RXFFST >= RXFFIL
//    SpibRegs.SPIFFCT.bit.TXDLY = 16; //Set delay between transmits to 16 spi clocks. Needed by DAN28027 chip
//    SpibRegs.SPICCR.bit.SPISWRESET = 1; // Pull the SPI out of reset
//    SpibRegs.SPIFFTX.bit.TXFIFO = 1; // Release transmit FIFO from reset.
//    SpibRegs.SPIFFRX.bit.RXFIFORESET = 1; // Re-enable receive FIFO operation
//    SpibRegs.SPICTL.bit.SPIINTENA = 1; // Enables SPI interrupt. !! I don’t think this is needed. Need to Test
//    SpibRegs.SPIFFRX.bit.RXFFIL = 10; //Interrupt Level to 16 words or more received into FIFO causes interrupt. This is just the initial setting for the register. Will be changed below


    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.  
    IER |= M_INT1;
    IER |= M_INT8;  // SCIC SCID
    IER |= M_INT9;  // SCIA
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;

    IER |= M_INT6;

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    // Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;
    // Enable SPIB_RX in the PIE: Group 6 interrupt 3
    PieCtrlRegs.PIEIER6.bit.INTx3 = 1;

    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;

    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    
    // IDLE loop. Just sit and loop forever (optional):
    while(1)
    {
        if (UARTPrint == 1 ) {
            //serial_printf(&SerialA,"AccelX: %.3f AccelY: %.3f AccelZ: %.3f GyroX: %.3f GyroY: %.3f GyroZ: %.3f\r\n", accelXG, accelYG, accelZG, gyroXG, gyroYG, gyroZG);
            //serial_printf(&SerialA,"RightWheel: %.3f LeftWheel: %.3f RightDistance: %.3f LeftDistance: %.3f VRight: %.3f Vleft: %.3f\r\n",RightWheel,LeftWheel, RightDistance, LeftDistance, VRightK, VLeftK);
            //serial_printf(&SerialA,"Xr: %.3f Yr: %.3f Phi: %.3f\r\n",Xr, Yr, phi_r);
            //serial_printf(&SerialA,"Joystick 1: %.3f Joystick 2: %.3f AccelZ: %.3f GyroX: %.3f Left Motor Angle: %.3f Right Motor Angle: %.3f\r\n",var1, var2, accelZG, gyroXG, left_motor_angle, right_motor_angle);
            serial_printf(&SerialA,"tilt_value: %.3f gyro_value: %.3f LeftWheel: %.3f RightWheel: %.3f\r\n",tilt_value, gyro_value, LeftWheel, RightWheel);
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

    //exercise 4
    //code to steer the segbot
    WhlDiff = LeftWheel - RightWheel;
    vel_WhlDiff = 0.333*vel_WhlDiff_1 + 166.667*(WhlDiff - WhlDiff_1); //using the transfer function given
    turnref = turnref_1 + (turnrate + turnrate_1)*0.004/2; //integrating using trapezoidal rule
    errDiff = turnref - WhlDiff;
    intDiff = intDiff_1 + (errDiff+errDiff_1)*0.004/2; //integrating using trapezoidal rule

    turn = Kp*errDiff + Ki*intDiff - Kd*vel_WhlDiff;

    if(fabs(turn >= 3))
        intDiff = intDiff_1;
    //saturating between -4 to 4
    if(turn >= 4)
        turn = 4;
    if(turn <= -4)
        turn = -4;

    //exercise 3
    //code to balance the segbot
    //using the transfer function given
    vel_right = 0.6*vel_right_1 + 100 *(RightWheel - RightWheel_1);
    vel_left = 0.6*vel_left_1 + 100 *(LeftWheel - LeftWheel_1);

    Ubal = -(K1*tilt_value) - (K2*gyro_value) - (K3*(vel_left + vel_right)/2);
    uleft = Ubal/2 + turn + FwdBackOffset;
    uright = Ubal/2 - turn + FwdBackOffset;

    setEPWM2B(-uleft);
    setEPWM2A(uright);

    //updating the old variables
    vel_right_1 = vel_right;
    vel_left_1 = vel_left;
    RightWheel_1 = RightWheel;
    LeftWheel_1 = LeftWheel;
    WhlDiff_1 = WhlDiff;
    vel_WhlDiff_1 = vel_WhlDiff;
    errDiff_1 = errDiff;
    intDiff_1 = intDiff;
    turnref_1 = turnref;
    turnrate_1 = turnrate;

    numSWIcalls++;
    DINT;

}

// cpu_timer0_isr - CPU Timer0 ISR
__interrupt void cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;

    numTimer0calls++;
//
//    if ((numTimer0calls%50) == 0) {
//        PieCtrlRegs.PIEIFR12.bit.INTx9 = 1;  // Manually cause the interrupt for the SWI
//    }
////
////    //Clear GPIO9 Low to act as a Slave Select. Right now, just to scope. Later to select DAN28027 chip
////    GpioDataRegs.GPACLEAR.bit.GPIO9 = 1;
////    SpibRegs.SPIFFRX.bit.RXFFIL = 2; // Issue the SPIB_RX_INT when two values are in the RX FIFO
////    SpibRegs.SPITXBUF = 0x4A3B; // 0x4A3B and 0xB517 have no special meaning. Wanted to send
////    SpibRegs.SPITXBUF = 0xB517; // something so you can see the pattern on the Oscilloscope
////
////    if ((numTimer0calls%250) == 0) {
////        displayLEDletter(LEDdisplaynum);
////        LEDdisplaynum++;
////        if (LEDdisplaynum == 0xFFFF) {  // prevent roll over exception
////            LEDdisplaynum = 0;
////        }
////    }
//
//    if(rcservo_flag == 0){
//       if(value < 4500)
//           value += 10;
//       else
//           rcservo_flag = 1;
//    }
//    else{
//       if(value > 1500)
//           value -= 10;
//       else
//           rcservo_flag = 0;
//    }
//
//    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
//    SpibRegs.SPIFFRX.bit.RXFFIL = 8;
//    SpibRegs.SPITXBUF = ((0x8000) | (0x3A00));
//    SpibRegs.SPITXBUF = 0;
//    SpibRegs.SPITXBUF = 0;
//    SpibRegs.SPITXBUF = 0;
//    SpibRegs.SPITXBUF = 0;
//    SpibRegs.SPITXBUF = 0;
//    SpibRegs.SPITXBUF = 0;
//    SpibRegs.SPITXBUF = 0;
////
////  // Blink LaunchPad Red LED
//    GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
//
//    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

    if ((CpuTimer0.InterruptCount % 200) == 0) {
       UARTPrint = 1;
    }
}


// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void)
{
    CpuTimer1.InterruptCount++;

    numTimer1calls++;

//    RightWheel = readEncRight();
//    LeftWheel = -readEncLeft();
//
//    RightDistance = RightWheel/9.55;
//    LeftDistance = LeftWheel/9.257;
//
//    VRightK = (RightDistance - RightDist_K_1)/0.004;
//    VLeftK = (LeftDistance - LeftDist_K_1)/0.004;
//
//    e_left = Vref - VLeftK;
//    e_right = Vref - VRightK;
//
//    I_left = I_left_K_1 + .004*((e_left + e_left_K_1)/2);
//    I_right = I_right_K_1 + .004*((e_right + e_right_K_1)/2);
//
//    e_turn = turn + (VLeftK - VRightK);
//    e_turn_left = Vref -VLeftK - (Kp_turn*e_turn);
//    e_turn_right = Vref -VRightK + (Kp_turn*e_turn);
//
//    uleft = Kp*e_turn_left + Ki*I_left;
//    uright = Kp*e_turn_right + Ki*I_right;
//
//    if(fabs(uright) > 10){
//        I_right = I_right_K_1;
//    }
//    if(fabs(uleft) > 10){
//        I_left = I_left_K_1;
//    }
//
//    setEPWM2B(-uleft);
//    setEPWM2A(uright);
//
//    I_left_K_1 = I_left;
//    I_right_K_1 = I_right;
//
//    e_left_K_1 = e_turn_left;
//    e_right_K_1 = e_turn_right;
//
//    RightDist_K_1 = RightDistance;
//    LeftDist_K_1 = LeftDistance;
//
//
//    //exercise 5 code
//    theta_dot_right = VRightK*9.55;
//    theta_dot_left = VLeftK*9.257;
//    phi_r = (Rwh/Wr)*(RightWheel-LeftWheel);
//
//    theta_dot_avg = .5*(theta_dot_right + theta_dot_left);
//
//    X_dot_r = Rwh * theta_dot_avg*cos(phi_r);
//    Y_dot_r = Rwh * theta_dot_avg*sin(phi_r);
//
//    Xr = X_K_1 + 0.004*(X_dot_r + X_dot_K_1)/2;
//    Yr = Y_K_1 + 0.004*(Y_dot_r + Y_dot_K_1)/2;
//
//    X_K_1 = Xr;
//    Y_K_1 = Yr;
//    X_dot_K_1 = X_dot_r;
//    Y_dot_K_1 = Y_dot_r;
//
//    if ((CpuTimer1.InterruptCount % 4) == 0) {
//
//       UARTPrint = 1;
//    }
}

// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void)
{

//
//  // Blink LaunchPad Blue LED
    //GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;
//
    CpuTimer2.InterruptCount++;
//
//  if ((CpuTimer2.InterruptCount % 50) == 0) {
//      UARTPrint = 1;
//  }
}


// This function is called each time a char is recieved over UARTA.
void serialRXA(serial_t *s, char data) {
    //copied from lab manual
    numRXA ++;
    if (data == 'q') {
        turnrate = turnrate - 0.2;
    } else if (data == 'r') {
        turnrate = turnrate + 0.2;
    } else if (data == '3') {
        FwdBackOffset = FwdBackOffset - 0.2;
    } else if (data == 's') {
        FwdBackOffset = FwdBackOffset + 0.2;
    } else {
        turnrate = 0;
        FwdBackOffset = 0;
    }
}

int16_t spivalue1 = 0;
int16_t spivalue2 = 0;
__interrupt void SPIB_isr(void){

    SPIB_count++;
//    ADC = SpibRegs.SPIRXBUF;
//    ADC1 = SpibRegs.SPIRXBUF;
//    ADC2 = SpibRegs.SPIRXBUF;

    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Set GPIO 9 high to end Slave Select. Now to Scope. Later to deselect DAN28027

    spivalue1 = SpibRegs.SPIRXBUF; // Read first 16 bit value off RX FIFO. Probably is zero since no chip
//    spivalue2 = SpibRegs.SPIRXBUF; // Read second 16 bit value off RX FIFO. Again probably zero
//    spivalue3 = SpibRegs.SPIRXBUF;
//
//
//    ADC1_reading = spivalue2 * (3.3/4095);
//    ADC2_reading = spivalue3 * (3.3/4095);


    accelX = SpibRegs.SPIRXBUF;
    accelY = SpibRegs.SPIRXBUF;
    accelZ = SpibRegs.SPIRXBUF;
    accelXG = accelX*4.0/32767.0;
    accelYG = accelY*4.0/32767.0;
    accelZG = accelZ*4.0/32767.0;

    spivalue2 = SpibRegs.SPIRXBUF;

    gyroX = SpibRegs.SPIRXBUF;
    gyroY = SpibRegs.SPIRXBUF;
    gyroZ = SpibRegs.SPIRXBUF;
    gyroXG = gyroX*250.0/32767.0;
    gyroYG = gyroY*250.0/32767.0;
    gyroZG = gyroZ*250.0/32767.0;

    //Code to be copied into SPIB_ISR interrupt function after the IMU measurements have been collected.
    //exercise 2
    if(calibration_state == 0){
        calibration_count++;
        if (calibration_count == 2000) {
            calibration_state = 1;
            calibration_count = 0;
        }
    } else if(calibration_state == 1){
        accelx_offset+=accelXG;
        accely_offset+=accelYG;
        accelz_offset+=accelZG;
        gyrox_offset+=gyroXG;
        gyroy_offset+=gyroYG;
        gyroz_offset+=gyroZG;
        calibration_count++;
        if (calibration_count == 2000) {
            calibration_state = 2;
            accelx_offset/=2000.0;
            accely_offset/=2000.0;
            accelz_offset/=2000.0;
            gyrox_offset/=2000.0;
            gyroy_offset/=2000.0;
            gyroz_offset/=2000.0;
            calibration_count = 0;
            doneCal = 1;
        }
    } else if(calibration_state == 2){
        accelXG -=(accelx_offset);
        accelYG -=(accely_offset);
        accelZG -=(accelz_offset-accelzBalancePoint);
        gyroXG -= gyrox_offset;
        gyroYG -= gyroy_offset;
        gyroZG -= gyroz_offset;
        /*--------------Kalman Filtering code start---------------------------------------------------------------------*/
        float tiltrate = (gyroXG*PI)/180.0; // rad/s
        float pred_tilt, z, y, S;
        // Prediction Step
        pred_tilt = kalman_tilt + T*tiltrate;
        pred_P = kalman_P + Q;
        // Update Step
        z = -accelZG; // Note the negative here due to the polarity of AccelZ
        y = z - pred_tilt;
        S = pred_P + R;
        kalman_K = pred_P/S;
        kalman_tilt = pred_tilt + kalman_K*y;
        kalman_P = (1 - kalman_K)*pred_P;
        SpibNumCalls++;
        // Kalman Filter used
        tilt_array[SpibNumCalls] = kalman_tilt;
        gyro_array[SpibNumCalls] = tiltrate;
        LeftWheelArray[SpibNumCalls] = -readEncLeft();
        RightWheelArray[SpibNumCalls] = readEncRight();
        if (SpibNumCalls >= 3) { // should never be greater than 3
            tilt_value = (tilt_array[0] + tilt_array[1] + tilt_array[2] + tilt_array[3])/4.0;
            gyro_value = (gyro_array[0] + gyro_array[1] + gyro_array[2] + gyro_array[3])/4.0;
            LeftWheel=(LeftWheelArray[0]+LeftWheelArray[1]+LeftWheelArray[2]+LeftWheelArray[3])/4.0;
            RightWheel=(RightWheelArray[0]+RightWheelArray[1]+RightWheelArray[2]+RightWheelArray[3])/4.0;
            SpibNumCalls = -1;
            PieCtrlRegs.PIEIFR12.bit.INTx9 = 1; // Manually cause the interrupt for the SWI
        }
    }
    timecount++;
    if((timecount%200) == 0)
    {
        if(doneCal == 0) {
            GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1; // Blink Blue LED while calibrating
        }
        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1; // Always Block Red LED
        UARTPrint = 1; // Tell While loop to print
    }
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR=1; // Clear Overflow flag
    SpibRegs.SPIFFRX.bit.RXFFINTCLR=1; // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;


//    exercise 1
    //working on gyros and polarities
//    left_motor_angle = -readEncLeft();
//    right_motor_angle = readEncRight();
//
//    setEPWM2B(-uleft);
//    setEPWM2A(uright);
//
//    if(SPIB_count % 200 == 0){
//        UARTPrint = 1;
//    }
//
//
//    // Later when actually communicating with the DAN28027 do something with the data. Now do nothing.
//    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Clear Overflow flag just in case of an overflow
//    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1; // Clear RX FIFO Interrupt flag so next interrupt will happen
//    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6; // Acknowledge INT6 PIE interrupt
}

void setupSPIB(void) //Call this function in main() somewhere after the DINT; line of code.
{
    int16_t temp = 0;
    //Step 1.
    //cut and paste here all the SpibRegs initializations you found for part 3. Make sure the TXdelay in
    //between each transfer to 0. Also don’t forget to cut and paste the GPIO settings for GPIO9, 63, 64, 65,
    //66 which are also a part of the SPIB setup.

    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0); // Set as GPIO9 and used as DAN28027 SS
    GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL); // Make GPIO9 an Output Pin
    GpioDataRegs.GPASET.bit.GPIO9 = 1; //Initially Set GPIO9/SS High so DAN28027 is not selected
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0); // Set as GPIO66 and used as MPU-9250 SS
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL); // Make GPIO66 an Output Pin
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; //Initially Set GPIO66/SS High so MPU-9250 is not selected
    GPIO_SetupPinMux(63, GPIO_MUX_CPU1, 15); //Set GPIO63 pin to SPISIMOB
    GPIO_SetupPinMux(64, GPIO_MUX_CPU1, 15); //Set GPIO64 pin to SPISOMIB
    GPIO_SetupPinMux(65, GPIO_MUX_CPU1, 15); //Set GPIO65 pin to SPICLKB


    SpibRegs.SPICCR.bit.SPISWRESET = 0; // Put SPI in Reset
    SpibRegs.SPICTL.bit.CLK_PHASE = 1; //This happens to be the mode for both the DAN28027 and
    SpibRegs.SPICCR.bit.CLKPOLARITY = 0; //The MPU-9250, Mode 01.
    SpibRegs.SPICTL.bit.MASTER_SLAVE = 1; // Set to SPI Master
    SpibRegs.SPICCR.bit.SPICHAR = 0xF; // Set to transmit and receive 16 bits each write to SPITXBUF
    SpibRegs.SPICTL.bit.TALK = 1; // Enable transmission
    SpibRegs.SPIPRI.bit.FREE = 1; // Free run, continue SPI operation
    SpibRegs.SPICTL.bit.SPIINTENA = 0; // Disables the SPI interrupt
    SpibRegs.SPIBRR.bit.SPI_BIT_RATE = 49; // Set SCLK bit rate to 1 MHz so 1us period. SPI base clock is
     // 50MHZ. And this setting divides that base clock to create SCLK’s period
    SpibRegs.SPISTS.all = 0x0000; // Clear status flags just in case they are set for some reasonSE423 4 Lab #5
    SpibRegs.SPIFFTX.bit.SPIRST = 1;// Pull SPI FIFO out of reset, SPI FIFO can resume transmit or receive.
    SpibRegs.SPIFFTX.bit.SPIFFENA = 1; // Enable SPI FIFO enhancements
    SpibRegs.SPIFFTX.bit.TXFIFO = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpibRegs.SPIFFTX.bit.TXFFINTCLR = 1; // Write 1 to clear SPIFFTX[TXFFINT] flag just in case it is set
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Write 1 to clear SPIFFRX[RXFFOVF] just in case it is set
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1; // Write 1 to clear SPIFFRX[RXFFINT] flag just in case it is set
    SpibRegs.SPIFFRX.bit.RXFFIENA = 1; // Enable the RX FIFO Interrupt. RXFFST >= RXFFIL
    SpibRegs.SPIFFCT.bit.TXDLY = 0; //Set delay between transmits to 16 spi clocks. Needed by DAN28027 chip
    SpibRegs.SPICCR.bit.SPISWRESET = 1; // Pull the SPI out of reset
    SpibRegs.SPIFFTX.bit.TXFIFO = 1; // Release transmit FIFO from reset.
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 1; // Re-enable receive FIFO operation
    SpibRegs.SPICTL.bit.SPIINTENA = 1; // Enables SPI interrupt. !! I don’t think this is needed. Need to Test
    SpibRegs.SPIFFRX.bit.RXFFIL = 10; //Interrupt Level to 16 words or more received into FIFO causes interrupt. This is just the initial setting for the register. Will be changed below
    //-----------------------------------------------------------------------------------------------------------------

    //Step 2.
    // perform a multiple 16 bit transfer to initialize MPU-9250 registers 0x13,0x14,0x15,0x16
    // 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C 0x1D, 0x1E, 0x1F. Use only one SS low to high for all these writes
    // some code is given, most you have to fill you yourself.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // Slave Select Low
    // Perform the number of needed writes to SPITXBUF to write to all 13 registers. Remember we are sending 16 bit transfers, so two registers at a time after the first 16 bit transfer.
    // To address 00x13 write 0x00
    // To address 00x14 write 0x00
    // To address 00x15 write 0x00
    // To address 00x16 write 0x00
    // To address 00x17 write 0x00
    // To address 00x18 write 0x00
    // To address 00x19 write 0x13
    // To address 00x1A write 0x02
    // To address 00x1B write 0x00
    // To address 00x1C write 0x08
    // To address 00x1D write 0x06
    // To address 00x1E write 0x00
    // To address 00x1F write 0x00

    SpibRegs.SPITXBUF = 0x1300;
    SpibRegs.SPITXBUF = 0x0000;
    SpibRegs.SPITXBUF = 0x0000;
    SpibRegs.SPITXBUF = 0x0013;
    SpibRegs.SPITXBUF = 0x0200;
    SpibRegs.SPITXBUF = 0x0806;
    SpibRegs.SPITXBUF = 0x0000;

    // wait for the correct number of 16 bit values to be received into the RX FIFO
    while(SpibRegs.SPIFFRX.bit.RXFFST != 7);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave Select High
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    // ???? read the additional number of garbage receive values off the RX FIFO to clear out the RX FIFO
    DELAY_US(10); // Delay 10us to allow time for the MPU-2950 to get ready for next transfer.

    //Step 3.
    // perform a multiple 16 bit transfer to initialize MPU-9250 registers 0x23,0x24,0x25,0x26
    // 0x27, 0x28, 0x29. Use only one SS low to high for all these writes
    // some code is given, most you have to fill you yourself.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // Slave Select Low

    // Perform the number of needed writes to SPITXBUF to write to all 7 registers
    // To address 00x23 write 0x00
    // To address 00x24 write 0x40
    // To address 00x25 write 0x8C
    // To address 00x26 write 0x02
    // To address 00x27 write 0x88
    // To address 00x28 write 0x0C
    // To address 00x29 write 0x0A

    SpibRegs.SPITXBUF = 0x2300;
    SpibRegs.SPITXBUF = 0x8C02;
    SpibRegs.SPITXBUF = 0x880C;
    SpibRegs.SPITXBUF = 0x0A00;

    // wait for the correct number of 16 bit values to be received into the RX FIFO
    while(SpibRegs.SPIFFRX.bit.RXFFST != 4);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave Select High
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;

    // ???? read the additional number of garbage receive values off the RX FIFO to clear out the RX FIFO
    DELAY_US(10); // Delay 10us to allow time for the MPU-2950 to get ready for next transfer.

    //Step 4.
    // perform a single 16 bit transfer to initialize MPU-9250 register 0x2A
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    // Write to address 0x2A the value 0x81
    SpibRegs.SPITXBUF = (0x2A81);

    // wait for one byte to be received
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);

    // The Remainder of this code is given to you and you do not need to make any changes.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x3800 | 0x0001); // 0x3800
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x3A00 | 0x0001); // 0x3A00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6400 | 0x0001); // 0x6400
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6700 | 0x0003); // 0x6700
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6A00 | 0x0020); // 0x6A00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6B00 | 0x0001); // 0x6B00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7500 | 0x0071); // 0x7500
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7700 | 0x00EB); // 0x7700
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7800 | 0x0012); // 0x7800
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7A00 | 0x0010); // 0x7A00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7B00 | 0x00FA); // 0x7B00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7D00 | 0x0021); // 0x7D00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7E00 | 0x0050); // 0x7E00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(50);

    // Clear SPIB interrupt source just in case it was issued due to any of the above initializations.
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR=1; // Clear Overflow flag
    SpibRegs.SPIFFRX.bit.RXFFINTCLR=1; // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
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

//b is the filter coefficients taken from Matlab
float b[101] = {  2.0089131384901197e-03,
                  6.4032873499578040e-04,
                  -1.7662310132503288e-03,
                  -1.8966231855838251e-03,
                  7.9038298787438197e-04,
                  2.8250866960543826e-03,
                  9.7274726769560108e-04,
                  -2.8535932093218977e-03,
                  -3.2069079180517828e-03,
                  1.3777460739364028e-03,
                  5.0108857805228734e-03,
                  1.7369488778204004e-03,
                  -5.0869489066624630e-03,
                  -5.6717260737981379e-03,
                  2.4066077632725297e-03,
                  8.6179538038498871e-03,
                  2.9352017836365030e-03,
                  -8.4357135384937401e-03,
                  -9.2235281203421979e-03,
                  3.8369713729420702e-03,
                  1.3470983718227284e-02,
                  4.4992711557421761e-03,
                  -1.2684979985041140e-02,
                  -1.3611937750688167e-02,
                  5.5600514925787251e-03,
                  1.9176967391055018e-02,
                  6.2956283333650978e-03,
                  -1.7455271677881148e-02,
                  -1.8429536833842467e-02,
                  7.4103785848253561e-03,
                  2.5171457314971404e-02,
                  8.1418571044648731e-03,
                  -2.2250769713411937e-02,
                  -2.3165078063428872e-02,
                  9.1879041586407240e-03,
                  3.0795414085640505e-02,
                  9.8318928762857697e-03,
                  -2.6528873794684965e-02,
                  -2.7276081156801475e-02,
                  1.0686709091186523e-02,
                  3.5390668308456406e-02,
                  1.1166118673320274e-02,
                  -2.9780034614308684e-02,
                  -3.0269173855075916e-02,
                  1.1725680290077527e-02,
                  3.8398491060813049e-02,
                  1.1981403290429368e-02,
                  -3.1604759414221834e-02,
                  -3.1774940699058361e-02,
                  1.2176082500102338e-02,
                  3.9444917234878515e-02,
                  1.2176082500102338e-02,
                  -3.1774940699058361e-02,
                  -3.1604759414221834e-02,
                  1.1981403290429368e-02,
                  3.8398491060813049e-02,
                  1.1725680290077527e-02,
                  -3.0269173855075916e-02,
                  -2.9780034614308684e-02,
                  1.1166118673320274e-02,
                  3.5390668308456406e-02,
                  1.0686709091186523e-02,
                  -2.7276081156801475e-02,
                  -2.6528873794684965e-02,
                  9.8318928762857697e-03,
                  3.0795414085640505e-02,
                  9.1879041586407240e-03,
                  -2.3165078063428872e-02,
                  -2.2250769713411937e-02,
                  8.1418571044648731e-03,
                  2.5171457314971404e-02,
                  7.4103785848253561e-03,
                  -1.8429536833842467e-02,
                  -1.7455271677881148e-02,
                  6.2956283333650978e-03,
                  1.9176967391055018e-02,
                  5.5600514925787251e-03,
                  -1.3611937750688167e-02,
                  -1.2684979985041140e-02,
                  4.4992711557421761e-03,
                  1.3470983718227284e-02,
                  3.8369713729420702e-03,
                  -9.2235281203421979e-03,
                  -8.4357135384937401e-03,
                  2.9352017836365030e-03,
                  8.6179538038498871e-03,
                  2.4066077632725297e-03,
                  -5.6717260737981379e-03,
                  -5.0869489066624630e-03,
                  1.7369488778204004e-03,
                  5.0108857805228734e-03,
                  1.3777460739364028e-03,
                  -3.2069079180517828e-03,
                  -2.8535932093218977e-03,
                  9.7274726769560108e-04,
                  2.8250866960543826e-03,
                  7.9038298787438197e-04,
                  -1.8966231855838251e-03,
                  -1.7662310132503288e-03,
                  6.4032873499578040e-04,
                  2.0089131384901197e-03};
//float b[5] = {.2, .2, .2, .2, .2}; // 0.2 is 1/5th therefore a 5 point average

__interrupt void ADCA_ISR(void) {

    int16_t i = 0;
    int16_t j = 0;

    adca2result = AdcaResultRegs.ADCRESULT0;
    adca3result = AdcaResultRegs.ADCRESULT1;

    // Here covert ADCIND0, ADCIND1 to volts
    var1 = adca2result/4095.0*3.0;
    var2 = adca3result/4095.0*3.0;

    // Here covert ADCIND0, ADCIND1 to volts
    xk[0] = var1;
    xk2[0]= var2;
    //yk = b[0]*xk + b[1]*xk_1 + b[2]*xk_2 + b[3]*xk_3 + b[4]*xk_4;

    yk = 0;
    yk2 = 0;
    for (i=0; i<22; i++){
        yk = yk + b[i] * xk[i];
        yk2 = yk2 +b[i] * xk2[i];
    }

    //Save past states before exiting from the function so that next sample they are the older state
    for (j = 22; j > 0; j--){
        xk[j] = xk[j-1];
        xk2[j] = xk2[j-1];
    }

//    // Here write yk to DACA channel
    setDACA(yk);
    setDACB(yk2);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPIFFRX.bit.RXFFIL = 8;
    SpibRegs.SPITXBUF = ((0x8000) | (0x3A00));
    SpibRegs.SPITXBUF = 0;
    SpibRegs.SPITXBUF = 0;
    SpibRegs.SPITXBUF = 0;
    SpibRegs.SPITXBUF = 0;
    SpibRegs.SPITXBUF = 0;
    SpibRegs.SPITXBUF = 0;
    SpibRegs.SPITXBUF = 0;

//    // Print ADCIND0 and ADCIND1’s voltage value to TeraTerm every 100ms
//
    if(count % 100 == 0){
        UARTPrint = 1;
    }
    count++;

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

