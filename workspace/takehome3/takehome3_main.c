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
//__interrupt void SPIB_isr(void);

//void setupSPIB(void);
void serialRXA(serial_t *s, char data);
void I2CB_Init(void);

uint16_t WriteDAN28027RCServo(uint16_t RC1, uint16_t RC2);
uint16_t ReadDAN28027ADC(uint16_t *ADC1, uint16_t *ADC2);
uint16_t WriteBQ32000(uint16_t second,uint16_t minute,uint16_t hour,uint16_t day,uint16_t date,uint16_t month,uint16_t year);
uint16_t ReadBQ32000(uint16_t *second,uint16_t *minute,uint16_t *hour,uint16_t *day,uint16_t *date,uint16_t *month,uint16_t *year);

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls = 0;
uint32_t numRXA = 0;
uint16_t UARTPrint = 0;
uint16_t LEDdisplaynum = 0;
uint16_t RC1 = 1500;
uint16_t RC2 = 1500;
uint16_t ADC1=0;
uint16_t ADC2=0;

uint16_t DanRC1 = 0;
uint16_t DanRC2 = 0;
uint16_t DanADC1 = 0;
uint16_t DanADC2 = 0;
float var1 = 0;
float var2 = 0;

uint16_t DanSec = 0;
uint16_t DanMin = 0;
uint16_t DanHour = 0;
uint16_t DanDay = 0;
uint16_t DanDate = 0;
uint16_t DanMonth = 0;
uint16_t DanYear = 0;

uint16_t count = 0;
uint16_t ReadI2CB = 0;

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
    //PieVectTable.SPIB_RX_INT = &SPIB_isr;

    PieVectTable.EMIF_ERROR_INT = &SWI_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers


    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every second:
    // 200MHz CPU Freq, 1 second Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, 200, 20000);
    ConfigCpuTimer(&CpuTimer1, 200, 20000);
    ConfigCpuTimer(&CpuTimer2, 200, 40000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    init_serial(&SerialA,115200,serialRXA);
//    init_serial(&SerialC,115200,serialRXC);
//    init_serial(&SerialD,115200,serialRXD);
    //setupSPIB();

    //calling required functions
    I2CB_Init();
    WriteBQ32000(0,13,4,6,27,4,33); //writing new date with this functions

//
//    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0); // Set as GPIO9 and used as DAN28027 SS
//    GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL); // Make GPIO9 an Output Pin
//    GpioDataRegs.GPASET.bit.GPIO9 = 1; //Initially Set GPIO9/SS High so DAN28027 is not selected
//    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0); // Set as GPIO66 and used as MPU-9250 SS
//    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL); // Make GPIO66 an Output Pin
//    GpioDataRegs.GPCSET.bit.GPIO66 = 1; //Initially Set GPIO66/SS High so MPU-9250 is not selected
//    GPIO_SetupPinMux(63, GPIO_MUX_CPU1, 15); //Set GPIO63 pin to SPISIMOB
//    GPIO_SetupPinMux(64, GPIO_MUX_CPU1, 15); //Set GPIO64 pin to SPISOMIB
//    GPIO_SetupPinMux(65, GPIO_MUX_CPU1, 15); //Set GPIO65 pin to SPICLKB
//    EALLOW;
//    GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0; // Enable Pull-ups on SPI PINs Recommended by TI for SPI Pins
//    GpioCtrlRegs.GPCPUD.bit.GPIO64 = 0;
//    GpioCtrlRegs.GPCPUD.bit.GPIO65 = 0;
//    GpioCtrlRegs.GPBQSEL2.bit.GPIO63 = 3; // Set I/O pin to asynchronous mode recommended for SPI
//    GpioCtrlRegs.GPCQSEL1.bit.GPIO64 = 3; // Set I/O pin to asynchronous mode recommended for SPI
//    GpioCtrlRegs.GPCQSEL1.bit.GPIO65 = 3; // Set I/O pin to asynchronous mode recommended for SPI
//    EDIS;
    // ---------------------------------------------------------------------------
//    //Setting up SPIB REGS
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
//    SpibRegs.SPIFFRX.bit.RXFFIL = 16; //Interrupt Level to 16 words or more received into FIFO causes interrupt. This is just the initial setting for the register. Will be changed below


    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.  
    IER |= M_INT1;
    IER |= M_INT8;  // SCIC SCID
    IER |= M_INT9;  // SCIA
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;

    //IER |= M_INT6;

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
	// Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;
    // Enable SPIB_RX in the PIE: Group 6 interrupt 3
    //PieCtrlRegs.PIEIER6.bit.INTx3 = 1;
	
    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM


    // IDLE loop. Just sit and loop forever (optional):
    while(1)
    {
        if (UARTPrint == 1 ) {
            //Printing variables

            //converting ADC signals to volts
            var1 = DanADC1*3.0/4095;
            var2 = DanADC2*3.0/4095;


            serial_printf(&SerialA,"ADC1: %.3f ADC2: %.3f\r\n", var1, var2);

            //if statements for different days based on whether DanDay is 1-7
            if(DanDay == 1){
                serial_printf(&SerialA,"day mm/dd/yy hour:min:second: Sunday %d/%d/%d %d:%d:%d\r\n", DanMonth, DanDate, DanYear, DanHour, DanMin, DanSec);
            }
            if(DanDay == 2){
                serial_printf(&SerialA,"day mm/dd/yy hour:min:second: Monday %d/%d/%d %d:%d:%d\r\n", DanMonth, DanDate, DanYear, DanHour, DanMin, DanSec);
            }
            if(DanDay == 3){
                serial_printf(&SerialA,"day mm/dd/yy hour:min:second: Tuesday %d/%d/%d %d:%d:%d\r\n", DanMonth, DanDate, DanYear, DanHour, DanMin, DanSec);
            }
            if(DanDay == 4){
                serial_printf(&SerialA,"day mm/dd/yy hour:min:second: Wednesday %d/%d/%d %d:%d:%d\r\n", DanMonth, DanDate, DanYear, DanHour, DanMin, DanSec);
            }
            if(DanDay == 5){
                serial_printf(&SerialA,"day mm/dd/yy hour:min:second: Thursday %d/%d/%d %d:%d:%d\r\n", DanMonth, DanDate, DanYear, DanHour, DanMin, DanSec);
            }
            if(DanDay == 6){
                serial_printf(&SerialA,"day mm/dd/yy hour:min:second: Friday %d/%d/%d %d:%d:%d\r\n", DanMonth, DanDate, DanYear, DanHour, DanMin, DanSec);
            }
            if(DanDay == 7){
                serial_printf(&SerialA,"day mm/dd/yy hour:min:second: Saturday %d/%d/%d %d:%d:%d\r\n", DanMonth, DanDate, DanYear, DanHour, DanMin, DanSec);
            }

            //serial_printf(&SerialA,"day mm/dd/yy hour:min:second: %d %d/%d/%d %d:%d:%d\r\n", DanDay, DanMonth, DanDate, DanYear, DanHour, DanMin, DanSec);
            UARTPrint = 0;
        }

        //Checking if ReadI2CB is 1 to call the Read/Write functions
        if(ReadI2CB == 1){
            ReadI2CB = 0;

            RC1 += 10;
            RC2 += 10;
            if(RC1>4500){
                RC1 = 1500;
            }
            if (RC2 > 4500){
                RC2 = 1500;
            }

            WriteDAN28027RCServo(RC1, RC2);
            ReadDAN28027ADC(&DanADC1, &DanADC2);
            ReadBQ32000(&DanSec,&DanMin,&DanHour,&DanDay, &DanDate, &DanMonth,&DanYear);
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
//
    if ((numTimer0calls%50) == 0) {
        PieCtrlRegs.PIEIFR12.bit.INTx9 = 1;  // Manually cause the interrupt for the SWI
    }
//
//    //Clear GPIO9 Low to act as a Slave Select. Right now, just to scope. Later to select DAN28027 chip
//    GpioDataRegs.GPACLEAR.bit.GPIO9 = 1;
//    SpibRegs.SPIFFRX.bit.RXFFIL = 2; // Issue the SPIB_RX_INT when two values are in the RX FIFO
//    SpibRegs.SPITXBUF = 0x4A3B; // 0x4A3B and 0xB517 have no special meaning. Wanted to send
//    SpibRegs.SPITXBUF = 0xB517; // something so you can see the pattern on the Oscilloscope
//
    if ((numTimer0calls%250) == 0) {
        displayLEDletter(LEDdisplaynum);
        LEDdisplaynum++;
        if (LEDdisplaynum == 0xFFFF) {  // prevent roll over exception
            LEDdisplaynum = 0;
        }
    }



//    GpioDataRegs.GPACLEAR.bit.GPIO9 = 1;
//    //Reading in values from the FIFO
//    SpibRegs.SPIFFRX.bit.RXFFIL = 3;
//    SpibRegs.SPITXBUF = 0xDA;
//    SpibRegs.SPITXBUF = RC1;
//    SpibRegs.SPITXBUF = RC2;

//	// Blink LaunchPad Red LED
    GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
//
//    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

    if ((CpuTimer0.InterruptCount % 5) == 0) {
       UARTPrint = 1;
    }
}

// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void)
{

    ReadI2CB = 1;

    CpuTimer1.InterruptCount++;

    if(CpuTimer1.InterruptCount % 5 == 0){
        UARTPrint = 1;
    }
}

// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void)
{

    CpuTimer2.InterruptCount++;

}


// This function is called each time a char is recieved over UARTA.
void serialRXA(serial_t *s, char data) {
    numRXA ++;

}

void I2CB_Init(void)
{
    EALLOW;
    /* Enable internal pull-up for the selected I2C pins */
    GpioCtrlRegs.GPBPUD.bit.GPIO40 = 1;
    GpioCtrlRegs.GPBPUD.bit.GPIO41 = 1;
    /* Set qualification for the selected I2C pins */
    GpioCtrlRegs.GPBQSEL1.bit.GPIO40 = 3;
    GpioCtrlRegs.GPBQSEL1.bit.GPIO41 = 3;
    /* Configure which of the possible GPIO pins will be I2C_B pins using GPIO regs*/
    GpioCtrlRegs.GPBGMUX1.bit.GPIO40 = 1;
    GpioCtrlRegs.GPBMUX1.bit.GPIO40 = 2;
    GpioCtrlRegs.GPBGMUX1.bit.GPIO41 = 1;
    GpioCtrlRegs.GPBMUX1.bit.GPIO41 = 2;
    EDIS;
    // Initialize I2C
    I2cbRegs.I2CMDR.bit.IRS = 0;
    // 200MHz / 20 = 10MHz
    I2cbRegs.I2CPSC.all = 19;
    // 10MHz/40 = 250KHz
    I2cbRegs.I2CCLKL = 45; //psc > 2 so d = 5 See Usersguide
    I2cbRegs.I2CCLKH = 45; //psc > 2 so d = 5 See Usersguide
    I2cbRegs.I2CIER.all = 0x00;
    I2cbRegs.I2CMDR.bit.IRS = 1;
    DELAY_US(5000);
}

//Write 2 16bit commands (LSB then MSB) to I2C Slave CHIPXYZ starting at CHIPXYZ's register 4,
uint16_t WriteDAN28027RCServo(uint16_t RC1, uint16_t RC2) {
    //Changed to match Dan28027 Register Values
    uint16_t Cmd1LSB = 0;
    uint16_t Cmd1MSB = 0;
    uint16_t Cmd2LSB = 0;
    uint16_t Cmd2MSB = 0;
    Cmd1LSB = RC1 & 0xFF; //Bottom 8 bits of command
    Cmd1MSB = (RC1 >> 8) & 0xFF; //Top 8 bits of command
    Cmd2LSB = RC2 & 0xFF; //Bottom 8 bits of command
    Cmd2MSB = (RC2 >> 8) & 0xFF; //Top 8 bits of command
    DELAY_US(200); // Allow time for I2C to finish up previous commands. It pains me to have this
    // delay here but I have not had time to figure out what status bit to poll on to
    // to check if the I2C peripheral is ready of the next command. I have tried the busy
    // bit but that gave me some issues especially at startup. This would be a great
    // choice for a part of your final project if you would like to learn more about I2C.
    if (I2cbRegs.I2CSTR.bit.BB == 1) { // Check if I2C busy, if it is better
        return 2; // to Exit and try again next sample
    } // This should not happen too often
    while(!I2cbRegs.I2CSTR.bit.XRDY); //Poll until I2C ready to Transmit
    I2cbRegs.I2CSAR.all = 0x7D; // I2C address of ChipXYZ
    I2cbRegs.I2CCNT = 5; //Num Values plus Start Register 4 + 1
    I2cbRegs.I2CDXR.all = 4; // First need to transfer the Register value
    // to start writing data
    // I2C in master mode (MST), I2C is in transmit mode (TRX) with start and stop
    I2cbRegs.I2CMDR.all = 0x6E20;
    while(!I2cbRegs.I2CSTR.bit.XRDY); //Poll until I2C ready to Transmit
    I2cbRegs.I2CDXR.all = Cmd1LSB; // Write Command 1 LSB
    if (I2cbRegs.I2CSTR.bit.NACK == 1){ // Check for No Acknowledgement
        return 3; // This should not happen
    }
    while(!I2cbRegs.I2CSTR.bit.XRDY); //Poll until I2C ready to Transmit
    I2cbRegs.I2CDXR.all = Cmd1MSB; // Write Command 1 MSB
    if (I2cbRegs.I2CSTR.bit.NACK == 1){ // Check for No Acknowledgement
        return 3; // This should not happen
    }
    while(!I2cbRegs.I2CSTR.bit.XRDY); //Poll until I2C ready to Transmit
    I2cbRegs.I2CDXR.all = Cmd2LSB; // Write Command 2 LSB
    if (I2cbRegs.I2CSTR.bit.NACK == 1){ // Check for No Acknowledgement
        return 3; // This should not happen
    }
    while(!I2cbRegs.I2CSTR.bit.XRDY); //Poll until I2C ready to Transmit
    I2cbRegs.I2CDXR.all = Cmd2MSB; // Write Command 2 MSB
    // After this write since I2CCNT = 0
    // A Stop condition will be issued
    if (I2cbRegs.I2CSTR.bit.NACK == 1){
        return 3;
    }
    return 0;
}

//Read Two 16 Bit values from I2C Slave CHIPXYZ starting at CHIPXYZ's register 10
//Notice the Rvalue1 and Rvalue2 passed as pointers (passed by reference)
//So pass address of uint16_t variable when using this function for example:
// uint16_t Rval1 = 0;
// uint16_t Rval2 = 0;
// err = ReadTwo8BitValuesFromCHIPXYZ(&Rval1,&Rval2);
// This allows Rval1 and Rval2 to be changed inside the function and return
// the values read inside the function.
uint16_t ReadDAN28027ADC(uint16_t *ADC1, uint16_t *ADC2) {
    //Changed to match Dan28027 Register Values
    uint16_t Val1LSB = 0;
    uint16_t Val1MSB = 0;
    uint16_t Val2LSB = 0;
    uint16_t Val2MSB = 0;
    DELAY_US(200); // Allow time for I2C to finish up previous commands. It pains me to have this
    // delay here but I have not had time to figure out what status bit to poll on to
    // to check if the I2C peripheral is ready of the next command. I have tried the busy
    // bit but that gave me some issues especially at startup. This would be a great
    // choice for a part of your final project if you would like to learn more about I2C.
    if (I2cbRegs.I2CSTR.bit.BB == 1) {
        return 2;
    }
    while(!I2cbRegs.I2CSTR.bit.XRDY); //Poll until I2C ready to Transmit
    I2cbRegs.I2CSAR.all = 0x7D; // I2C address of ChipXYZ
    I2cbRegs.I2CCNT = 1; // Just Sending Address to start reading from
    I2cbRegs.I2CDXR.all = 0; // Start Reading at this Register location
    // I2C in master mode (MST), I2C is in transmit mode (TRX) with start
    I2cbRegs.I2CMDR.all = 0x6620;
    if (I2cbRegs.I2CSTR.bit.NACK == 1) {
        return 3;
    }
    while(!I2cbRegs.I2CSTR.bit.XRDY); //Poll until I2C ready to Transmit
    I2cbRegs.I2CSAR.all = 0x7D; // I2C address of ChipXYZ
    I2cbRegs.I2CCNT = 4;
    // I2C in master mode (MST), TRX=0, receive mode start stop
    I2cbRegs.I2CMDR.all = 0x6C20; // Reissuing another Start with Read
    if (I2cbRegs.I2CSTR.bit.NACK == 1) {
        return 3;
    }
    while(!I2cbRegs.I2CSTR.bit.RRDY); //Poll until I2C has Recieved 8 bit value
    Val1LSB = I2cbRegs.I2CDRR.all; // Read CHIPXYZ
    if (I2cbRegs.I2CSTR.bit.NACK == 1) {
        return 3;
    }
    while(!I2cbRegs.I2CSTR.bit.RRDY); //Poll until I2C has Recieved 8 bit value
    Val1MSB = I2cbRegs.I2CDRR.all; // Read CHIPXYZ
    if (I2cbRegs.I2CSTR.bit.NACK == 1) {
        return 3;
    }
    while(!I2cbRegs.I2CSTR.bit.RRDY); //Poll until I2C has Recieved 8 bit value
    Val2LSB = I2cbRegs.I2CDRR.all; // Read CHIPXYZ
    if (I2cbRegs.I2CSTR.bit.NACK == 1) {
        return 3;
    }
    while(!I2cbRegs.I2CSTR.bit.RRDY); //Poll until I2C has Recieved 8 bit value
    Val2MSB = I2cbRegs.I2CDRR.all; // Read CHIPXYZ
    if (I2cbRegs.I2CSTR.bit.NACK == 1) {
        return 3;
    }
    // After this read since I2CCNT = 0
    // A Stop condition will be issued
    *ADC1 = (Val1MSB << 8) | (Val1LSB & 0xFF);
    *ADC2 = (Val2MSB << 8) | (Val2LSB & 0xFF);

    return 0;
}

uint16_t WriteBQ32000(uint16_t second,uint16_t minute,uint16_t hour,uint16_t day,uint16_t date,uint16_t month,uint16_t year){
    //Changed to match BQ32000 Register Values
    DELAY_US(200); // Allow time for I2C to finish up previous commands. It pains me to have this
    // delay here but I have not had time to figure out what status bit to poll on to
    // to check if the I2C peripheral is ready of the next command. I have tried the busy
    // bit but that gave me some issues especially at startup. This would be a great
    // choice for a part of your final project if you would like to learn more about I2C.
    if (I2cbRegs.I2CSTR.bit.BB == 1) { // Check if I2C busy, if it is better
        return 2; // to Exit and try again next sample
    } // This should not happen too often
    while(!I2cbRegs.I2CSTR.bit.XRDY); //Poll until I2C ready to Transmit
    I2cbRegs.I2CSAR.all = 0x68; // I2C address of ChipXYZ
    I2cbRegs.I2CCNT = 8; //Num Values plus Start Register 4 + 1
    I2cbRegs.I2CDXR.all = 0; // First need to transfer the Register value
    // to start writing data
    // I2C in master mode (MST), I2C is in transmit mode (TRX) with start and stop
    I2cbRegs.I2CMDR.all = 0x6E20;
    while(!I2cbRegs.I2CSTR.bit.XRDY); //Poll until I2C ready to Transmit
    I2cbRegs.I2CDXR.all = second; // Write Command 1 MSB
    if (I2cbRegs.I2CSTR.bit.NACK == 1){ // Check for No Acknowledgement
        return 3; // This should not happen
    }
    while(!I2cbRegs.I2CSTR.bit.XRDY); //Poll until I2C ready to Transmit
    I2cbRegs.I2CDXR.all = minute; // Write Command 2 LSB
    if (I2cbRegs.I2CSTR.bit.NACK == 1){ // Check for No Acknowledgement
        return 3; // This should not happen
    }

    while(!I2cbRegs.I2CSTR.bit.XRDY); //Poll until I2C ready to Transmit
    I2cbRegs.I2CDXR.all = hour; // Write Command 2 MSB
    // After this write since I2CCNT = 0
    // A Stop condition will be issued
    if (I2cbRegs.I2CSTR.bit.NACK == 1){
        return 3;
    }
    while(!I2cbRegs.I2CSTR.bit.XRDY); //Poll until I2C ready to Transmit
    I2cbRegs.I2CDXR.all = day; // Write Command 2 LSB
    if (I2cbRegs.I2CSTR.bit.NACK == 1){ // Check for No Acknowledgement
        return 3; // This should not happen
    }

    while(!I2cbRegs.I2CSTR.bit.XRDY); //Poll until I2C ready to Transmit
    I2cbRegs.I2CDXR.all = date; // Write Command 2 MSB
    // After this write since I2CCNT = 0
    // A Stop condition will be issued
    if (I2cbRegs.I2CSTR.bit.NACK == 1){
        return 3;
    }
    while(!I2cbRegs.I2CSTR.bit.XRDY); //Poll until I2C ready to Transmit
    I2cbRegs.I2CDXR.all = month; // Write Command 2 LSB
    if (I2cbRegs.I2CSTR.bit.NACK == 1){ // Check for No Acknowledgement
        return 3; // This should not happen
    }
    while(!I2cbRegs.I2CSTR.bit.XRDY); //Poll until I2C ready to Transmit
    I2cbRegs.I2CDXR.all = year; // Write Command 2 LSB
    if (I2cbRegs.I2CSTR.bit.NACK == 1){ // Check for No Acknowledgement
        return 3; // This should not happen
    }


    //getting the ones and tens place for the required parameters to write

    uint16_t ones = second % 10;
    uint16_t tens = second/10;
    uint16_t onesMin = minute%10;
    uint16_t tensMin = minute/10;
    uint16_t onesHour = hour&10;
    uint16_t tensHour = hour/10;
    uint16_t onesDate = date%10;
    uint16_t tensDate = date/10;
    uint16_t onesMonth = month%10;
    uint16_t tensMonth = month/10;
    uint16_t onesYear = year%10;
    uint16_t tensYear = year/10;

    //adding the ones and tens place together
    second = tens*10 + ones;
    minute = tensMin*10 + onesMin;
    hour = tensHour*10 + onesHour;
    date = tensDate*10 + onesDate;
    month = tensMonth*10 + onesMonth;
    year =tensYear*10 + onesYear;


    return 0;
}
uint16_t ReadBQ32000(uint16_t *second,uint16_t *minute,uint16_t *hour,uint16_t *day,uint16_t *date,uint16_t *month,uint16_t *year){
    //Changed to match BQ32000 Register Values
    DELAY_US(200); // Allow time for I2C to finish up previous commands. It pains me to have this
    // delay here but I have not had time to figure out what status bit to poll on to
    // to check if the I2C peripheral is ready of the next command. I have tried the busy
    // bit but that gave me some issues especially at startup. This would be a great
    // choice for a part of your final project if you would like to learn more about I2C.
    if (I2cbRegs.I2CSTR.bit.BB == 1) {
        return 2;
    }
    while(!I2cbRegs.I2CSTR.bit.XRDY); //Poll until I2C ready to Transmit
    I2cbRegs.I2CSAR.all = 0x68; // I2C address of ChipXYZ
    I2cbRegs.I2CCNT = 1; // Just Sending Address to start reading from
    I2cbRegs.I2CDXR.all = 0; // Start Reading at this Register location
    // I2C in master mode (MST), I2C is in transmit mode (TRX) with start
    I2cbRegs.I2CMDR.all = 0x6620;
    if (I2cbRegs.I2CSTR.bit.NACK == 1) {
        return 3;
    }
    while(!I2cbRegs.I2CSTR.bit.XRDY); //Poll until I2C ready to Transmit
    I2cbRegs.I2CSAR.all = 0x68; // I2C address of ChipXYZ
    I2cbRegs.I2CCNT = 7;
    // I2C in master mode (MST), TRX=0, receive mode start stop
    I2cbRegs.I2CMDR.all = 0x6C20; // Reissuing another Start with Read
    if (I2cbRegs.I2CSTR.bit.NACK == 1) {
        return 3;
    }
    while(!I2cbRegs.I2CSTR.bit.RRDY); //Poll until I2C has Recieved 8 bit value
    *second = I2cbRegs.I2CDRR.all; // Read CHIPXYZ
    if (I2cbRegs.I2CSTR.bit.NACK == 1) {
        return 3;
    }
    while(!I2cbRegs.I2CSTR.bit.RRDY); //Poll until I2C has Recieved 8 bit value
    *minute = I2cbRegs.I2CDRR.all; // Read CHIPXYZ
    if (I2cbRegs.I2CSTR.bit.NACK == 1) {
        return 3;
    }
    while(!I2cbRegs.I2CSTR.bit.RRDY); //Poll until I2C has Recieved 8 bit value
    *hour = I2cbRegs.I2CDRR.all; // Read CHIPXYZ
    if (I2cbRegs.I2CSTR.bit.NACK == 1) {
        return 3;
    }
    while(!I2cbRegs.I2CSTR.bit.RRDY); //Poll until I2C has Recieved 8 bit value
    *day = I2cbRegs.I2CDRR.all; // Read CHIPXYZ
    if (I2cbRegs.I2CSTR.bit.NACK == 1) {
        return 3;
    }
    while(!I2cbRegs.I2CSTR.bit.RRDY); //Poll until I2C has Recieved 8 bit value
    *date = I2cbRegs.I2CDRR.all; // Read CHIPXYZ
    if (I2cbRegs.I2CSTR.bit.NACK == 1) {
        return 3;
    }
    while(!I2cbRegs.I2CSTR.bit.RRDY); //Poll until I2C has Recieved 8 bit value
    *month = I2cbRegs.I2CDRR.all; // Read CHIPXYZ
    if (I2cbRegs.I2CSTR.bit.NACK == 1) {
        return 3;
    }
    while(!I2cbRegs.I2CSTR.bit.RRDY); //Poll until I2C has Recieved 8 bit value
    *year = I2cbRegs.I2CDRR.all; // Read CHIPXYZ
    if (I2cbRegs.I2CSTR.bit.NACK == 1) {
        return 3;
    }
    // After this read since I2CCNT = 0
    // A Stop condition will be issued

    //getting the ones and tens place for the required parameters to read

    uint16_t ones = *second & 0xF;
    uint16_t tens = (*second >> 4) & 0x7;
    uint16_t onesMin = *minute & 0xF;
    uint16_t tensMin = (*minute >> 4) & 0x7;
    uint16_t onesHour = *hour & 0xF;
    uint16_t tensHour = (*hour >> 4) & 0x7;
    uint16_t onesDate = *date & 0xF;
    uint16_t tensDate = (*date >> 4) & 0x7;
    uint16_t onesMonth = *month & 0xF;
    uint16_t tensMonth = (*month >> 4) & 0x7;
    uint16_t onesYear = *year & 0xF;
    uint16_t tensYear = (*year >> 4) & 0x7;

    //adding the ones and tens place together
    *second = tens*10 + ones;
    *minute = tensMin*10 + onesMin;
    *hour = tensHour*10 + onesHour;
    *date = tensDate*10 + onesDate;
    *month = tensMonth*10 + onesMonth;
    *year =tensYear*10 + onesYear;
    return 0;
}

