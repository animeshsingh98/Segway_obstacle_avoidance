//#############################################################################
// FILE:   LABstarter_main.c
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
#include "F28379dSerial.h"
#include "LEDPatterns.h"
#include "song.h"
#include "dsp.h"
#include "fpu32/fpu_rfft.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
// The Launchpad's CPU Frequency set to 200 you should not change this value
#define LAUNCHPAD_CPU_FREQUENCY 200


// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI_isr(void);

//pks11 ex1
__interrupt void SPIB_isr(void);

//pks11/ ex3
void setupSpib(void);

//pks11 //lab6 // ex1 // to avoid warnings
void init_eQEPs(void);
float readEncLeft(void);
float readEncRight(void);
void setEPWM2A(float controleffort);
void setEPWM2B(float controleffort);

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numTimer1calls = 0; //pks11 // ex5
uint32_t numSWIcalls = 0;
extern uint32_t numRXA;
uint16_t UARTPrint = 0;
uint16_t LEDdisplaynum = 0;

//pks11//ex1/
int16_t spivalue1 = 0;
int16_t spivalue2 = 0;
int16_t spivalue3 = 0;
//pks11 // ex2
int16_t pwmValue = 0;
int16_t direction = 1;
float adc_volt1 = 0;
float adc_volt2 = 0;

//pks11/ex3
int16_t dummy = 0;
int16_t Accel_X_Raw = 0;
int16_t Accel_Y_Raw = 0;
int16_t Accel_Z_Raw = 0;
int16_t Temp = 0;
int16_t Gyro_X_Raw = 0;
int16_t Gyro_Y_Raw = 0;
int16_t Gyro_Z_Raw = 0;

float Accel_X = 0;
float Accel_Y = 0;
float Accel_Z = 0;
float Gyro_X = 0;
float Gyro_Y = 0;
float Gyro_Z = 0;

//pks11 // lab6 // ex1
float LeftWheel = 0;
float RightWheel = 0;
float LeftWheel_update = 0;
float RightWheel_update = 0;
float LeftRadius = 2.37; // inch //initial guess
float RightRadius = 2.37; //inch // initial radius

float LeftWheel_Dist = 0;
float RightWheel_Dist = 0;

float LeftRadius_final = 0.2093; //foot// measured radius
float RightRadius_final = 0.2093; // foot // measured radius

float uLeft = 5; // setting up control action
float uRight = 5; // setting up control action

float PosLeft_K = 0 ; //setting up global position for left at time instance K;
float PosLeft_K_1 = 0; //setting up global position for left at time K-1

float PosRight_K = 0; //setting up global position for left at time instance K;
float PosRight_K_1 = 0 ; //setting up global position for left at time K-1

float  VLeftK = 0;
float  VRightK = 0;

//pks11 // controllers
float Kp = 3;
float Ki = 25;
float Vref = 1;
float Vref_Left = 1;
float eK_Left = 0;
float Vref_Right = 1;
float eK_Right = 0;
float IK_Left = 0;
float IK_1_Left = 0;
float IK_Right = 0;
float IK_1_Right = 0;
float eK_Left_1 = 0;
float eK_Right_1 = 0;
//pks11 setting up turn
float Kpturn = 3;
float turn = 0;
float eK_turn = 0;
float eK_turn_1 = 0;

//pks11 // ex5
float printLV3 = 0;
float printLV4 = 0;
float printLV5 = 0;
float printLV6 = 0;
float printLV7 = 0;
float printLV8 = 0;
float x = 0;
float y = 0;
float bearing = 0;
extern uint16_t NewLVData;
extern float fromLVvalues[LVNUM_TOFROM_FLOATS];
extern LVSendFloats_t DataToLabView;
extern char LVsenddata[LVNUM_TOFROM_FLOATS*4+2];
extern uint16_t newLinuxCommands;
extern float LinuxCommands[CMDNUM_FROM_FLOATS];

//pks11 //ex6
float Rwh = 0.19460; //feet //Radius of Wheel
float Wr = 0.56759; //feet //width of robot
float xR_K = 0; //feet // Robot pose X coordinate
float yR_K = 0; //feet //Robot pose Y coordinate
float phiR = 0; // radians // robot pose or bearing
float RightWheel_K = 0; // rightwheel in radians at time K
float LeftWheel_K = 0;  // leftwheel in radians at time K
float RightWheel_K_1 = 0; // Rightwheel in radians at time k-1
float LeftWheel_K_1 = 0; // Leftwheel in radians at time k-1
float thetaAvg = 0;
float thetaAvg_dot = 0;
float xdot_K = 0;
float ydot_K = 0;
float xdot_K_1 = 0;
float ydot_K_1 = 0;
float xR_K_1 = 0;
float yR_K_1 = 0;

void main(void)
{
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xD_SysCtrl.c file.
    InitSysCtrl();

    InitGpio();

    // Blue LED on LaunchPad
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

    //pks11 EPWM2A EPWM2B // lab6 //ex2
    //Count up Mode
    EPwm2Regs.TBCTL.bit.CTRMODE = 0;
    // Free Soft emulation mode to Free Run so that the PWM continues when you set a break point in your code
    EPwm2Regs.TBCTL.bit.FREE_SOFT = 2;
    //disable the phase loading
    EPwm2Regs.TBCTL.bit.PHSEN  = 0;
    //Clock divide by 1.
    EPwm2Regs.TBCTL.bit.CLKDIV = 0;

    //: Start the timer at zero.
    EPwm2Regs.TBCTR = 0;

    //Setting the period of PWM Signal 20 KHZ (50 mhz/ 20KKHZ = 2500)
    EPwm2Regs.TBPRD = 2500;

    //Starting duty cycle to 0%, that means CMPA 0.
    EPwm2Regs.CMPA.bit.CMPA = 0;
    EPwm2Regs.CMPB.bit.CMPB = 0;

    //Setting the actions
    //When TBCTR = CMPA then set low
    //EPwm12Regs.AQCTLA.CAU = 1;
    EPwm2Regs.AQCTLA.bit.CAU = 1;
    EPwm2Regs.AQCTLB.bit.CBU = 1;
    //When TBCTR= TBPRD then set high
    EPwm2Regs.AQCTLA.bit.ZRO = 2;
    EPwm2Regs.AQCTLB.bit.ZRO = 2;

    // Set the phase zero
    EPwm2Regs.TBPHS.bit.TBPHS =0;

    //EPWM2A, EPWM2B //pks11 // lab6 // setting up pin mux
    GPIO_SetupPinMux(2, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinMux(3, GPIO_MUX_CPU1, 1);

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
    PieVectTable.SCIB_RX_INT = &RXBINT_recv_ready;
    PieVectTable.SCIC_RX_INT = &RXCINT_recv_ready;
    PieVectTable.SCID_RX_INT = &RXDINT_recv_ready;
    PieVectTable.SCIA_TX_INT = &TXAINT_data_sent;
    PieVectTable.SCIB_TX_INT = &TXBINT_data_sent;
    PieVectTable.SCIC_TX_INT = &TXCINT_data_sent;
    PieVectTable.SCID_TX_INT = &TXDINT_data_sent;

    PieVectTable.EMIF_ERROR_INT = &SWI_isr;
    PieVectTable.SPIB_RX_INT = &SPIB_isr; //pks11 ///ex1

    EDIS;    // This is needed to disable write to EALLOW protected registers


    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every given period:
    // 200MHz CPU Freq,                       Period (in uSeconds)
    //ConfigCpuTimer(&CpuTimer0, LAUNCHPAD_CPU_FREQUENCY, 10000);
    //ConfigCpuTimer(&CpuTimer0, LAUNCHPAD_CPU_FREQUENCY, 20000); //pks11 // ex2 // calling the interrupt at every 20ms
    ConfigCpuTimer(&CpuTimer0, LAUNCHPAD_CPU_FREQUENCY, 1000); //pks11 // ex3 // 1ms interupt calling
    ConfigCpuTimer(&CpuTimer1, LAUNCHPAD_CPU_FREQUENCY, 4000); //pks11 // lab 6 // timer 1 will be 4 ms interupt calling
    ConfigCpuTimer(&CpuTimer2, LAUNCHPAD_CPU_FREQUENCY, 40000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    init_serialSCIA(&SerialA,115200);

    //pks11 // lab6 //ex1
    init_eQEPs();
    //pks11 //ex3
    //calling the setupSpib function
    setupSpib();

    //pks11 // lab5/ ex1 // copy -pasting the code to setup SPIB

    //    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0); // Set as GPIO9 and used as DAN28027 SS
    //    GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL); // Make GPIO9 an Output Pin
    //    GpioDataRegs.GPASET.bit.GPIO9 = 1; //Initially Set GPIO9/SS High so DAN28027 is not selected
    //
    //    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0); // Set as GPIO66 and used as MPU-9250 SS
    //    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL); // Make GPIO66 an Output Pin
    //    GpioDataRegs.GPCSET.bit.GPIO66 = 1; //Initially Set GPIO66/SS High so MPU-9250 is not selected
    //
    //    GPIO_SetupPinMux(63, GPIO_MUX_CPU1, 15); //Set GPIO63 pin to SPISIMOB //pks11/ ex1 // mux selection
    //    GPIO_SetupPinMux(64, GPIO_MUX_CPU1, 15); //Set GPIO64 pin to SPISOMIB //pks11/ ex1 // mux selection
    //    GPIO_SetupPinMux(65, GPIO_MUX_CPU1, 15); //Set GPIO65 pin to SPICLKB //pks11/ ex1 // mux selection
    //
    //
    //    EALLOW;
    //    GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0; // Enable Pull-ups on SPI PINs Recommended by TI for SPI Pins
    //    GpioCtrlRegs.GPCPUD.bit.GPIO64 = 0;
    //    GpioCtrlRegs.GPCPUD.bit.GPIO65 = 0;
    //    GpioCtrlRegs.GPBQSEL2.bit.GPIO63 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    //    GpioCtrlRegs.GPCQSEL1.bit.GPIO64 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    //    GpioCtrlRegs.GPCQSEL1.bit.GPIO65 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    //    EDIS;
    //
    //    // ---------------------------------------------------------------------------
    //    SpibRegs.SPICCR.bit.SPISWRESET = 0; // Put SPI in Reset //pks11 // ex1 // reseting spi because we need to configure
    //
    //    SpibRegs.SPICTL.bit.CLK_PHASE = 1; //This happens to be the mode for both the DAN28027 and
    //    SpibRegs.SPICCR.bit.CLKPOLARITY = 0; //The MPU-9250, Mode 01.
    //    SpibRegs.SPICTL.bit.MASTER_SLAVE = 1; // Set to SPI Master // pks11 // ex1// according to guide, if we are in reset configuration this SPI network become slave hence we need to set up high to create masters
    //    SpibRegs.SPICCR.bit.SPICHAR = 15; // Set to transmit and receive 16-bits each write to SPITXBUF //pks11// ex1 // we need to shift 16 bits together and hence SPICHAR will be Fh (15 in decimal)
    //    SpibRegs.SPICTL.bit.TALK = 1; // Enable transmission //pks11 // ex1// enabling master transmission
    //    SpibRegs.SPIPRI.bit.FREE = 1; // Free run, continue SPI operation
    //    SpibRegs.SPICTL.bit.SPIINTENA = 0; // Disables the SPI interrupt //pks11 //ex1 // disabling SPI interrupt
    //
    //    SpibRegs.SPIBRR.bit.SPI_BIT_RATE = 49; // Set SCLK bit rate to 1 MHz so 1us period. SPI base clock is
    //    // 50MHZ. And this setting divides that base clock to create SCLK�s period //pks11 //ex1 // SPI Baud Rate = SPCLK / (SPIBRR + 1), SPIBRR = 49
    //    SpibRegs.SPISTS.all = 0x0000; // Clear status flags just in case they are set for some reason
    //
    //    SpibRegs.SPIFFTX.bit.SPIRST = 1;// Pull SPI FIFO out of reset, SPI FIFO can resume transmit or receive. //PKS11 //ex1// taking out FIFO from rest configuration
    //    SpibRegs.SPIFFTX.bit.SPIFFENA = 1; // Enable SPI FIFO enhancements // pks11// ex1// enabling the enhancement
    //    SpibRegs.SPIFFTX.bit.TXFIFO = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    //    SpibRegs.SPIFFTX.bit.TXFFINTCLR = 1; // Write 1 to clear SPIFFTX[TXFFINT] flag just in case it is set
    //
    //    SpibRegs.SPIFFRX.bit.RXFIFORESET = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    //    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Write 1 to clear SPIFFRX[RXFFOVF] just in case it is set
    //    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1; // Write 1 to clear SPIFFRX[RXFFINT] flag just in case it is set //pks11 //ex1
    //    SpibRegs.SPIFFRX.bit.RXFFIENA = 1; // Enable the RX FIFO Interrupt. RXFFST >= RXFFIL //pks11 //ex1
    //
    //    SpibRegs.SPIFFCT.bit.TXDLY = 16; //Set delay between transmits to 16 spi clocks. Needed by DAN28027 chip //pks11// ex1// the delay has nothing to do with data transfer bits, that is requirement of DAN's chip
    //
    //    SpibRegs.SPICCR.bit.SPISWRESET = 1; // Pull the SPI out of reset //pks11 //ex1// taking out from the rest configuration
    //
    //    SpibRegs.SPIFFTX.bit.TXFIFO = 1; // Release transmit FIFO from reset. //pks11 //ex1 // taking out the transmission from the rest
    //    SpibRegs.SPIFFRX.bit.RXFIFORESET = 1; // Re-enable receive FIFO operation
    //    SpibRegs.SPICTL.bit.SPIINTENA = 1; // Enables SPI interrupt. !! I don�t think this is needed. Need to Test
    //
    //    SpibRegs.SPIFFRX.bit.RXFFIL = 16; //Interrupt Level to 16 words or more received into FIFO causes interrupt. This is just the initial setting for the register. Will be changed below //pks11 //ex1




    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.  
    IER |= M_INT1;
    IER |= M_INT8;  // SCIC SCID
    IER |= M_INT9;  // SCIA
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;

    //SPIB_RX is group 6
    IER |= M_INT6;

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    // Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;

    //Enabling SPIB_RX interupt in the PIE : Group 6 interupt 3
    PieCtrlRegs.PIEIER6.bit.INTx3 = 1;

    //init_serialSCIB(&SerialB,115200);
    init_serialSCIC(&SerialC,115200);
    init_serialSCID(&SerialD,115200);
    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM


    // IDLE loop. Just sit and loop forever (optional):
    while(1)
    {
        if (UARTPrint == 1 ) {
            //serial_printf(&SerialA,"Num Timer2:%ld Num SerialRX: %ld\r\n",CpuTimer2.InterruptCount,numRXA);
            //serial_printf(&SerialA,"adc_volt1:%.3f adv_volt2: %.3f\r\n",adc_volt1,adc_volt2); //pks11 // ex2
            //serial_printf(&SerialA,"Accel_X :%.3f Accel_Y: %.3f Accel_Z: %.3f Gyro_X : %.3f Gyro_Y : %.3f Gyro_Z : %.3f\r\n",Accel_X,Accel_Y,Accel_Z,Gyro_X,Gyro_Y,Gyro_Z);

            //pks11 //ex1 // lab6
            serial_printf(&SerialA,"Leftwheel enc (radians):%.6f Rightwheel enc (radians): %.6f\r\n",LeftWheel,RightWheel);
            serial_printf(&SerialA,"Leftwheel velocity (foot/s):%.6f RightWheel velocity (foot/s): %.6f\r\n", VLeftK, VRightK);

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

    //pks11 //ex2
    //this interrput is called at every 20ms, now, i order to print at every 100ms
    // taking module by 5

    //commented this out for ex3
    //    if ((CpuTimer0.InterruptCount % 5) == 0) {
    //        UARTPrint = 1;
    //    }

    //pks11 ex3 // I am running this interupt by 1ms and in order to print very 200 ms taking module by 200
    //    if ((CpuTimer0.InterruptCount % 200) == 0)
    //    {
    //        UARTPrint = 1;
    //    }


    //pks11 //ex2 // increasig duty cycle of pwm by value 10
    // if pwmvalue is greater than 3000 then start to reduce
    //if pwmvalue is less than or equal to 0, then start to increase by 10
    if(direction == 1)
    {
        pwmValue = pwmValue + 10;
        if (pwmValue >= 3000)
        {
            direction = -1;
        }
    }

    if(direction == -1)
    {
        pwmValue = pwmValue - 10;
        if (pwmValue <=0)
        {
            direction = 1;
        }
    }

    numTimer0calls++;

    //    if ((numTimer0calls%50) == 0) {
    //        PieCtrlRegs.PIEIFR12.bit.INTx9 = 1;  // Manually cause the interrupt for the SWI
    //    }

    if ((numTimer0calls%25) == 0) {
        displayLEDletter(LEDdisplaynum);
        LEDdisplaynum++;
        if (LEDdisplaynum == 0xFFFF) {  // prevent roll over exception
            LEDdisplaynum = 0;
        }
    }

    if ((numTimer0calls%50) == 0) {
        // Blink LaunchPad Red LED
        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
    }

    //    //Clear GPIO9 Low to act as a Slave Select. Right now, just to scope. Later to select DAN28027 chip
    //    GpioDataRegs.GPACLEAR.bit.GPIO9 = 1; //pks11// ex1 // clearing gpio9 to start the data transmission
    //    //SpibRegs.SPIFFRX.bit.RXFFIL = 2; //pks11 // ex1 Issue the SPIB_RX_INT when two values are in the RX FIFO
    //    //SpibRegs.SPITXBUF = 0x4A3B; // 0x4A3B and 0xB517 have no special meaning. Wanted to send
    //    //SpibRegs.SPITXBUF = 0xB517; // something so you can see the pattern on the Oscilloscope
    //
    //    //pks11 // ex2
    //    SpibRegs.SPIFFRX.bit.RXFFIL = 3; //pks11 // ex2// Issue the SPIB_RX_INT when three values are in the RX FIFO
    //    SpibRegs.SPITXBUF = 0x00DA; //start command to MOSI
    //    SpibRegs.SPITXBUF = pwmValue; //Sending 2 PWM values to DAN chip
    //    SpibRegs.SPITXBUF = pwmValue;

    // Clear GPIO66 Low to act as a Slave Select
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // pks11 // ex3 // clearing gpio66 to start the data transimission
    SpibRegs.SPIFFRX.bit.RXFFIL = 8; //pks11 // ex3 Issue the SPIB_RX_INT // Note we need to receive total 3 accelerometer and three gyro readings
    // Now, temp will in between of this so that makes total 7 readings.
    //if we add int_status (0x3A) in order to get Accel_XOut_H and Accel_XOut_L together
    SpibRegs.SPITXBUF = 0xBA00;// Reading from the INT_STATUS, giving the 00 // Note, we need to read the data so the most significant bit will change to 1 which makes 3A to change to BA
    SpibRegs.SPITXBUF = 0x0000; // THIS WILL HANDLE Accel_XOut_H andAccel_Xout_L
    SpibRegs.SPITXBUF = 0X0000; // This will handle Accel_Yout_H and Accel_Yout_L
    SpibRegs.SPITXBUF = 0X0000; // This will handle Accel_Zout_H and Accel_Zout_L
    SpibRegs.SPITXBUF = 0X0000; // This will handle Temp_out_H and Temp_out_L
    SpibRegs.SPITXBUF = 0X0000; // This will handle Gyro_Xout_H and Gyro_Xout_L
    SpibRegs.SPITXBUF = 0X0000; // This will handle Gyro_Yout_H and Gyro_Yout_L
    SpibRegs.SPITXBUF = 0X0000; // This will handle Gyro_Zout_H and Gyro_Zout_L






    // Acknowledge this interrupt to receive more interrupts from group 1

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void)
{

    CpuTimer1.InterruptCount++;
    numTimer1calls++;



    //pks11// lab6// ex5
    if (NewLVData == 1) {
        NewLVData = 0;
        Vref = fromLVvalues[0];
        turn = fromLVvalues[1];
        printLV3 = fromLVvalues[2];
        printLV4 = fromLVvalues[3];
        printLV5 = fromLVvalues[4];
        printLV6 = fromLVvalues[5];
        printLV7 = fromLVvalues[6];
        printLV8 = fromLVvalues[7];
    }

    if((numTimer1calls%62) == 0) { // change to the counter variable of you selected 4ms. timer : //pks11 // ex5 We want to communicate at 248 ms
        DataToLabView.floatData[0] = xR_K;
        DataToLabView.floatData[1] = yR_K;
        DataToLabView.floatData[2] = phiR;
        DataToLabView.floatData[3] = 2.0*((float)numTimer1calls)*.001;
        DataToLabView.floatData[4] = 3.0*((float)numTimer1calls)*.001;
        DataToLabView.floatData[5] = (float)numTimer1calls;
        DataToLabView.floatData[6] = (float)numTimer1calls*4.0;
        DataToLabView.floatData[7] = (float)numTimer1calls*5.0;
        LVsenddata[0] = '*'; // header for LVdata
        LVsenddata[1] = '$';
        for (int i=0;i<LVNUM_TOFROM_FLOATS*4;i++) {
            if (i%2==0) {
                LVsenddata[i+2] = DataToLabView.rawData[i/2] & 0xFF;
            } else {
                LVsenddata[i+2] = (DataToLabView.rawData[i/2]>>8) & 0xFF;
            }
        }
        serial_sendSCID(&SerialD, LVsenddata, 4*LVNUM_TOFROM_FLOATS + 2);
    }


    //pks11 //lab6 // ex1
    Vref_Left = Vref;
    Vref_Right = Vref;
    //assigning or reading radians value from readENCLeft and readEncRight
    LeftWheel = readEncLeft();
    RightWheel = readEncRight();

    //Left Distance
    LeftWheel_Dist = LeftRadius_final * LeftWheel;
    RightWheel_Dist = RightRadius_final * RightWheel;



    //pks11 //calculating radius
    //1 foot = radius *theta;
    //LeftRadius = 1 / LeftWheel;
    //RightRadius = 1 / RightWheel;

    //    LeftWheel_update = LeftWheel % (2*3.14);
    //    RightWheel_update = RightWheel % (2*3.14);

    //Updating exisitng location
    PosLeft_K = LeftWheel_Dist;
    PosRight_K = RightWheel_Dist;

    //    PosLeft_K = PosLeft_K_1 + LeftWheel_update * LeftRadius_final;
    //    PosRight_K = PosRight_K_1 + RightWheel_update * RightRadius_final;

    //calculating velocity at each time steps
    VLeftK = (PosLeft_K - PosLeft_K_1)/0.004;
    VRightK = (PosRight_K - PosRight_K_1)/ 0.004;

    //Calculate the turn error
    eK_turn = turn + (VLeftK - VRightK);


    //Design a controller
    eK_Left  = Vref_Left - VLeftK - Kpturn*eK_turn ;
    eK_Right = Vref_Right - VRightK + Kpturn*eK_turn;

    IK_Left = IK_1_Left + (0.004*(eK_Left + eK_Left_1))/2;
    IK_Right = IK_1_Right + (0.004*(eK_Right + eK_Right_1)/2);

    uLeft = Kp*eK_Left + Ki*IK_Left;
    uRight = Kp*eK_Right + Ki*IK_Right;

    //Designing an AntiWindup Controller
    //Note : I am not resetting, I am just holding the integrator
    if(uLeft > 10 || uLeft < -10)
    {
        IK_Left = IK_1_Left;

    }


    if(uRight > 10 || uRight < -10)
    {
        IK_Right = IK_1_Right;
    }



    //Updating POS
    PosLeft_K_1 = PosLeft_K;
    PosRight_K_1 = PosRight_K;
    eK_Left_1 = eK_Left;
    eK_Right_1  = eK_Right;
    IK_1_Left = IK_Left;
    IK_1_Right = IK_Right;


    //pks11// ex6
    //calculating x, y and pose of bearing
    RightWheel_K = RightWheel;
    LeftWheel_K = LeftWheel;
    phiR = (Rwh/Wr)*(RightWheel_K - LeftWheel_K);
    thetaAvg = 0.5*(RightWheel_K + LeftWheel_K);
    thetaAvg_dot = 0.5*((RightWheel_K - RightWheel_K_1) + (LeftWheel_K - LeftWheel_K_1))*250;
    xdot_K = Rwh*(thetaAvg_dot)*cos(phiR);
    ydot_K = Rwh*(thetaAvg_dot)*sin(phiR);

    //writing trapezoidal rule
    xR_K = xR_K_1 + (0.5)*(xdot_K + xdot_K_1)*(0.004);
    yR_K = yR_K_1 + (0.5)*(ydot_K + ydot_K_1)*(0.004);




    //pos update
    RightWheel_K_1 = RightWheel_K;
    LeftWheel_K_1 = LeftWheel_K;
    xdot_K_1 = xdot_K;
    ydot_K_1 = ydot_K;
    xR_K_1 = xR_K;
    yR_K_1 = yR_K;


    //giving the control input // changing uRight to uLeft as uLeft is associated t EPWM2B
    //Also, I need to negate the uLeft inorder it to go to positive direction
    setEPWM2A(uRight);
    setEPWM2B(-1 * uLeft);

    if ((CpuTimer0.InterruptCount % 100) == 0)
    {
        UARTPrint = 1;
    }





}

// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void)
{
    // Blink LaunchPad Blue LED
    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;

    CpuTimer2.InterruptCount++;

    //pks11 //ex2 // commenting out this interrupt for UARTPrint!
    // if ((CpuTimer2.InterruptCount % 10) == 0) {
    //   UARTPrint = 1;
    //}
}

//pks11 //ex1 // spi interrupt
__interrupt void SPIB_isr(void)
{
    //    spivalue1 = SpibRegs.SPIRXBUF; // Read first 16-bit value off RX FIFO. Probably is zero since no chip //pks11 // Not sure about this
    //    spivalue2 = SpibRegs.SPIRXBUF; // Read second 16-bit value off RX FIFO. first ADC value associated to pwmvalue
    //    spivalue3 = SpibRegs.SPIRXBUF; // Read third 16-bit value off RX FIFO, Second ADC value associated to pwmvalue
    //
    //    //pks11 //ex2 // these values are coming from ADCs, (Digital value varing from 0 - 4095) , changing this to voltage
    //    adc_volt1 =  (3.3/4096) * spivalue2;
    //    adc_volt2 =  (3.3/4096) * spivalue3;
    //
    //
    //
    //    GpioDataRegs.GPASET.bit.GPIO9 = 1; // Set GPIO9 high to end Slave Select. Now Scope. Later to deselect DAN28027 //pks11 //ex1
    //    // Later when actually communicating with the DAN28027 do something with the data. Now do nothing.

    dummy = SpibRegs.SPIRXBUF; // Read first receive value which is not useful for us as  they are related to INIT_Status
    Accel_X_Raw = SpibRegs.SPIRXBUF; // Accelerometer X data
    Accel_Y_Raw = SpibRegs.SPIRXBUF; // Acceleroeter Y data
    Accel_Z_Raw = SpibRegs.SPIRXBUF; // Accelerometer Z data
    Temp = SpibRegs.SPIRXBUF; // //Temp data // not gonna use it
    Gyro_X_Raw = SpibRegs.SPIRXBUF; // Gyro X data
    Gyro_Y_Raw = SpibRegs.SPIRXBUF; // Gyro Y data
    Gyro_Z_Raw = SpibRegs.SPIRXBUF; // Gyro Z data

    //We receive the value from -32768 to 32767, it should be within the range of -4g to 4g //pks11
    Accel_X = (8.0/65535.0)*(Accel_X_Raw);
    Accel_Y = (8.0/65535.0)*(Accel_Y_Raw);
    Accel_Z = (8.0/65535.0)*(Accel_Z_Raw);

    //Setting up a Gyro data, which has offset of -250 to 250 deg/s // pks11
    Gyro_X = (500.0/65535.0)*Gyro_X_Raw;
    Gyro_Y = (500.0/65535.0)*Gyro_Y_Raw;
    Gyro_Z = (500.0/65535.0)*Gyro_Z_Raw;


    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Set GPIO66 high to end Slave Select.

    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Clear Overflow flag just in case of an overflow
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1; // Clear RX FIFO Interrupt flag so next interrupt will happen

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6; // Acknowledge INT6 PIE interrupt
}

//pks11 //ex3
void setupSpib(void) //Call this function in main() somewhere after the DINT; line of code.
{
    int16_t temp = 0;
    ////Step 1.
    //// cut and paste here all the SpibRegs initializations you found for part 3. Make sure the TXdelay in
    ////between each transfer to 0. Also don�t forget to cut and paste the GPIO settings for GPIO9, 63, 64, 65,
    ////66 which are also a part of the SPIB setup.
    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0); // Set as GPIO9 and used as DAN28027 SS
    GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL); // Make GPIO9 an Output Pin
    GpioDataRegs.GPASET.bit.GPIO9 = 1; //Initially Set GPIO9/SS High so DAN28027 is not selected

    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0); // Set as GPIO66 and used as MPU-9250 SS
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL); // Make GPIO66 an Output Pin
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; //Initially Set GPIO66/SS High so MPU-9250 is not selected

    GPIO_SetupPinMux(63, GPIO_MUX_CPU1, 15); //Set GPIO63 pin to SPISIMOB //pks11/ ex1 // mux selection
    GPIO_SetupPinMux(64, GPIO_MUX_CPU1, 15); //Set GPIO64 pin to SPISOMIB //pks11/ ex1 // mux selection
    GPIO_SetupPinMux(65, GPIO_MUX_CPU1, 15); //Set GPIO65 pin to SPICLKB //pks11/ ex1 // mux selection


    EALLOW;
    GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0; // Enable Pull-ups on SPI PINs Recommended by TI for SPI Pins
    GpioCtrlRegs.GPCPUD.bit.GPIO64 = 0;
    GpioCtrlRegs.GPCPUD.bit.GPIO65 = 0;
    GpioCtrlRegs.GPBQSEL2.bit.GPIO63 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    GpioCtrlRegs.GPCQSEL1.bit.GPIO64 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    GpioCtrlRegs.GPCQSEL1.bit.GPIO65 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    EDIS;

    // ---------------------------------------------------------------------------
    SpibRegs.SPICCR.bit.SPISWRESET = 0; // Put SPI in Reset //pks11 // ex1 // reseting spi because we need to configure

    SpibRegs.SPICTL.bit.CLK_PHASE = 1; //This happens to be the mode for both the DAN28027 and
    SpibRegs.SPICCR.bit.CLKPOLARITY = 0; //The MPU-9250, Mode 01.
    SpibRegs.SPICTL.bit.MASTER_SLAVE = 1; // Set to SPI Master // pks11 // ex1// according to guide, if we are in reset configuration this SPI network become slave hence we need to set up high to create masters
    SpibRegs.SPICCR.bit.SPICHAR = 15; // Set to transmit and receive 16-bits each write to SPITXBUF //pks11// ex1 // we need to shift 16 bits together and hence SPICHAR will be Fh (15 in decimal)
    SpibRegs.SPICTL.bit.TALK = 1; // Enable transmission //pks11 // ex1// enabling master transmission
    SpibRegs.SPIPRI.bit.FREE = 1; // Free run, continue SPI operation
    SpibRegs.SPICTL.bit.SPIINTENA = 0; // Disables the SPI interrupt //pks11 //ex1 // disabling SPI interrupt

    SpibRegs.SPIBRR.bit.SPI_BIT_RATE = 49; // Set SCLK bit rate to 1 MHz so 1us period. SPI base clock is
    // 50MHZ. And this setting divides that base clock to create SCLK�s period //pks11 //ex1 // SPI Baud Rate = SPCLK / (SPIBRR + 1), SPIBRR = 49
    SpibRegs.SPISTS.all = 0x0000; // Clear status flags just in case they are set for some reason

    SpibRegs.SPIFFTX.bit.SPIRST = 1;// Pull SPI FIFO out of reset, SPI FIFO can resume transmit or receive. //PKS11 //ex1// taking out FIFO from rest configuration
    SpibRegs.SPIFFTX.bit.SPIFFENA = 1; // Enable SPI FIFO enhancements // pks11// ex1// enabling the enhancement
    SpibRegs.SPIFFTX.bit.TXFIFO = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpibRegs.SPIFFTX.bit.TXFFINTCLR = 1; // Write 1 to clear SPIFFTX[TXFFINT] flag just in case it is set

    SpibRegs.SPIFFRX.bit.RXFIFORESET = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Write 1 to clear SPIFFRX[RXFFOVF] just in case it is set
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1; // Write 1 to clear SPIFFRX[RXFFINT] flag just in case it is set //pks11 //ex1
    SpibRegs.SPIFFRX.bit.RXFFIENA = 1; // Enable the RX FIFO Interrupt. RXFFST >= RXFFIL //pks11 //ex1

    SpibRegs.SPIFFCT.bit.TXDLY = 16; //Set delay between transmits to 16 spi clocks. Needed by DAN28027 chip //pks11// ex1// the delay has nothing to do with data transfer bits, that is requirement of DAN's chip

    SpibRegs.SPICCR.bit.SPISWRESET = 1; // Pull the SPI out of reset //pks11 //ex1// taking out from the rest configuration

    SpibRegs.SPIFFTX.bit.TXFIFO = 1; // Release transmit FIFO from reset. //pks11 //ex1 // taking out the transmission from the rest
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 1; // Re-enable receive FIFO operation
    SpibRegs.SPICTL.bit.SPIINTENA = 1; // Enables SPI interrupt. !! I don�t think this is needed. Need to Test

    SpibRegs.SPIFFRX.bit.RXFFIL = 16; //Interrupt Level to 16 words or more received into FIFO causes interrupt. This is just the initial setting for the register. Will be changed below //pks11 //ex1

    SpibRegs.SPICCR.bit.SPICHAR = 0xF; // Making sure the data transfer will happen for 16 bits //pks11
    SpibRegs.SPIFFCT.bit.TXDLY = 0x00; // delay to 0
    //

    //Step 2.
    // perform a multiple 16-bit transfer to initialize MPU-9250 registers 0x13,0x14,0x15,0x16
    // 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C 0x1D, 0x1E, 0x1F. Use only one SS low to high for all these writes
    // some code is given, most you have to fill you yourself.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // Slave Select Low

    // Perform the number of needed writes to SPITXBUF to write to all 13 registers. Remember we are sending
    // 16-bit transfers, so two registers at a time after the first 16-bit transfer.




    // To address 00x13 write 0x00
    SpibRegs.SPITXBUF = 0x1300;
    //Auto incrementing 0x14 and 0x15 to give value 00
    SpibRegs.SPITXBUF = 0x0000;
    //Auto incrementing 0x16 and 0x17 to give value 00
    SpibRegs.SPITXBUF = 0x0000;
    //Auto incrementing 0x18 to give value 0 and 0x19 to give value 0x0013
    SpibRegs.SPITXBUF = 0x0013;
    //Auto incrementing 0x1A to value 0x02 and 1B to value 00
    SpibRegs.SPITXBUF = 0x0200;
    //Auto incrementing 0x1C to value 0x08 and 1D to value 06
    SpibRegs.SPITXBUF = 0x0806;
    //Auto incrementing to 0x1E and 0x1F to value 00
    SpibRegs.SPITXBUF = 0x0000;

    // wait for the correct number of 16-bit values to be received into the RX FIFO
    while(SpibRegs.SPIFFRX.bit.RXFFST !=7);//pks11 total 7 - 16 bits data// Receive buffer to acknolwedge all data has been transfered
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave Select High
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    // read the additional number of 6 (total 7)  values off the RX FIFO to clear out the RX FIFO
    DELAY_US(10); // Delay 10us to allow time for the MPU-2950 to get ready for next transfer.

    //Step 3
    // perform a multiple 16-bit transfer to initialize MPU-9250 registers 0x23,0x24,0x25,0x26
    // 0x27, 0x28, 0x29. Use only one SS low to high for all these writes
    // some code is given, most you have to fill you yourself.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;

    // Perform the number of needed writes to SPITXBUF to write to all 7 registers
    // To address 00x23 write 0x00
    SpibRegs.SPITXBUF = 0x2300;
    // Auto incrementing 0x24 to set value 0x40 and 0x25 to 0x8c
    SpibRegs.SPITXBUF = 0x408C;
    // Auto incrementing 0x26 to set value 0x02 and 0x27 to 0x88
    SpibRegs.SPITXBUF = 0x0288;
    // Auto incrementing 0x28 to set value 0x0c and 0x29 to 0x8A
    SpibRegs.SPITXBUF = 0x0C8A;

    // wait for the correct number of 16-bit values to be received into the RX FIFO
    while(SpibRegs.SPIFFRX.bit.RXFFST !=4); // pks11 // ex3 // 4 16 bits data
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave Select High
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    //  read the additional number 3 (total 4)  of garbage receive values off the RX FIFO to clear out the RX FIFO
    DELAY_US(10); // Delay 10us to allow time for the MPU-2950 to get ready for next transfer.

    //Step 4.
    // perform a single 16-bit transfer to initialize MPU-9250 register 0x2A
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    // Write to address 0x2A the value 0x81 //pks11 // writing this 0x2A address to value 0x81
    SpibRegs.SPITXBUF = 0x2A81;
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
    // SpibRegs.SPITXBUF = (0x7700 | 0x00EB); // 0x7700
    SpibRegs.SPITXBUF = (0x7700 | 0x00E8); // 0x7700 // pks11 //ex3 // setting offset to 0 first to find value of offset which is 2.934 // E890 is equivalent to -2.394 (2.934/0.00098 = 3000, we need to give negative of that and left shift by 1 as it is 15 bit register)
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    // SpibRegs.SPITXBUF = (0x7800 | 0x0012); // 0x7800
    SpibRegs.SPITXBUF = (0x7800 | 0x0090); // 0x7800 //pks11 // E890 from bits [6-0] which will be 0x0090
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    // SpibRegs.SPITXBUF = (0x7A00 | 0x0010); // 0x7A00
    SpibRegs.SPITXBUF = (0x7A00 | 0x000E); // 0x7A00 // pks11 // setting offset to 0 first to find value of offset which is -1.845 // EB6 is equivalent to 1.845 ( 1.845/0.0098 = 1883, we need to give the same value and left shit by 1 as it is 15 bit register)
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    // SpibRegs.SPITXBUF = (0x7B00 | 0x00FA); // 0x7B00
    SpibRegs.SPITXBUF = (0x7B00 | 0x00B6); // 0x7B00 // pks11 // 0EB6  from bits [6-0] will be B6
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    // SpibRegs.SPITXBUF = (0x7D00 | 0x0021); // 0x7D00
    SpibRegs.SPITXBUF = (0x7D00 | 0x0026); // 0x7D00 // pks11 //setting offset to 0 first to find value of offset which is -4.817 (Note that :Maximum saturation is -4 however, I need to reevalaute after giving offset to 4, which gives me -0.817 offset) // 4.817/ 0.00098 : 4915.3, we need to give the same value and left shift by 1 as it is 15 bit register)
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    // SpibRegs.SPITXBUF = (0x7E00 | 0x0050); // 0x7E00
    SpibRegs.SPITXBUF = (0x7E00 | 0x0066); // 0x7E00 //pks11 // offset to 0
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(50);

    // Clear SPIB interrupt source just in case it was issued due to any of the above initializations.
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR=1; // Clear Overflow flag
    SpibRegs.SPIFFRX.bit.RXFFINTCLR=1; // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
}

//pks11 // lab 6 // setting up function
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

//pks11
//reading left encoder value

float readEncLeft(void) {
    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU; //4294967295U
    raw = EQep1Regs.QPOSCNT;
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue; // I don't think this is needed and never true
    // 100 slits in the encoder disk so 100 square waves per one revolution of the
    // DC motor's back shaft. Then Quadrature Decoder mode multiplies this by 4 so 400 counts per one rev
    //pks11 lab 6
    // of the DC motor's back shaft. Then the gear motor's gear ratio is 30:1.
    // logic behind this calculation : 30 revolution is equivalent to 2pi radians, 1 revalution (400 counts) is equivalent to (2*pi/30)
    // 1 count is equivalent to (2*pi/30)/400 = pi/6000
    //left wheel i giving negative value hence, factor pi/600 will be multiplied by -1
    return (raw*(-1*0.0005233));
}
//pks11
//reading right encoder value
float readEncRight(void) {
    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU; //4294967295U -1 32bit signed int
    raw = EQep2Regs.QPOSCNT;
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue; // I don't think this is needed and never true
    // 100 slits in the encoder disk so 100 square waves per one revolution of the
    // DC motor's back shaft. Then Quadrature Decoder mode multiplies this by 4 so 400 counts per one rev
    // of the DC motor's back shaft. Then the gear motor's gear ratio is 30:1.
    //pks11 lab 6
    // of the DC motor's back shaft. Then the gear motor's gear ratio is 30:1.
    // logic behind this calculatio : 30 revolution is equivalent to 2pi radians, 1 revalution (400 counts) is equivalent to (2*pi/30)
    // 1 count is equivalent to (2*pi/30)/400 = pi/600
    return (raw*(0.0005233));
}

//pks11 // ex2 // lab6
//copy & paste from lab3
void setEPWM2A(float controleffort)
{
    //pks11
    //Saturating input parameter
    if (controleffort > 10.0)
    {
        controleffort = 10;
    }

    if (controleffort < -10)
    {
        controleffort = -10;
    }
    //pks11
    //ex2
    //linear scaling to make sure 0% duty cycle is at controleffort -10, 50% dutycycle is at controleffort 0
    EPwm2Regs.CMPA.bit.CMPA = ((controleffort + 10.0)/(20.0))* (EPwm2Regs.TBPRD);
}

void setEPWM2B(float controleffort)
{
    //pks11
    //Saturating input parameter
    if (controleffort > 10)
    {
        controleffort = 10;
    }

    if (controleffort < -10)
    {
        controleffort = -10;
    }
    //pks11
    //ex2
    //linear scaling to make sure 0% duty cycle is at controleffort -10, 50% dutycycle is at controleffort 0
    EPwm2Regs.CMPB.bit.CMPB = ((controleffort + 10.0)/(20.0))* (EPwm2Regs.TBPRD);
}
