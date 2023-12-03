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
__interrupt void ADCD_ISR (void);
//pks //ex3
__interrupt void ADCA_ISR (void);
//PKS //EX4
__interrupt void ADCB_ISR (void);
// Count variables
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls = 0;
extern uint32_t numRXA;
uint16_t UARTPrint = 0;
uint16_t LEDdisplaynum = 0;
//pks //  global variable
uint16_t adcd0result = 0;
uint16_t adcd1result = 0;

//pks //global variable ex3
uint16_t adca0result = 0;
uint16_t adca1result = 0;

//pks //global variable ex4
uint16_t adcb0result = 0;

float adc0Volt = 0;
///pks //global varibale ex3
float adc1Volt = 1;
uint32_t count_ISR = 0; //PKS // For ADC INterupt function

//pks Lab4 ex2.
float xk = 0;
float xk_1 = 0;
float xk_2 = 0;
float xk_3 = 0;
float xk_4 = 0;
//yk is the filtered value
float yk = 0;
//pks11 //ex3
float yk1 = 0;
float yk2 = 0;
float yk3 = 0; //ex4 pks11

//pks11 //ex4
float yk4 = 0;

float yk1_result = 0;
float yk2_result = 0;
float yk3_result = 0; //ex4 pks11
//b is the filter coefficients
//pks average filter
//float b[5] = {0.2,0.2,0.2,0.2,0.2}; // 0.2 is 1/5th therefore a 5 point average
//pks fir filter


//pks initializing array for x_k
#define array_size  22
//uint16_t array_size = 5;
float x_k[array_size] = {0};
uint16_t i_array = 0;

//pks initializing new 2 array for ex3
#define array1_size  22
//uint16_t array_size = 5;
float x1_k[array1_size] = {0};

#define array2_size  22
//uint16_t array_size = 5;
float x2_k[array2_size] = {0};

#define array3_size 32
float x3_k[array3_size] = {0};

//pks11 ex4 bandpass filter order 100 with frequency range 1900Hz to 2100Hz
#define array4_size 101
float x4_k[array4_size] = {0};
float b[101]={  -4.1384865093955942e-18,    2.4158288163475484e-05, -1.3320677588450310e-04,    -2.1438469575543533e-04,    1.1898399936030848e-04, 5.3082205069710680e-04, 2.1893290271722703e-04, -7.4768481245699380e-04,    -9.5792023943993328e-04,    4.6168341621969679e-04, 1.8598657706234230e-03, 7.0670080707196015e-04, -2.2492456754091747e-03,    -2.7055293027965603e-03,    1.2307634272343665e-03, 4.6993269406780981e-03, 1.6984303126684232e-03, -5.1577427312871921e-03,    -5.9361687426490355e-03,    2.5904429699616822e-03, 9.5104864390879260e-03, 3.3122378905612003e-03, -9.7118714382866452e-03,    -1.0812123641265282e-02,    4.5715859206989177e-03, 1.6287321385412081e-02, 5.5122975619431172e-03, -1.5726675339333283e-02,    -1.7056081487002734e-02,    7.0329483752597077e-03, 2.4459678182842035e-02, 8.0882772704277336e-03, -2.2565290379886044e-02,    -2.3949227569375457e-02,    9.6706866781569138e-03, 3.2957303117234021e-02, 1.0685317933349465e-02, -2.9243552530078470e-02,    -3.0461077862757931e-02,    1.2077118105660343e-02, 4.0427773465971463e-02, 1.2879264031643200e-02, -3.4645501075422983e-02,    -3.5481182001261206e-02,    1.3834430631479126e-02, 4.5553192553114567e-02, 1.4277570256188015e-02, -3.7792491047513456e-02,    -3.8090059866479127e-02,    1.4617663668474229e-02, 4.7377897417654163e-02, 1.4617663668474229e-02, -3.8090059866479127e-02,    -3.7792491047513456e-02,    1.4277570256188015e-02, 4.5553192553114567e-02, 1.3834430631479126e-02, -3.5481182001261206e-02,    -3.4645501075422983e-02,    1.2879264031643200e-02, 4.0427773465971463e-02, 1.2077118105660343e-02, -3.0461077862757931e-02,    -2.9243552530078470e-02,    1.0685317933349465e-02, 3.2957303117234021e-02, 9.6706866781569138e-03, -2.3949227569375457e-02,    -2.2565290379886044e-02,    8.0882772704277336e-03, 2.4459678182842035e-02, 7.0329483752597077e-03, -1.7056081487002734e-02,    -1.5726675339333283e-02,    5.5122975619431172e-03, 1.6287321385412081e-02, 4.5715859206989177e-03, -1.0812123641265282e-02,    -9.7118714382866452e-03,    3.3122378905612003e-03, 9.5104864390879260e-03, 2.5904429699616822e-03, -5.9361687426490355e-03,    -5.1577427312871921e-03,    1.6984303126684232e-03, 4.6993269406780981e-03, 1.2307634272343665e-03, -2.7055293027965603e-03,    -2.2492456754091747e-03,    7.0670080707196015e-04, 1.8598657706234230e-03, 4.6168341621969679e-04, -9.5792023943993328e-04,    -7.4768481245699380e-04,    2.1893290271722703e-04, 5.3082205069710680e-04, 1.1898399936030848e-04, -2.1438469575543533e-04,    -1.3320677588450310e-04,    2.4158288163475484e-05, -4.1384865093955942e-18};

//float b[5]={    3.3833240118424500e-02,
//    2.4012702387971543e-01,
//    4.5207947200372001e-01,
//    2.4012702387971543e-01,
//    3.3833240118424500e-02};

//float b[22]={   -2.3890045153263611e-03,
//                -3.3150057635348224e-03,
//                -4.6136191242627002e-03,
//                -4.1659855521681268e-03,
//                1.4477422497795286e-03,
//                1.5489414225159667e-02,
//                3.9247886844071371e-02,
//                7.0723964095458614e-02,
//                1.0453473887246176e-01,
//                1.3325672639406205e-01,
//                1.4978314227429904e-01,
//                1.4978314227429904e-01,
//                1.3325672639406205e-01,
//                1.0453473887246176e-01,
//                7.0723964095458614e-02,
//                3.9247886844071371e-02,
//                1.5489414225159667e-02,
//                1.4477422497795286e-03,
//                -4.1659855521681268e-03,
//                -4.6136191242627002e-03,
//                -3.3150057635348224e-03,
//                -2.3890045153263611e-03};

//float b[32]= {   -6.3046914864397922e-04,    -1.8185681242784432e-03,    -2.5619416124584822e-03,    -1.5874939943956356e-03,    2.3695126689747326e-03, 8.3324969783531780e-03, 1.1803612855040625e-02, 6.7592967793297151e-03, -9.1745119977290398e-03,    -2.9730906886035850e-02,    -3.9816452266421651e-02,    -2.2301647638687881e-02,    3.1027965907247105e-02, 1.1114350049251465e-01, 1.9245540210070616e-01, 2.4373020388648489e-01, 2.4373020388648489e-01, 1.9245540210070616e-01, 1.1114350049251465e-01, 3.1027965907247105e-02, -2.2301647638687881e-02,    -3.9816452266421651e-02,    -2.9730906886035850e-02,    -9.1745119977290398e-03,    6.7592967793297151e-03, 1.1803612855040625e-02, 8.3324969783531780e-03, 2.3695126689747326e-03, -1.5874939943956356e-03,    -2.5619416124584822e-03,    -1.8185681242784432e-03,    -6.3046914864397922e-04};

//pks lab4
//This function sets DACA to the voltage between 0V and 3V passed to this function.
//If outside 0V to 3V the output is saturated at 0V to 3V
//Example code
//float myu = 2.25;
//setDACA(myu); // DACA will now output 2.25 Volts
void setDACA(float dacouta0) {
    int16_t DACOutInt = 0;
    DACOutInt = (4096.0/3.0)*dacouta0; // perform scaling of 0 – almost 3V to 0 - 4095 // pks11 //  using resolution of 4096 instead of 4095
    if (DACOutInt > 4095) DACOutInt = 4095;
    if (DACOutInt < 0) DACOutInt = 0;
    DacaRegs.DACVALS.bit.DACVALS = DACOutInt;
}
void setDACB(float dacouta1) {
    int16_t DACOutInt = 0;
    DACOutInt =(4096.0/3.0)*dacouta1 ; // perform scaling of 0 – almost 3V to 0 - 4095 // pks11 // using resolution of 4096 instead of 4095
    if (DACOutInt > 4095) DACOutInt = 4095;
    if (DACOutInt < 0) DACOutInt = 0;
    DacbRegs.DACVALS.bit.DACVALS = DACOutInt;
}




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

    // pks
    // Setting GPIO 52 as output and to see time taken by my code to run
    GPIO_SetupPinMux(52, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(52, GPIO_OUTPUT, GPIO_PUSHPULL);
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
    PieVectTable.SCIB_RX_INT = &RXBINT_recv_ready;
    PieVectTable.SCIC_RX_INT = &RXCINT_recv_ready;
    PieVectTable.SCID_RX_INT = &RXDINT_recv_ready;
    PieVectTable.SCIA_TX_INT = &TXAINT_data_sent;
    PieVectTable.SCIB_TX_INT = &TXBINT_data_sent;
    PieVectTable.SCIC_TX_INT = &TXCINT_data_sent;
    PieVectTable.SCID_TX_INT = &TXDINT_data_sent;

    //pks11 lab4
    //PieVectTable.ADCD1_INT = &ADCD_ISR;
    //PieVectTable.EMIF_ERROR_INT = &SWI_isr;

    //pks11 lab4 ex3 : ADCA1 address
    //PieVectTable.ADCA1_INT = &ADCA_ISR;
    PieVectTable.EMIF_ERROR_INT = &SWI_isr;

    PieVectTable.ADCB1_INT = &ADCB_ISR;

    EDIS;    // This is needed to disable write to EALLOW protected registers




    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every given period:
    // 200MHz CPU Freq,                       Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, LAUNCHPAD_CPU_FREQUENCY, 10000);
    ConfigCpuTimer(&CpuTimer1, LAUNCHPAD_CPU_FREQUENCY, 20000);
    ConfigCpuTimer(&CpuTimer2, LAUNCHPAD_CPU_FREQUENCY, 40000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    init_serialSCIA(&SerialA,115200);

    //pks11
    //lab4ex1

    EALLOW;
    EPwm5Regs.ETSEL.bit.SOCAEN = 0; // Disable SOC on A group
    EPwm5Regs.TBCTL.bit.CTRMODE = 3; // freeze counter
    EPwm5Regs.ETSEL.bit.SOCASEL = 2; // Select Event when counter equal to PRD //PKS11 // IT WILL BE 010 according to guide
    EPwm5Regs.ETPS.bit.SOCAPRD = 1; // Generate pulse on 1st event (“pulse” is the same as “trigger”) //PKS11 // It will be 001 according to guide
    EPwm5Regs.TBCTR = 0x0; // Clear counter
    EPwm5Regs.TBPHS.bit.TBPHS = 0x0000; // Phase is 0
    EPwm5Regs.TBCTL.bit.PHSEN = 0; // Disable phase loading
    EPwm5Regs.TBCTL.bit.CLKDIV = 0; // divide by 1 50Mhz Clock
    //EPwm5Regs.TBPRD = 50000; // Set Period to 1ms sample. Input clock is 50MHz. //pks11 // (Clock Freq* period sample)
    //EPwm5Regs.TBPRD = 12500; // Set Period to 0.25ms sample. Input clock is 50MHz. //pks11 // (Clock Freq* period sample)
    EPwm5Regs.TBPRD = 5000; // Set Period to 0.1ms (10,000 Hz) sample. Input clock is 50MHz. //pks11 // (Clock Freq* period sample)
    // Notice here that we are not setting CMPA or CMPB because we are not using the PWM signal
    EPwm5Regs.ETSEL.bit.SOCAEN = 1; //enable SOCA
    EPwm5Regs.TBCTL.bit.CTRMODE = 0; //unfreeze, and enter up count mode //PKS11/ UPCOUNT MODE
    EDIS;

    // pks11
    // adding ADC initialisation
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
    //Select the channels to convert and end of conversion flag
    //Many statements commented out, To be used when using ADCA or ADCB
    //ADCA
    //PKS11/
    //EX3 : Do not forget to remeber we are gonna use ADCINA2 (CHSEL : 2) and ADCINA3 (chsel : 3)
    //    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 2; // ADCINA2 : SOC0 will convert Channel you choose Does not have to be A0
    //    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    //    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 13;// EPWM5 ADCSOCA or another trigger you choose will trigger SOC0
    //    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 3; //ADCINA3 : SOC1 will convert Channel you choose Does not have to be A1
    //    AdcaRegs.ADCSOC1CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    //    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 13;// EPWM5 ADCSOCA or another trigger you choose will trigger SOC1
    //    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 1; //set to last SOC that is converted and it will set INT1 flag ADCA1
    //    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1; //enable INT1 flag
    //    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared

    //ADCB
    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 4; // ADCINB4 : SOC0 will convert Channel you choose Does not have to be B0
    AdcbRegs.ADCSOC0CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 13; // EPWM5 ADCSOCA or another trigger you choose will trigger SOC0
    //AdcbRegs.ADCSOC1CTL.bit.CHSEL = ???; //SOC1 will convert Channel you choose Does not have to be B1
    //AdcbRegs.ADCSOC1CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    //AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = ???; // EPWM5 ADCSOCA or another trigger you choose will trigger SOC1
    //AdcbRegs.ADCSOC2CTL.bit.CHSEL = ???; //SOC2 will convert Channel you choose Does not have to be B2
    //AdcbRegs.ADCSOC2CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    //AdcbRegs.ADCSOC2CTL.bit.TRIGSEL = ???; // EPWM5 ADCSOCA or another trigger you choose will trigger SOC2
    //AdcbRegs.ADCSOC3CTL.bit.CHSEL = ???; //SOC3 will convert Channel you choose Does not have to be B3
    //AdcbRegs.ADCSOC3CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    //AdcbRegs.ADCSOC3CTL.bit.TRIGSEL = ???; // EPWM5 ADCSOCA or another trigger you choose will trigger SOC3
    AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 0; //set to last SOC that is converted and it will set INT1 flag ADCB1
    AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1; //enable INT1 flag
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared

    //ADCD
    //    AdcdRegs.ADCSOC0CTL.bit.CHSEL = 0; // set SOC0 to convert pin D0
    //    AdcdRegs.ADCSOC0CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    //    AdcdRegs.ADCSOC0CTL.bit.TRIGSEL = 13; // EPWM5 ADCSOCA will trigger SOC0 // pks11// it will be 0Dh 0D in hexa which convert to 13
    //    AdcdRegs.ADCSOC1CTL.bit.CHSEL = 1; //set SOC1 to convert pin D1 //pks11/ 1 in hexa is 1
    //    AdcdRegs.ADCSOC1CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    //    AdcdRegs.ADCSOC1CTL.bit.TRIGSEL = 13; // EPWM5 ADCSOCA will trigger SOC1 // pks11// it will be 0Dh 0D in hexa which convert to 13
    //    //AdcdRegs.ADCSOC2CTL.bit.CHSEL = ???; //set SOC2 to convert pin D2
    //    //AdcdRegs.ADCSOC2CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    //    //AdcdRegs.ADCSOC2CTL.bit.TRIGSEL = ???; // EPWM5 ADCSOCA will trigger SOC2
    //    //AdcdRegs.ADCSOC3CTL.bit.CHSEL = ???; //set SOC3 to convert pin D3
    //    //AdcdRegs.ADCSOC3CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    //    //AdcdRegs.ADCSOC3CTL.bit.TRIGSEL = ???; // EPWM5 ADCSOCA will trigger SOC3
    //    AdcdRegs.ADCINTSEL1N2.bit.INT1SEL = 1; //set to SOC1, the last converted, and it will set INT1 flag ADCD1 // end of conversion and I am dtelling that it will be done after SOC1 is triggered
    //    AdcdRegs.ADCINTSEL1N2.bit.INT1E = 1; //enable INT1 flag
    //    AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
    EDIS;



    // Enable DACA and DACB outputs
    EALLOW;
    DacaRegs.DACOUTEN.bit.DACOUTEN = 1; //enable dacA output-->uses ADCINA0
    DacaRegs.DACCTL.bit.LOADMODE = 0; //load on next sysclk
    DacaRegs.DACCTL.bit.DACREFSEL = 1; //use ADC VREF as reference voltage
    DacbRegs.DACOUTEN.bit.DACOUTEN = 1; //enable dacB output-->uses ADCINA1
    DacbRegs.DACCTL.bit.LOADMODE = 0; //load on next sysclk
    DacbRegs.DACCTL.bit.DACREFSEL = 1; //use ADC VREF as reference voltage
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
    //PKS Enabling Pie Interupt 1.6
    //PieCtrlRegs.PIEIER1.bit.INTx6 = 1;

    //PKS ex3 Enabaling Pie Interupt 1.1 for ADCA1
    //PieCtrlRegs.PIEIER1.bit.INTx1 = 1;

    //PKS EX4 Enabling Pie Interupt for ADCB1 : 1.2
    PieCtrlRegs.PIEIER1.bit.INTx2 = 1;

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
            serial_printf(&SerialA,"Value 1 :%f Value 2 : %f \r\n",yk1_result,yk2_result);
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

    if ((CpuTimer2.InterruptCount % 10) == 0) {
        UARTPrint = 1;
    }
}

//pks11 // interupt function hen ADCIN0 and ADCIN1 inputs given

//adcd1 pie interrupt
__interrupt void ADCD_ISR (void)
{
    adcd0result = AdcdResultRegs.ADCRESULT0;
    adcd1result = AdcdResultRegs.ADCRESULT1;
    // Here covert ADCIND0 to volts
    adc0Volt = (3.0/4096.0)*adcd0result;

    //ex2 lab 4 pks
    //xk =  adc0Volt;
    //yk = b[0]*xk + b[1]*xk_1 + b[2]*xk_2 + b[3]*xk_3 + b[4]*xk_4;


    // for loop for ex2 lab4
    x_k[0] = adc0Volt;
    for (i_array = 0; i_array < array_size; i_array++)
    {
        yk = x_k[i_array] * b[i_array] + yk;
    }

    for (i_array = array_size-1; i_array > 0 ; i_array--)
    {
        x_k[i_array] = x_k[i_array-1];

    }

    //Save past states before exiting from the function so that next sample they are the older state
    //    xk_4 = xk_3;
    //    xk_3 = xk_2;
    //    xk_2 = xk_1;
    //    xk_1 = xk;
    // Here write voltages value to DACA
    //setDACA(adc0Volt);
    //pks11
    setDACA(yk);
    yk = 0;

    // Print ADCIND0’s voltage value to TeraTerm every 100ms
    count_ISR++;

    if((count_ISR % 100) == 0)
    {
        UARTPrint = 1;
    }

    AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
//PKS11
//EX3
__interrupt void ADCA_ISR (void)
{
    adca0result = AdcaResultRegs.ADCRESULT0;
    adca1result = AdcaResultRegs.ADCRESULT1;
    // Here covert ADCIND0 to volts
    adc0Volt = (3.0/4096.0)*adca0result;
    adc1Volt = (3.0/4096.0)*adca1result;
    //ex2 lab 4 pks
    //xk =  adc0Volt;
    //yk = b[0]*xk + b[1]*xk_1 + b[2]*xk_2 + b[3]*xk_3 + b[4]*xk_4;


    // for loop for ex3 lab4 : Loop 1 : Joystick 1
    x1_k[0] = adc0Volt;
    for (i_array = 0; i_array < array_size; i_array++)
    {
        yk1 = x1_k[i_array] * b[i_array] + yk1;
    }

    for (i_array = array_size-1; i_array > 0 ; i_array--)
    {
        x1_k[i_array] = x1_k[i_array-1];

    }
    yk1_result = yk1;
    setDACA(yk1);


    // for loop for ex3 lab4 : Loop 1 : Joystick 2
    x2_k[0] = adc1Volt;
    for (i_array = 0; i_array < array_size; i_array++)
    {
        yk2 = x2_k[i_array] * b[i_array] + yk2;
    }

    for (i_array = array_size-1; i_array > 0 ; i_array--)
    {
        x2_k[i_array] = x2_k[i_array-1];

    }
    yk2_result = yk2;
    setDACB(yk2);


    //Save past states before exiting from the function so that next sample they are the older state
    //    xk_4 = xk_3;
    //    xk_3 = xk_2;
    //    xk_2 = xk_1;
    //    xk_1 = xk;
    // Here write voltages value to DACA
    //setDACA(adc0Volt);
    //pks11
    //    setDACA(yk);
    //    yk = 0;

    // Print ADCIND0’s voltage value to TeraTerm every 100ms
    count_ISR++;

    if((count_ISR % 100) == 0)
    {
        UARTPrint = 1;
    }
    yk1 = 0;
    yk2 = 0;

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

}

__interrupt void ADCB_ISR (void)
{
    //pks ex4 part 3 setting GPIO high
    GpioDataRegs.GPBSET.bit.GPIO52 = 1;


    adcb0result = AdcbResultRegs.ADCRESULT0;
    adc0Volt = (3.0/4096.0)*adcb0result;

    //    x3_k[0] = adc0Volt;
//    for (i_array = 0; i_array < array3_size; i_array++)
//    {
//        yk3 = x3_k[i_array] * b[i_array] + yk3;
//    }
//
//    for (i_array = array3_size-1; i_array > 0 ; i_array--)
//    {
//        x3_k[i_array] = x3_k[i_array-1];
//
//    }
//    yk3_result = yk3;
//
//    setDACA(yk3);
//
//    yk3 = 0;

    x4_k[0] = adc0Volt;
    for (i_array = 0; i_array < array4_size; i_array++)
    {
        yk4 = x4_k[i_array] * b[i_array] + yk4;
    }

    for (i_array = array4_size-1; i_array > 0 ; i_array--)
    {
        x4_k[i_array] = x4_k[i_array-1];

    }
//pks11 // ex4 // I have added 1.5 v offset because DAC varies from 0 -3 volt so taking avereage of that will be 3/2 = 1.5v so that we can see full sinusoidal
    setDACA(yk4+1.5);

    yk4 = 0;

    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

    //gPIGPIO clear bit
    GpioDataRegs.GPBCLEAR.bit.GPIO52 = 1;



}



