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
#define PI_INV      0.318309886183790671537767526745
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
// The Launchpad's CPU Frequency set to 200 you should not change this value
#define LAUNCHPAD_CPU_FREQUENCY 200
#define FALSE 0
#define TRUE 1
#define SIZEOFARRAY 22

// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI_isr(void);
__interrupt void SPIB_isr(void);
__interrupt void ADCA_ISR (void);

// Function Definitions
void setupSpib(void);
void init_eQEPs(void);
float readEncLeft(void);
float readEncRight(void);
void setEPWM2A(float controleffort);
void setEPWM2B(float controleffort);
float min_val(float a, float b);
int xy_control(float turn_thres, float x_pos,float y_pos,float x_desired,float y_desired,
               float thetaabs,float target_radius,float target_radius_near);

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numTimer1calls = 0;
uint32_t numSWIcalls = 0;
extern uint32_t numRXA;
uint16_t UARTPrint = 0;
uint16_t LEDdisplaynum = 0;

// IMU readings
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

// Wheel Variables
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

// Balance Control
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

// Turn Control
float Kpturn = 3;
float turn = 0;
float eK_turn = 0;
float eK_turn_1 = 0;
float turnerror = 0;
float alpha = 0;
float alpha_old = 0;

// Lab View variables
float printLV3 = 0;
float printLV4 = 0;
float printLV5 = 0;
float printLV6 = 0;
float printLV7 = 0;
float printLV8 = 0;
float mode = 0;
float x = 0;
float y = 0;
float bearing = 0;
extern uint16_t NewLVData;
extern float fromLVvalues[LVNUM_TOFROM_FLOATS];
extern LVSendFloats_t DataToLabView;
extern char LVsenddata[LVNUM_TOFROM_FLOATS*4+2];
extern uint16_t newLinuxCommands;
extern float LinuxCommands[CMDNUM_FROM_FLOATS];

// Wheel
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


// IMU data filter
int count_ISR = 0;
float accelx = 0;
float accely = 0;
float accelz = 0;
float gyrox = 0;
float gyroy = 0;
float gyroz = 0;
float LeftWheel_New = 0;
float RightWheel_New = 0;
int countSPIB_ISR =0;

// Needed global Variables
float accelx_offset = 0;
float accely_offset = 0;
float accelz_offset = 0;
float gyrox_offset = 0;
float gyroy_offset = 0;
float gyroz_offset = 0;
float accelzBalancePoint = -.64;
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

float vel_Left_K = 0;
float vel_Left_K_1 = 0;
float vel_Right_K = 0;
float vel_Right_K_1 = 0;
float gyrorate_dot_K = 0;
float gyrorate_dot_K_1 = 0;
float gyro_value_K = 0;
float gyro_value_K_1 = 0;
float ubal = 0;
float K1 = -60.0;
float K2 = -4.5;
float K3 = -1.1;
float K4 = -0.1;

// Balance and Turn control variables
float WhlDiff = 0;
float WhlDiff_K = 0;
float WhlDiff_K_1 = 0;
float vel_WhlDiff = 0;
float vel_WhlDiff_K = 0;
float vel_WhlDiff_K_1 = 0;
float turn_angle = 0;
float turn_angle_K = 0;
float turn_angle_K_1 = 0;
float turnref = 0;
float turnref_K = 0;
float turnref_K_1 = 0;
float errorDiff = 0;
float errorDiff_K = 0;
float errorDiff_K_1 = 0;
float intDiff_K = 0;
float intDiff_K_1 = 0;
float ubalturn = 0;
float Kp_turn = 3.0;
float Ki_turn = 20.0;
float Kd_turn = 0.08;
float uleft_balturn = 0;
float uright_balturn = 0;

float Kp_speed = 0.35;
float Ki_speed = 1.5;
float errorSpeed = 0;
float refV = 0;
float avgSpeed = 0;
float errorSpeed_K = 0;
float errorSpeed_K_1 = 0;
float intSpeed_K =0;
float intSpeed_K_1 = 0;
float forwardback_command = 0;

// XY control
float vref_forxy = 0;
float turn_forxy = 0;
float x_nav = 0;
float y_nav = 0;
int target_near = 0;

// Obstacle
float left_obs_det = 0;
float center_obs_det = 0;
float right_obs_det = 0;
float alpha_turn_left= 0;
float alpha_turn_right = 0;
float count_turn = 0;
float right_count = 0;
float left_count = 0;
float delay_count = 0;


// KH & LJM: Initialization of variables for Right Wall Following
int16_t right_wall_follow_state = 2; //By default, do the right wall follow which is case 2
float c[SIZEOFARRAY]={   -2.3890045153263611e-03,    -3.3150057635348224e-03,    -4.6136191242627002e-03,    -4.1659855521681268e-03,    1.4477422497795286e-03, 1.5489414225159667e-02, 3.9247886844071371e-02, 7.0723964095458614e-02, 1.0453473887246176e-01, 1.3325672639406205e-01, 1.4978314227429904e-01, 1.4978314227429904e-01, 1.3325672639406205e-01, 1.0453473887246176e-01, 7.0723964095458614e-02, 3.9247886844071371e-02, 1.5489414225159667e-02, 1.4477422497795286e-03, -4.1659855521681268e-03,    -4.6136191242627002e-03,    -3.3150057635348224e-03,    -2.3890045153263611e-03};
//KH & LJM: Initializing raw reading array of ADCINA2 and 3 for reading IR Sensor values
uint16_t raw_ADCINA2 = 0;
uint16_t raw_ADCINA3 = 0;
uint16_t raw_ADCINA4 = 0;
//KH & LJM: Initializing arrays for pre-filtered values of IR sensors
float xk2[SIZEOFARRAY];
float xk3[SIZEOFARRAY];
float xk4[SIZEOFARRAY];
//KH & LJM: Initializing arrays for filtered values of IR sensors
float yk2 = 0;
float yk3 = 0;
float yk4 = 0;
float dist_IR_ADCINA2 = 0;
float dist_IR_ADCINA3 = 0;
float dist_IR_ADCINA4 = 0;
float left_wall_Start_threshold = 2.2;
float right_wall_Start_threshold = 2.2;

float forward_velocity = 0.5;
float Kp_right_front_wall = 0.8;
float Kp_left_front_wall = 1.1;
float Kp_left_wall = 1.2;
float Kp_right_wall = 1.2;
float ref_right_wall = 2.3;
float ref_left_wall = 2.6;
uint16_t machine_state = 3;
float left_turn_Start_threshold = 2.0;
float left_turn_Stop_threshold = 2.3;
float alpha_K_1 = 0;


// Follow Human
uint16_t follow_machine_state = 0;
uint16_t count_follow1 = 0;
float distance_ref = 2.1;
float Kp_follow = 3.0;
float leftfollow_count = 0;
float rightfollow_count = 0;
float ref_sensor_diff = 0.1;
float Kp_turnleft_follow = 6; //Kp_turnleft_follow = 3
float Kp_turnright_follow = 6; // Kp_turnright_follow = 3
float follow_count = 0;

uint32_t ADCA_count = 0;
void setDACA(float dacouta0) {
    int16_t DACOutInt = 0;
    DACOutInt = dacouta0*(4095.0/3.1); // perform scaling of 0 � almost 3.1V to 0 - 4095
    if (DACOutInt > 4095) DACOutInt = 4095;
    if (DACOutInt < 0) DACOutInt = 0;
    DacaRegs.DACVALS.bit.DACVALS = DACOutInt;
}
void setDACB(float dacouta1) {
    int16_t DACOutInt = 0;
    DACOutInt = dacouta1*(4095.0/3.1); // perform scaling of 0 � almost 3.1V to 0 - 4095
    if (DACOutInt > 4095) DACOutInt = 4095;
    if (DACOutInt < 0) DACOutInt = 0;
    DacbRegs.DACVALS.bit.DACVALS = DACOutInt;
}
void setDACC(float dacouta2) {
    int16_t DACOutInt = 0;
    DACOutInt = dacouta2*(4095.0/3.1); // perform scaling of 0 � almost 3.1V to 0 - 4095
    if (DACOutInt > 4095) DACOutInt = 4095;
    if (DACOutInt < 0) DACOutInt = 0;
    DaccRegs.DACVALS.bit.DACVALS = DACOutInt;
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

    //pks11 // lab7 // ex1
    PieVectTable.ADCA1_INT = &ADCA_ISR;

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

    //pks11
    //lab7 ex1

    EALLOW;
    EPwm5Regs.ETSEL.bit.SOCAEN = 0; // Disable SOC on A group
    EPwm5Regs.TBCTL.bit.CTRMODE = 3; // freeze counter
    EPwm5Regs.ETSEL.bit.SOCASEL = 2; // Select Event when counter equal to PRD //PKS11 // IT WILL BE 010 according to guide
    EPwm5Regs.ETPS.bit.SOCAPRD = 1; // Generate pulse on 1st event (�pulse� is the same as �trigger�) //PKS11 // It will be 001 according to guide
    EPwm5Regs.TBCTR = 0x0; // Clear counter
    EPwm5Regs.TBPHS.bit.TBPHS = 0x0000; // Phase is 0
    EPwm5Regs.TBCTL.bit.PHSEN = 0; // Disable phase loading
    EPwm5Regs.TBCTL.bit.CLKDIV = 0; // divide by 1 50Mhz Clock
    EPwm5Regs.TBPRD = 50000; // Set Period to 1ms sample. Input clock is 50MHz. //pks11 // (Clock Freq* period sample)
    //EPwm5Regs.TBPRD = 12500; // Set Period to 0.25ms sample. Input clock is 50MHz. //pks11 // (Clock Freq* period sample)
    //EPwm5Regs.TBPRD = 5000; // Set Period to 0.1ms (10,000 Hz) sample. Input clock is 50MHz. //pks11 // (Clock Freq* period sample)
    // Notice here that we are not setting CMPA or CMPB because we are not using the PWM signal
    EPwm5Regs.ETSEL.bit.SOCAEN = 1; //enable SOCA
    EPwm5Regs.TBCTL.bit.CTRMODE = 0; //unfreeze, and enter up count mode //PKS11/ UPCOUNT MODE
    EDIS;


    // pks11 // lab 7 // ex1
    // adding ADC initialisation
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
    //Select the channels to convert and end of conversion flag
    //Many statements commented out, To be used when using ADCA or ADCB
    //ADCA
    //PKS11/
    //EX3 : Do not forget to remeber we are gonna use ADCINA2 (CHSEL : 2) and ADCINA3 (chsel : 3)
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 2; // ADCINA2 : SOC0 will convert Channel you choose Does not have to be A0
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 13;// EPWM5 ADCSOCA or another trigger you choose will trigger SOC0
    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 3; //ADCINA3 : SOC1 will convert Channel you choose Does not have to be A1
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 13;// EPWM5 ADCSOCA or another trigger you choose will trigger SOC1
    AdcaRegs.ADCSOC2CTL.bit.CHSEL = 4; //ADCINA3 : SOC2 will convert Channel you choose Does not have to be A1
    AdcaRegs.ADCSOC2CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = 13;// EPWM5 ADCSOCA or another trigger you choose will trigger SOC1
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 2; //set to last SOC that is converted and it will set INT1 flag ADCA1
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1; //enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared

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

    //SPIB_RX is group 6
    IER |= M_INT6;

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    // Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;

    //Enabling SPIB_RX interupt in the PIE : Group 6 interupt 3
    PieCtrlRegs.PIEIER6.bit.INTx3 = 1;

    //Enabling Pie Interupt 1.1 for ADCA1
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;

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
            serial_printf(&SerialA,"tilt_value:%.3f gyro_value: %.3f LeftWheel: %.3f RightWheel : %.3f \r\n", tilt_value, gyro_value,LeftWheel, RightWheel);
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

    //pks11 //lab7 //ex5

    //pks11// lab6// ex5
    if (NewLVData == 1) {
        NewLVData = 0;
        x_nav = fromLVvalues[0];
        y_nav = fromLVvalues[1];
        mode = fromLVvalues[2];
        printLV4 = fromLVvalues[3];
        printLV5 = fromLVvalues[4];
        printLV6 = fromLVvalues[5];
        printLV7 = fromLVvalues[6];
        printLV8 = fromLVvalues[7];
    }

    if((numSWIcalls%62) == 0) { // change to the counter variable of you selected 4ms. timer : //pks11 // ex5 We want to communicate at 248 ms
        DataToLabView.floatData[0] = xR_K;
        DataToLabView.floatData[1] = yR_K;
        DataToLabView.floatData[2] = phiR;
        DataToLabView.floatData[3] = yk2;
        DataToLabView.floatData[4] = yk3;
        DataToLabView.floatData[5] = yk4;
        DataToLabView.floatData[6] = alpha;
        DataToLabView.floatData[7] = machine_state;
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

    // Balance Control
    RightWheel_K = RightWheel;
    LeftWheel_K = LeftWheel;
    phiR = (Rwh/Wr)*(RightWheel_K - LeftWheel_K);
    thetaAvg = 0.5*(RightWheel_K + LeftWheel_K);
    thetaAvg_dot = 0.5*((RightWheel_K - RightWheel_K_1) + (LeftWheel_K - LeftWheel_K_1))*250;
    xdot_K = Rwh*(thetaAvg_dot)*cos(phiR);
    ydot_K = Rwh*(thetaAvg_dot)*sin(phiR);

    // Calculating current position
    xR_K = xR_K_1 + (0.5)*(xdot_K + xdot_K_1)*(0.004);
    yR_K = yR_K_1 + (0.5)*(ydot_K + ydot_K_1)*(0.004);

    if (mode == 1)
    {
        switch(follow_machine_state){
        case 0:
            vref_forxy = 0.0;
            turnref = 1;   // KH & LJM: yk4 is the right IR sensor
            alpha = alpha_K_1 + (turnref + turnref_K_1)*0.002;

            alpha_K_1 = alpha;
            if(yk3 < 2.2){
                count_follow1 ++;
                if(count_follow1 >= 50)
                    follow_machine_state = 1;
            }
            break;
        case 1:  //move to follow
            //alpha = 0;
            vref_forxy = Kp_follow*-1*(distance_ref-yk3);
            if (yk4 > distance_ref) // When right sensor sees no object then turn left
            {
                rightfollow_count++;
                if (rightfollow_count >= 25){                // Ensuring constant distance reading instead of noise
                    alpha_K_1 = alpha;
                    follow_machine_state = 2;               // Then go to right_wall_following mode
                    rightfollow_count = 0;
                }

            }
            else if (yk2 < 2.25)  // When left sensor sees an object at distance less than right_wall_start_threshold
            {
                leftfollow_count++;
                if (leftfollow_count >= 25){                // Ensuring constant distance reading instead of noise
                    alpha_K_1 = alpha;
                    follow_machine_state = 3;           // Then go to left_wall_following_mode
                    leftfollow_count = 0;
                }
            }

            else if (yk3 > 3.0)
            {
                follow_count++;
                if(follow_count > 25){

                    follow_count = 0;
                    follow_machine_state = 0;
                }
            }

            break;
        case 2: //leftturn to follow
            turnref = Kp_turnleft_follow*(yk3-yk2-ref_sensor_diff);
            alpha = alpha_K_1 + (turnref + turnref_K_1)*0.002;
            alpha_K_1 = alpha;
            turnref_K_1 = turnref;

            if(yk4 < 2.4)
            {
                rightfollow_count++;
                if (rightfollow_count >= 25){                // Ensuring constant distance reading instead of noise
                    alpha_K_1 = alpha;

                    rightfollow_count = 0;
                    follow_machine_state = 1;               // Then go to right_wall_following mode
                }

            }
            else if (yk3 > 3.0)
            {
                follow_count++;
                if(follow_count > 25){

                    follow_count = 0;
                    follow_machine_state = 0;
                }
            }
            break;
        case 3: //righttun to follow
            turnref = Kp_turnright_follow*(yk3-yk4-ref_sensor_diff);
            alpha = alpha_K_1 + (turnref + turnref_K_1)*0.002;
            alpha_K_1 = alpha;
            turnref_K_1 = turnref;

            if(yk2 < 2.4)
            {
                leftfollow_count++;
                if (leftfollow_count >= 25){                // Ensuring constant distance reading instead of noise
                    alpha_K_1 = alpha;

                    leftfollow_count = 0;
                    follow_machine_state = 1;               // Then go to right_wall_following mode
                }

            }
            else if (yk3 > 3.0)
            {
                follow_count++;
                if(follow_count > 25){

                    follow_count = 0;
                    follow_machine_state = 0;
                }
            }
            break;



        }

    }
    else
    {

        if (machine_state==3)
        {
            count_turn = 0;

            // Sending the current position to calculate the next control inputs
            target_near = xy_control(2.0 , xR_K, yR_K, x_nav, y_nav, phiR , 0.5, 0.75);
        }

        // State Machine
        switch (machine_state) {
        case 1: // Left_wall_following
            if (yk3 > left_wall_Start_threshold){

                // Turn towards the wall
                turnref = -1*Kp_left_wall*(ref_left_wall - yk2);   // KH & LJM: yk4 is the right IR sensor
                alpha = alpha_K_1 + (turnref + turnref_K_1)*0.002;
                alpha_K_1 = alpha;
                turnref_K_1 = turnref;
                vref_forxy = forward_velocity;
            } else if(yk3 < left_wall_Start_threshold) {
                // Turn away from the wall
                turnref = -1*Kp_left_front_wall*(3.1 - yk3);   //KH & LJM: yk3 is the middle IR sensor
                alpha = alpha_K_1 + (turnref + turnref_K_1)*0.002;
                alpha_K_1 = alpha;
                turnref_K_1 = turnref;
                vref_forxy = 0.0;                           //KH & LJM: Halt while turning away from the wall
            }
            //count_turn++;
            if(yk2 > 2.8){
                left_count = 0;                             // Resetting left_count to stop constant machine state 1
                alpha = alpha - PI*15.0/180;
                machine_state = 4;
            }
            break;
        case 2: // Right_wall_following
            if (yk3 > right_wall_Start_threshold){
                // Turn towards the wall
                turnref = Kp_right_wall*(ref_right_wall - yk4);   // KH & LJM: yk4 is the right IR sensor
                alpha = alpha_K_1 + (turnref + turnref_K_1)*0.002;
                alpha_K_1 = alpha;
                turnref_K_1 = turnref;
                vref_forxy = forward_velocity;
            } else if (yk3 < right_wall_Start_threshold){
                // Turn away from the wall
                turnref = Kp_right_front_wall*(3.1 - yk3);   //KH & LJM: yk3 is the middle IR sensor
                alpha = alpha_K_1 + (turnref + turnref_K_1)*0.002;
                alpha_K_1 = alpha;
                turnref_K_1 = turnref;
                vref_forxy = 0.0;                           //KH & LJM: Halt while turning away from the wall

            }
            //count_turn++;
            if(yk4 > 2.8){
                right_count = 0;                            // Resetting right_count to stop constant machine state 2
                alpha = alpha + PI*15.0/180;
                machine_state = 4;

            }
            break;
        case 3: // Navigation
            if (yk4 < 2.25) // When right sensor sees an object at distance less than left_wall_start_threshold
            {
                right_count++;
                if (right_count >= 25){                // Ensuring constant distance reading instead of noise
                    alpha_K_1 = alpha;
                    machine_state = 2;               // Then go to right_wall_following mode
                }

            }
            else if (yk4>2.8)
            {
                right_count = 0;
            }
            else if (yk2 < 2.25)  // When left sensor sees an object at distance less than right_wall_start_threshold
            {
                left_count++;
                if (left_count >= 25){                // Ensuring constant distance reading instead of noise
                    alpha_K_1 = alpha;
                    machine_state = 1;           // Then go to left_wall_following_mode
                }
            }
            else if (yk2>2.8)
            {
                left_count = 0;
            }
            break;
        case 4: // Delay and wall/position option
            vref_forxy = 0.5;
            if (yk4 < 2.25) // When right sensor sees an object at distance less than left_wall_start_threshold
            {
                right_count++;
                if (right_count >= 60){                // Ensuring constant distance reading instead of noise
                    alpha_K_1 = alpha;
                    machine_state = 2;               // Then go to right_wall_following mode
                }

            }
            else if (yk2 < 2.25)  // When left sensor sees an object at distance less than right_wall_start_threshold
            {
                left_count++;
                if (left_count >= 60){                // Ensuring constant distance reading instead of noise
                    alpha_K_1 = alpha;
                    machine_state = 1;           // Then go to left_wall_following_mode
                }
            }
            if(delay_count > 500){ //After 0.5 seconds
                delay_count = 0;
                right_count = 0;
                left_count = 0;
                machine_state = 3;
            }
            delay_count++;
        }
    }


    gyro_value_K = gyro_value;

    vel_Left_K = 0.6*(vel_Left_K_1) + 100*LeftWheel_K - 100*LeftWheel_K_1;
    vel_Right_K = 0.6*(vel_Right_K_1) + 100*RightWheel_K - 100*RightWheel_K_1;
    gyrorate_dot_K = 0.6*(gyrorate_dot_K_1) + 100*(gyro_value_K) - 100*(gyro_value_K_1);

    // Balance Control law
    ubal = -K1*tilt_value - K2*gyro_value_K - K3*(0.5)*(vel_Left_K + vel_Right_K) - K4*gyrorate_dot_K;

    // State update for balance
    LeftWheel_K_1 = LeftWheel_K;
    RightWheel_K_1 = RightWheel_K;
    gyro_value_K_1 = gyro_value_K;
    xdot_K_1 = xdot_K;
    ydot_K_1 = ydot_K;
    xR_K_1 = xR_K;
    yR_K_1 = yR_K;
    vel_Left_K_1 = vel_Left_K;
    vel_Right_K_1 = vel_Right_K;
    gyrorate_dot_K_1 = gyrorate_dot_K;


    // Turn Control
    WhlDiff = LeftWheel - RightWheel;
    WhlDiff_K = WhlDiff;

    turn_angle_K = -1*alpha*Wr/Rwh;

    vel_WhlDiff_K = 0.3333*(vel_WhlDiff_K_1) + 166.667*(WhlDiff_K) - 166.667*(WhlDiff_K_1);

    // Control Law for turn
    errorDiff = turn_angle_K - WhlDiff;
    errorDiff_K = errorDiff;

    intDiff_K = intDiff_K_1 + (errorDiff_K + errorDiff_K_1)*(0.002);
    if(machine_state == 1 || machine_state == 2) {
        Kp_turn = 5.0;
        Ki_turn = 0.1;
    }
    ubalturn = Kp_turn*(errorDiff_K) + Ki_turn*(intDiff_K) - Kd_turn*(vel_WhlDiff_K);

    if(ubalturn > 4){
        ubalturn = 4;
    }

    if(ubalturn < -4){
        ubalturn = -4;
    }

    if(ubalturn > 3){
        intDiff_K = intDiff_K_1;
    }

    if(ubalturn < -3){
        intDiff_K = intDiff_K_1;
    }


    // Speed Control
    avgSpeed = (0.5)*(vel_Left_K + vel_Right_K);
    errorSpeed = (vref_forxy/Rwh) - avgSpeed;
    errorSpeed_K = errorSpeed;

    intSpeed_K = intSpeed_K_1 + (errorSpeed_K + errorSpeed_K_1)*(0.002);
    forwardback_command = Kp_speed*(errorSpeed_K) + Ki_speed*(intSpeed_K);

    if(forwardback_command > 4){
        forwardback_command = 4;
    }

    if(forwardback_command < -4){
        forwardback_command = -4;
    }

    if(forwardback_command > 3){
        intSpeed_K = intSpeed_K_1;
    }

    if(forwardback_command < -3){
        intSpeed_K = intSpeed_K_1;
    }


    // state update wheel diff
    WhlDiff_K_1 = WhlDiff_K;
    vel_WhlDiff_K_1 = vel_WhlDiff_K;
    errorDiff_K_1 = errorDiff_K;
    intDiff_K_1 = intDiff_K;
    turn_angle_K_1 = turn_angle_K;
    turnref_K_1 = turnref_K;
    errorSpeed_K_1 = errorSpeed_K;
    intSpeed_K_1 = intSpeed_K;

    // Complete control
    uright_balturn = 0.5*ubal - ubalturn - forwardback_command;
    uleft_balturn = 0.5*ubal + ubalturn - forwardback_command;

    // Assigning the control to the two wheel
    setEPWM2A(+1*uright_balturn);
    setEPWM2B(-1*uleft_balturn);

    numSWIcalls++;
    DINT;

}

// cpu_timer0_isr - CPU Timer0 ISR
__interrupt void cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;
    numTimer0calls++;

    if ((numTimer0calls%25) == 0) {
        displayLEDletter(LEDdisplaynum);
        LEDdisplaynum++;
        if (LEDdisplaynum == 0xFFFF) {  // prevent roll over exception
            LEDdisplaynum = 0;
        }
    }

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void)
{

    CpuTimer1.InterruptCount++;
    numTimer1calls++;

}

// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void)
{
    // Blink LaunchPad Blue LED
    //GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;

    CpuTimer2.InterruptCount++;
}

// SPI interrupt
__interrupt void SPIB_isr(void)
{
    countSPIB_ISR++;

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

    //pks11 //lab7// Assigning new variables to accomodate the code
    accelx = Accel_X;
    accely = Accel_Y;
    accelz = Accel_Z;

    gyrox = Gyro_X;
    gyroy = Gyro_Y;
    gyroz = Gyro_Z;

    //pks11 // lab7 //ex1
    LeftWheel_New = readEncLeft();
    RightWheel_New = readEncRight();

    //pks11 //lab7
    //    setEPWM2A(uRight);
    //    setEPWM2B(-1*uLeft);

    if(calibration_state == 0)
    {
        calibration_count++;
        if (calibration_count == 2000)
        {
            calibration_state = 1;
            calibration_count = 0;
        }
    }
    else if(calibration_state == 1)
    {
        accelx_offset+=accelx;
        accely_offset+=accely;
        accelz_offset+=accelz;
        gyrox_offset+=gyrox;
        gyroy_offset+=gyroy;
        gyroz_offset+=gyroz;
        calibration_count++;
        if (calibration_count == 2000)
        {
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
    }
    else if(calibration_state == 2)
    {
        accelx -=(accelx_offset);
        accely -=(accely_offset);
        accelz -=(accelz_offset-accelzBalancePoint);
        gyrox -= gyrox_offset;
        gyroy -= gyroy_offset;
        gyroz -= gyroz_offset;
        /*--------------Kalman Filtering code start---------------------------------------------------------------------*/
        float tiltrate = (gyrox*PI)/180.0; // rad/s
        float pred_tilt, z, y, S;
        // Prediction Step
        pred_tilt = kalman_tilt + T*tiltrate;
        pred_P = kalman_P + Q;
        // Update Step
        z = -accelz; // Note the negative here due to the polarity of AccelZ
        y = z - pred_tilt;
        S = pred_P + R;
        kalman_K = pred_P/S;
        kalman_tilt = pred_tilt + kalman_K*y;
        kalman_P = (1 - kalman_K)*pred_P;
        SpibNumCalls++;
        // Kalman Filter used
        tilt_array[SpibNumCalls] = kalman_tilt;
        gyro_array[SpibNumCalls] = tiltrate;
        LeftWheelArray[SpibNumCalls] = readEncLeft();
        RightWheelArray[SpibNumCalls] = readEncRight();
        if (SpibNumCalls >= 3) // should never be greater than 3
        {
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
    }

    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Set GPIO66 high to end Slave Select.

    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Clear Overflow flag just in case of an overflow
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1; // Clear RX FIFO Interrupt flag so next interrupt will happen

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6; // Acknowledge INT6 PIE interrupt

    if((countSPIB_ISR % 200) == 0)
    {
        UARTPrint = 1;
    }
}

// SPI setup
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

// Motor encoder reading
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

// Reading Left encoder value
float readEncLeft(void) {
    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU; //4294967295U
    raw = EQep1Regs.QPOSCNT;
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue; // I don't think this is needed and never true
    // 100 slits in the encoder disk so 100 square waves per one revolution of the
    // DC motor's back shaft. Then Quadrature Decoder mode multiplies this by 4 so 400 counts per one rev
    return (raw*(-1*0.0005233));
}

// Reading Right encoder value
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

// Control Right wheel
void setEPWM2A(float controleffort)
{
    //Saturating input parameter
    if (controleffort > 10.0)
    {
        controleffort = 10;
    }

    if (controleffort < -10)
    {
        controleffort = -10;
    }

    //linear scaling to make sure 0% duty cycle is at controleffort -10, 50% dutycycle is at controleffort 0
    EPwm2Regs.CMPA.bit.CMPA = ((controleffort + 10.0)/(20.0))* (EPwm2Regs.TBPRD);
}

// Control Left wheel
void setEPWM2B(float controleffort)
{
    //Saturating input parameter
    if (controleffort > 10)
    {
        controleffort = 10;
    }

    if (controleffort < -10)
    {
        controleffort = -10;
    }

    //linear scaling to make sure 0% duty cycle is at controleffort -10, 50% dutycycle is at controleffort 0
    EPwm2Regs.CMPB.bit.CMPB = ((controleffort + 10.0)/(20.0))* (EPwm2Regs.TBPRD);
}

// ADCA interrupt acting as time trigger for reading SPI
__interrupt void ADCA_ISR (void)
{
    // KL start : Wall Following
    yk2 = 0; //KH & LJM : Reset YK otherwise it will keep increasing
    yk3 = 0; //KH & LJM : Reset YK otherwise it will keep increasing
    yk4 = 0;
    raw_ADCINA2 = AdcaResultRegs.ADCRESULT0;
    raw_ADCINA3 = AdcaResultRegs.ADCRESULT1;
    raw_ADCINA4 = AdcaResultRegs.ADCRESULT2;
    // KH: Map the IR sensor reading to 0-3.1V (based on output voltage characteristic from the datasheet, the max voltage output seems to be at 3.1V)
    xk2[0] = raw_ADCINA2 * (3.1/4095.0);
    xk3[0] = raw_ADCINA3 * (3.1/4095.0);
    xk4[0] = raw_ADCINA4 * (3.1/4095.0);

    // KH & LJM: For new FIR filter with 21st order and 75Hz cutoff
    // This for loop performs element-wise multiplication and adds all the multiplied elements
    for (int i = 0; i < SIZEOFARRAY; i++){
        yk2 += c[i]*xk2[i]; // Apply filter
        yk3 += c[i]*xk3[i]; // Apply filter
        yk4 += c[i]*xk4[i]; // Apply filter
    }
    //Save past states before exiting from the function so that next sample they are the older state
    // KH & LJM: This for loop saves the previous elements, runs for size - 1 times because otherwise it would
    // go out of bounds
    for (int i = 0; i < (SIZEOFARRAY-1); i++){
        xk2[SIZEOFARRAY-(i+1)] = xk2[SIZEOFARRAY-(i+2)]; // Save old state (unfiltered)
        xk3[SIZEOFARRAY-(i+1)] = xk3[SIZEOFARRAY-(i+2)]; // Save old state (unfiltered)
        xk4[SIZEOFARRAY-(i+1)] = xk4[SIZEOFARRAY-(i+2)]; // Save old state (unfiltered)
    }

    // KH & LJM: Here write voltages value to DACA, Exercise 3

    dist_IR_ADCINA2 = 21.853 / (yk2 - 0.1753);
    dist_IR_ADCINA3 = 21.853 / (yk3 - 0.1753);
    dist_IR_ADCINA4 = 21.853 / (yk4 - 0.1753);

    setDACA(yk2);
    setDACB(yk3);
    setDACC(yk4);

    yk2 = 3.1 - yk2;
    yk3 = 3.1 - yk3;
    yk4 = 3.1 - yk4;

    // KL end:

    count_ISR++;
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

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// Calculation of turn angle
float my_atanf(float dy, float dx)
{
    float ang = 0;

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

// XY navigation control of segway
int xy_control(float turn_thres, float x_pos,float y_pos,float x_desired,float y_desired,
               float thetaabs,float target_radius,float target_radius_near)
{
    float dx = 0;
    float dy = 0;
    float dist = 0.0F;
    float dir = 0;
    float theta = 0;
    int target_near = FALSE;

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
    turnerror = alpha - theta;

    // check for shortest path
    if (fabsf(turnerror + 2.0*PI) < fabsf(turnerror)) turnerror += 2.0*PI;
    else if (fabsf(turnerror - 2.0*PI) < fabsf(turnerror)) turnerror -= 2.0*PI;

    if (dist<target_radius)
    {
        vref_forxy = 0;
        turnerror = 0;
        return(TRUE);
    }

    if (dist < target_radius_near)
    {
        target_near = TRUE;
        // Arrived to the target's (X,Y)
        // if we overshot target, we must change direction. This can cause the robot to bounce back and forth when
        // remaining at a point.
        if (fabsf(turnerror) > HALFPI)
        {
            vref_forxy = 0;
            return(TRUE);
        }
        vref_forxy = 0.25*dir*min_val(dist,1);
    }
    else
    {
        target_near = FALSE;
    }

    if (fabsf(turnerror) > turn_thres) {
        vref_forxy = 0;
        return(TRUE);
    }

    // vref is 1 tile/sec; but slower when close to target.
    vref_forxy = 0.5*dir*min_val(dist,1);

    return(target_near);
}

float min_val(float a, float b)
{
    if (a>b) return b;
    return a;
}
