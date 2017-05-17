//*****************************************************************************
//
// compdcm_mpu9150.c - Example use of the SensorLib with the MPU9150
//
// Copyright (c) 2013-2016 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.3.156 of the EK-TM4C123GXL Firmware Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/pwm.h"
#include "driverlib/timer.h"
#include "driverlib/eeprom.h"
#include "utils/uartstdio.h"
#include "sensorlib/hw_mpu9150.h"
#include "sensorlib/hw_ak8975.h"
#include "sensorlib/i2cm_drv.h"
#include "sensorlib/ak8975.h"
#include "sensorlib/mpu9150.h"
#include "sensorlib/comp_dcm.h"
#include "tPID.h"
#include "tMotor.h"


//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>Nine Axis Sensor Fusion with the MPU9150 and Complimentary-Filtered
//! DCM (compdcm_mpu9150)</h1>
//!
//! This example demonstrates the basic use of the Sensor Library, TM4C123G
//! LaunchPad and SensHub BoosterPack to obtain nine axis motion measurements
//! from the MPU9150.  The example fuses the nine axis measurements into a set
//! of Euler angles: roll, pitch and yaw.  It also produces the rotation
//! quaternions.  The fusion mechanism demonstrated is complimentary-filtered
//! direct cosine matrix (DCM) algorithm is provided as part of the Sensor
//! Library.
//!
//! Connect a serial terminal program to the LaunchPad's ICDI virtual serial
//! port at 115,200 baud.  Use eight bits per byte, no parity and one stop bit.
//! The raw sensor measurements, Euler angles and quaternions are printed to
//! the terminal.  The RGB LED begins to blink at 1Hz after initialization is
//! completed and the example application is running.
//
//*****************************************************************************

//*****************************************************************************
//
// Define MPU9150 I2C Address.
//
//*****************************************************************************
#define MPU9150_I2C_ADDRESS     0x68

//*****************************************************************************
//
// Global array for holding the color values for the RGB.
//
//*****************************************************************************
uint32_t g_pui32Colors[3];

//*****************************************************************************
//
// Global instance structure for the I2C master driver.
//
//*****************************************************************************
tI2CMInstance g_sI2CInst;

//*****************************************************************************
//
// Global instance structure for the ISL29023 sensor driver.
//
//*****************************************************************************
tMPU9150 g_sMPU9150Inst;

//*****************************************************************************
//
// Global Instance structure to manage the DCM state.
//
//*****************************************************************************
tCompDCM g_sCompDCMInst;

//PID Structure
tPID g_sPIDInst;
tPID g_sPIDInst_Pitch;

//PID Constants
pidConstants pitch_consts;
pidConstants roll_consts;
pidConstants yaw_consts;

tMotor g_sMotorInst;

const int ROLL_TUNE = 1;
const int PITCH_TUNE = 2;
const int YAW_TUNE = 3;
int TUNING_MODE = 1;

uint32_t ui32EEPROMInit; //flag for eeprom ready

//*****************************************************************************
//
// Global flags to alert main that MPU9150 I2C transaction is complete
//
//*****************************************************************************
volatile uint_fast8_t g_vui8I2CDoneFlag;

//*****************************************************************************
//
// Global flags to alert main that MPU9150 I2C transaction error has occurred.
//
//*****************************************************************************
volatile uint_fast8_t g_vui8ErrorFlag;

//*****************************************************************************
//
// Global flags to alert main that MPU9150 data is ready to be retrieved.
//
//*****************************************************************************
volatile uint_fast8_t g_vui8DataFlag;

//*****************************************************************************
//
// Global counter to control and slow down the rate of data to the terminal.
//
//*****************************************************************************
#define PRINT_SKIP_COUNT        10

uint32_t g_ui32PrintSkipCounter;

//Degree ranges for the roll angles of the sensor
//Angles go from -180 to 180 degrees
float ROLL_FULL	= 360;
float ROLL_HALF = 180;
float ROLL_STABLE_ANGLE = 0;
float PITCH_FULL = 180;
float PITCH_HALF = 90;
float PITCH_STABLE_ANGLE = 0;

bool FLY = false;


//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

void initEEPROM()
{
	
	// Enable the EEPROM module.
	//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);
	//
	// Wait for the EEPROM module to be ready.
	//
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_EEPROM0))
	{
		
	}
	//
	// Wait for the EEPROM Initialization to complete
	//
	ui32EEPROMInit = EEPROMInit();
	//
	// Check if the EEPROM Initialization returned an error
	// and inform the application
	//
	if(ui32EEPROMInit != EEPROM_INIT_OK)
	{
		while(1)
		{
		
		}
	
	}

}

//Arrays used for storing and reading PID Gain Values from EEPROM
float pidConstantsData [12];
float pidConstantsRead [12];
void IncreaseProportionalAngle(int TUNING_MODE)
{
	switch(TUNING_MODE){
								case ROLL_TUNE:
										pidConstantsData[0] = roll_consts.KpEuler += .01;	
										break;
								case PITCH_TUNE:
										pidConstantsData[4] = pitch_consts.KpEuler += .01;
										break;
								case YAW_TUNE:
										pidConstantsData[8] = yaw_consts.KpEuler += .01;
										break;
							}
}

void DecreaseProportionalAngle(int TUNING_MODE)
{
	switch(TUNING_MODE){
								case ROLL_TUNE:
										pidConstantsData[0] = roll_consts.KpEuler -= .01;	
										break;
								case PITCH_TUNE:
										pidConstantsData[4] = pitch_consts.KpEuler -= .01;
										break;
								case YAW_TUNE:
										pidConstantsData[8] = yaw_consts.KpEuler -= .01;
										break;
							}
}
void IncreaseIntegralAngle(int TUNING_MODE)
{
	switch(TUNING_MODE){
								case ROLL_TUNE:
										pidConstantsData[1] = roll_consts.KiEuler += .001;	
										break;
								case PITCH_TUNE:
										pidConstantsData[5] = pitch_consts.KiEuler += .001;
										break;
								case YAW_TUNE:
										pidConstantsData[9] = yaw_consts.KiEuler += .001;
										break;
							}
}

void DecreaseIntegralAngle(int TUNING_MODE)
{
	switch(TUNING_MODE){
								case ROLL_TUNE:
										pidConstantsData[1] = roll_consts.KiEuler -= .001;	
										break;
								case PITCH_TUNE:
										pidConstantsData[5] = pitch_consts.KiEuler -= .001;
										break;
								case YAW_TUNE:
										pidConstantsData[9] = yaw_consts.KiEuler -= .001;
										break;
							}
}


void IncreaseProportionalRate(int TUNING_MODE)
{
	switch(TUNING_MODE){
								case ROLL_TUNE:
										pidConstantsData[2] = roll_consts.KpGyro += .01;	
										break;
								case PITCH_TUNE:
										pidConstantsData[6] = pitch_consts.KpGyro += .01;
										break;
								case YAW_TUNE:
										pidConstantsData[10] = yaw_consts.KpGyro += .01;
										break;
							}
}

void DecreaseProportionalRate(int TUNING_MODE)
{
	switch(TUNING_MODE){
								case ROLL_TUNE:
										pidConstantsData[2] = roll_consts.KpGyro -= .01;	
										break;
								case PITCH_TUNE:
										pidConstantsData[6] = pitch_consts.KpGyro -= .01;
										break;
								case YAW_TUNE:
										pidConstantsData[10] = yaw_consts.KpGyro -= .01;
										break;
							}
}

void IncreaseIntegralRate(int TUNING_MODE)
{
	switch(TUNING_MODE){
								case ROLL_TUNE:
										pidConstantsData[3] = roll_consts.KiGyro += .001;	
										break;
								case PITCH_TUNE:
										pidConstantsData[7] = pitch_consts.KiGyro += .001;
										break;
								case YAW_TUNE:
										pidConstantsData[11] = yaw_consts.KiGyro += .001;
										break;
							}
}

void DecreaseIntegralRate(int TUNING_MODE)
{
	switch(TUNING_MODE){
								case ROLL_TUNE:
										pidConstantsData[3] = roll_consts.KiGyro -= .001;	
										break;
								case PITCH_TUNE:
										pidConstantsData[7] = pitch_consts.KiGyro -= .001;
										break;
								case YAW_TUNE:
										pidConstantsData[11] = yaw_consts.KiGyro -= .001;
										break;
							}
}


//*****************************************************************************
//
// MPU9150 Sensor callback function.  Called at the end of MPU9150 sensor
// driver transactions. This is called from I2C interrupt context. Therefore,
// we just set a flag and let main do the bulk of the computations and display.
//
//*****************************************************************************
void
MPU9150AppCallback(void *pvCallbackData, uint_fast8_t ui8Status)
{
    //
    // If the transaction succeeded set the data flag to indicate to
    // application that this transaction is complete and data may be ready.
    //
    if(ui8Status == I2CM_STATUS_SUCCESS)
    {
        g_vui8I2CDoneFlag = 1;
    }

    //
    // Store the most recent status in case it was an error condition
    //
    g_vui8ErrorFlag = ui8Status;
}

//*****************************************************************************
//
// Update the Motor Speed by changing the duty cycle of the PWM signals that
// drive the motors.
//
//*****************************************************************************
void MOTOR_SPEED_UPDATE(int back, int front, int left, int right){
		ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, back);  // PF2
		ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, front); // PF3
		ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, left); //PB4 pin
		ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, right); //PB5 pin
}

//*****************************************************************************
//
// Called by the NVIC as a result of GPIO port B interrupt event. For this
// application GPIO port B pin 2 is the interrupt line for the MPU9150
//
//*****************************************************************************
void
IntGPIOb(void)
{
    unsigned long ulStatus;

    ulStatus = GPIOIntStatus(GPIO_PORTB_BASE, true);

    //
    // Clear all the pin interrupts that are set
    //
    GPIOIntClear(GPIO_PORTB_BASE, ulStatus);

    if(ulStatus & GPIO_PIN_2)
    {
        //
        // MPU9150 Data is ready for retrieval and processing.
        //
        MPU9150DataRead(&g_sMPU9150Inst, MPU9150AppCallback, &g_sMPU9150Inst);
    }
}
bool intGPIODFirstTime = true;
void IntGPIOd(void)
{
	unsigned long ulstatus;
	
	ulstatus = GPIOIntStatus(GPIO_PORTD_BASE, true);
	
	GPIOIntClear(GPIO_PORTD_BASE, ulstatus);
	
	if(ulstatus & GPIO_PIN_7)
	{
		if((GPIOPinRead(GPIO_PORTD_BASE, 128)&(0x80)) == 0)
		{
			
				//g_sMotorInst.currentSpeed = MINSPEED;
			MOTOR_SPEED_UPDATE(MINSPEED,MINSPEED, MINSPEED, MINSPEED);
			g_sMotorInst.currentSpeed = MINSPEED;
			g_sMotorInst.motorBack = MINSPEED;
			g_sMotorInst.motorFront = MINSPEED;
			g_sMotorInst.motorLeft = MINSPEED;
			g_sMotorInst.motorRight = MINSPEED;
			
			//EEPROMProgram(pidConstantsData, 0x400, sizeof(pidConstantsData));
			
				
				FLY = false;
		}
		else
		{
			/*
			EEPROMRead(pidConstantsRead, 0x400, sizeof(pidConstantsRead));
						
								roll_consts.KpEuler = pidConstantsRead[0];
								roll_consts.KiEuler = pidConstantsRead[1];
								roll_consts.KpGyro = pidConstantsRead[2];
								roll_consts.KiGyro = pidConstantsRead[3];
								
								pitch_consts.KpEuler = pidConstantsRead[4];
								pitch_consts.KiEuler = pidConstantsRead[5];
								pitch_consts.KpGyro = pidConstantsRead[6];
								pitch_consts.KiGyro = pidConstantsRead[7];
			*/
		}
		
	}
}

//*****************************************************************************
//
// Called by the NVIC as a result of I2C3 Interrupt. I2C3 is the I2C connection
// to the MPU9150.
//
//*****************************************************************************
void
MPU9150I2CIntHandler(void)
{
    //
    // Pass through to the I2CM interrupt handler provided by sensor library.
    // This is required to be at application level so that I2CMIntHandler can
    // receive the instance structure pointer as an argument.
    //
    I2CMIntHandler(&g_sI2CInst);
}

//*****************************************************************************
//
// MPU9150 Application error handler. Show the user if we have encountered an
// I2C error.
//
//*****************************************************************************
void
MPU9150AppErrorHandler(char *pcFilename, uint_fast32_t ui32Line)
{
    //
    // Set terminal color to red and print error status and locations
    //
    UARTprintf("\033[31;1m");
    UARTprintf("Error: %d, File: %s, Line: %d\n"
               "See I2C status definitions in sensorlib\\i2cm_drv.h\n",
               g_vui8ErrorFlag, pcFilename, ui32Line);

    //
    // Return terminal color to normal
    //
    UARTprintf("\033[0m");

    //
    // Set RGB Color to RED
    //
    g_pui32Colors[0] = 0xFFFF;
    g_pui32Colors[1] = 0;
    g_pui32Colors[2] = 0;
//    RGBColorSet(g_pui32Colors);

    //
    // Increase blink rate to get attention
    //
  //  RGBBlinkRateSet(10.0f);

    //
    // Go to sleep wait for interventions.  A more robust application could
    // attempt corrective actions here.
    //
    while(1)
    {
        //
        // Do Nothing
        //
    }
}

//*****************************************************************************
//
// Function to wait for the MPU9150 transactions to complete. Use this to spin
// wait on the I2C bus.
//
//*****************************************************************************
void
MPU9150AppI2CWait(char *pcFilename, uint_fast32_t ui32Line)
{
    //
    // Put the processor to sleep while we wait for the I2C driver to
    // indicate that the transaction is complete.
    //
    while((g_vui8I2CDoneFlag == 0) && (g_vui8ErrorFlag == 0))
    {
        //
        // Do Nothing
        //
    }

    //
    // If an error occurred call the error handler immediately.
    //
    if(g_vui8ErrorFlag)
    {
        MPU9150AppErrorHandler(pcFilename, ui32Line);
    }

    //
    // clear the data flag for next use.
    //
    g_vui8I2CDoneFlag = 0;
}

//*****************************************************************************
//
//	Timer delay for calibrting the ESC's, set to 120,000,000
//  delay time = delay value/40MHz
//
//*****************************************************************************

#define Three_sec		120000000

//*****************************************************************************
//
// Timer Init initializes a fill width one shot timer
// Takes in an unsigned 32 bit value as the length of the delay.
// Timer delay time = delay value/System clock (40MHz)
//
//*****************************************************************************
void TIMER_Init_delay(uint32_t delay){
		// 
		// Enable the Timer0 peripheral 
		// 
		SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
		// 
		// Wait for the Timer0 module to be ready. 
		// 
		while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0)) { }
		// 
		// Configure TimerA as a full-width one-shot timer
		// 
		TimerConfigure(TIMER0_BASE, (TIMER_CFG_ONE_SHOT | TIMER_CFG_A_ONE_SHOT));
		//
		// Set the count time for the the one-shot timer (TimerA). 
		// 
		TimerLoadSet(TIMER0_BASE, TIMER_A, delay);
		// 
		// Enable the timers. 
		// 
		TimerEnable(TIMER0_BASE, TIMER_A);
		//
		//Wait for timer to complete one shot cycle.
		//
		while(TimerValueGet(TIMER0_BASE, TIMER_A) != delay)
		{		}
	}



//*****************************************************************************
//
//	Creates a 2ms pulse to give ESC max speed, waits three secods, and then \
//	gives min speed (1ms pulse)
//
//*****************************************************************************
void Calibrate_ESC(){
	MOTOR_SPEED_UPDATE(MAXSPEED,MAXSPEED,MAXSPEED,MAXSPEED);
	TIMER_Init_delay(Three_sec);
	MOTOR_SPEED_UPDATE(MINSPEED,MINSPEED,MINSPEED,MINSPEED);
}

//*****************************************************************************
//
// Configure the PWM. PWM on PB4, PB5, PF2, and PF3 with a 400Hz (not 50Hz) frequenzy and
// initial duty cycle to 5% (1 ms pulse)
//
//*****************************************************************************
void PWM_Init(){
		ROM_SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

		//Enable PWM0 Module (There are 4 generators inside each module)
		ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
		
		//Enable PWM1 Module
		ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
		
    //
    // Enable port B used for motion interrupt. and PWM on PB4 and PB5
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
		
		
		//Enable port F for PWM on PF2 and PF3
		ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
		
		
		ROM_GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_4);
		ROM_GPIOPinConfigure(GPIO_PB4_M0PWM2);
		ROM_GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_5);
		ROM_GPIOPinConfigure(GPIO_PB5_M0PWM3);
		ROM_GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);
		ROM_GPIOPinConfigure(GPIO_PF2_M1PWM6);
		ROM_GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_3);
		ROM_GPIOPinConfigure(GPIO_PF3_M1PWM7);
		
		
		// Wait for the PWM0 module to be ready.

		while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0)&&!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM1))
		{
		}
		
		

		//Configuring Generator 3 Module 1
		ROM_PWMGenConfigure(PWM1_BASE, PWM_GEN_3,
		                PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
		
		//Configuring Generator 1 Module 0 
		ROM_PWMGenConfigure(PWM0_BASE, PWM_GEN_1,
		                PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
		
		// Set the period. For a 50 Hz frequency, the period = 1/50, or 20
    // millliseconds. For a 625 KHz PWM clock, this translates to 12500 clock ticks.
		// 1ms pulse is 625 and 2ms pulse is 1250 for this configuration
    // Use this value to set the period.
    //For 400hz use 1562 
		// Use 156 for 2ms pulse and 78 for 1ms
    ROM_PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, 1562);
		
		//Setting same period as generator 1
		ROM_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, 1562);
				
		ROM_PWMGenEnable(PWM1_BASE, PWM_GEN_3);
		ROM_PWMGenEnable(PWM0_BASE, PWM_GEN_1);
		
		//Initialize motor values
		g_sMotorInst.motorLeft = MINSPEED;
		g_sMotorInst.motorRight = MINSPEED;
		g_sMotorInst.motorBack = MINSPEED;
		g_sMotorInst.motorFront = MINSPEED;
				
		// Enable the outputs.
		//
    ROM_PWMOutputState(PWM0_BASE, (PWM_OUT_2_BIT | PWM_OUT_3_BIT), true);
		
		ROM_PWMOutputState(PWM1_BASE, (PWM_OUT_6_BIT | PWM_OUT_7_BIT), true);
		
	  //Calibrate_ESC(); // ESC'S HAVE ALREADY BEEN CALIBRATED
		
}

//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void
ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	  
    //
    // Enable UART0
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	
    //
    // Configure GPIO Pins for UART mode.
    //
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	  

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
			

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);	
}

void ConfigureUART1(void)
{
	//
  // Enable the GPIO Peripheral used by the UART.
  //
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	
	//
	// Enable UART1
	//
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
	
	//
  // Configure GPIO Pins for UART mode.
  //
	ROM_GPIOPinConfigure(GPIO_PC4_U1RX);
  ROM_GPIOPinConfigure(GPIO_PC5_U1TX);
  ROM_GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);
	
	
	UARTClockSourceSet(UART1_BASE, UART_CLOCK_PIOSC);
	
	UARTConfigSetExpClk(UART1_BASE, 16000000, 115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
}

//*****************************************************************************
//
// Main application entry point.
//
//*****************************************************************************
int main(void)
{
//    int_fast32_t i32IPart[16], i32FPart[16];
		
    uint_fast32_t ui32Idx, ui32CompDCMStarted;
    float pfData[16];
    float *pfAccel, *pfGyro, *pfMag, *pfEulers, *pfQuaternion;
		bool setPointIsSet = false;
	  float pvRoll, pvPitch, pvYaw;
	
		float rollkpEulerValue;
	  float rollkiEulerValue;
	  float rollkpGyroValue;
		float rollkiGyroValue;
	
		float pitchkpEulerValue;
	  float pitchkiEulerValue;
	  float pitchkpGyroValue;
		float pitchkiGyroValue;
	
		float yawkpEulerValue;
	  float yawkiEulerValue;
	  float yawkpGyroValue;
	  
	  //int motor1, motor2, motor3, motor4;//PB6, PB7, PB4, PB5
	  float gyroRoll;
		float pvGyroPitch;
		
		//Used to store the resulting string in sprintf function at the bottom of the code
		char motorLeftStr[5];
		char motorRightStr[5];
		
		char pvstr[80];
		char mvstr[80];
		char spstr[80];
		char kpstr[80];
		
		char integralStr[80];
		char proportionalStr[80];
		char gyroPitchStr[80];
		char pitchIntegralGyroStr[80];
		
		char rollIntegralStr[80];
		char rollProportionalStr[80];
		char rollGyroStr[80];
		char rollIntegralGyroStr[80];
		
		char yawIntegralStr[80];
		char yawProportionalStr[80];
		char yawGyroStr[80];
		
		char rangestr[80];
		char errorstr[80];
		char currentspeedstr[80];
		int32_t character = '\0';
		bool canPrintOut = true;
		
		g_sMotorInst.currentSpeed = MINSPEED;
    
    //
    // Initialize convenience pointers that clean up and clarify the code
    // meaning. We want all the data in a single contiguous array so that
    // we can make our pretty printing easier later.
    //
    pfAccel = pfData;
    pfGyro = pfData + 3;
    pfMag = pfData + 6;
    pfEulers = pfData + 9;
    pfQuaternion = pfData + 12;

    //
    // Setup the system clock to run at 40 Mhz from PLL with crystal reference
    //
		
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                      SYSCTL_OSC_MAIN);
				
		//
		// Configure the PWM. PWM on PB4, PB5, PF2, and PF3 with a 50Hz frequenzy and
		// initial duty cycle to 5% (1 ms pulse), slowest Motor speed.
		//
		PWM_Init();		
		
		
		
    //
    // Initialize the UART.
    //
    ConfigureUART();
		ConfigureUART1();
		
    //
    // The I2C3 peripheral must be enabled before use.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C3);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    //
    // Configure the pin muxing for I2C3 functions on port D0 and D1.
    //
    ROM_GPIOPinConfigure(GPIO_PD0_I2C3SCL);
    ROM_GPIOPinConfigure(GPIO_PD1_I2C3SDA);

    //
    // Select the I2C function for these pins.  This function will also
    // configure the GPIO pins pins for I2C operation, setting them to
    // open-drain operation with weak pull-ups.  Consult the data sheet
    // to see which functions are allocated per pin.
    //
    GPIOPinTypeI2CSCL(GPIO_PORTD_BASE, GPIO_PIN_0);
    ROM_GPIOPinTypeI2C(GPIO_PORTD_BASE, GPIO_PIN_1);
		
		//Unlocking PD7 to use as GPIO. Because PD7 defaults to NMI fucntionality
		//Need to unlock to use as an interrupt source
		HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;
    HWREG(GPIO_PORTD_BASE + GPIO_O_AFSEL) &= ~0x80;   
    HWREG(GPIO_PORTD_BASE + GPIO_O_DEN) |= 0x80;
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;

    //
    // Configure and Enable the GPIO interrupt. Used for INT signal from the
    // MPU9150
    //
    ROM_GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOIntEnable(GPIO_PORTB_BASE, GPIO_PIN_2);
    ROM_GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_FALLING_EDGE);
    ROM_IntEnable(INT_GPIOB);

		//Setting Interrupt on PD7 for HC-05 Bluetooth STATE signal. 
		//This will tell us if the bluetooth module is currently connected
		ROM_GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_7);
		GPIOIntEnable(GPIO_PORTD_BASE, GPIO_PIN_7);
		ROM_GPIOIntTypeSet(GPIO_PORTD_BASE, GPIO_PIN_7, GPIO_FALLING_EDGE);
		ROM_IntEnable(INT_GPIOD);
    //
    // Keep only some parts of the systems running while in sleep mode.
    // GPIOB is for the MPU9150 interrupt pin.
    // UART0 is the virtual serial port
    // TIMER0, TIMER1 and WTIMER5 are used by the RGB driver
    // I2C3 is the I2C interface to the ISL29023
    //
    ROM_SysCtlPeripheralClockGating(true);
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOB);
		ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOD);
		ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_PWM0);
		ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_PWM1);
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UART0);
		ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UART1);
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER0);
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER1);
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_I2C3);
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_WTIMER5);

    //
    // Enable interrupts to the processor.
    //
    ROM_IntMasterEnable();
		
		//initEEPROM(); //Initialize EEPROM

    //
    // Initialize I2C3 peripheral.
    //
    I2CMInit(&g_sI2CInst, I2C3_BASE, INT_I2C3, 0xff, 0xff,
             ROM_SysCtlClockGet());

    //
    // Initialize the MPU9150 Driver.
    //
    MPU9150Init(&g_sMPU9150Inst, &g_sI2CInst, MPU9150_I2C_ADDRESS,
                MPU9150AppCallback, &g_sMPU9150Inst);

    //
    // Wait for transaction to complete
    //
    MPU9150AppI2CWait(__FILE__, __LINE__);

    //
    // Write application specifice sensor configuration such as filter settings
    // and sensor range settings.
    //
    g_sMPU9150Inst.pui8Data[0] = MPU9150_CONFIG_DLPF_CFG_94_98;
    g_sMPU9150Inst.pui8Data[1] = MPU9150_GYRO_CONFIG_FS_SEL_250;
    g_sMPU9150Inst.pui8Data[2] = (MPU9150_ACCEL_CONFIG_ACCEL_HPF_5HZ |
                                  MPU9150_ACCEL_CONFIG_AFS_SEL_2G);
    MPU9150Write(&g_sMPU9150Inst, MPU9150_O_CONFIG, g_sMPU9150Inst.pui8Data, 3,
                 MPU9150AppCallback, &g_sMPU9150Inst);

    //
    // Wait for transaction to complete
    //
    MPU9150AppI2CWait(__FILE__, __LINE__);

    //
    // Configure the data ready interrupt pin output of the MPU9150.
    //
    g_sMPU9150Inst.pui8Data[0] = MPU9150_INT_PIN_CFG_INT_LEVEL |
                                    MPU9150_INT_PIN_CFG_INT_RD_CLEAR |
                                    MPU9150_INT_PIN_CFG_LATCH_INT_EN;
    g_sMPU9150Inst.pui8Data[1] = MPU9150_INT_ENABLE_DATA_RDY_EN;
    MPU9150Write(&g_sMPU9150Inst, MPU9150_O_INT_PIN_CFG,
                 g_sMPU9150Inst.pui8Data, 2, MPU9150AppCallback,
                 &g_sMPU9150Inst);

    //
    // Wait for transaction to complete
    //
    MPU9150AppI2CWait(__FILE__, __LINE__);

    //
    // Initialize the DCM system. 50 hz sample rate.
    // accel weight = .2, gyro weight = .8, mag weight = .2
    //
    CompDCMInit(&g_sCompDCMInst, 1.0f / 50.0f, 0.2f, 0.6f, 0.2f);


    ui32CompDCMStarted = 0;
		MOTOR_SPEED_UPDATE(MINSPEED,MINSPEED,MINSPEED,MINSPEED);
		g_sMotorInst.increaseMotorBack = 0;
		
    while(1)
    {
			
				///////*Bluetooth Code Block**/////////
				if(UARTCharsAvail(UART1_BASE))
				{
						//Check for input from Phone.
						character = UARTCharGetNonBlocking(UART1_BASE);
						//UARTprintf("UART 1 Recieved %c", character);
										
						if(character == 'F' || character == 'f'){//Fly Mode
								FLY = true;
								UARTprintf("In Fly Mode.\n\r");
							
								//Set all PWM pulses to 5%. So this generates a 1ms pulse
								//Min speed, ESC will not be Calibrated.
								MOTOR_SPEED_UPDATE(675,675,675,675);
								
								g_sMotorInst.currentSpeed = 675;
							g_sMotorInst.increaseMotorBack = 0;
								
								//canPrintOut = true;
							
						}
						else if(character == 'W' || character == 'w'){//Increase Speed.
							UARTprintf("Increase Speed by 25\n\r");
							g_sMotorInst.currentSpeed = g_sMotorInst.currentSpeed + 25;

							if(g_sMotorInst.currentSpeed > MAXSPEED)
								g_sMotorInst.currentSpeed = MAXSPEED;//Out of Range, Motor stops						
						}
						else if(character == 'S' || character == 's'){//Decrease Speed.
							UARTprintf("Decrease Speed by 25\n\r");
							g_sMotorInst.currentSpeed = g_sMotorInst.currentSpeed - 25;
							
							if(g_sMotorInst.currentSpeed < MINSPEED)
								g_sMotorInst.currentSpeed = MINSPEED;
						}
						else if(character == '1')
						{
							TUNING_MODE = ROLL_TUNE;
							UARTprintf("Roll Tune Mode\n\r");
						}
						else if(character == '2')
						{
							TUNING_MODE = PITCH_TUNE;
							UARTprintf("Pitch Tune Mode\n\r");
						}
						else if(character == '3')
						{
							TUNING_MODE = YAW_TUNE;
							UARTprintf("Yaw Tune Mode\n\r");
						}
						else if(character == 'P' || character == 'p'){
							IncreaseProportionalAngle(TUNING_MODE);
							canPrintOut = true;
						}	
						else if(character == 'M' || character == 'm')
						{
							DecreaseProportionalAngle(TUNING_MODE);
							canPrintOut = true;
						}
						else if(character == 'I' || character == 'i')
						{
							IncreaseIntegralAngle(TUNING_MODE);
							canPrintOut = true;
						}
						else if(character == 'K' || character == 'k')
						{
							DecreaseIntegralAngle(TUNING_MODE);
							canPrintOut = true;
						}
						else if(character == 'R' || character == 'r')
						{
							IncreaseProportionalRate(TUNING_MODE);
							canPrintOut = true;
						}
						else if(character == 'J' || character == 'j')
						{
							DecreaseProportionalRate(TUNING_MODE);
							canPrintOut = true;
						}
						else if(character == 'G' || character == 'g')
						{
							IncreaseIntegralRate(TUNING_MODE);
							canPrintOut = true;
						}
						else if(character == 'N' || character == 'n')
						{
							DecreaseIntegralRate(TUNING_MODE);
							canPrintOut = true;
						}
						else if(character == 'Z' || character == 'z')
						{
							pitch_consts.KpEuler = 0;
							pitch_consts.KiEuler = 0;
							pitch_consts.KpGyro = 0;
							pitch_consts.KiGyro = 0;
							
							roll_consts.KpEuler = 0;
							roll_consts.KiEuler = 0;
							roll_consts.KpGyro = 0;
							roll_consts.KiGyro = 0;
							canPrintOut = true;
						}
						
						else if(character == 'C' || character == 'c'){//Calibrate
								
							/*
							//escCalibrated = true;
							UARTprintf("Calibrate ESC's\n\r");
							Calibrate_ESC();
							UARTprintf("Finish Calibrate ESC's\n\r");
							*/
						}
						/*
						else if(character == '3')
						{
							g_sMotorInst.increaseMotorBack += 1;
						}
						*/
						else
							canPrintOut = false; //Stop printing values			
				}
				////////***End Bluetooth Code Block*****///////////////
				
				
				//
				// Go to sleep mode while waiting for data ready.
				//
				while(!g_vui8I2CDoneFlag)
				{
						ROM_SysCtlSleep();
				}

				//
				// Clear the flag
				//
				g_vui8I2CDoneFlag = 0;

				//
				// Get floating point version of the Accel Data in m/s^2.
				//
				MPU9150DataAccelGetFloat(&g_sMPU9150Inst, pfAccel, pfAccel + 1,
																 pfAccel + 2);

				//
				// Get floating point version of angular velocities in rad/sec
				//
				MPU9150DataGyroGetFloat(&g_sMPU9150Inst, pfGyro, pfGyro + 1,
																pfGyro + 2);

				//
				// Get floating point version of magnetic fields strength in tesla
				//
				MPU9150DataMagnetoGetFloat(&g_sMPU9150Inst, pfMag, pfMag + 1,
																	 pfMag + 2);

				//
				// Check if this is our first data ever.
				//
				if(ui32CompDCMStarted == 0)
				{
						//
						// Set flag indicating that DCM is started.
						// Perform the seeding of the DCM with the first data set.
						//
						ui32CompDCMStarted = 1;
						CompDCMMagnetoUpdate(&g_sCompDCMInst, pfMag[0], pfMag[1],
																 pfMag[2]);
						CompDCMAccelUpdate(&g_sCompDCMInst, pfAccel[0], pfAccel[1],
															 pfAccel[2]);
						CompDCMGyroUpdate(&g_sCompDCMInst, pfGyro[0], pfGyro[1],
															pfGyro[2]);
						CompDCMStart(&g_sCompDCMInst);
				}
				else
				{
						//
						// DCM Is already started.  Perform the incremental update.
						//
						CompDCMMagnetoUpdate(&g_sCompDCMInst, pfMag[0], pfMag[1],
																 pfMag[2]);
						CompDCMAccelUpdate(&g_sCompDCMInst, pfAccel[0], pfAccel[1],
															 pfAccel[2]);
						CompDCMGyroUpdate(&g_sCompDCMInst, -pfGyro[0], -pfGyro[1],
															-pfGyro[2]);
						CompDCMUpdate(&g_sCompDCMInst);
				}

				gyroRoll = pfGyro[0] * 57.295779513082320876798154814105f;
				pvGyroPitch = pfGyro[1] * 57.295779513082320876798154814105f;
				
				//
				// Increment the skip counter.  Skip counter is used so we do not
				// overflow the UART with data.
				//
				g_ui32PrintSkipCounter++;
						
        if(g_ui32PrintSkipCounter >= PRINT_SKIP_COUNT)
        {
				
				
				// Get Euler data. (Roll Pitch Yaw)
				//
				CompDCMComputeEulers(&g_sCompDCMInst, pfEulers, pfEulers + 1,
														 pfEulers + 2);

				//
				// Get Quaternions.
				//
				//CompDCMComputeQuaternion(&g_sCompDCMInst, pfQuaternion);

				//
				// convert mag data to micro-tesla for better human interpretation.
				//
				pfMag[0] *= (float)1e6;
				pfMag[1] *= (float)1e6;
				pfMag[2] *= (float)1e6;

				//
				// Convert Eulers to degrees. 180/PI = 57.29...
				// Convert Yaw to 0 to 360 to approximate compass headings.
				//
				pfEulers[0] *= 57.295779513082320876798154814105f;
				pfEulers[1] *= 57.295779513082320876798154814105f;
				pfEulers[2] *= 57.295779513082320876798154814105f;
				
				
				
				if(pfEulers[2] < 0)
				{
						pfEulers[2] += 360.0f;
				}
				
				pvRoll = (pfEulers[0] + ROLL_HALF)/ROLL_FULL;
				pvPitch = (pfEulers[1] + PITCH_HALF)/PITCH_FULL;
				
				
				if(!setPointIsSet)
				{
					PID_InitSetPoint(&g_sPIDInst, pvRoll, gyroRoll);
					PID_InitSetPoint(&g_sPIDInst_Pitch, pvPitch, pvGyroPitch);
					
					initPidConstants(&roll_consts);
					initPidConstants(&pitch_consts);
					setPointIsSet = true;
				}
				
				
				if(FLY)
				{
					//Removed codition that checks if esc's are calibrated
				
				//Update process variables for stabilize and rate loops
						updateProcessVar(&g_sPIDInst, pvRoll, gyroRoll); //Updating Roll PV
						updateProcessVar(&g_sPIDInst_Pitch, pvPitch, pvGyroPitch);     //Updating Pitch PV 
				
				//Now run stabilize loop which will run pid loop with angles
				//The manipulated variable from these loops will also serve as the input
				//to the Set Point of the rate loop
						stabilizePID(&g_sPIDInst, &roll_consts);				//Updating Roll
						stabilizePID(&g_sPIDInst_Pitch, &pitch_consts);  //Updating Pitch
				
						g_sPIDInst_Pitch.mvEuler = (g_sPIDInst_Pitch.mvEuler*PITCH_FULL);
						g_sPIDInst.mvEuler *= ROLL_FULL;
				
						ratePID(&g_sPIDInst, &roll_consts); // Rate loop for Roll axis
					
				//Now run the rate loops with the mv from the stabilize loop used as the set point
						ratePID(&g_sPIDInst_Pitch, &pitch_consts);
						
						motorUpdate(&g_sMotorInst, g_sPIDInst.mvGyro, g_sPIDInst_Pitch.mvGyro, g_sPIDInst.pvGyro, g_sPIDInst_Pitch.pvGyro);
				}
				
				rollkpEulerValue = roll_consts.KpEuler;
				rollkiEulerValue = roll_consts.KiEuler;
				rollkpGyroValue = roll_consts.KpGyro;
				
				pitchkpEulerValue = pitch_consts.KpEuler; //Calculating Proportional value for display
				pitchkiEulerValue = pitch_consts.KiEuler;
				pitchkpGyroValue = pitch_consts.KpGyro;
				
				if(canPrintOut)
				{
					sprintf(motorLeftStr, "%5d", (int)g_sMotorInst.motorLeft);
						
							//sprintf(motorRightStr, "%5d", (int)g_sMotorInst.motorRight);
							//sprintf(pvstr, "%5f", g_sPIDInst.pvEuler); //PID process variable
							//sprintf(mvstr, "%5f", g_sPIDInst.mvEuler); // Manipulated Variable
							//sprintf(spstr, "%5f", g_sPIDInst.spEuler); // Set Point
							sprintf(rollProportionalStr, "%5f", rollkpEulerValue);
							sprintf(rollIntegralStr, "%5f", rollkiEulerValue);
							sprintf(rollGyroStr, "%5f", rollkpGyroValue);
							sprintf(rollIntegralGyroStr, "%5f", rollkiGyroValue);
							sprintf(integralStr, "%5f", pitchkiEulerValue); //Integral += Ki*error
							sprintf(proportionalStr, "%5f", pitchkpEulerValue);
							sprintf(gyroPitchStr, "%5f", pitchkpGyroValue); //Gyro roll value (deg/s)
							sprintf(pitchIntegralGyroStr, "%5f", pitchkiGyroValue);
							
							
							UARTprintf("Kp Roll = %s ", rollProportionalStr);
							UARTprintf("\tKi Roll = %s ", rollIntegralStr);
							UARTprintf("\tKpGyro Roll = %s ", rollGyroStr);
							UARTprintf("\tKiGyro = %s ", rollIntegralGyroStr);
							UARTprintf("\n\r\tKp Pitch = : %s\t", proportionalStr);
							UARTprintf("\tKi Pitch = %s", integralStr);
							UARTprintf("\tKpGyro Pitch = %s", gyroPitchStr);
							UARTprintf("\tKiGyro Pitch = %s \n\r", pitchIntegralGyroStr);
							/*
							UARTprintf("\tSP = %s ", spstr);
							UARTprintf("\tPV = %s ", pvstr);
							UARTprintf("\tMV = %s\n\r", mvstr);
							
							UARTprintf("Motor Left: %s ", motorLeftStr);
							UARTprintf("\tMotor Right: %s\n\r", motorRightStr);
							*/
							
							canPrintOut = false;			
				}
				//Update Motor Speed
						MOTOR_SPEED_UPDATE(g_sMotorInst.motorBack,g_sMotorInst.motorFront,g_sMotorInst.motorLeft,g_sMotorInst.motorRight);
				
					
    }
	}
}
