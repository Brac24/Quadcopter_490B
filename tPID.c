#include <stdint.h>
#include <stdbool.h>
#include "tPID.h"

//Whats Up



//This function will set the roll set point for the PID 
void PID_InitSetPoint(tPID *pidInstance, float SetPointAngle, float SetPointRate)
{

	pidInstance->spEuler = SetPointAngle; //Keep setpoint between 0 and 1. .5 ~ stable setpoint angle
	pidInstance->spGyro = SetPointRate;
	
	pidInstance->Integral = 0;
 
}

void ratePID(tPID *pidInst, pidConstants *rollConsts)
{
	pidInst->errorGyro = pidInst->mvEuler-pidInst->pvGyro;
	pidInst->IntegralRate += rollConsts->KiGyro*pidInst->errorGyro;
	
	pidInst->mvGyro = rollConsts->KpGyro*(pidInst->errorGyro) + pidInst->IntegralRate;
}

//Function updates a value used for pwm duty cycle
//Duty Cycle values: 625 = 5% duty cycle for a 50Hz pwm signal but 0% speed for the motor and is the minimum for the motor(No Movement)
//                   1250 = 10% duty cycle for a 50Hz signal and 100% speed for the motor(Top Speed)
void stabilizePID(tPID *pidInstance, pidConstants *rollConsts)
{
	// u(t) = Kp*e(t) + Ki*[sum(e(t)dt)] + Kd*[e(t)/dt]
	
	//float error,p_error;
	
	pidInstance->errorEuler = pidInstance->spEuler-pidInstance->pvEuler;  // error = sp - pv
	
	////pidInstance->integralErrorEuler += pidInstance->errorEuler;
	//pidInstance->p_errorEuler = pidInstance->errorEuler;
//	pidInstance->errorGyro = pidInstance->pvGyro - pidInstance->spGyro; //Error in angular speed (deg/s)
	
	////pidInstance->Integral = rollConsts->KiEuler*pidInstance->integralErrorEuler; //Ki*sum(e(t))
	
  pidInstance->Integral += rollConsts->KiEuler*pidInstance->errorEuler; //added by eh
	//Checking the Integral wind up to make sure it does not get out of control
	if(pidInstance->Integral > 1)
	{
		pidInstance->Integral = 1;
	}
	else if(pidInstance->Integral < -1)
	{
		pidInstance->Integral = -1;
	}
			
	//Calculating the Manipulated variable
	pidInstance->mvEuler = rollConsts->KpEuler*pidInstance->errorEuler + pidInstance->Integral; // Derivative: rollConsts->Kd*(pidInstance->error - pidInstance->p_error)(add dt);
	////pidInstance->mvGyro = rollConsts->KpGyro*pidInstance->errorGyro;
	
	
	
}

void initPidConstants(pidConstants *consts)
{
	consts->KpEuler = .67;//.879;//.9;//.46;//.27;//.18 ;  //old 0.57
	consts->KiEuler = .03;//.026;//.06  ;//.0017;//.0023;//.00475; havent tried //0.0017
	consts->KpGyro = 1.121;//.61;//.9;
	consts->KiGyro = 0;
	
	consts->KdEuler = 0;
	
	
}
//Using only for roll as of now 
void updateProcessVar(tPID *pidInst, float pvEuler, float pvGyroValue)
{
	pidInst->pvEuler = pvEuler;

	pidInst->pvGyro = pvGyroValue;
}



