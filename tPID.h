#include <stdint.h>
#include <stdbool.h>



typedef struct
{
	//Set Point value input to the PID
	float spEuler;
	float spGyro;
	
	//The Process variable i.e. the feedback value attained from the sensor
	float pvEuler;
	float pvGyro;
	
	
	//Manipulated Variable. The value that will be use to set the new PWM duty cycle to drive the motors for stabilization
	float mvEuler;
	float mvGyro;
	
	float errorEuler;
	float integralErrorEuler;
	float p_errorEuler;
	float errorGyro;
	
	float Integral;
	float IntegralRate;
		
}
tPID;

typedef struct
{
	float KpEuler;
	float KpGyro;
	
	float KiEuler;
	float KiGyro;
	
	float KdEuler;
	float KdGyro;
	
}
pidConstants;

//Function prototype for setting the initial roll set point (Roll Calibration)
void PID_InitSetPoint(tPID *pidInstance, float SetPointAngle, float SetPointRate);

void stabilizePID(tPID *pidInstance, pidConstants *rollConsts);
void ratePID(tPID *pidInstance, pidConstants *rollConsts);

void initPidConstants(pidConstants *consts);

void updateProcessVar(tPID *pidInst, float pvEuler, float pvGyroValue);





