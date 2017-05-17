#include <stdint.h>
#include <stdbool.h>
#include "tMotor.h"

float PPM_RESOLUTION = 625; // for 400hz use 156-78 = 78 1250-625 = 625 Range from largest to smallest pwm signal to esc's

void initMotor(tMotor *motorInst, int motorLeft, int motorRight)
{
	//Initial motor right speed
	motorInst->motorLeft = motorLeft;
	
	//Initial motor left speed
	motorInst->motorRight = motorRight;
}

void motorUpdate(tMotor *motorInstance, float mvRoll, float mvPitch, float pvRoll, float pvPitch)
{
	// This checks that mv will not set a value that will make the motor go past its max speed
	////if(pidInstance->mvEuler > ((MAXSPEED - motorInstance->currentSpeed)/PPM_RESOLUTION))
	////{
	////	pidInstance->mvEuler = (MAXSPEED-motorInstance->currentSpeed)/PPM_RESOLUTION;
	///}
	
	// For Derivative: pidInstance->p_error = pidInstance->error;
	//Perspective of quadcopter is looking at the copter from the front side
	//TM4C JTAG usb should be considered the left side
	//Meaning whenever it moves right the Euler angles are positive
	//When it moves left the Euler angles are negative
	/*
	if(pvRoll < .5f)
	{
		motorInstance->motorLeft = motorInstance->currentSpeed - ((mvRoll*PPM_RESOLUTION));//
		motorInstance->motorRight = motorInstance->currentSpeed;
	}
	else
	{
		motorInstance->motorRight = motorInstance->currentSpeed + (mvRoll*PPM_RESOLUTION); //
		motorInstance->motorLeft = motorInstance->currentSpeed;
	}
	*/
	
	motorInstance->motorFront = motorInstance->currentSpeed + mvPitch;//((mvPitch*PPM_RESOLUTION));
	motorInstance->motorBack = motorInstance->currentSpeed - mvPitch;//(mvPitch*PPM_RESOLUTION);
	motorInstance->motorLeft = motorInstance->currentSpeed + mvRoll;
	motorInstance->motorRight = motorInstance->currentSpeed - mvRoll;
	
	/*
	if(pvPitch < .5f)
	{
		motorInstance->motorFront = motorInstance->currentSpeed - ((mvPitch*PPM_RESOLUTION));//
		motorInstance->motorBack = motorInstance->currentSpeed + motorInstance->increaseMotorBack;
	}
	else
	{
		motorInstance->motorBack = motorInstance->currentSpeed + (mvPitch*PPM_RESOLUTION) + motorInstance->increaseMotorBack; //
		motorInstance->motorFront = motorInstance->currentSpeed;
	}
	*/
	
	
	
		
	
	//motorInstance->motorLeft = motorInstance->currentSpeed - pidInstance->mvGyro;
	
	
//	motorInstance->motorRight = motorInstance->currentSpeed + pidInstance->mvGyro;
	
			
			if(motorInstance->motorLeft > MAXSPEED)
			{
				motorInstance->motorLeft = motorInstance->currentSpeed; //If we increase past the maximum turn off motors
							
			}
			else if(motorInstance->motorRight > MAXSPEED)
			{
				motorInstance->motorRight = motorInstance->currentSpeed; //Turn off motor if past max speed value
			}
			else if(motorInstance->motorLeft < MINSPEED)
			{
				motorInstance->motorLeft = MINSPEED;
			}
			else if(motorInstance->motorRight < MINSPEED)
			{
				motorInstance->motorRight = MINSPEED;
			}
	
}
