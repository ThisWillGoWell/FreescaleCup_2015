

#include "MK64F12.h"
#include "PID.h"
#include "UART.h"
#include "stdio.h"
char string[100];

int abs(int v);
uint8_t PIDcal(uint8_t actual_position)
{
	static double pre_error= 0;
	static double integral=0;
	double error;
	double derivative;
	uint8_t	 output;

	//CaculateP,I,D
	error = setpoint - actual_position;
	//Incaseof error too small then stop integration
	integral =integral+ error*dt;
	
	
	derivative = (error - pre_error)/dt;
	
	output= (uint8_t) ((Kp*error+ Ki*integral+ Kd*derivative) + setpoint);
	//Saturation Filter
	if(output> MAX)
	{
	output= MAX;
	}
	else if(output< MIN)
	{
	output= MIN;
	}
	//Update error
	pre_error= error;
	/*
	sprintf(string, "%10.6f", output);
	putln(string);
	
	sprintf(string, "%i", output_i);
	putln(string);
	*/
	
	return output;
	
	
}

int abs(int v) 
{
	return v * ((v>0) - (v<0));
}

