

#include "MK64F12.h"
#include "PID.h"
#include "UART.h"
#include "stdio.h"
char string[100];

int abs(int v);
int PIDcal(int actual_position)
{
	static double pre_error= 0;
	static double integral=0;
	double error;
	double derivative;
	double	 output;

	//CaculateP,I,D
	error = setpoint - actual_position;
	//Incaseof error too small then stop integration
	integral =integral+ error*dt;
	
	
	derivative = (error - pre_error)/dt;
	
	output= (uint8_t) ((Kp*error+ Ki*integral+ Kd*derivative) + setpoint);
	//Saturation Filter
	if(output> MAX)
	{
	output= 128;
	}
	else if(output< MIN)
	{
	output= 0;
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

