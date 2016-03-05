#ifndef PID_H_
#define PID_H_


//Defineparameter

#define epsilon 0.01
#define dt 0.01 //100mslooptime
#define MAX 100 //ForCurrent Saturation
#define MIN 0
#define Kp 0.1
#define Kd 0.01
#define Ki 0.005

#define setpoint 50

int PIDcal(int actual_position)
{
	static float pre_error= 0;
	static float integral=0;
	float error;
	float derivative;
	float output_f;
	int output;
	//CaculateP,I,D
	error = setpoint -actual_position;
	//Incaseof errortoosmall then stop integration
	if(error > 0 && error> epsilon)
	{
		integral =integral+ error*dt;
	}
	else if(error <0 && error < epsilon)
	{
		integral =integral+ error*dt;
	}
	derivative= (error -pre_error)/dt;
	
	output=Kp*error+ Ki*integral+ Kd*derivative;
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
	return output;
}
#endif /*PID_H_*/