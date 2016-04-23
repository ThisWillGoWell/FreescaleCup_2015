/*
 * Main Method for testing the PWM Code for the K64F
 * PWM signal can be connected to output pins are PC3 and PC4
 * 
 * Author:William Gowell	  
 * Created: 10/16/2015	  
 * Modified: 10/16/2015  
 */

#include "MK64F12.h"
#include "PWM.h"
#include "UART.h"
#include "stdio.h"
#include "math.h"


void initialize(void);
void en_interrupts(void);
void delay(int del);
	

uint16_t* line_point;
int j;

int main(void)
{
	
	initialize();
	
	line_point = getCameraArray();
	
	//SetDutyCycleMotor(LEFT_MOTOR,29);
	//SetDutyCycleMotor(RIGHT_MOTOR,29);
	//
	//SetDutyCycleServo(10);
	//Step 9
	for(;;)  //loop forever
	{

		uart_getchar();
		for(j=0;j<NUM_PIXELS;j++)
		{
			putByte(line_point[j]>>4);
		}	
		uart_getchar();
		putByte(getCurrentPosition());
		putByte(get_output());
		putByte(get_last_min());
		putByte(get_last_max());
	
		
	}
}



/**
 * Waits for a delay (in milliseconds)
 * 
 * del - The delay in milliseconds
 */
void delay(int del){
	int i;
	for (i=0; i<del*50000; i++){
		// Do nothing
	}
}

void initialize()
{

	uart_init();
	//put("Start");
	InitPWM();
}






