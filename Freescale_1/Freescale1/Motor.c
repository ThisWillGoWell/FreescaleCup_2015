
#include "MK64F12.h"
#include "PWM.h"

void driveMotor(int motor, int speed)
{
	uint16_t mod = (uint16_t) (((CLOCK/Frequency) * DutyCycle) / 100);
	
	if( motor== 0)
	{
	}
	else if(motor == 1)
	{
		
	}
}