#ifndef PID_H_
#define PID_H_

#include "MK64F12.h"

//Defineparameter

#define epsilon (0.01)
#define dt ((float) 0.02) //100mslooptime
#define MAX 100 //ForCurrent Saturation
#define MIN 20
#define MAX_TURN 128
#define MIN_TURN 0
#define Kp 1
#define Kd 0.02
#define Ki 0.0

#define setpoint 64

uint8_t PIDcal(uint8_t actual_position);

#endif /*PID_H_*/
