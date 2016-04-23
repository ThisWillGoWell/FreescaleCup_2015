#ifndef PMW_H_
#define PMW_H_
#include "MK64F12.h"

#define FORWARD 1
#define BACKWARD 0

#define LEFT_MOTOR 0
#define RIGHT_MOTOR 1

#define CAMERA_AVERAGE_READ 10
#define NUM_PIXELS 128




void SetDutyCycleMotor(unsigned int motorNum, unsigned int DutyCycle);
void SetDutyCycleServo(int pos);
void InitPWM(void);
void PWM_ISR(void);
void process(void);


uint8_t getCurrentPosition(void);
uint16_t* getCameraArray(void);
uint8_t get_output(void);
uint8_t get_last_min();
uint8_t get_last_max();


#endif /* PWM_H_ */
