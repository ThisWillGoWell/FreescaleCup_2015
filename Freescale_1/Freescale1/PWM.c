	/*
 * Pulse-Width-Modulation Code for K64
 * PWM signal can be connected to output pins PC3 and PC4
 * 
 * Author: Brent Dimmig <bnd8678@rit.edu>
 * Modified by: 
 * Created: 2/20/2014
 * Modified: 3/07/2015
 */

#include "PWM.h"
#include "PID.h"
#include "analog_read.h"
#include "UART.h"

/*From clock setup 0 in system_MK64f12.c*/
#define DEFAULT_SYSTEM_CLOCK 20485760u /* Default System clock value */
#define CLOCK					20485760u
#define PWM_FREQUENCY			10000



#define MOTOR_PWM_FREQUECNY 10000
#define CAMERA_PWM_FREQUECNY 50000

#define FTM0_MOD_VALUE			(CLOCK/MOTOR_PWM_FREQUECNY)
#define FTM3_MOD_VALUE			(CLOCK/CAMERA_PWM_FREQUECNY)
#define CAMERA_DUTY_CYCLE 50

#define SERVO_PWM_FREQUECNY 1000
#define SERVO_MOD_VALUE 645

#define LEFT 0
#define RIGHT 128

#define MIN_SERVO_MOD 34
#define MAX_SERVO_MOD 58

#define THROW_OUT 20
#define TRACK_WIDTH 80
#define THRESHOLD 100
#define DELTA 40
#define POS_THRESH 250
#define NEG_THRESH -250

//Camera defines
#define CAMERA_SI_PULSE_HIGH 20 //Number of cycles to give the SI Pulse High
#define CAMERA_INTERGRATION_CYCLE 700 //Number of cycles for the intergraion time

#define SCALEDOWN .01

#define DIFFER_DRIVE_HIGH 60
#define DIFFER_DRIVE_LOW 10 
#define DEAFULT_DRIVE 50
#define STRIGHT_DRIVE 60
#define DIFFER_KICK_IN 27

#define JUST_DRIVE_STRIGHT 3

#define HAVE_LINE_TIMES 2	

void averageLine(void);

void derivative(void);
void dectectLine(void);
uint8_t map(uint8_t x, uint8_t in_min, uint8_t in_max, uint8_t out_min, uint8_t out_max);

int PWMTick = 0;

int servoCount = 1;
int currentServoCount = 0;

int servoHighMod = 0 ;
int servoLowMod = 0;
int servoState = 0;

int leftMotorDuty;
int rightMotorDuty;

uint16_t cameraCaptureBuffer[128];
uint16_t line[128];
int16_t derivative_line[128];

uint8_t last_valid_min, last_valid_max;
uint8_t last_pos;

//Line Decteing var
uint8_t min1, max1;
int min1Val, max1Val;
uint16_t line1;
double output;
uint16_t current_pos;

uint16_t lastMin, lastMax;	

uint8_t haveLineTimes;

int currentCameraTick;
int fallingEdge;
int currentLineRead;

int using_last_pos;

double weights [5] = {0.4, 0.2, 0.2, 0.1, 0.1};

void InitPins(void);

/*
 * Change the Motor Duty Cycle and Frequency
 * @param DutyCycle (0 to 100)
 * @param Frequency (~1000 Hz to 20000 Hz)
 * @param dir: 1 for C3 active, else C2 active 
 */
void SetDutyCycleMotor(unsigned int motorNum, unsigned int DutyCycle	)
{
	// Calculate the new cutoff value
	uint16_t mod = (uint16_t) (((CLOCK/MOTOR_PWM_FREQUECNY) * DutyCycle) / 100);
  
	if(motorNum == LEFT_MOTOR)
	{
		FTM0_C3V = mod;
	}
	else
	{
		FTM0_C2V = mod;
	}
	/*/ Set outputs 
	if(motorNum==RIGHT_MOTOR)
    {
			//Channel 0 and Channel 3
			if(direciton == BACKWARD)
			{
				FTM0_C0V = mod;
				FTM0_C3V = 0;
			}
			else
			{
				FTM0_C0V = 0;
				FTM0_C3V = mod;
			}
		}
  else if(motorNum == LEFT_MOTOR)
    {			
			//Channel 2 and Channel 5
			if(direciton == BACKWARD)
			{
				FTM0_C2V = mod;
				FTM0_C5V = 0;
			}
			else
			{
				FTM0_C2V = 0;
				FTM0_C5V = mod;
			}
		}

	*/
	FTM0_MOD = FTM0_MOD_VALUE;
}




void InitFTM0()
{
	// 12.2.13 Enable the Clock to the FTM0 Module
	SIM_SCGC6 |= SIM_SCGC6_FTM0_MASK;
	// 11.4.1 Route the output of TPM channel 0 to the pins
	// Use drive strength enable flag to high drive strength
	//These port/pins may need to be updated for the K64 <Yes, they do. Here are two that work.>
	// 39.3.10 Disable Write Protection
	FTM0_MODE |= FTM_MODE_WPDIS_MASK;
	
	// 39.3.4 FTM Counter Value
	// Initialize the CNT to 0 before writing to MOD
	FTM0_CNT = 0;
	
	// 39.3.8 Set the Counter Initial Value to 0
	FTM0_CNTIN = 0;
	
	// 39.3.5 Set the Modulo resister
	FTM0_MOD = FTM0_MOD_VALUE;
	//FTM0->MOD = (DEFAULT_SYSTEM_CLOCK/(1<<7))/1000;

	// 39.3.6 Set the Status and Control of both channels
	// Used to configure mode, edge and level selection
	// See Table 39-67,  Edge-aligned PWM, High-true pulses (clear out on match)
	FTM0_C3SC |= FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
	FTM0_C3SC &= ~FTM_CnSC_ELSA_MASK;
	
	// See Table 39-67,  Edge-aligned PWM, Low-true pulses (clear out on match)
	FTM0_C2SC |= FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
	FTM0_C2SC &= ~FTM_CnSC_ELSA_MASK;
	
	
	// 39.3.3 FTM Setup
	// Set prescale value to 1 
	// Chose system clock source
	// Timer Overflow Interrupt Enable
	FTM0_SC = FTM_SC_PS(0) | FTM_SC_CLKS(1);

	// Enable Interrupt Vector for FTM
    //NVIC_EnableIRQ(FTM0_IRQn);	
}


void PIT0_IRQHandler(void)
{
	PIT_TFLG0 |= PIT_TFLG_TIF_MASK;
	PTD->PTOR = 1;
	//putln("PIT Inter");
	if(fallingEdge)
	{
		if(currentCameraTick == 0) // give the SI pulse
		{
			PTD->PSOR = 4; //set it to 1
		}
		else
		{
			PTD->PCOR = 4; //clear it
			ADC1_SC1A = 0x00000001;//Start Conversion for next Run
			
		}
		if(currentCameraTick >= 1 && currentCameraTick <129)
		{		
			cameraCaptureBuffer[currentCameraTick - 1] = ADC1_RA>>4;
			PTD->PSOR = 8;
			ADC1_SC1A = 0x00000001;
		}
		
		
		if(currentCameraTick > (129 + CAMERA_INTERGRATION_CYCLE))
		{
			currentCameraTick = 0;
		}
		else
		{
			currentCameraTick++;
		}
	}
	fallingEdge = ~fallingEdge;
}


void InitPDB()
{
		//Enable clock for PDB module
	SIM_SCGC6 |= SIM_SCGC6_PDB_MASK;
	
	// Set continuous mode, prescaler of 128, multiplication factor of 20,
	// software triggering, and PDB enabled

	PDB0_SC |= PDB_SC_CONT_MASK | PDB_SC_PRESCALER(0x5) | PDB_SC_MULT(0x2) | PDB_SC_TRGSEL(0xF) | PDB_SC_PDBEN_MASK;

	
	//Set the mod field to get a 1 second period.
	//There is a division by 2 to make the LED blinking period 1 second.
	//This translates to two mod counts in one second (one for on, one for off)
	PDB0_MOD = MIN_SERVO_MOD;            
	
	//Configure the Interrupt Delay register.
	PDB0_IDLY = 10;
	
	//Enable the interrupt mask.

	PDB0_SC |= PDB_SC_PDBIE_MASK;
	PDB0_SC |= PDB_SC_PDBEN_MASK; //enable the the timer
	PDB0_SC |= PDB_SC_SWTRIG_MASK; //start with trigger
	//Enable LDOK to have PDB0_SC register changes loaded. 
	
	PDB0_SC |= PDB_SC_LDOK_MASK;
	

	
		
	//Configure Output

	
	servoHighMod = (MIN_SERVO_MOD + MAX_SERVO_MOD) / 2;
	servoLowMod = SERVO_MOD_VALUE - servoHighMod;
}

void SetDutyCycleServo(int pos)
{
	servoHighMod =  MIN_SERVO_MOD + ((MAX_SERVO_MOD - MIN_SERVO_MOD) * (pos / 128.0));
	servoLowMod = SERVO_MOD_VALUE - servoHighMod;
}

void PDB0_IRQHandler(void){ //For PDB timer
	//PTB->PDOR = (0 << 22); 
	
	if(servoState) //we just did a high time
	{
		
		PDB0_MOD = servoLowMod;
		PTD->PTOR = 2;
	}
	else
	{
		PDB0_MOD = servoHighMod;
		PTD->PTOR = 2;
	}
	PDB0_SC |= PDB_SC_LDOK_MASK;
	servoState = ~servoState;
	PDB0_SC &= ~(PDB_SC_PDBIF_MASK);
	if(~servoState)
	{
		process();
	}
}


void InitPIT()
{
	SIM_SCGC6 |= SIM_SCGC6_PIT_MASK;
	// Enable clock for timers
	//PIT_MCR &= ~(PIT_MCR_MDIS_MASK); // I don't think we need this
	// Enable timers to continue in debug mode
	PIT_MCR = PIT_MCR_FRZ_MASK; // In case you need to debug
	// PIT clock frequency is the system clock
	// Load the value that the timer will count down from
	//PIT_LDVAL0 = 0x01389680; // about 1 second
	PIT_LDVAL0 = DEFAULT_SYSTEM_CLOCK/CAMERA_PWM_FREQUECNY/2; // about 1 second
	// Enable timer interrupts
	PIT_TCTRL0 |= PIT_TCTRL_TIE_MASK;
	// Enable the timer
	PIT_TCTRL0 |= PIT_TCTRL_TEN_MASK;
	
	// Clear interrupt flag
	PIT_TFLG0 |= PIT_TFLG_TIF_MASK; 
	// Enable PIT interrupt in the interrupt controller
	NVIC_EnableIRQ(PIT0_IRQn);
	
}
void InitPWM()
{

	InitPins();
	ADC1_INIT();
	InitFTM0();
	InitPIT();
	InitPDB();
	NVIC_EnableIRQ(PDB0_IRQn); 
	NVIC_EnableIRQ(FTM3_IRQn);

	
}

void InitPins()
{
	
			// Enable clock on PORT A so it can output
	SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK ;
	
	
	//FTM 0
	PORTC_PCR3  = PORT_PCR_MUX(4)  | PORT_PCR_DSE_MASK; //FTM 0 Ch2
	PORTC_PCR4  = PORT_PCR_MUX(4)  | PORT_PCR_DSE_MASK;//FTM 0 Ch3


	
	//FTM3 Clock
	//Configure Port D pin 0 for SI pulse
	GPIOD_PDDR |= 1;
	GPIOD_PDDR |= 4;
	PORTD_PCR0 = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;
	PORTD_PCR2 = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;
	//PORTC_PCR10  = PORT_PCR_MUX(3)  | PORT_PCR_DSE_MASK; //Ch2 for Clock
	
	//PDB for Serov
	//Port D Pin 1
	//PORTB_PCR22 = PORT_PCR_MUX(1);
	//GPIOB_PDDR =  (1<<22);
	GPIOD_PDDR |= 2; 
	PORTD_PCR1 = PORT_PCR_MUX(1)  | PORT_PCR_DSE_MASK;
}

uint16_t* getCameraArray(void)
{
	return (line);
}




void process()
{	
	averageLine();
	derivative();
	dectectLine();
	output = PIDcal(current_pos);

	
	//output = 128 - current_pos;
	SetDutyCycleServo(output);
	//SetDutyCycleMotor(LEFT_MOTOR, 40 );
	//SetDutyCycleMotor(RIGHT_MOTOR, 40	);
	
	
	if(output < 64 + JUST_DRIVE_STRIGHT && output > 64 - JUST_DRIVE_STRIGHT )
	{
		SetDutyCycleMotor(LEFT_MOTOR,  STRIGHT_DRIVE );
		SetDutyCycleMotor(RIGHT_MOTOR, STRIGHT_DRIVE );
		
	}
	if (output > 64 + DIFFER_KICK_IN)
	{
		SetDutyCycleMotor(LEFT_MOTOR,  DIFFER_DRIVE_HIGH);
		SetDutyCycleMotor(RIGHT_MOTOR, DIFFER_DRIVE_LOW);
	}
	else if (output < 64 - DIFFER_KICK_IN)
	{
		SetDutyCycleMotor(RIGHT_MOTOR, DIFFER_DRIVE_HIGH);
		SetDutyCycleMotor(LEFT_MOTOR,  DIFFER_DRIVE_LOW);
	}
	else
	{
		SetDutyCycleMotor(RIGHT_MOTOR, DEAFULT_DRIVE);
		SetDutyCycleMotor(LEFT_MOTOR, DEAFULT_DRIVE);
	}
	
}

void averageLine()
{
	int i;

	for(i=THROW_OUT;i<(128 - THROW_OUT) ;i++)
	{
			line[i] = (weights[0] * cameraCaptureBuffer[i] + weights[1] * cameraCaptureBuffer[i-1] + weights[2] * cameraCaptureBuffer[i+1] + weights[3] * cameraCaptureBuffer[i-2] + weights[4] * cameraCaptureBuffer[i+2]);
	}
	
}

void derivative()
{
	int i;
	for(i=THROW_OUT + 1;i<128 - THROW_OUT - 1 ;i++)
	{
			derivative_line[i] = line[i]-line[i-1];
	}
}



void dectectLine()
{
	int i;
	max1=0;
	max1Val=0;

	min1= 0;
	min1Val=0;

	for(i=THROW_OUT + 1;i<NUM_PIXELS-THROW_OUT -1;i++)
	{
		if( (derivative_line[i] < NEG_THRESH) && (derivative_line[i] < min1Val) ) //
		{
			last_valid_min = i;
			min1Val = derivative_line[i];
			min1 = i;
		}
		if( (derivative_line[i] > POS_THRESH) &&  (derivative_line[i] > max1Val)) //
		{
			last_valid_max = i;
			max1Val = derivative_line[i];
			max1 = i;
		}
		

	}
	
	
	if(min1 == 0 && max1 == 0) //we are in the middle, not seeing either side of the track
	{
		current_pos = 64;
	}
	else if(min1 == 0) //only see the Right side
	{
		current_pos = max1 + TRACK_WIDTH /2 ;
	}
	else if(max1 == 0) //only see the left side
	{
		current_pos = min1 - TRACK_WIDTH/2;
	}
	else
	{
		current_pos	= (min1 + max1) >> 1;
	}

	
}


uint8_t getCurrentPosition()
{
	return current_pos;
}

uint8_t get_output()
{
	return ((uint8_t) output);
}

uint8_t get_last_min()
{
	return last_valid_min;
}

uint8_t get_last_max()
{
	return last_valid_max;
}



