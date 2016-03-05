/*
 * Pulse-Width-Modulation Code for K64
 * PWM signal can be connected to output pins PC3 and PC4
 * 
 * Author: Brent Dimmig <bnd8678@rit.edu>
 * Modified by: 
 * Created: 2/20/2014
 * Modified: 3/07/2015
 */
#include "MK64F12.h"
#include "PWM.h"
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

#define MIN_SERVO_MOD 40
#define MAX_SERVO_MOD 1

//Camera defines
#define CAMERA_SI_PULSE_HIGH 20 //Number of cycles to give the SI Pulse High
#define CAMERA_INTERGRATION_CYCLE 50 //Number of cycles for the intergraion time
//#define LAST_CAMERA_CLOCK  (128 + CAMERA_INTERGRATION_CYCLE + CAMERA_SI_PULSE_HIGH) //The Number of cycles to when the last pixel is read in in
//#define TOTAL_CAMERA_CYCLES  (10 + LAST_CAMERA_CLOCK)  //The total Number of cycles in 1 total read 
int PWMTick = 0;

int servoCount = 1;
int currentServoCount = 0;

int servoHighMod = 0 ;
int servoLowMod = 0;
int servoState = 0;

char cameraCaptureBuffer[128];
int currentCameraTick;
int fallingEdge;

void InitPins();

/*
 * Change the Motor Duty Cycle and Frequency
 * @param DutyCycle (0 to 100)
 * @param Frequency (~1000 Hz to 20000 Hz)
 * @param dir: 1 for C3 active, else C2 active 
 */
void SetDutyCycleMotor(unsigned int motorNum, unsigned int DutyCycle)
{
	// Calculate the new cutoff value
	uint16_t mod = (uint16_t) (((CLOCK/MOTOR_PWM_FREQUECNY) * DutyCycle) / 100);
  
	// Set outputs 
	if(motorNum==1)
    {FTM0_C3V = mod;}
  else if(motorNum == 0)
    {FTM0_C2V = mod;}

	// Update the clock to the new frequency
}

/*
 * Initialize the FlexTimer for PWM
 */


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
	
	FTM0_C5SC |= FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
	FTM0_C5SC &= ~FTM_CnSC_ELSA_MASK;
	
		FTM0_C7SC |= FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
	FTM0_C7SC &= ~FTM_CnSC_ELSA_MASK;
	
	// 39.3.3 FTM Setup
	// Set prescale value to 1 
	// Chose system clock source
	// Timer Overflow Interrupt Enable
	FTM0_SC = FTM_SC_PS(0) | FTM_SC_CLKS(1);

	// Enable Interrupt Vector for FTM
    //NVIC_EnableIRQ(FTM0_IRQn);
	
}
void InitFTM3()
{
	SIM_SCGC3 |= SIM_SCGC3_FTM3_MASK ;
	

	
	// 11.4.1 Route the output of TPM channel 0 to the pins
	// Use drive strength enable flag to high drive strength
	//These port/pins may need to be updated for the K64 <Yes, they do. Here are two that work.>

   
	
	// 39.3.10 Disable Write Protection
	FTM3_MODE |= FTM_MODE_WPDIS_MASK;
	
	// 39.3.4 FTM Counter Value
	// Initialize the CNT to 0 before writing to MOD
	FTM3_CNT = 0;
	
	// 39.3.8 Set the Counter Initial Value to 0
	FTM3_CNTIN = 0;
	
	// 39.3.5 Set the Modulo resister
	FTM3_MOD = (CLOCK/(1<<6))/300000; //FTM3_MOD_VALUE;

	// 39.3.6 Set the Status and Control of both channels
	// Used to configure mode, edge and level selection
	// See Table 39-67,  Edge-aligned PWM, High-true pulses (clear out on match)
	FTM3_C6SC |= FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
	FTM3_C6SC &= ~FTM_CnSC_ELSA_MASK;
	

	
	// 39.3.3 FTM Setup
	// Set prescale value to 1 
	// Chose system clock source
	// Timer Overflow Interrupt Enable	
	FTM3_SC = FTM_SC_PS(0x7) | FTM_SC_CLKS(1) | FTM_SC_TOIE_MASK;
	
	FTM3_C6SC |= FTM_CnSC_CHIE_MASK;
	FTM3_C6V = (uint16_t) ((CLOCK/(1<<7))/100000)/2;
	
	//SI output for camra PORT D_1
	
}


//void FTM3_IRQHandler(void){ //For FTM timer
 //PTB->PTOR = (1 << 22);
	//putln("FTM3_IRQ");
	//FTM3_CNT = 0;	
void PIT0_IRQHandler(void)
{

	PIT_TFLG0 |= PIT_TFLG_TIF_MASK;
	PTD->PTOR = 1;
	if(fallingEdge)
	{
		if(currentCameraTick == 0) // give the SI pulse
		{
			PTD->PSOR = 4; //set it to 1
		}
		else
		{
			PTD->PCOR = 4; //clear it
		}
		if(currentCameraTick > 1 && currentCameraTick <129)
		{
			
			cameraCaptureBuffer[currentCameraTick - 1] = ADC0_RA;
			ADC0_SC1A = 26 & ADC_SC1_ADCH_MASK;
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
	else
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

	
	servoHighMod = MIN_SERVO_MOD;
	servoLowMod = SERVO_MOD_VALUE - servoHighMod;
}

void setServoPosition(int pos)
{
	servoHighMod =  MIN_SERVO_MOD + ((MAX_SERVO_MOD - MIN_SERVO_MOD) * pos / 100);
	servoLowMod = SERVO_MOD_VALUE - servoHighMod;
}

void PDB0_IRQHandler(void){ //For PDB timer
	//PTB->PDOR = (0 << 22); 
	
	if(servoState) //we just did a high time
	{
		PDB0_MOD = servoLowMod;
		PTD->PDOR &= ~(2);
	}
	else
	{
		PDB0_MOD = servoHighMod;
		PTD->PDOR |= 2;
	}
	PDB0_SC |= PDB_SC_LDOK_MASK;
	servoState = ~servoState;
	PDB0_SC &= ~(PDB_SC_PDBIF_MASK);
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
	//InitFTM3();	
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

	PORTA_PCR2 = PORT_PCR_MUX(3) | PORT_PCR_DSE_MASK;//FTM 0 Channel 5
	PORTA_PCR0 = PORT_PCR_MUX(3) | PORT_PCR_DSE_MASK;//FTM 0 Channel 0
	
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


//char* getCameraArray(void)
//{
	//return (&caemraCaptureBuffer);
//}
