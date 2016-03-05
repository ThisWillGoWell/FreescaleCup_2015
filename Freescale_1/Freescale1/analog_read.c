
#include "MK64F12.h"
#include "UART.h"

int time = 0;
void ADC1_INIT(void) {
    unsigned int calib;
 
    
    SIM_SCGC3 |= SIM_SCGC3_ADC1_MASK;
 
    // Configure CFG Registers
    // Configure ADC to divide 50 MHz down to 6.25 MHz AD Clock, 16-bit single ended
    ADC1_CFG1 |= ADC_CFG1_MODE(3);
 
    // Do ADC Calibration for Singled Ended ADC. Do not touch.
    ADC1_SC3 = ADC_SC3_CAL_MASK;
    while ( (ADC1_SC3 & ADC_SC3_CAL_MASK) != 0 );
    calib = ADC1_CLP0;
    calib += ADC1_CLP1;
    calib += ADC1_CLP2;
    calib += ADC1_CLP3;
    calib += ADC1_CLP4;
    calib += ADC1_CLPS;
    calib = calib >> 1;
    calib |= 0x8000;
    ADC1_PG = calib;
 
     // Configure SC registers.
    // Select Software trigger.
    
    //ADC1_SC2=  0x00000000;
 
    // Configure SC1A register.
    // Select ADC Channel and enable interrupts. Use ADC1 channel DAD3  in single ended mode.
    
    ADC1_SC1A =  0x00000001;
    
    
    // Enable NVIC interrupt
		//NVIC_EnableIRQ(ADC1_IRQn);//ADC1 Interrupt
	//TO be rmeove later
	GPIOD_PDDR |= 8;
	PORTD_PCR3 = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;
}

void ADC1_IRQHandler(void) {
    // Read the result (upper 12-bits). This also clears the Conversion complete flag.
		unsigned short i = ADC1_RA >> 4;	
		PTD->PCOR = 8;
		//putln("iter");
}




