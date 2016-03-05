
/*
 * File:        uart.c
 * Purpose:     Provide UART routines for serial IO
 *
 * Notes:
 *
 */

#include "UART.h"

void uart_init()
{
//define variables for baud rate and baud rate fine adjust
uint16_t ubd, brfa;

//Enable clock for UART
SIM->SCGC4 = SIM->SCGC4 | SIM_SCGC4_UART0_MASK; //bit 10 to 1
SIM->SCGC5 = SIM->SCGC5 | SIM_SCGC5_PORTB_MASK;	//endable port B clock, where UART Pins are

//Configure the port control register to alternative 3 (which is UART mode for K64)
PORTB->PCR[16] = PORTB->PCR[16] | PORT_PCR_MUX(3);  //Set to alt-3
PORTB->PCR[17] = PORTB->PCR[17] | PORT_PCR_MUX(3);  //Set to alt-3
	

/*Configure the UART for establishing serial communication*/

//Disable transmitter and receiver until proper settings are chosen for the UART module
UART0->C2 = UART0_C2 & ~(UART_C2_TE_MASK | UART_C2_RE_MASK ); 

//Select default transmission/reception settings for serial communication of UART by clearing the control register 1
UART0->C1 = 0x00; 

//UART Baud rate is calculated by: baud rate = UART module clock / (16 × (SBR[12:0] + BRFD))
//13 bits of SBR are shared by the 8 bits of UART3_BDL and the lower 5 bits of UART3_BDH 
//BRFD is dependent on BRFA, refer Table 52-234 in K64 reference manual
//BRFA is defined by the lower 4 bits of control register, UART3_C4 

//calculate baud rate settings: ubd = UART module clock/16* baud rate
ubd = (uint16_t)((SYS_CLOCK)/(BAUD_RATE * 16));  

//clear SBR bits of BDH
UART0->BDH &= 0xE0;
UART0->BDL = 0x00;
//distribute this ubd in BDH and BDL

UART0->BDH |= (uint8_t) ((ubd & 0x1F00) >> 8);
UART0->BDL |= (uint8_t)  (ubd & UART_BDL_SBR_MASK);

//BRFD = (1/32)*BRFA 
//make the baud rate closer to the desired value by using BRFA
brfa = (((SYS_CLOCK*32)/(BAUD_RATE * 16)) - (ubd * 32));

//write the value of brfa in UART0_C4
//UART0->C4 = brfa;
UART0->C4 = 0;
UART0->C4 = UART_C4_BRFA(brfa);
//Enable transmitter and receiver of UART
 
UART0->C2 = UART0->C2 | (UART_C2_TE_MASK | UART_C2_RE_MASK ); //TE and RE are the proper masks to enable 
}

uint8_t uart_getchar()
{
/* Wait until there is space for more data in the receiver buffer*/
while (!(UART0_S1 & UART_S1_RDRF_MASK));
/* Return the 8-bit data from the receiver */
return UART0->D;
}

void uart_putchar(char ch)
{
/* Wait until space is available in the FIFO */
while(!(UART0->S1 & UART_S1_TDRE_MASK));
/* Send the character */
UART0->D = (uint8_t)ch;

}

void put(char *ptr_str)
{
	while(*ptr_str)
		uart_putchar(*ptr_str++);
}
void putln(char *ptr_str)
{
	while(*ptr_str)
		uart_putchar(*ptr_str++);
	uart_putchar('\n');
	uart_putchar('\r');
}

void putByte(char b)
{
	uart_putchar(b);
}

