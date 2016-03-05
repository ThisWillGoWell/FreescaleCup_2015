
#ifndef UART_H_
#define UART_H_

#include "MK64F12.h"
#define BAUD_RATE 115200      //default baud rate 
#define SYS_CLOCK 20485760 //default system clock (see DEFAULT_SYSTEM_CLOCK  in system_MK64F12.c)


void uart_init(void);
uint8_t uart_getchar(void);
void uart_putchar(char ch);
void put(char *ptr_str);
void putln(char *ptr_str);
void putByte(char b);

#endif /* UART_H_ */

