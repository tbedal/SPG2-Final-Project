/**
 * uart.h
 *
 * Contains functions to communicate with the CyBot via UART
 * 
 * @date October 29, 2025
 * @author Thiago Bedal
 * @author Joseph Vesterby
**/

#ifndef UART_H_
#define UART_H_

/* <----------| INCLUDES |----------> */

#include <stdint.h>
#include <stdbool.h>
#include <inc/tm4c123gh6pm.h>
#include "driverlib/interrupt.h"
#include "timer.h"
#include <stdio.h>

/* <----------| DEFINITIONS |----------> */

#define UART_MESSAGE_LEN 80

// TODO: figure out whether we want these to be external or private
extern volatile char uart_data;
extern volatile char flag;

/* <----------| FUNCTIONS |----------> */

/**
 * Sets the registers necessary for communicating with the CyBot over UART.
 * Uses UART1 with GPIO PMC0 on PB0 (output) and PMC1 on PB1 (input).
 * 
 * @param baud desired baud rate on 16 MHz clock
 */
void uart_init(int baud);

/**
 * Sends 8-bit character from CyBot to server over UART.
 * 
 * @warning Waits indefinitely for successful completion of task
 * @param data desired ASCII char to be sent to server
 */
void uart_sendChar(char data);

/**
 * Sends 80-character string from CyBot to server over UART.
 * 
 * @param data desired message to be sent to server
**/
void uart_sendStr(const char *data);

/**
 * Listens for data sent from the server.
 * 
 * @warning Waits indefinitely for successful completion of task
 * @returns ASCII character sent from server
**/
char uart_getChar(void);

/**
 * Sets registers necessary for utilizing interrupts on UART; binds event trigger to ISR.
**/
void uart_interruptInit();

/**
 * Update flag and data volatile variables upon interrupt trip.
**/
void uart_interruptHandler();

#endif /* UART_H_ */
