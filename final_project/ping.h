/**
 * ping.h
 *
 * Contains functions to operate the PING)) ultrasonic sensory on the CyBot.
 * 
 * @date November 5, 2025
 * @author Thiago Bedal
 * @author Joseph Vesterby
**/

#ifndef PING_H_
#define PING_H_

/* <----------| INCLUDES |----------> */

#include <inc/tm4c123gh6pm.h>
#include <stdbool.h>
#include <stdint.h>
#include "driverlib/interrupt.h"
#include "Timer.h"

/* <----------| FUNCTIONS |----------> */

/**
 * Sets registers necessary for operating ultrasonic sensor.
 * Uses GPIO PB3 with no AFSEL in Digital Mode with a 24-bit timer on Timer 3.
**/
void ping_init(void);

/**
 * Measures distance from object to sensor in centimeters using a 5 microsecond pulse.
 * 
 * @return Object distance (cm)
**/
double ping_read(void);

#endif /* PING_H_ */
