/**
 * servo.h
 *
 *  Contains functions to operate the Servo motor on the CyBot
 *
 * @date November 5, 2025
 * @author Thiago Bedal
 * @author Joseph Vesterby
**/

#ifndef SERVO_H_
#define SERVO_H_

/* <----------| INCLUDES |----------> */

#include <inc/tm4c123gh6pm.h>
#include <stdbool.h>
#include <stdint.h>
#include "driverlib/interrupt.h"
#include "Timer.h"

/* <----------| DEFINITIONS |----------> */

extern uint16_t servo_rightBound, servo_leftBound;

/* <----------| FUNCTIONS |----------> */

/**
 * Sets registers necessary for operating ultrasonic sensor.
 * Uses GPIO Out TCCP1 on PB5 with 24-bit Timer 1 on PB3.
**/
void servo_init(void);

/**
 * Initiates callibration mode for servo to find right/left match value. Must be called twice to be used effectively.
 * - Button 1: Turns servo to the left;
 * - Button 2: Displays callibration value for servo's current positions to LCD then exits;
 * - Button 3: N/A;
 * - Button 4: Turns servo to the right.
**/
void servo_callibrate();

// Sends out 5 us pulse and times length of pulse in to calculate distance from sensor in cm
/**
 * Directs servo to the specified angle in degrees
 * 
 * @param degrees Desired servo position angle (0 to 180 degrees)
**/
void servo_move(float degrees);

#endif /* SERVO_H_ */
