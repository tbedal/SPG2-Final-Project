/**
 * servo.h
 *
 *  Contains functions to operate the Servo motor on the CyBot
 *
 * @date November 5, 2025
 * @authors Thiago Bedal, Joseph Vesterby
 * @authors Griffin Cegielsi, Andrew Pham, Joseph Zeffiro
**/

#ifndef SERVO_H_
#define SERVO_H_

/* <----------| INCLUDES |----------> */

#include <stdint.h>
#include <inc/tm4c123gh6pm.h>
#include "Timer.h"

/* <----------| FUNCTIONS |----------> */

/**
 * Sets registers necessary for operating ultrasonic sensor.
 * Uses GPIO Out TCCP1 on PB5 with 24-bit Timer 1 on PB3.
**/
void servo_init(void);

/**
 * Directs servo to the specified angle in degrees
 * 
 * @param degrees Desired servo position angle (0 to 180 degrees)
**/
int servo_move(float degrees);

#endif /* SERVO_H_ */