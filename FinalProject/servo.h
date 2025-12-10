/*
 * servo.h
 *
 *  Created on: Nov 7, 2025
 *      Author: jz23
 */

#include <stdint.h>
#include <inc/tm4c123gh6pm.h>
#include "Timer.h"


void servo_init(void);
int servo_move(float degrees);
