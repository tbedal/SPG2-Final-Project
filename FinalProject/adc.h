/**
 * adc.h
 *
 * Contains functions to help operate the ADC on the IR sensor of the Cybot
 * 
 * @date October 29, 2025
 * @authors Thiago Bedal, Joseph Vesterby
 * @authors Griffin Cegielsi, Andrew Pham, Joseph Zeffiro
**/

#ifndef ADC_H_
#define ADC_H_

/* <----------| INCLUDES |----------> */

#include <inc/tm4c123gh6pm.h>
#include <stdbool.h>
#include <stdint.h>
#include "driverlib/interrupt.h"

/* <----------| FUNCTIONS |----------> */

/**
 * Sets the registers necessary for reading raw IR data through the ADC.
 * Uses ADC Module 0 with AIN10 running through PB4 and 16x oversampling to SS3.
**/
void adc_init(void);

/**
 * Calls and waits for ADC to complete sample conversion then returns value
 * 
 * @return IR "raw" value (mV)
**/
uint16_t adc_read(void);

/** 
 * Converts IR "raw" values to a measured distance in centimeters.
 * 
 * @param millivolts IR "raw" value
 * @return Object distance (cm)
**/
uint8_t adc_calculateIRDistance(uint16_t millivolts);

#endif /* ADC_H_ */
