/**
 * scan.h
 *
 * Contains functions useful for scanning a field populated with objects
 * 
 * @date November 19, 2025
 * @authors Thiago Bedal, Joseph Vesterby
 * @authors Griffin Cegielsi, Andrew Pham, Joseph Zeffiro
**/

#ifndef SCAN_H_
#define SCAN_H_

/* <----------| INCLUDES |----------> */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "uart_extra_help.h"
#include "adc.h"
#include "ping.h"
#include "servo.h"

/* <----------| DEFINITIONS |----------> */

#define SCAN_START  0
#define SCAN_END 180
#define SCAN_INCREMENT 2
#define SCAN_TOTAL (((SCAN_END - SCAN_START) / SCAN_INCREMENT) + 1)

// Wrapper struct for angle and distance values vector measured by the ultrasonic and IR sensors
typedef struct scan_angleDistVector {
    uint8_t angle;
    uint8_t pingDistance;
    uint8_t irDistance;
} scan_t;

/* <----------| FUNCTIONS |----------> */

/**
 * Scans object distance in centimeters at given servo angle with IR and ultrasonic sensors.
 * 
 * @param angle Angle to point servo to
 * @returns Vector struct with angle (degrees), ultrasonic sensor distance (cm), and IR sensor distance (cm) readings
**/
scan_t scan_read(uint8_t angle);

/**
 * Takes a given number of scans between two given angle and stores values.
 * 
 * @param startAngle Field scan left bound (degrees)
 * @param endAnlge Field scan right bound (degrees)
 * @param incrementAngle Number of scans to take between left and right bound
 * @param vectors Data array to store scans in
**/
void scan_readField(uint8_t startAngle, uint8_t endAngle, uint8_t incrementAngle, scan_t vectors[]);

/**
 * Filters noise in given data through rolling average filter.
 * 
 * @param vectors Unfiltered data array
 * @param numValues Length of unfiltered data array
**/
void scan_filterNoise(scan_t vectors[], uint8_t numValues);

/**
 * Finds the smallest object in a field scan.
 * 
 * @param vectors Data array of object angle-distance vectors
 * @param numValues Length of data array
 * @returns Median angle (degrees) where smallest object is located
**/
uint8_t scan_findSmallestObject(scan_t vectors[], uint8_t numValues);

// Calcualte width of object based on sound vector values
/**
 * Calculates width of object in centimeters based on angle-distance vectors.
 * 
 * @param medianDistance Measured distance (cm) object is away from CyBot
 * @param startAngle First angle (degrees) in which the object appears in view of CyBot
 * @param endAngle Last angle (degrees) in which the object appears in view of CyBot
 * @returns Cartesian width of object (cm)
**/
uint8_t scan_calculateObjectWidth(uint8_t medianDistance, uint8_t startAngle, uint8_t endAngle);

#endif /* SCAN_H_ */
