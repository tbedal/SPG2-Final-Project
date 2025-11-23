/**
 * scan.h
 *
 * Contains functions useful for scanning a field populated with objects
 * 
 * @date November 19, 2025
 * @author Thiago Bedal
 * @author Joseph Vesterby
**/

/*
 * TODO:
 * [ ] Standardize header file templates
 * [ ] Figure out where `#ifndef` should be placed in header files
 * [ ] Convert function comments to the formatted @return stuff
 */

#include <stdbool.h>
#include <stdint.h>

#define SCAN_START  0
#define SCAN_END 180
#define SCAN_INCREMENT 2
#define NUM_SCANS (((SCAN_END - SCAN_START) / SCAN_INCREMENT) + 1)

#ifndef SCAN_H_
#define SCAN_H_

// Wrapper struct for angle and distance values vector measured by the ultrasonic and IR sensors
typedef struct scanResultData {
    uint8_t angle;
    uint8_t pingDistance;
    uint8_t irDistance;
} scanVector;

// Direct servo to point at given angle, and perform a scan at that angle and return a vector of the converted data
scanVector scan_read(uint8_t angle);

// Perform ultrasonic scan of field from startAngle to endAngle in incrementAngle increments, storing values in vectors array
void scan_readField(uint8_t startAngle, uint8_t endAngle, uint8_t incrementAngle, scanVector vectors[]);

// Filters noise in data by averaging values across a rolling average buffer. Generates new array, buffer-by-buffer
void scan_filterNoise(scanVector vectors[], uint8_t numValues, uint8_t bufferSize);

// Finds the smallest object in a scan and returns the median angle at which it is located
uint8_t scan_findSmallestObject(scanVector vectors[], uint8_t numValues);

// Calcualte width of object based on sound vector values
uint8_t scan_calculateObjectWidth(uint8_t medianDistance, uint8_t startAngle, uint8_t endAngle);

// Loop through array of distance values and print table of scanned angles
void scan_printVectors(scanVector vectors[], uint8_t numVectors);

#endif /* SCAN_H_ */