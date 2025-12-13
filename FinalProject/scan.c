/**
 * scan.c
 *
 * Contains functions useful for scanning a field populated with objects
 * 
 * @date November 19, 2025
 * @authors Thiago Bedal, Joseph Vesterby
 * @authors Griffin Cegielsi, Andrew Pham, Joseph Zeffiro
**/

/* <----------| INCLUDES |----------> */

#include "scan.h"

/* <----------| DEFINTIONS |----------> */

#define BUFFER_SIZE 7
#define MAX_OBJECTS 15
#define NO_OBJECT_DISTANCE 50
#define TOLERANCE 3
#define M_PI 3.14159265358979323846

/* <----------| FUNCTIONS |----------> */

// Private helper method to return the mean (average) of all values up to index length-1 in array
static uint8_t mean(uint8_t values[], uint8_t length);

// Private helper method which returns a 1 if given value is within +/- tolerance of target, 0 if not
static uint8_t isWithinTolerance(uint8_t value, uint8_t target, uint8_t tolerance);

// Private helper method to shift all items to the left, remove first item, and append newValue to length-1 index
static void updateBuffer(uint8_t buffer[], uint8_t length, uint8_t newValue);

/* <----------| IMPLEMENTATIONS |----------> */

scan_t scan_read(uint8_t angle) {
    scan_t returnedVector;

    // Move servo to input angle and store in degrees
    servo_move((float)angle);
    timer_waitMillis(100);
    returnedVector.angle = angle;

    // Scan and store ultrasound in centimeters (capped at 250cm)
    uint8_t pingDistanceRaw = (uint8_t)ping_read();
    returnedVector.pingDistance = pingDistanceRaw > 250.0 ? (uint8_t)(250) : (uint8_t)(pingDistanceRaw);

    // Scan and store converted IR data in centimeters
    returnedVector.irDistance = adc_calculateIRDistance(adc_read());

    return returnedVector;
}

void scan_readField(uint8_t startAngle, uint8_t endAngle, uint8_t incrementAngle, scan_t vectors[]) {
    uint8_t index = 0;
    uint8_t angle = startAngle;

    // Iterate through each angle in array (Chopped For loop)
    while (angle <= endAngle) {
        // Poll sensor and add value to array
        vectors[index] = scan_read(angle);

        index += 1;
        angle += incrementAngle;
    }
}

void scan_filterNoise(scan_t vectors[], uint8_t numValues) {
    // Initialize variables
    uint8_t buffer[BUFFER_SIZE];
    uint8_t i = 0;
    uint8_t j = 0;

    // Fill buffer with first bufferSize items
    for (i = 0; i < BUFFER_SIZE; i++) {
        updateBuffer(buffer, BUFFER_SIZE, vectors[i].pingDistance);
    }

    // Generate values for new, filtered array up to the length - buffer size index
    for (i = 0; i < numValues - BUFFER_SIZE; i++) {
        vectors[i].pingDistance = mean(buffer, BUFFER_SIZE);
        updateBuffer(buffer, BUFFER_SIZE, vectors[i + BUFFER_SIZE].pingDistance);
    }

    // Generate values for last bufferSize items. Theoretically unneccessary but I don't have the energy to FAAFO
    for (i = numValues - BUFFER_SIZE; i < numValues; i++) {
        vectors[i].pingDistance = mean(buffer, BUFFER_SIZE - j);
        j++; // TODO: technically an unnecessary variable, but again, I want to slam my head into my desk rn and this works
    }
}

// TODO: Make it not store PING data when it doesn't USE it
uint8_t scan_findSmallestObject(scan_t vectors[], uint8_t numValues) {
    uint8_t index = 0;

    // Find objects from data and record their start and end angles into the corresponding arrays
    uint8_t objectStartAngles[MAX_OBJECTS], objectEndAngles[MAX_OBJECTS];
    uint8_t currentDistance = 0, nextDistance = 0;
    uint8_t objectCount = 0;
    uint8_t lookingAtObject = 0;
    for (index = 0; index < numValues - 1; index++) { // TODO: i'm aware i can probably incremment by two since i'm always checking the next value but i do not care
        currentDistance = vectors[index].irDistance;
        nextDistance = vectors[index + 1].irDistance;

        // Found BEGINNING of NEW object if (NOT looking at object) AND (object is within range) AND (next value is within tolerance)
        if ((!lookingAtObject) && (currentDistance < NO_OBJECT_DISTANCE) && (isWithinTolerance(currentDistance, nextDistance, TOLERANCE))) {
            objectStartAngles[objectCount] = vectors[index].angle;
            lookingAtObject = 1;
        }
        // Found END of CURRENT object if (looking at object) AND (object is within range) AND (next value is out of range)
        else if ((lookingAtObject) && (currentDistance < NO_OBJECT_DISTANCE) && (nextDistance >= NO_OBJECT_DISTANCE)) {
            objectEndAngles[objectCount] = vectors[index].angle;
            lookingAtObject = 0;
            objectCount++;
        }
    }

    // Calculate true widths of each object and store the object with the smallest width
    uint8_t currentWidth = 255, currentStartAngle = 255, currentEndAngle = 255;
    uint8_t smallestWidth = 255, smallestDegree = 255;
    for (index = 0; index < objectCount; index++) {
        // Calculate width of object
        currentStartAngle = objectStartAngles[index];
        currentEndAngle = objectEndAngles[index];
        currentDistance = vectors[(objectStartAngles[index] + objectEndAngles[index]) / (SCAN_INCREMENT * 2)].pingDistance; // divide by four because joe is better at math than me
        currentWidth = scan_calculateObjectWidth(currentDistance, currentStartAngle, currentEndAngle);

        // Determine if current object is the smallest object
        if (currentWidth < smallestWidth) {
            smallestWidth = currentWidth;
            smallestDegree = ((uint16_t)(currentStartAngle) + (uint16_t)(currentEndAngle)) / 2;
        }
    }

    // Return angle of smallest object in degrees
    return smallestDegree;
}

uint8_t scan_calculateObjectWidth(uint8_t medianDistance, uint8_t startAngle, uint8_t endAngle) {
    return sqrt(((pow(medianDistance, 2)) * 2) * (1 - cos(((endAngle - startAngle) / 180.0) * M_PI)));
}

static uint8_t mean(uint8_t values[], uint8_t length) {
    uint8_t i = 0;
    uint16_t total = 0;
    for (i = 0; i < length; i++) {
        total += values[i];
    }
    return total / length;
}

static uint8_t isWithinTolerance(uint8_t value, uint8_t target, uint8_t tolerance) {
    return abs(value - target) < tolerance;
}

static void updateBuffer(uint8_t buffer[], uint8_t length, uint8_t newValue) {
    uint8_t i = 0;
    for (i = 0; i < length - 1; i++) {
        buffer[i] = buffer[i + 1];
    }
    buffer[length - 1] = newValue;
}

// TODO: comment me!
// OBJECT SCAN
void objectScan() {
    int angle = 0;
    int index = 0;

    scan_t vectors[91];
    scan_readField(0, 180, 2, vectors);
    scan_filterNoise(vectors, 91);

    for (angle = 0; angle < 180; angle += 2) {
        // Send scan result for plotting
        char msg[40];
        sprintf(msg, "%d %.2f %.2f\n", angle, vectors[index].irDistance / 100.0, vectors[index].pingDistance / 100.0);
        uart_sendStr(msg);

        index++;
    }

    uart_sendStr("END\n");
}
