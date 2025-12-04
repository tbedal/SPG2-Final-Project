/**
 * position.c
 *
 * Functions useful for tracking the position and path of the CyBot
 *
 * @date December 3, 2025
 * @author Thiago Bedal
 * @author Joseph Vesterby
**/

/* <----------| INCLUDES |----------> */

#include "position.h"

/* <----------| DEFINITIONS |----------> */

#define TABLE_RADIUS 6
#define TABLE_TOLERANCE 50 // Center of table to center of table in each axis tolerance
                           // (ie object can be max 50 cm in the x and 50 cm in y giving a true distance between of 70.7 cm)
#define DEGREES_TO_RADIANS 0.01745329251 // (M_PI / 180)

/* <----------| FUNCTIONS |----------> */

// Private helper method which returns a 1 if given value is within +/- tolerance of target, 0 if not
// FIXME: This code is duplicated in scan.c but I really can't think of a good justifcation for creating 
//        a utils library unless we run into more one-off functions like these... Someone should figure out 
//        what to do with this and then delete this comment once that decision is made.
static inline uint8_t isWithinTolerance(uint8_t value, uint8_t target, uint8_t tolerance) {
    return abs(value - target) < tolerance;
}

// Bounds signed 16-bit integers from 0 (lower) to 360 (upper)
static inline void wrapAroundDegrees(int16_t* degrees){
    if      (*degrees >= 360) { *degrees -= 360; }
    else if (*degrees < 0)    { *degrees += 360; }
}

/* <----------| IMPLEMENTATIONS |----------> */

void updateCyBotPosition(gridPointFull* currentPositionFull, uint8_t traveledDistance, int16_t traveledDegrees) {
    // Updating degrees
    (*currentPositionFull).degrees += traveledDegrees;

    // Degree wrap around logic
    wrapAroundDegrees(&((*currentPositionFull).degrees));

    // Updating currentPosition
    (*currentPositionFull).position.x += round(traveledDistance * cos(((*currentPositionFull).degrees) * DEGREES_TO_RADIANS));
    (*currentPositionFull).position.y += round(traveledDistance * sin(((*currentPositionFull).degrees) * DEGREES_TO_RADIANS));

}

gridPoint findTableLocation(gridPointFull* currentPositionFull, uint8_t tableDistance, int16_t tableDegrees) {
    // Where the bot is
    gridPointFull currentPositionStorage = *currentPositionFull;

    // Finds the coords of the table's center
    updateCyBotPosition(&currentPositionStorage, tableDistance + TABLE_RADIUS, tableDegrees);

    return currentPositionStorage.position;
}


uint8_t isVisitedTable(gridPoint tablePositions[POS_MAX_TABLES], gridPoint tableLocation) {
    uint8_t isVisitedTable = 0;
    uint8_t tableCursor;

    // Goes through each of the tables assuming any 0, 0s are not real
    for (tableCursor = 0; !(tablePositions[tableCursor].x == 0 && tablePositions[tableCursor].y == 0) && tableCursor < POS_MAX_TABLES; tableCursor++) {
        if (isWithinTolerance(tablePositions[tableCursor].x, tableLocation.x, TABLE_TOLERANCE) && isWithinTolerance(tablePositions[tableCursor].y, tableLocation.y, TABLE_TOLERANCE)) {
            isVisitedTable = 1;
            break;
        }
    }

    return isVisitedTable;
}

gridPoint calculateWall(gridPointFull firstWall, gridPointFull secondWall){
    gridPoint corner;

    // Saving on tan and cot calculations by doing them once and carrying throughout

    float tanDegree = tan(firstWall.degrees * DEGREES_TO_RADIANS);
    // Tertiary because 1 / 0 is undefined but 1 / (basically 0) isn't
    float cotDegree = 1 / (tanDegree == 0 ? 0.00000000000001 : tanDegree);

    // Calculates where a corner must be using Desmos math (trust if this doesn't kill the bot it works)
    corner.x = round((cotDegree * firstWall.position.x + tanDegree * secondWall.position.x - firstWall.position.y + secondWall.position.y)/(cotDegree + tanDegree));
    corner.y = round(cotDegree * (corner.x - firstWall.position.x) + firstWall.position.y);

    return corner;
}