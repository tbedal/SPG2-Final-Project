/**
 * position.h
 *
 * Functions useful for tracking the position and path of the CyBot
 * 
 * @date Decmeber 3, 2025
 * @authors Thiago Bedal, Joseph Vesterby
 * @authors Griffin Cegielsi, Andrew Pham, Joseph Zeffiro
**/

#ifndef POSITION_H_
#define POSITION_H_

/* <----------| INCLUDES |----------> */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

/* <----------| DEFINITIONS |----------> */

#define POS_MAX_TABLES 5

// Creates a gridPoint struct
// TODO: This could get moved to floats to further reduce compounding error
typedef struct gridPointData {
    int16_t x;
    int16_t y;
} gridPoint;

// Creates a gridPoint struct with degree tracking
// TODO: This could get moved to floats to further reduce compounding error
typedef struct gridPointFullData {
    gridPoint position;
    uint16_t degrees;
} gridPointFull;

/* <----------| FUNCTIONS |----------> */

// TODO: Comment me!
void updateCyBotPosition(gridPointFull* currentPostionFull, uint8_t traveledDistance, int16_t traveledDegrees);

// TODO: Comment me!
// NOTE: Pretends the bot moved to where the table is to get the coordinates of the table
gridPoint findTableLocation(gridPointFull* currentPostionFull, uint8_t tableDistance, int16_t tableDegrees);

// TODO: Comment me!
uint8_t isVisitedTable(gridPoint tablePositions[POS_MAX_TABLES], gridPoint tableLocation);

// TODO: Comment me!
// NOTE: Assumes that firstWall and secondWall had a FL and FR sensed border aka bot was perpendicular to the wall 
//       AND Assumes that first Wall and secondWall are perpendicular
gridPoint calculateWall(gridPointFull firstWall, gridPointFull secondWall);

#endif /* POSITION_H_ */
