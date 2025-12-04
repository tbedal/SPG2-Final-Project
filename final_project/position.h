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

/** 
 * DESCRIPTION
 * 
 * @param currentPositionFull TODO
 * @param traveledDistance TODO
 * @param traveledDegrees TODO
**/
void updateCyBotPosition(gridPointFull* currentPostionFull, uint8_t traveledDistance, int16_t traveledDegrees);

// NOTE: Pretends the bot moved to where the table is to get the coordinates of the table
/** 
 * DESCRIPTION
 * 
 * @param currentPositionFull TODO
 * @param tableDistance TODO
 * @param tableDegrees TODO
 * @returns TODO
**/
gridPoint findTableLocation(gridPointFull* currentPostionFull, uint8_t tableDistance, int16_t tableDegrees);

/** 
 * DESCRIPTION
 * 
 * @param tablePositions TODO
 * @param tableLocation TODO
 * @returns TODO 
**/
uint8_t isVisitedTable(gridPoint tablePositions[POS_MAX_TABLES], gridPoint tableLocation);

// NOTE: Assumes that firstWall and secondWall had a FL and FR sensed border aka bot was perpendicular to the wall 
//       AND Assumes that first Wall and secondWall are perpendicular
/** 
 * DESCRIPTION
 * 
 * @param firstWall TODO
 * @param secondWall TODO
 * @returns TODO
**/
gridPoint calculateWall(gridPointFull firstWall, gridPointFull secondWall);

#endif /* POSITION_H_ */
