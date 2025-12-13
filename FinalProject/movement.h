/**
 * movement.h
 *
 * Contains helpful functions to navigate the CyBot around obstacles.
 * 
 * @date October 29, 2025
 * @authors Thiago Bedal, Joseph Vesterby
 * @authors Griffin Cegielsi, Andrew Pham, Joseph Zeffiro
**/

#ifndef MOVEMENT_H_
#define MOVEMENT_H_

/* <----------| INCLUDES |----------> */

#include "open_interface.h"
#include "uart_extra_help.h"

/* <----------| DEFINITIONS |----------> */

// Location Variables for MiniMap
extern static float botx;
extern static float boty;
extern static float bottheta;

/* <----------| FUNCTIONS |----------> */

/**
 * Updates CyBot position data
 * 
 * @warning Updates static variables botx, boty, and bottheta
**/
void updateodometry();

/**
 * Sets wheels to tank turn cybot to left for +velocity; right for -velocity
 * 
 * @param sensor Address of OI sensor values
 * @param velocity Arbitrary wheel speed value (-500 to 500)
 * @param degrees Relative degrees to turn Cybot (0.0 to 360.0)
**/
void bot_turn(int velocity);

/**
 * Tank turn the CyBot counter-clockwise by degrees
 * 
 * @param sensor Address of OI sensor values
 * @param velocity Arbitrary wheel speed value (-500 to 500)
 * @param degrees Relative degrees to turn Cybot (0.0 to 360.0)
**/
void bot_turnDegrees(oi_t *sensor_data, int velocity, double degrees);

/**
 * Drives the CyBot a specified distance in centimeters.
 * Stops bot early upon bumping into an object.
 * 
 * @param sensor Address of OI sensor values
 * @param velocity Arbitrary wheel speed value (-500 to 500)
 * @param distanceCM Distance (cm) for CyBot to drive forward
 * @returns True distance (cm) traveled
**/
double bot_driveDistance(oi_t *sensor_data, int velocity, double distanceCM);

/**
 * Sets CyBot wheels to drive forward at given velocity.
 * 
 * @param velocity Arbitrary wheel speed value (-500 to 500)
**/
void bot_drive(int velocity);

/**
 * Checks for bumped objects and updates UART.
**/
void objectCollision();


/** 
 * Cecks for boundary lines and updates UART.
**/
void boundaryCheck();

/**
 * Checks for holes in field and update UART.
**/
void holeCheck();

/**
 * Play sound on CyBot to indicate table found.
**/
void playNoise();

#endif /* MOVEMENT_H_ */