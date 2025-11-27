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

#include <math.h>
#include "scan.h"
#include "open_interface.h"

/* <----------| DEFINITIONS |----------> */

#define BOT_MAX_SPEED 500
#define BOT_CRUISE_SPEED 200
#define BOT_CRAWL_SPEED 50
#define BOT_TURN_SPEED 50

/* <----------| FUNCTIONS: Sensor Detection |----------> */

/**
 * Reads state of CyBot bumper plates.
 * 
 * @param sensor Address of OI sensor values
 * @returns 1 if bumped; 0 otherwise
**/
int bot_isBumped(oi_t *sensor);

/* <----------| FUNCTIONS: Drive Commands |----------> */

/**
 * Sets CyBot wheels to drive forward at given velocity.
 * 
 * @param velocity Arbitrary wheel speed value (-500 to 500)
**/
void bot_drive(int velocity);

/**
 * Drives the CyBot a specified distance in centimeters.
 * Stops bot early upon bumping into an object.
 * 
 * @param sensor Address of OI sensor values
 * @param velocity Arbitrary wheel speed value (-500 to 500)
 * @param distanceCM Distance (cm) for CyBot to drive forward
 * @returns True distance (cm) traveled
**/
double bot_driveDistance(oi_t *sensor, int velocity, double distanceCM);

/**
 * Accurately drives the CyBot a specified distance in centimeters.
 * Slowly ramps up to speed and gently brakes before reaching desired distance.
 * Stops bot early upon bumping into an object.
 * 
 * @param sensor Address of OI sensor values
 * @param velocity Arbitrary wheel speed value (-500 to 500)
 * @param distanceCM Distance (cm) for CyBot to drive forward
 * @returns True distance (cm) traveled
**/
double bot_driveDistancePrecise(oi_t *sensor, int velocity, double distanceCM);

/**
 * Accurately drives the CyBot a specified distance.
 * Attempts to avoid encountered obstacles by navigating around them.
 * 
 * @param sensor Address of OI sensor values
 * @param distanceCM Distance (cm) for CyBot to drive forward
 * @returns True distance (cm) traveled
**/
void bot_driveObstacles(oi_t *sensor, double distanceCM);

/**
 * Sets wheels to tank turn cybot to left for +velocity; right for -velocity
 * 
 * @param sensor Address of OI sensor values
 * @param velocity Arbitrary wheel speed value (-500 to 500)
 * @param degrees Relative degrees to turn Cybot (0.0 to 360.0)
**/
void bot_turn(int velocity);

// Tank turn the CyBot counter-clockwise by degrees
/**
 * Tank turn the CyBot counter-clockwise by degrees
 * 
 * @param sensor Address of OI sensor values
 * @param velocity Arbitrary wheel speed value (-500 to 500)
 * @param degrees Relative degrees to turn Cybot (0.0 to 360.0)
**/
void bot_turnDegrees(oi_t *sensor, int velocity, double degrees);

/**
 * Stops the CyBot wheels.
**/
void bot_stopWheels(void);

/* <----------| FUNCTIONS: User Control |----------> */

/**
 * Execute a certain movement action on the CyBot based on user input.
 * 
 * @param sensor Address of OI sensor values
 * @param vectors Data array for field scans to update
 * @param input ASCII character "code" corresponding to a respective bot command
 * @returns 1 upon successful read and execution of user input; 0 otherwise
**/
int bot_executeCommand(oi_t* sensor, scan_t vectors[], char input);

#endif /* MOVEMENT_H_ */
