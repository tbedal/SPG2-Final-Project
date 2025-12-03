/**
 * movement.c
 *
 * Contains helpful functions to navigate the CyBot around obstacles.
 * 
 * @date October 29, 2025
 * @authors Thiago Bedal, Joseph Vesterby
 * @authors Griffin Cegielsi, Andrew Pham, Joseph Zeffiro
**/

/* <----------| INCLUDES |----------> */

#include "movement.h"

/* <----------| IMPLEMENTATIONS |----------> */

int bot_isBumped(oi_t *sensor) {
    return sensor -> bumpLeft || sensor -> bumpRight;
}

void bot_drive(int velocity) {
    oi_setWheels(velocity, velocity);
}

double bot_driveDistance(oi_t *sensor, int velocity, double distanceCM) {
    // Initialize variables
    oi_update(sensor);
    double desiredDistanceMM = distanceCM * 10;
    double distanceTraveledMM = 0;

    // Drive forward until desired distance is reached
    bot_drive(velocity);
    while (distanceTraveledMM < desiredDistanceMM && !(sensor -> bumpLeft || sensor -> bumpRight )) {
        oi_update(sensor);
        if      (velocity > 0) { distanceTraveledMM += sensor -> distance; }
        else if (velocity < 0) { distanceTraveledMM -= sensor -> distance; }
        else { return 0.0; }
    }
    bot_stopWheels();

    return distanceTraveledMM / 10;
}

double bot_driveDistancePrecise(oi_t *sensor, int velocity, double distanceCM) {
    // Declare constants
    const double CRAWL_DISTANCE_MM = 30.0 * 10.0;
    const double DESIRED_DISTANCE_MM = distanceCM * 10.0;
    const double CRUISING_DISTANCE_MM = DESIRED_DISTANCE_MM - CRAWL_DISTANCE_MM;
    const int RAMP_UP_COEFFICIENT = 50;
    const int RAMP_DOWN_COEFFICIENT = 40;

    // Initialize variables
    oi_update(sensor);
    double distanceTraveledMM = 0.0;
    int velocitySet = (sensor -> requestedRightVelocity);
    int direction = velocity < 0 ? -1 : 1;

    // Ramp up to desired velocity linearly
    while (!bot_isBumped(sensor) && abs(velocitySet) < abs(velocity) && abs(distanceTraveledMM) < CRUISING_DISTANCE_MM) {
        bot_drive(velocitySet);

        oi_update(sensor);
        distanceTraveledMM += sensor -> distance;

        velocitySet += RAMP_UP_COEFFICIENT * direction;
    }

    // Cruise until within 30 cm of total distance
    bot_drive(velocity);
    while (!bot_isBumped(sensor) && abs(distanceTraveledMM) < CRUISING_DISTANCE_MM) {
        oi_update(sensor);
        distanceTraveledMM += sensor -> distance;
    }

    // Ramp down to crawl speed
    while (!bot_isBumped(sensor) && abs(velocitySet) > BOT_CRAWL_SPEED) {
        bot_drive(velocitySet);

        oi_update(sensor);
        distanceTraveledMM += sensor -> distance;

        velocitySet -= RAMP_DOWN_COEFFICIENT * direction;
    }

    // Drive slowly until desired distance reached
    while (!bot_isBumped(sensor) && abs(distanceTraveledMM) < DESIRED_DISTANCE_MM) {
        oi_update(sensor);
        distanceTraveledMM += sensor -> distance;
    }

    // Hard stop and returned true distance traveled
    bot_stopWheels();
    return distanceTraveledMM / 10;
}

void bot_driveObstacles(oi_t *sensor, double distanceCM) {
    double distanceTraveledCM = 0.0;

    // Continue navigating forward until desired distance has been traveled in the forward direction
    while (distanceTraveledCM < distanceCM) {
        distanceTraveledCM += bot_driveDistancePrecise(sensor, BOT_CRUISE_SPEED, distanceCM - distanceTraveledCM);

        // Follow collision response protocol if either bumper is hit
        if (bot_isBumped(sensor)) {
            double turnDegrees = sensor -> bumpLeft ? -90.0 : 90.0;
            distanceTraveledCM -= bot_driveDistancePrecise(sensor, -BOT_CRUISE_SPEED, 15.0);
            bot_turnDegrees(sensor, BOT_TURN_SPEED, turnDegrees);
            bot_driveDistancePrecise(sensor, BOT_CRUISE_SPEED, 10.0);
            bot_turnDegrees(sensor, BOT_TURN_SPEED, -turnDegrees);
        }
    }
}

void bot_turn(int velocity) {
    oi_setWheels(velocity, -velocity);
}

// FIXME: negative velocity values are technically legal here
void bot_turnDegrees(oi_t *sensor, int velocity, double degrees) {
    // Initialize variables
    oi_update(sensor);
    double degreesTurned = 0.0;

    // Tank turn left for +degrees; tank turn right for -degrees
    if      (degrees > 0.0) { bot_turn(velocity); }
    else if (degrees < 0.0) { bot_turn(-velocity); }

    // Continue turning until desired degrees reached
    while (abs(degreesTurned) < abs(degrees) - 0.1) { // TODO: do we need this absolute value here?
        oi_update(sensor);
        degreesTurned += sensor -> angle;
    }
    bot_stopWheels();
}

void bot_stopWheels(void) {
    oi_setWheels(0, 0);
}

int bot_executeCommand(oi_t* sensor, scan_t vectors[], char input) {
    oi_update(sensor);

    // Convert input character into command for bot to execute
    switch (input) {
        case 'w': bot_drive(BOT_MAX_SPEED); break;
        case 's': bot_drive(-BOT_MAX_SPEED); break;
        case 'a': bot_turn(BOT_TURN_SPEED); break;
        case 'd': bot_turn(-BOT_TURN_SPEED); break;
        case 'm': scan_readField(SCAN_START, SCAN_END, SCAN_INCREMENT, vectors); scan_printVectors(vectors, SCAN_TOTAL); break;
        case ' ': bot_stopWheels(); break;
        case '4': bot_driveObstacles(sensor, 200); break;
        case 'q': bot_turnDegrees(sensor, BOT_TURN_SPEED, 5.0); break;
        case 'e': bot_turnDegrees(sensor, BOT_TURN_SPEED, -5.0); break;
        default:
            uart_sendStr("Command not recognized\n");
            return 0;
    }

    // Ready for new input
    uart_sendChar('\n');

    // Successful completion of function
    return 1;
}
