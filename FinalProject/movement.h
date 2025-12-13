#include "open_interface.h"
#include "uart_extra_help.h"

// Location Variables for MiniMap
extern static float botx;
extern static float boty;
extern static float bottheta;

void updateodometry()

double move_forward(oi_t *sensor, int centimeters);

// Turns the robot clockwise by a given number of degrees.
void turn_clockwise(oi_t *sensor, double degrees);

// Turns the robot counter-clockwise by a given number of degrees.
void turn_counter_clockwise(oi_t *sensor, double degrees);

void bot_turn(int velocity);

void bot_turnDegrees(oi_t *sensor_data, int velocity, double degrees);

double bot_driveDistance(oi_t *sensor_data, int velocity, double distanceCM);

void bot_drive(int velocity);

void objectCollision();

void boundaryCheck();

void holeCheck();

// TODO: comment me!
// PLAY SOUND FOR FINDING TABLE
void playNoise();