#include "open_interface.h"
double move_forward(oi_t *sensor, int centimeters);

// Turns the robot clockwise by a given number of degrees.
void turn_clockwise(oi_t *sensor, double degrees);

// Turns the robot counter-clockwise by a given number of degrees.
void turn_counter_clockwise(oi_t *sensor, double degrees);
