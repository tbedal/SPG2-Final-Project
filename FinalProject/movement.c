#include "movement.h"

// TODO: comment me!
//UPDATE POISITION
void updateodometry() {
    double d_mm = (double)sensor_data->distance; // mm
    double da_deg = (double)sensor_data->angle;  // degrees

    // Update heading
    bottheta += da_deg;

    // Wrap degrees to 0-360
    while (bottheta >= 360.0f) bottheta -= 360.0f;
    while (bottheta < 0.0f) bottheta += 360.0f;

    // Convert heading to radians for position update */
    double theta_rad = bottheta * (M_PI / 180.0f);

    // Integrate distance into x,y (treat d_mm as forward in current heading)
    botx += d_mm * cosf(theta_rad);
    boty += d_mm * sinf(theta_rad);
}

// Move forward (or backward if negative)
double move_forward(oi_t* sensor, int centimeters) {
    double sum = 0;
    int speed = (centimeters < 0) ? -150 : 150;
    double target = fabs(centimeters);

    oi_setWheels(speed, speed);

    while (fabs(sum) < target) {
        oi_update(sensor);
        sum += fabs(sensor->distance);

        // Only stop early on bump if moving forward
        if (speed > 0 && (sensor->bumpLeft || sensor->bumpRight)) {
            oi_setWheels(0, 0);
            return sum;
        }
    }

    oi_setWheels(0, 0);
    return sum;
}

// Clockwise turn
void turn_clockwise(oi_t *sensor, double degrees) {
    double sum = 0;
    oi_setWheels(50, -50);
    while (fabs(sum) < degrees) {
        oi_update(sensor);
        sum += sensor->angle; // Clockwise: angle is negative
    }
    oi_setWheels(0, 0);
}

// Counter-clockwise turn
void turn_counter_clockwise(oi_t *sensor, double degrees) {
    double sum = 0;
    oi_setWheels(-50, 50);
    while (fabs(sum) < degrees) {
        oi_update(sensor);
        sum -= sensor->angle; // Counter-clockwise: angle is positive
    }
    oi_setWheels(0, 0);
}

// TODO: comment me!
// TURN BOT AT SPECIFIED SPEED
void bot_turn(int velocity) {
    oi_setWheels(velocity, -velocity);
}

// TODO: comment me!
// TURN BOT SPECIFICED DEGREES
void bot_turnDegrees(oi_t *sensor_data, int velocity, double degrees) {
    oi_update(sensor_data);
    double degreesTurned = 0.0;

    // Turn left for +degrees; right for -degrees
    if (degrees > 0.0) {
        bot_turn(velocity);
    } else {
        bot_turn(-velocity);
    }

    while (fabs(degreesTurned) < fabs(degrees)) {
        oi_update(sensor_data);

        // Apply incremental angle to odometry
        updateodometry();

        // Send updated absolute position (x, y, theta)
        sendPosition();

        // accumulate angle
        degreesTurned += sensor_data->angle;

        // reset sensor angle for next increment
        sensor_data->angle = 0;

        // smooth update timing
        timer_waitMillis(20);
    }

    oi_setWheels(0, 0);
}

// TODO: comment me!
// DRIVE SPECIFICED DISTANCE AND UPDATE POISTION FOR MINIMAP
double bot_driveDistance(oi_t *sensor_data, int velocity, double distanceCM) {
    oi_update(sensor_data);

    double desiredDistanceMM = distanceCM * 10.0;
    double distanceTraveledMM = 0.0;

    bot_drive(velocity);

    while (fabs(distanceTraveledMM) < fabs(desiredDistanceMM)) {
        oi_update(sensor_data);

        // Update odometry based on incremental distance/angle
        updateodometry();

        // Send new absolute position
        sendPosition();

        // Accumulate incremental distance
        distanceTraveledMM += sensor_data->distance;

        // Reset distance for next increment
        sensor_data->distance = 0;

        timer_waitMillis(20);
    }

    oi_setWheels(0, 0);

    return distanceTraveledMM / 10.0; // return cm
}

// TODO: comment me!
//DRIVE BOT AT CHOSEN SPEED
void bot_drive(int velocity) {
    oi_setWheels(velocity, velocity);
}

// TODO: comment me!
// CHECK FOR BUMPING OBJECTS
void objectCollision() {
    oi_update(sensor_data);
    if (sensor_data->bumpLeft || sensor_data->bumpRight) {
        uart_sendStr("Object bumped!\n");
    }
}

// TODO: comment me!
//CHECK FOR BOUNDARY LINES
void boundaryCheck() {
    oi_update(sensor_data);
    lcd_init();
    if (sensor_data->cliffLeftSignal > 2600 || sensor_data->cliffFrontLeftSignal > 2600 || sensor_data->cliffFrontRightSignal > 2600 || sensor_data->cliffRightSignal > 2600) {
        lcd_printf("L:%4d FL:%4d\nFR:%4d R:%4d",
                           sensor_data->cliffLeftSignal,
                           sensor_data->cliffFrontLeftSignal,
                           sensor_data->cliffFrontRightSignal,
                           sensor_data->cliffRightSignal);
        uart_sendStr("Wall detected!\n");
    }
}

// TODO: comment me!
//CHECK FOR HOLES
void holeCheck() {
    oi_update(sensor_data);
    lcd_init();
    if (sensor_data->cliffLeftSignal < 100 || sensor_data->cliffFrontLeftSignal < 100 || sensor_data->cliffFrontRightSignal < 100 || sensor_data->cliffRightSignal < 100) {
        lcd_printf("L:%4d FL:%4d\nFR:%4d R:%4d",
                           sensor_data->cliffLeftSignal,
                           sensor_data->cliffFrontLeftSignal,
                           sensor_data->cliffFrontRightSignal,
                           sensor_data->cliffRightSignal);
        uart_sendStr("Hole detected!\n");
    }
}

// TODO: comment me!
// PLAY SOUND FOR FINDING TABLE
void playNoise() {
   unsigned char notes[1] = {84};
   unsigned char duration[1] = {50};
   oi_loadSong(0,1,notes,duration);
   oi_play_song(0);
}
