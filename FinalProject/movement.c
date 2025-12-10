//#include "open_interface.h"
//#include "movement.h"
//double move_forward(oi_t*sensor,int centimeters) {
// oi_t *sensor_data = oi_alloc();
// oi_init(sensor_data);
// double sum = 0;
// double length = centimeters;
// if(length < 0){
//     oi_setWheels(-150, -150);
//      while (sum > length) {
//      oi_update(sensor_data);
//      sum += sensor_data->distance;
//  }
//  }
// oi_setWheels(150, 150);
// while (sum < length) {
// oi_update(sensor_data);
// sum += sensor_data->distance;
// if (sensor_data->bumpLeft || sensor_data->bumpRight) {
//  // respond to left bumper being pressed
//     oi_setWheels(0, 0);
//     return sensor_data->distance;
// }
//
// }
// oi_setWheels(0, 0); // stop
// oi_free(sensor_data);
// return 0;
//
//}
//void turn_clockwise(oi_t*sensor,double degrees) {
// oi_t *sensor_data = oi_alloc();
// oi_init(sensor_data);
// double degrees1 = degrees;
// double sum = 0;
// oi_setWheels(50, -50);
// while (sum < degrees1){
//     oi_update(sensor_data);
//     sum += sensor_data->angle;
//
// }
// oi_setWheels(0,0);
// oi_free(sensor_data);
//}
//
//void turn_counter_clockwise(oi_t*sensor,double degrees) {
//    oi_t *sensor_data = oi_alloc();
//     oi_init(sensor_data);
//     double degrees2 = degrees;
//     double sum = 0;
//     oi_setWheels(-50, 50);
//     while (sum < degrees2){
//         oi_update(sensor_data);
//         sum += sensor_data->angle;
//
//     }
//     oi_setWheels(0,0);
//     oi_free(sensor_data);
//    }

#include "movement.h"
#include "open_interface.h"

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






