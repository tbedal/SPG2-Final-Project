//Includes
#include "open_interface.h"
#include "timer.h"
#include "lcd.h"
#include "uart_extra_help.h"
#include "adc.h"
#include <stdint.h>
#include <math.h>
#include "movement.h"
#include "servo.h"
#include "ping.h"
#include "scan.h"
oi_t *sensor_data;
//Location Variables for MiniMap
static float botx = 0.0f;
static float boty = 0.0f;
static float bottheta = 0.0f;

//STRUCT DEFINITIONS
typedef struct smallObject
{
    int angle;
    double distance;
    int width;
} smallObject;
typedef struct obstacle_object
{
    int angle;
    double distance;
    int width;
} obstacle_object;

smallObject objectScan();
void uart_sendStr(const char *str);

//UART STRING FUNCTION
void uart_sendStr(const char *str)
{
    int i = 0;
    while (str[i] != '\0')
    {
        uart_sendChar(str[i]);
        i++;
    }
}

// DISTANCE SCAN USING PING
float scan(int degrees)
{
    servo_move(degrees);
    int cycles = ping_read();
    float distance = (float)cycles / 16000000.0;
    distance /= 2.0;
    distance *= 34000.0;   // cm
    timer_waitMillis(100);
    return distance;
}
//CHECK FOR BOUNDARY LINES
void boundaryCheck(){

    oi_update(sensor_data);
    lcd_init();
    if(sensor_data->cliffLeftSignal > 2600 || sensor_data->cliffFrontLeftSignal > 2600 || sensor_data->cliffFrontRightSignal > 2600 || sensor_data->cliffRightSignal > 2600){
        lcd_printf("L:%4d FL:%4d\nFR:%4d R:%4d",
                           sensor_data->cliffLeftSignal,
                           sensor_data->cliffFrontLeftSignal,
                           sensor_data->cliffFrontRightSignal,
                           sensor_data->cliffRightSignal);
        uart_sendStr("Wall detected!\n");
    }

}
//CHECK FOR HOLES
void holeCheck(){

    oi_update(sensor_data);
    lcd_init();
    if(sensor_data->cliffLeftSignal < 100 || sensor_data->cliffFrontLeftSignal < 100 || sensor_data->cliffFrontRightSignal < 100 || sensor_data->cliffRightSignal < 100){
        lcd_printf("L:%4d FL:%4d\nFR:%4d R:%4d",
                           sensor_data->cliffLeftSignal,
                           sensor_data->cliffFrontLeftSignal,
                           sensor_data->cliffFrontRightSignal,
                           sensor_data->cliffRightSignal);
         uart_sendStr("Hole detected!\n");
       }

}
//CHECK FOR BUMPING OBJECTS
void objectCollision(){

    oi_update(sensor_data);
    if (sensor_data->bumpLeft || sensor_data->bumpRight){
        uart_sendStr("Object bumped!\n");
    }
}
//SEND POSITION TO PYTHON FOR MINIMAP
void sendPosition(){
    char message[64];
    float x_cm = botx / 10;
    float y_cm = boty/10;
    float thetadeg = bottheta;
    sprintf(message,"pos %.2f %.2f %.2f\n",x_cm,y_cm,thetadeg);
    uart_sendStr(message);
}
//UPDATE POISITION
void updateodometry(){
        double d_mm = (double)sensor_data->distance; /* mm */
        double da_deg = (double)sensor_data->angle;  /* degrees */

        /* Update heading first */
        bottheta += da_deg;
        /* Normalize to [0,360) */
        while (bottheta >= 360.0f) bottheta -= 360.0f;
        while (bottheta < 0.0f) bottheta += 360.0f;

        /* Convert heading to radians for position update */
        double theta_rad = bottheta * (M_PI / 180.0f);

        /* Integrate distance into x,y (we treat d_mm as forward in current heading) */
        botx += d_mm * cosf(theta_rad);
        boty += d_mm * sinf(theta_rad);
}
//DRIVE BOT AT CHOSEN SPEED
void bot_drive(int velocity) {
    oi_setWheels(velocity, velocity);
}
//DRIVE SPECIFICED DISTANCE AND UPDATE POISTION FOR MINIMAP
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

    return distanceTraveledMM / 10.0;   // return cm
}

//TURN BOT AT SPECIFIED SPEED
void bot_turn(int velocity) {
    oi_setWheels(velocity, -velocity);
}
//TURN BOT SPECIFICED DEGREES
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

//  MANUAL MODE
void manualMode()
{
    char my_data;
    int flag = 0;

    while (!flag)
    {
        my_data = uart_receive();
        oi_update(sensor_data);
        switch (my_data)
        {
            case 'w':
                bot_driveDistance(sensor_data,100,5);
                oi_update(sensor_data);
                updateodometry();
                sendPosition();
                boundaryCheck();
                holeCheck();
                objectCollision();
                break;

            case 's':
                bot_driveDistance(sensor_data,-100,5);
                oi_update(sensor_data);
                updateodometry();
                sendPosition();
                boundaryCheck();
                holeCheck();
                objectCollision();
                break;

            case 'a':
                bot_turnDegrees(sensor_data,50,22.5);
                oi_update(sensor_data);
                updateodometry();
                sendPosition();
                boundaryCheck();
                holeCheck();
                objectCollision();
                break;

            case 'd':
                bot_turnDegrees(sensor_data,50,-22.5);
                oi_update(sensor_data);
                updateodometry();
                sendPosition();
                boundaryCheck();
                holeCheck();
                objectCollision();
                break;

            case 'b':   // exit manual mode
                oi_setWheels(0, 0);
                flag = 1;
                break;

            default:
                oi_setWheels(0, 0);
                break;
        }
    }
}

// OBJECT SCAN
smallObject objectScan()
{
    //uint16_t IR_raw;
    //float IR_dist[91];
    int angle = 0;
    int index = 0;
    //int max_object_distance = 50;

    //int object_detected = 0;
    //int start_degree = 0;
    //int objCount = 0;

    //obstacle_object object_data[10];

    smallObject smallest;
    smallest.angle = -1;
    smallest.distance = -1;
    smallest.width = -1;

    scan_t vectors[91];
    scan_readField(0, 180, 2, vectors);
    scan_filterNoise(vectors, 91);

    for (angle = 0; angle < 180; angle += 2)
    {
        // Send scan result for plotting
        char msg[40];
        sprintf(msg, "%d %.2f %.2f\n", angle, vectors[index].irDistance / 100.0, vectors[index].pingDistance / 100.0);
        uart_sendStr(msg);

        index++;
    }

    uart_sendStr("END\n");
    return smallest;
}
//PLAY SOUND FOR FINDING TABLE
void playNoise(){
   unsigned char notes[1] = {84};
   unsigned char duration[1] = {50};
   oi_loadSong(0,1,notes,duration);
   oi_play_song(0);

}
// MAIN
int main(void)
 {

    //Initialize libraries
    timer_init();
    lcd_init();
    uart_init();
    adc_init();
    servo_init();
    ping_init();
    IntMasterEnable();


    sensor_data = oi_alloc();
    oi_init(sensor_data);

    char my_data;
    char command[20];
    int index = 0;

    while (1)
    {
        // Read command
        index = 0;
        my_data = uart_receive();

        while (my_data != '\n')
        {
            command[index++] = my_data;
            my_data = uart_receive();
        }

        command[index] = '\0';

        // Handle commands
        // m = manual drive mode
        if (command[0] == 'm')
        {
            manualMode();
        }
        // h = scan
        else if (command[0] == 'h')
        {
            smallObject found = objectScan();

            char msg[80];
            sprintf(msg,
                    "large object angle: %d distance: %.2f cm\n",
                    found.angle, found.distance);

            uart_sendStr(msg);
        }
        // x = play noise
        else if(command[0] == 'x'){
            playNoise();
        }

    }
}
