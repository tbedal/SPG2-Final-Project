/**
 * main.c
 * 
 * The main file. 
 *
 * @date November 11, 2025
 * @authors Thiago Bedal, Joseph Vesterby
 * @authors Griffin Cegielsi, Andrew Pham, Joseph Zeffiro
**/

/* <----------| INCLUDES |----------> */

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

/* <----------| DEFINITIONS |----------> */

oi_t *sensor_data;

static float botx = 0.0f;
static float boty = 0.0f;
static float bottheta = 0.0f;

typedef struct smallObject {
    int angle;
    double distance;
    int width;
} smallObject;


/* <----------| FUNCTONS |----------> */

/**
 * Scans the field for objects and sends results to Python via UART
**/
void objectScan();

/**
 * Sends position to python to generate minimap
 * 
 * @warning Function reads static variables botx, boty, bottheta
**/
void sendPosition();

/**
 * Converts user input into executable commands
**/
void manualMode();

/* <----------| IMPLEMENTATIONS |----------> */

int main(void) {
    // Initialize libraries
    timer_init();
    lcd_init();
    uart_init();
    adc_init();
    servo_init();
    ping_init();
    IntMasterEnable();

    // Initialize OI sensors
    sensor_data = oi_alloc();
    oi_init(sensor_data);

    // Declare UART variables
    char my_data;
    char command[20];
    int index = 0;

    // Main loop
    while (1) {
        // Read command
        index = 0;
        my_data = uart_receive();
        while (my_data != '\n') {
            command[index++] = my_data;
            my_data = uart_receive();
        }
        command[index] = '\0';

        // Execute command
        if (command[0] == 'm') {
            manualMode();
        }
        else if (command[0] == 'h') {
            objectScan();
            char msg[80];
            sprintf(msg, "large object angle: %d distance: %.2f cm\n", -1, -1); // UART usage here is deprecated; sends dummy values
            uart_sendStr(msg);
        }
        else if (command[0] == 'x') {
            playNoise();
        }
    }
}

void objectScan() {
    int angle = 0;
    int index = 0;

    scan_t vectors[91];
    scan_readField(0, 180, 2, vectors);
    scan_filterNoise(vectors, 91);

    for (angle = 0; angle < 180; angle += 2) {
        // Send scan result for plotting
        char msg[40];
        sprintf(msg, "%d %.2f %.2f\n", angle, vectors[index].irDistance / 100.0, vectors[index].pingDistance / 100.0);
        uart_sendStr(msg);

        index++;
    }

    uart_sendStr("END\n");
}

void sendPosition() {
    char message[64];
    float x_cm = botx / 10;
    float y_cm = boty / 10;
    float thetadeg = bottheta;
    sprintf(message,"pos %.2f %.2f %.2f\n",x_cm,y_cm,thetadeg);
    uart_sendStr(message);
}

void manualMode() {
    char my_data;
    int flag = 0;

    while (!flag) {
        my_data = uart_receive();
        oi_update(sensor_data);

        switch (my_data) {
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