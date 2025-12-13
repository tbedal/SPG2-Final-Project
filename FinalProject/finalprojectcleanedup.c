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

botx = 0.0f;
boty = 0.0f;
bottheta = 0.0f;

typedef struct smallObject {
    int angle;
    double distance;
    int width;
} smallObject;


/* <----------| FUNCTONS |----------> */

// TODO: comment me!
smallObject objectScan();

// TODO: comment me!
void sendPosition();

// TODO: comment me!
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

    sensor_data = oi_alloc();
    oi_init(sensor_data);

    char my_data;
    char command[20];
    int index = 0;

    while (1) {
        // Read command
        index = 0;
        my_data = uart_receive();

        while (my_data != '\n') {
            command[index++] = my_data;
            my_data = uart_receive();
        }

        command[index] = '\0';

        // Handle commands

        // m = manual drive mode
        if (command[0] == 'm') {
            manualMode();
        }

        // h = scan
        else if (command[0] == 'h') {
            objectScan();
            char msg[80];
            sprintf(msg, "large object angle: %d distance: %.2f cm\n", 0, 0); // FIXME: dummy values
            uart_sendStr(msg);
        }

        // x = play noise
        else if (command[0] == 'x') {
            playNoise();
        }
    }
}

// TODO: comment me!
// SEND POSITION TO PYTHON FOR MINIMAP
void sendPosition() {
    char message[64];
    float x_cm = botx / 10;
    float y_cm = boty / 10;
    float thetadeg = bottheta;
    sprintf(message,"pos %.2f %.2f %.2f\n",x_cm,y_cm,thetadeg);
    uart_sendStr(message);
}

// TODO: comment me!
//  MANUAL MODE
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