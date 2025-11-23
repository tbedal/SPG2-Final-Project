/**
 * main.c
 * 
 * The main file.
 *
 * @date November 11, 2025
 * @author Thiago Bedal
 * @author Joseph Vesterby
**/

/* <----------| INCLUDES |----------> */

#include "adc.h"
#include "uart.h"
#include "lcd.h"
#include "Timer.h"
#include "math.h"
#include "open_interface.h"
#include "movement.h"
#include "bot_callibration.h"
#include "ping.h"
#include "servo.h"
#include "button.h"
#include "scan.h"


/* <----------| DEFINITIONS |----------> */

// Magic values
#define CRASH_AVOIDANCE_OFFSET 10

// Initialization values
#define BAUD_RATE 115200

// TODO: use these for interrupts
volatile char uart_data;
volatile char flag;

// Pulled from Lab 10's old main
volatile int button_num; // Current value of LCD pushbuttons

uint16_t servo_rightBound;
uint16_t servo_leftBound;

/* <----------| UART METHODS |----------> */

// Execute a certain movement action on the cybot based on user input
int executeBotCommand(oi_t* sensor, scanVector vectors[], char input);

// Sets bot into manual mode and until user exits
void engageManualMode(oi_t* sensor, scanVector vectors[]);

/* <----------| IMPLEMENTATIONS |----------> */

uint8_t main(void)
{
    // Declare variables
    oi_t *sensor_data = oi_alloc();
    scanVector measuredVectors[NUM_SCANS];
    char puttyMessage[MAX_MESSAGE_LEN];
    char inputChar = 0;
    double smallestObjectDistance;
    double nextTurnDegrees;
    uint8_t smallestObjectAngle;

    // Initialize variables
    oi_init(sensor_data);
    timer_init();
    adc_init();
    uart_init(BAUD_RATE);
    ping_init();
    servo_init();
    servo_rightBound = 49295;
    servo_leftBound = 21764;


    // Uncomment and run to find cybot servo callibration values:
    /*button_init();
    init_button_interrupts();
    lcd_init();
    servo_callibrate();
    servo_callibrate();*/

    // Update putty once serial connection is successful
    uart_sendStr("Serial connection established.\r\n");

    // Primary autonomous/manual operation loop
    while (1) {
        /* <----------| STEP 0: WAIT FOR USER COMMAND |----------> */

        // FIXME: GUI gets mad after faulty instructions
        do { inputChar = uart_getChar(); } while (inputChar != 't' && inputChar != 'h');
        if (inputChar == 't') {
            engageManualMode(sensor_data, measuredVectors);
            continue;
        }

        /* <----------| STEP 1: SCAN FIELD |----------> */

        // Perform scan across field and print raw distances and filter noise with rolling average
        scanField(SCAN_START, SCAN_END, SCAN_INCREMENT, measuredVectors);
        rollingAverageFilter(measuredVectors, NUM_SCANS, BUFFER_SIZE);

        // Find smallest object in filtered data
        smallestObjectAngle = findSmallestObject(measuredVectors, NUM_SCANS);

        // Point, turn, and drive to smallest object found
        servo_move(smallestObjectAngle);
        smallestObjectDistance = (double)measuredVectors[smallestObjectAngle / SCAN_INCREMENT].pingDistance;

        // Notify client of scan results
        snprintf(puttyMessage, MAX_MESSAGE_LEN, "Wants to turn %u Degrees, then drive: %.1f cm. Press `h` to continue.\r\n", smallestObjectAngle, smallestObjectDistance);
        uart_sendStr(puttyMessage);

        /* <----------| STEP 2: WAIT FOR USER COMMAND |----------> */

        do { inputChar = uart_getChar(); } while (inputChar != 't' && inputChar != 'h');
        if (inputChar == 't') {
            engageManualMode(sensor_data, measuredVectors);
            continue;
        }

        /* <----------| STEP 3: ATTEMPT DRIVE |----------> */

        // Turn and drive to smallest object found in field
        bot_turnDegrees(sensor_data, BOT_TURN_SPEED, 90.0 - smallestObjectAngle);
        bot_driveDistancePrecise(sensor_data, BOT_CRUISE_SPEED, smallestObjectDistance - CRASH_AVOIDANCE_OFFSET);

        // Follow collision response protocol if either bumper is hit
        if (bot_isBumped(sensor_data)) {
            nextTurnDegrees = sensor_data -> bumpLeft ? -90.0 : 90.0;

            // Update user of collision
            snprintf(puttyMessage, MAX_MESSAGE_LEN, "Wants to go around object by turning %.1f degrees. Press `h` to execute.\r\n", nextTurnDegrees);
            uart_sendStr(puttyMessage);
        }
        else {
            // Request new input from user
            uart_sendChar('\n');
            continue;
        }

        /* <----------| STEP 4: WAIT FOR USER COMMAND |----------> */

        do { inputChar = uart_getChar(); } while (inputChar != 't' && inputChar != 'h');
        if (inputChar == 't') {
            engageManualMode(sensor_data, measuredVectors);
            continue;
        }

        /* <----------| STEP 5: KEEP TURNING TILL NO BUMP |----------> */

        bot_driveDistance(sensor_data, -BOT_CRUISE_SPEED, 5.0);
        bot_turnDegrees(sensor_data, BOT_TURN_SPEED, nextTurnDegrees);
        bot_driveDistancePrecise(sensor_data, BOT_CRUISE_SPEED, 10.0);
        bot_turnDegrees(sensor_data, BOT_TURN_SPEED, -nextTurnDegrees);
    }
}

int executeBotCommand(oi_t* sensor, scanVector vectors[], char input) {
    oi_update(sensor);

    // Convert input character into command for bot to execute
    switch (input) {
        case 'w': bot_drive(BOT_MAX_SPEED); break;
        case 's': bot_drive(-BOT_MAX_SPEED); break;
        case 'a': bot_turn(BOT_TURN_SPEED); break;
        case 'd': bot_turn(-BOT_TURN_SPEED); break;
        case 'm': scan_readField(SCAN_START, SCAN_END, SCAN_INCREMENT, vectors); scan_printVectorsvectors, NUM_SCANS); break;
        case ' ': bot_stopWheels(); break;
        case '3': bot_driveSquare(sensor); break;
        case '4': bot_driveObstacles(sensor, 200); break;
        case 'q': bot_turnDegrees(sensor, BOT_TURN_SPEED, 5.0); break;
        case 'e': bot_turnDegrees(sensor, BOT_TURN_SPEED, -5.0); break;
        default:
            // Command not recognized
            return 0;
    }

    // FIXME: code breaks when user sends back nonexistent command because server is expecting to receive back this character but executeBotCommand()
    // early returns a 0 and no '\n' on the swtich() default. Can we queue commands by putting uart_sendChar() at top of function? Or should we send a string
    // informing server that input was not recognized
    // Requests socket for new input
    uart_sendChar('\n');

    // Successful completion of function
    return 1;
}

void engageManualMode(oi_t* sensor, scanVector vectors[]) {
    char input = 0;
    char output[MAX_MESSAGE_LEN];

    // Status update
    snprintf(output, MAX_MESSAGE_LEN, "Toggled manual\r\n");
    uart_sendStr(output);

    // Continue requesting input from user until user toggles out of auto
    while (1) {
        input = uart_getChar();

        if (input == 't') {
            oi_setWheels(0, 0);
            snprintf(output, MAX_MESSAGE_LEN, "Toggled auto\r\n");
            uart_sendStr(output);
            break;
        }

        executeBotCommand(sensor, vectors, input);

        if (input == 'm') {
            scan_readField(0, 180, 2, vectors);
            scan_printVectors(vectors, NUM_SCANS);
            uart_sendChar('\n');
        }
        else {
            executeBotCommand(sensor, input);
        }
    }

    return;
}
