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

// Longest single putty message
#define MAX_MESSAGE_LEN 100

// Magic values for scans
#define CRASH_AVOIDANCE_OFFSET 10

// Initialization values
#define BAUD_RATE 115200
#define INIT_SERVO 0b0001
#define INIT_PING 0b0010
#define INIT_IR 0b0100

// TODO: use these for interrupts
volatile char uart_data;
volatile char flag;


// Pulled from Lab 10's old main
volatile int button_num; // Current value of LCD pushbuttons

uint16_t servo_rightBound;
uint16_t servo_leftBound;

/* <----------| UART METHODS |----------> */

// Loop through array of distance values and print table of scanned angles
void printScanData(scanVector vectors[], uint8_t numVectors);

// Execute a certain movement action on the cybot based on user input
int executeBotCommand(oi_t* sensor, scanVector vectors[], char input);

// Sets bot into manual mode and until user exits
void engageManualMode(oi_t* sensor, scanVector vectors[]);

/* <----------| MATH & HELPER METHODS |----------> */

// Returns a 1 if given value is within +/- tolerance of target, 0 if not
uint8_t isWithinTolerance(uint8_t value, uint8_t target, uint8_t tolerance);

// Returns the mean (average) of all values up to index length-1 in array
uint8_t mean(uint8_t values[], uint8_t length);

// Shifts all items to the left, remove first item, and appends newValue to length-1 index
void updateBuffer(uint8_t buffer[], uint8_t length, uint8_t newValue);

// TODO: comment me!
scanVector scanAngle (uint8_t angle);

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

scanVector scanAngle(uint8_t angle) {
    scanVector returnedVector;

    // Move servo to input angle and store in degrees
    servo_move((float)angle);
    returnedVector.angle = angle;

    // Scan and store ultrasound in centimeters (capped at 250cm)
    uint8_t pingDistanceRaw = (uint8_t)ping_read();
    returnedVector.pingDistance = pingDistanceRaw > 250.0 ? (uint8_t)(250) : (uint8_t)(pingDistanceRaw);

    // Scan and store converted IR data in centimeters
    returnedVector.irDistance = adc_calculateIRDistance(adc_read());

    return returnedVector;
}

void printScanData(scanVector vectors[], uint8_t numVectors) {
    char output[MAX_MESSAGE_LEN];
    uint8_t i = 0;

    // Print table header
    uart_sendStr("Angle(Degrees)\tSound_Dist(cm)\tIR_Dist(cm)\r\n");

    // Loop through values and print angle and distance to putty in table format
    for (i = 0; i < numVectors; i++) {
        snprintf(output, MAX_MESSAGE_LEN, "%u\t%u\t%u\r\n", vectors[i].angle, vectors[i].pingDistance, vectors[i].irDistance);
        uart_sendStr(output);
    }

    uart_sendStr("END\n");
}

uint8_t isWithinTolerance(uint8_t value, uint8_t target, uint8_t tolerance) {
    return abs(value - target) < tolerance;
}

uint8_t mean(uint8_t values[], uint8_t length) {
    uint8_t i = 0;
    uint16_t total = 0;
    for (i = 0; i < length; i++) {
        total += values[i];
    }
    return total / length;
}

void updateBuffer(uint8_t buffer[], uint8_t length, uint8_t newValue) {
    uint8_t i = 0;
    for (i = 0; i < length - 1; i++) {
        buffer[i] = buffer[i + 1];
    }
    buffer[length - 1] = newValue;
}

int executeBotCommand(oi_t* sensor, scanVector vectors[], char input) {
    oi_update(sensor);

    // Convert input character into command for bot to execute
    switch (input) {
        case 'w': bot_drive(BOT_MAX_SPEED); break;
        case 's': bot_drive(-BOT_MAX_SPEED); break;
        case 'a': bot_turn(BOT_TURN_SPEED); break;
        case 'd': bot_turn(-BOT_TURN_SPEED); break;
        case 'm': scanField(SCAN_START, SCAN_END, SCAN_INCREMENT, vectors); printScanData(vectors, NUM_SCANS); break;
        case ' ': bot_stopWheels(); break;
        case '3': bot_driveSquare(sensor); break;
        case '4': bot_driveObstacles(sensor, 200); break;
        case 'q': bot_turnDegrees(sensor, BOT_TURN_SPEED, 5.0); break;
        case 'e': bot_turnDegrees(sensor, BOT_TURN_SPEED, -5.0); break;
        default:
            // Command not recognized
            return 0;
    }

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
    }

    return;
}
