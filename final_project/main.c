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

// Initialization values
#define BAUD_RATE 115200

// For use with UART interrupts
// FIXME: why are these necessary to declare for compilation?
volatile char uart_data, flag;

// Servo callibration values
uint16_t servo_rightBound, servo_leftBound;

/* <----------| FUNCTONS |----------> */

// Sets bot into manual mode and until user exits
void engageManualMode(oi_t* sensor, scan_t vectors[]);

/* <----------| IMPLEMENTATIONS |----------> */

uint8_t main(void) {
    // Declare variables
    oi_t *sensorData = oi_alloc();
    scan_t scanData[SCAN_TOTAL];
    char puttyMessage[UART_MESSAGE_LEN];
    char userInput = 0;

    // Initialize variables
    oi_init(sensorData);
    timer_init();
    adc_init();
    uart_init(BAUD_RATE);
    ping_init();
    servo_init();
    servo_rightBound = BOTCAL_B23_SERVO_BOUND_R;
    servo_leftBound = BOTCAL_B23_SERVO_BOUND_L;

    // Uncomment and run to find cybot servo callibration values:
    /*volatile int button_num;
    button_init();
    init_button_interrupts();
    lcd_init();
    servo_callibrate();
    servo_callibrate();*/

    // Update putty once serial connection is successful
    uart_sendStr("Serial connection established.\r\n");
}

void engageManualMode(oi_t* sensor, scan_t vectors[]) {
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

        bot_executeCommand(sensor, vectors, input);

        if (input == 'm') {
            scan_readField(0, 180, 2, vectors);
            scan_printVectors(vectors, NUM_SCANS);
            uart_sendChar('\n');
        }
        else {
            bot_executeCommand(sensor, input);
        }
    }

    return;
}
