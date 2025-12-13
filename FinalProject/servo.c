/**
 * servo.c
 *
 * Contains functions to operate the Servo motor on the CyBot
 *
 * @date November 5, 2025
 * @authors Thiago Bedal, Joseph Vesterby
 * @authors Griffin Cegielsi, Andrew Pham, Joseph Zeffiro
**/

/* <----------| INCLUDES |----------> */

#include <stdint.h>
#include "timer.h"
#include "servo.h"
#include <inc/tm4c123gh6pm.h>

/* <----------| IMPLEMENTATIONS |----------> */

void servo_init(void) {
    /* <----------| INIT GPIO |----------> */

    SYSCTL_RCGCGPIO_R |= 0b000010; // Clock for Port B
    SYSCTL_RCGCTIMER_R |= 0b00000010; // Clock for Timer 1
    while((SYSCTL_PRTIMER_R & 0x02)==0);
    
    // Enable port PB5 for output
    GPIO_PORTB_AFSEL_R |= 0b00100000;
    GPIO_PORTB_DIR_R |= 0b00100000;
    GPIO_PORTB_DEN_R |= 0b00100000;
    GPIO_PORTB_PCTL_R &= ~0x00F00000;
    GPIO_PORTB_PCTL_R |= 0x00700000;

    /* <----------| INIT COUNTER |----------> */

    TIMER1_CTL_R &= ~0b100000000;
    TIMER1_CFG_R = 0x4;
    TIMER1_TBMR_R = 0x0A;
    TIMER1_TBPR_R = 0x4;
    TIMER1_TBILR_R = 320000 & 0xFFFF; // 20 ms period 16 MHz clock

    uint32_t match = 320000 - 16000;
    TIMER1_TBMATCHR_R = match & 0xFFFF;
    TIMER1_TBPMR_R =  (match >> 16) & 0xFF;
    TIMER1_CTL_R |= 0b100000000;
}

int servo_move(float degrees) {
    // Set callibration values
    int matchdeg0 = 313156;
    int matchdeg180 = 283822;
    int matchrange = matchdeg0 - matchdeg180;

    // Normalize degree range to 0-180
    if      (degrees < 0  ) { degrees = 0;   }
    else if (degrees > 180) { degrees = 180; }

    // Send PWM signal to servo
    float pulse = 1.0f + (degrees /180) * 1.0f;
    uint32_t match = matchdeg0 - ((matchrange * degrees)/180);
    TIMER1_TBMATCHR_R = match & 0xFFFF;
    TIMER1_TBPMR_R =  (match >> 16) & 0xFF;

    // Return calculated match value
    return match;
}
