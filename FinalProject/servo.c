/*
 * servo.c
 *
 *  Created on: Nov 7, 2025
 *      Author: jz23
 */
#include <stdint.h>
#include "timer.h"
#include "servo.h"
#include <inc/tm4c123gh6pm.h>

void servo_init(void){

    SYSCTL_RCGCGPIO_R |= 0b000010; //clock for port B
    SYSCTL_RCGCTIMER_R |= 0b00000010; //Clock for timer 1
    while((SYSCTL_PRTIMER_R & 0x02)==0);
    //enable port PB5 for output
    GPIO_PORTB_AFSEL_R |= 0b00100000;
    GPIO_PORTB_DIR_R |= 0b00100000;
    GPIO_PORTB_DEN_R |= 0b00100000;
    GPIO_PORTB_PCTL_R &= ~0x00F00000;
    GPIO_PORTB_PCTL_R |= 0x00700000;


    TIMER1_CTL_R &= ~0b100000000;
    TIMER1_CFG_R = 0x4;
    TIMER1_TBMR_R = 0x0A;
    TIMER1_TBPR_R = 0x4;
    TIMER1_TBILR_R = 320000 & 0xFFFF; //20 ms period 16 MHz clock

    uint32_t match = 320000 - 16000;
    TIMER1_TBMATCHR_R = match & 0xFFFF;
    TIMER1_TBPMR_R =  (match >> 16) & 0xFF;
    TIMER1_CTL_R |= 0b100000000;

}

int servo_move(float degrees){
    int matchdeg0 = 313156;
    int matchdeg180 = 283822;
    int matchrange = matchdeg0 - matchdeg180;
    if(degrees < 0){
        degrees = 0;
    }
    if(degrees > 180){
        degrees = 180;
    }
    float pulse = 1.0f + (degrees /180) * 1.0f;
    uint32_t match = matchdeg0 - ((matchrange * degrees)/180);
    TIMER1_TBMATCHR_R = match & 0xFFFF;
    TIMER1_TBPMR_R =  (match >> 16) & 0xFF;

    return match;
}
