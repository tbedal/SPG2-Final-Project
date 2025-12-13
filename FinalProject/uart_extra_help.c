/**
 * uart_extra_help.c
 *
 * Contains functions to communicate with the CyBot via UART
 * 
 * @date October 29, 2025
 * @authors Thiago Bedal, Joseph Vesterby
 * @authors Griffin Cegielsi, Andrew Pham, Joseph Zeffiro
**/

#include "uart_extra_help.h"
#include "timer.h"
#include <stdint.h>
#include <stdbool.h>
#include "lcd.h"
#include <inc/tm4c123gh6pm.h>
#include "driverlib/interrupt.h"

void uart_init(void) {
    SYSCTL_RCGCUART_R |= 0b00000010;
    SYSCTL_RCGCGPIO_R |= 0b0000010;
    timer_waitMillis(1);            // Small delay before accessing device after turning on clock
    GPIO_PORTB_AFSEL_R |= 0b00000011;

    GPIO_PORTB_PCTL_R |= 0x00000011;     // Force 1's in the disired locations
    GPIO_PORTB_DEN_R |= 0b00000011;

    GPIO_PORTB_DIR_R |= 0b00000011;

    UART1_CTL_R &= 0xFFFE;      // disable UART1 (page 918)
    UART1_IBRD_R = 0x0008;        // write integer portion of BRD to IBRD
    UART1_FBRD_R = 0b101100;   // write fractional portion of BRD to FBRD
    UART1_LCRH_R = 0b01100000;        // write serial communication parameters (page 916) * 8bit and no parity
    UART1_CC_R   = 0x0;          // use system clock as clock source (page 939)
    UART1_CTL_R |= 0x0001;        // enable UART1
}

void uart_sendChar(char data) {
    while ((UART1_FR_R & 0x20)) {
        // Wait
    }
    UART1_DR_R = data;
}

char uart_receive(void) {
    while ((UART1_FR_R & 0x10) != 0) {
        // Wait
    }
    return (char)(UART1_DR_R & 0xFF);
 
}

void uart_sendStr(const char *str) {
    int i = 0;
    while (str[i] != '\0') {
        uart_sendChar(str[i]);
        i++;
    }
}

void uart_interrupt_init() {
    UART1_IM_R |= 0x0010; // Enable interrupt on receive

    NVIC_EN0_R |= 0x00000040; // Enable UART1 interrupts

    IntRegister(INT_UART1, uart_interrupt_handler); // Give the microcontroller the address of our interrupt handler

}

void uart_interrupt_handler() {
    if (UART1_MIS_R & 0x0010) {
        char copy = UART1_DR_R & 0xFF;
        uart_sendChar(copy);
        UART1_ICR_R = 0b11111111;
    }
}
