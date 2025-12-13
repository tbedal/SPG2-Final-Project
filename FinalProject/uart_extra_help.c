/*
*
*   uart_extra_help.c
* Description: This is file is meant for those that would like a little
*              extra help with formatting their code, and followig the Datasheet.
*/

#include "uart_extra_help.h"
#include "timer.h"
#include <stdint.h>
#include <stdbool.h>
#include "lcd.h"
#include <inc/tm4c123gh6pm.h>
#include "driverlib/interrupt.h"


void uart_init(void)
{
//    SYSCTL_RCGCGPIO_R |= REPLACE_ME;      // enable clock GPIOB (page 340)
//    SYSCTL_RCGCUART_R |= REPLACE_ME;      // enable clock UART1 (page 344)
//    GPIO_PORTB_AFSEL_R = REPLACE_ME;      // sets PB0 and PB1 as peripherals (page 671)
//    GPIO_PORTB_PCTL_R  = REPLACE_ME;       // pmc0 and pmc1       (page 688)  also refer to page 650
//    GPIO_PORTB_DEN_R   = REPLACE_ME;        // enables pb0 and pb1
//    GPIO_PORTB_DIR_R   = REPLACE_ME;        // sets pb0 as output, pb1 as input


    SYSCTL_RCGCUART_R |= 0b00000010;
          SYSCTL_RCGCGPIO_R |= 0b0000010;
          timer_waitMillis(1);            // Small delay before accessing device after turning on clock
          GPIO_PORTB_AFSEL_R |= 0b00000011;

          GPIO_PORTB_PCTL_R |= 0x00000011;     // Force 1's in the disired locations
          GPIO_PORTB_DEN_R |= 0b00000011;


          GPIO_PORTB_DIR_R |= 0b00000011;

    //compute baud values [UART clock= 16 MHz] 
//    double fbrd;
//    int    ibrd;
//
//    fbrd = 44; // page 903
//    ibrd = 8;


    UART1_CTL_R &= 0xFFFE;      // disable UART1 (page 918)
    UART1_IBRD_R = 0x0008;        // write integer portion of BRD to IBRD
    UART1_FBRD_R = 0b101100;   // write fractional portion of BRD to FBRD
    UART1_LCRH_R = 0b01100000;        // write serial communication parameters (page 916) * 8bit and no parity
    UART1_CC_R   = 0x0;          // use system clock as clock source (page 939)
    UART1_CTL_R |= 0x0001;        // enable UART1

}

void uart_sendChar(char data)
{
   //TODO
   

   while((UART1_FR_R & 0x20)){

}
UART1_DR_R = data;
}
char uart_receive(void)
{
 //TODO
    while((UART1_FR_R & 0x10)!=0){
    }
    return (char)(UART1_DR_R & 0xFF);
 
}

// TODO: comment me!
// UART STRING FUNCTION
void uart_sendStr(const char *str) {
    int i = 0;
    while (str[i] != '\0') {
        uart_sendChar(str[i]);
        i++;
    }
}

// _PART3


void uart_interrupt_init()
{
    // Enable interrupts for receiving bytes through UART1
    UART1_IM_R |= 0x0010; //enable interrupt on receive - page 924

    // Find the NVIC enable register and bit responsible for UART1 in table 2-9
    // Note: NVIC register descriptions are found in chapter 3.4
    NVIC_EN0_R |= 0x00000040; //enable uart1 interrupts - page 104

    // Find the vector number of UART1 in table 2-9 ! UART1 is 22 from vector number page 104
    IntRegister(INT_UART1, uart_interrupt_handler); //give the microcontroller the address of our interrupt handler - page 104 22 is the vector number

}

void uart_interrupt_handler()
{
// STEP1: Check the Masked Interrupt Status
    if(UART1_MIS_R & 0x0010){

        char copy = UART1_DR_R & 0xFF;
        uart_sendChar(copy);
        UART1_ICR_R = 0b11111111;
    }
//STEP2:  Copy the data

//STEP3:  Clear the interrupt



}
