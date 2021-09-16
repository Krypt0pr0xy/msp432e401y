
//UART TEST
#include <stdint.h>
#include "msp.h"



int main(void)
{
    //INIT UART
    //Universal Asynchronous Receiver/Transmitter (UART)
    //Slau 723a page 1620

    //UART PA0 33 – U0Rx
    //UART PA1 34 – U0Tx


    //UART Initialization and Configuration
    //26.4 page 1629
    //-----------------------------------------------------------------------
    //******1. Enable the UART module page 331
    // UART Module 0 Run Mode Clock Gating Control
    SYSCTL->RCGCUART |= BIT0;//0x1 = Enable and provide a clock to UART module 0 in run mode


    //******2. Enable the clock to the appropriate GPIO module page 326
    //GPIO Port A Run Mode Clock Gating Control
    SYSCTL->RCGCGPIO |= BIT0;//0x1 = Enable and provide a clock to GPIO port A in run mode.


    //******3.Set the GPIO AFSEL bits for the appropriate pins page 1213
    //GPIO Alternate Function Select
    GPIOA->AFSEL |= BIT0;//0x1 = The associated pin functions as a peripheral signal and is controlled by the alternate hardware function.
    GPIOA->AFSEL |= BIT1;//0x1 = The associated pin functions as a peripheral signal and is controlled by the alternate hardware function.


    //******4.Configure the GPIO current level or slew rate as specified for the mode selected
    //GPIO Slew Rate Control Select
    GPIOA->SLR |= BIT0;//0x1 = Slew rate control is enabled for the corresponding pin.
    GPIOA->SLR |= BIT1;//0x1 = Slew rate control is enabled for the corresponding pin.

    //GPIO 8-mA Drive Select
    GPIOA->DR8R |= BIT0;//The corresponding GPIO pin has 8-mA drive
    GPIOA->DR8R |= BIT1;//The corresponding GPIO pin has 8-mA drive


    //*****5.Configure the PMCn fields in the GPIOPCTL register to assign the UART signals to the appropriate pins
    GPIOA->PCTL |= BIT1;//GPIO Pin Multiplexing page 30/157


    //UART configuration page 1630
    //*****1. Disable the UART by clearing the UARTEN bit in the UARTCTL register
    UART0->CTL &= ~UART_CTL_UARTEN;
    //*****2. Write the integer portion of the BRD to the UARTIBRD register.
    UART0->IBRD = 10.8507;//BRD = 20000000 / (16 × 115200) = 10.8507
    //*****3. Write the fractional portion of the BRD to the UARTFBRD register
    UART0->FBRD = 54;//UARTFBRD[DIVFRAC] = integer(0.8507 × 64 + 0.5) = 54
    //*****4. Write the desired serial parameters to the UARTLCRH register
    UART0->LCRH = 0x00000060;
    //*****5. Configure the UART clock source by writing to the UARTCC register
    UART0->CC;
    //*****6. Enable the UART by setting the UARTEN bit in the UARTCTL register.
    UART0->CTL |= UART_CTL_UARTEN;
    while(1)
    {
        __delay_cycles(1000);//NOP
    }

    return 0;
}
