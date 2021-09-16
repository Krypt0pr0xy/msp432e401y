
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

    //GPIO 12-mA Drive Select
    GPIOA->DR12R |= BIT0;//The corresponding GPIO pin has 12-mA drive
    GPIOA->DR12R |= BIT1;//The corresponding GPIO pin has 12-mA drive


    //*****5.Configure the PMCn fields in the GPIOPCTL register to assign the UART signals to the appropriate pins


    while(1)
    {
        __delay_cycles(1000);//NOP
    }

    return 0;
}
