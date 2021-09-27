
//UART TEST
#include <stdint.h>
#include "msp.h"


void UARTSendArray(char array_to_send[])//send char array
{
    unsigned int array_length = strlen(array_to_send);//size of char
    unsigned int counter = 0;
    for(counter = 0; counter <= array_length; counter++)//for loop with length
    {
        while(UART0->FR & UART_FR_TXFF);
        __delay_cycles(1000);//NOP
        UART0->DR = array_to_send[counter];//Take array at specific place

    }

}



int main(void)
{


    //INIT UART
    //Universal Asynchronous Receiver/Transmitter (UART)
    //Slau 723a page 1620

    //UART PA0 33 � U0Rx
    //UART PA1 34 � U0Tx

    //UART Initialization and Configuration
    //26.4 page 1629
    //-----------------------------------------------------------------------
    //******1. Enable the UART module page 331
    // UART Module 0 Run Mode Clock Gating Control
    SYSCTL->RCGCUART |= BIT0;//0x1 = Enable and provide a clock to UART module 0 in run mode
    __delay_cycles(10);//NOP

    //******2. Enable the clock to the appropriate GPIO module page 326
    //GPIO Port A Run Mode Clock Gating Control
    SYSCTL->RCGCGPIO |= BIT0;//0x1 = Enable and provide a clock to GPIO port A in run mode.
    __delay_cycles(10);//NOP


    //= The corresponding GPIOAFSEL, GPIOPUR, GPIOPDR, or GPIODEN bits can be written.
    GPIOA->CR |= 0x03;
    //GPIOA->CR |= BIT1;

    //******3.Set the GPIO AFSEL bits for the appropriate pins page 1213
    //GPIO Alternate Function Select page 1199
    //GPIOPCTL is shown in Figure 17-26 and described in Table 17-33. page 1230 Port Mux
    GPIOA->AFSEL |= BIT0;//0x1 = The associated pin functions as a peripheral signal and is controlled by the alternate hardware function.
    GPIOA->AFSEL |= BIT1;//0x1 = The associated pin functions as a peripheral signal and is controlled by the alternate hardware function.

    GPIOA->PC |= BIT0;//0x1 = GPIO port A is powered but does not receive a clock. In this case, the module is inactive.
    //******4.Configure the GPIO current level or slew rate as specified for the mode selected


    //set as Digital Pin
    GPIOA->DEN |= BIT0;
    GPIOA->DEN |= BIT1;

    //Pullup Select
    GPIOA->PUR |= BIT0;
    GPIOA->PUR |= BIT1;

    //GPIO Slew Rate Control Select
    GPIOA->SLR |= BIT0;//0x1 = Slew rate control is enabled for the corresponding pin.
    GPIOA->SLR |= BIT1;//0x1 = Slew rate control is enabled for the corresponding pin.

    //GPIO 8-mA Drive Select
    GPIOA->DR8R |= BIT0;//The corresponding GPIO pin has 8-mA drive
    GPIOA->DR8R |= BIT1;//The corresponding GPIO pin has 8-mA drive

    //*****5.Configure the PMCn fields in the GPIOPCTL register to assign the UART signals to the appropriate pins
    GPIOA->PCTL |= BIT0;//GPIO Pin Multiplexing page 30/157
    GPIOA->PCTL |= BIT4;//GPIO Pin Multiplexing page 30/157
    //UART configuration page 1630
    //*****1. Disable the UART by clearing the UARTEN bit in the UARTCTL register
    UART0->CTL &= ~UART_CTL_UARTEN;
    //*****2. Write the integer portion of the BRD to the UARTIBRD register.
    UART0->IBRD = 8.680555;//BRD = 16000000 / (16 � 115200) = 8.680555
    //*****3. Write the fractional portion of the BRD to the UARTFBRD register
    UART0->FBRD = 54;//UARTFBRD[DIVFRAC] = integer(0.8507 � 64 + 0.5) = 54
    //*****4. Write the desired serial parameters to the UARTLCRH register
    UART0->LCRH = 0x00000060;
    //*****5. Configure the UART clock source by writing to the UARTCC register
    UART0->CC = 0x0;//page 1659
    //*****6. Enable the UART by setting the UARTEN bit in the UARTCTL register.


    UART0->CTL &= ~UART_CTL_LBE;//UART Loop Back Disable
    UART0->CTL &= ~UART_CTL_SIREN;//0x0 = Normal operation

    UART0->LCRH |= UART_LCRH_FEN;//UART  FIFOs
    UART0->LCRH |= 0x00110000;//UART Word Length 8Bits page 1640


    UART0->CTL |= UART_CTL_RXE;//UART Receive Enable
    //UART0->CTL &= ~UART_CTL_RXE;//UART Receive Disable

    UART0->CTL |= UART_CTL_TXE;//UART Transmit Enable
    //UART0->CTL &= ~UART_CTL_TXE;//UART Transmit Disable

    UART0->PP &= ~UART_PP_MSE;//0x0 = The UART module does not provide extended support for modem control.
    UART0->PP &= ~UART_PP_MS;//0x0 = The UART module does not provide support for modem control.
    UART0->PP &= ~UART_PP_SC;//0x0 = The UART module does not provide smart card support.

    UART0->CTL &= ~UART_CTL_CTSEN;//0x0 = CTS hardware flow control is disabled.
    UART0->CTL &= ~UART_CTL_RTSEN;//0x0 = RTS hardware flow control is disabled
    UART0->CTL &= ~UART_CTL_SMART;//0x0 = Normal operation.
    UART0->CTL |= UART_CTL_UARTEN; //0x1 = The UART is enabled.


    //Interrupt Setting

    UART0->IM |= UART_IM_RXIM; //An interrupt is sent to the interrupt controller when the RXRIS bit in the UARTRIS register is set.

    _enable_interrupts();

#define buffer_size 16
char test[buffer_size] = 0;
unsigned int index = 0;

UARTSendArray("Startup Finished \r\n");
    while(1)
    {

        __delay_cycles(100000);//NOP

        while(UART0->FR & UART_FR_RXFF);

        index = 0;
        do
        {
            test[index] = UART0->DR;
            index++;
        }while(((index <= (buffer_size-1)) && (test[index-1] != 0x00)));

        __delay_cycles(100000);//NOP
        UARTSendArray("You Send:  ");
        UARTSendArray(test);
        UARTSendArray("\r\n");
        for(index = 0; index <= buffer_size-1; index++)
        {
            test[index] = 0x00;
        }

    }

}


__interrupt void EUSCIA0_IRQHandler(void)
{
    __delay_cycles(100000);//NOP
}

