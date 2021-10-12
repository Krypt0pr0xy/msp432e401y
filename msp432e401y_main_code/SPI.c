#include <stdint.h>
#include <stdio.h>
#include <msp.h>
#include "SPI.h"
//####################################################################################################################
void SPI_INIT()
{
    //Quad Synchronous Serial Interface (QSSI)
        //Slau 723 Page 1534 / 1537
        //Header    Pin StandardFunction    GPIO    MCU Pin     Digital Function (FPIOPCTL Bit Encoding) + Function
        //J5        7   SPI CLK             PQ0     5           SSI3Clk + BIT14
        //J6        16  SPI MOSI            PQ2     11          SSI3XDAT0 + BIT14
        //J6        17  SPI MISO            PQ3     27          SSI3XDAT1 + BIT14
        //J6        19  CSz                 PQ1     6           SSI3Fss + BIT14
        //J5        2   LDAC                PD2     3
        //Initialization and Configuration
        //To enable and initialize the QSSI, the following steps are necessary:
        //1. Enable the QSSI module using the RCGCSSI register (see Section 4.2.92)
        SYSCTL->RCGCSSI |= BIT3; //0x1 = Enable and provide a clock to SSI module 3 in run mode
        __delay_cycles(10);//NOP
        //2. Enable the clock to the appropriate GPIO module through the RCGCGPIO register (see Section 4.2.87). To find out which GPIO port to enable, see the device-specific data sheet
        //GPIO Port Q Run Mode Clock Gating Control
        SYSCTL->RCGCGPIO |= 0x4000;//0x1 = Enable and provide a clock to GPIO port Q in run mode. 0x4000 = Bit 14 Port Q
        __delay_cycles(10);//NOP
        //SYSCTL->RCGCGPIO |= BIT3;//0x1 = Enable and provide a clock to GPIO port D in run mode. Port D
        __delay_cycles(10);//NOP
        //3. Set the GPIO AFSEL bits for the appropriate pins (see Section 17.5.10). To determine which GPIOs to configure, see the device-specific data sheet.
        GPIOQ->AFSEL |= BIT0;//0x1 = The associated pin functions as a peripheral signal and is controlled by the alternate hardware function.
        GPIOQ->AFSEL&= ~BIT1;//0x1 = The associated pin functions as a peripheral signal and is controlled by the alternate hardware function.
        GPIOQ->AFSEL |= BIT2;//0x1 = The associated pin functions as a peripheral signal and is controlled by the alternate hardware function.
        GPIOQ->AFSEL |= BIT3;//0x1 = The associated pin functions as a peripheral signal and is controlled by the alternate hardware function.
        //4. Configure the PMCn fields in the GPIOPCTL register to assign the QSSI signals to the appropriate pins. See Section 17.5.22 and the device-specific data sheet.
        //GPIO Pin Multiplexing page 30/157
        GPIOQ->PCTL |= 0xE;//SSI3Clk BIT14 Port0
        //GPIOQ->PCTL |= 0xE0;//SSI3Fss BIT14 Port1
        GPIOQ->PCTL |= 0xE00;//SSI3XDAT0 BIT14 Port2
        GPIOQ->PCTL |= 0xE000;//SSI3XDAT1 BIT14 Port3
        //5. Program the GPIODEN register to enable the pin's digital function. In addition, the drive strength, drain select and pullup and pulldown functions must be configured. See Chapter 17 for more information.
        //set as Digital Pin
        GPIOQ->DEN |= BIT0;//Port PQ0
        GPIOQ->DEN |= BIT1;//Port PQ1
        GPIOQ->DEN |= BIT2;//Port PQ2

        //For each of the frame formats, the QSSI is configured using the following steps:
        /*
        1. If initializing out of reset, ensure that the SSE bit in the SSICR1 register is clear before making any
        configuration changes. Otherwise, configuration changes for advanced SSI can be made while the
        SSE bit is set.
        */
        SSI3->CR1&= ~SSI_CR1_SSE;
        //2. Select whether the QSSI is a master or slave:
        SSI3->CR1 = 0x00000000;//For master operations, set the SSICR1 register to 0x0000.0000.
        //3. Configure the QSSI clock source by writing to the SSICC register.
        SSI3->CC = 0x0;//0x0 = System clock (based on clock source and divisor factor programmed in RSCLKCFG register in the System Control Module)
        //SCR = 1 => SSInClk = SysClk / (CPSDVSR * (1 + SCR)) = 500kHz
        //4. Configure the clock prescale divisor by writing the SSICPSR register.
        SSI3->CR0 |= BIT8;
        //5. Write the SSICR0 register with the following configuration:
        //Serial clock rate (SCR)
        SSI3->CPSR = 16;
        //QSSI Serial Clock Phase
        SSI3->CR0 |= BIT7;//0x1 = Data is captured on the second clock edge transition.
        //QSSI Serial Clock Polarity
        SSI3->CR0 |= BIT6;//0x1 = A steady state high value is placed on the SSInClk pin when data is not being transferred // 0x0 = A steady state low value is placed on the SSInClk pin.
        //SSI Frame Format Select. When operating in Advanced, Bi-, or Quad-SSI mode these bits must be 0x0.
        SSI3->CR0 &= ~BIT4;//FRF Freescale SPI Frame Format
        SSI3->CR0 &= ~BIT5;
        //DDS QSSI Data Size Select
        SSI3->CR0 |= BIT0;
        SSI3->CR0 |= BIT1;
        SSI3->CR0 |= BIT2;//0x7 = 8-bit data

        //Eanble SSE

        SSI3->CR1 |= SSI_CR1_SSE;
}
//####################################################################################################################
void SPISendArray(char array_to_send)//send char array
{
    while(SSI3->SR & SSI_SR_BSY);
    //__delay_cycles(1000);//NOP
    SSI3->DR = array_to_send;//Take array at specific place
}

void SPIsend4bytes(char dataset1, char dataset2, char dataset3, char dataset4)
{
    SPI_CS_OFF;
    SPISendArray(dataset1);
    //Bit23-16
    SPISendArray(dataset2);
    //Bit15-8
    SPISendArray(dataset3);
    //Bit7-0
    SPISendArray(dataset4);
    __delay_cycles(400);//NOP //delay for CS in the SPI communication value measuerd with locig analyser
    SPI_CS_ON;
}
//####################################################################################################################
