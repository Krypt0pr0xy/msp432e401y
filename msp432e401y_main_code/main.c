#include <stdint.h>
#include <stdio.h>
#include "msp.h"

#include "UART.h"
//#define DEBUGG
#define SPI_CS_OFF (GPIOQ->DATA &= ~BIT1)
#define SPI_CS_ON (GPIOQ->DATA |= BIT1)

#define LDAC_OFF (GPIOD->DATA &= ~BIT2)
#define LDAC_ON (GPIOD->DATA |= BIT2)
#define VREF 5.0


//#define buffer_size 20
//#define END_Symbol 13//Enter Key

//char buffer_RX[buffer_size];
//unsigned int index_buffer_RX = 0;

#define buflen_cmd 20//!!DO not change them unless you discussed it with the creator of this document!!
#define COMMANDCHAR '_'//Separator
char cmd_1[buflen_cmd] = "";//command 1 buffer
char cmd_2[buflen_cmd] = "";//command 2 buffer
char cmd_3[buflen_cmd] = "";//command 3 buffer
char cmd_4[buflen_cmd] = "";//command 4 buffer
char cmd_5[buflen_cmd] = "";//command 5 buffer


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
    __delay_cycles(1000);//NOP
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
    __delay_cycles(500);//NOP //delay for CS in the SPI communication value measuerd with locig analyser
    SPI_CS_ON;
}
//####################################################################################################################
unsigned long DAC_value = 0;//Only for tests
void DACsendVoltage(double set_voltage)
{
    //DAC_value = ((set_voltage + VREF)/VREF) * 524288;//Calc DAC value from Voltage value --> in a range from -Vref to +Vref
    DAC_value = DAC_value << 4;//Shift operation to the left with 4 Bits

    uint8_t DAC_value_1 = DAC_value>>16;//Shift operation to the right with 16 Bits
    uint8_t DAC_value_2 = DAC_value>>8;//Shift operation to the right with 8 Bits
    uint8_t DAC_value_3 = DAC_value;

    SPIsend4bytes(0x01,DAC_value_1, DAC_value_2, DAC_value_3);
    LDAC_OFF;// Set the load DAC for a short moment low
    __delay_cycles(500);//NOP
    LDAC_ON;
}

double get_Voltage(char value[buflen_cmd])
{
    //_____________________________________________________________________________
        //Extracting the Voltage
        char input[buflen_cmd] = "";//input buffer
        strcpy(input, value);//copy it from value
        input[19] = '\0';//securing the last buffer as end symbol
        int firstdigit = 0;
        unsigned long long commavalue = 0;
        unsigned long long Divider = 1E18;
        firstdigit = atoi(strtok(value, ','));//take the first digit only
        if(firstdigit < 0){firstdigit *= (-1);}//be sure it is positiv
        char offset = 0;
        if(input[0] == '-')//check if first buffer has an minus symbol
        {
            offset = 1;
            input[2] = 0;//clearing buffer
        }
        input[0] = 0;//clearing buffer
        input[1] = 0;//clearing buffer

        char counter = 0;
        for(counter = (2+offset); counter <=19; counter++)//counting through the array
        {
            char temp_buf[2] = {input[counter], '\0' };//filling an temporary buffer ech time
            commavalue+=(atoi(temp_buf)*Divider);//

            Divider /= 10;//setting divider one place down
        }

        double vout = 0.0;

        vout+=firstdigit;//adding the first digit to Vout
        vout+=(commavalue/1E19);//adding the comma value to Vout

        if(offset > 0){vout *= (-1);}//multiplying with -1 when the value should be negativ
        return vout;
    //_____________________________________________________________________________________________
}

//#########################################################################################################################################################
//____________________________________________Command Decoder SET_CH1_2.5_OUT10_Low
void CommandDecoder(char input_command[])
{
    unsigned char clearCounter=0;
    //Clearing all Char arrays
    for(clearCounter = 0; clearCounter <=19;clearCounter++)
    {
        cmd_1[clearCounter] = '\0';
        cmd_2[clearCounter] = '\0';
        cmd_3[clearCounter] = '\0';
        cmd_4[clearCounter] = '\0';
        cmd_5[clearCounter] = '\0';
    }
    unsigned char length = strlen(input_command);
    unsigned char counter = 0;
    unsigned char cmd_counter = 0;

    //SET=CH1=4.5=OUT10_Low

    cmd_counter=0;//Reseting counter for out but array
    for(counter = 0; (input_command[counter] != COMMANDCHAR) && (counter <= length); counter++)//Searching to Special Symbol
    {
        cmd_1[cmd_counter] = input_command[counter];//Copy char to comand char
        cmd_counter++;
    }

    cmd_counter=0;//Reseting counter for out but array
    for(counter = counter+1; (input_command[counter] != COMMANDCHAR) && (counter <= length); counter++)//Searching to Special Symbol
    {
        cmd_2[cmd_counter] = input_command[counter];//Copy char to comand char
        cmd_counter++;
    }

    cmd_counter=0;//Reseting counter for out but array
    for(counter = counter+1; (input_command[counter] != COMMANDCHAR) && (counter <= length); counter++)//Searching to Special Symbol
    {
        cmd_3[cmd_counter] = input_command[counter];//Copy char to comand char
        cmd_counter++;
    }

    cmd_counter=0;//Reseting counter for out but array
    for(counter = counter+1; (input_command[counter] != COMMANDCHAR) && (counter <= length); counter++)//Searching to Special Symbol
    {
        cmd_4[cmd_counter] = input_command[counter];//Copy char to comand char
        cmd_counter++;
    }

    cmd_counter=0;//Reseting counter for out but array
    for(counter = counter+1; (input_command[counter] != '\r') && (counter <= length); counter++)//Searching to Special Symbol
    {
        cmd_5[cmd_counter] = input_command[counter];//Copy char to comand char
        cmd_counter++;
    }

    //ending the char array with a end Symbol
    cmd_1[strlen(cmd_1)+1] = '\0';
    cmd_2[strlen(cmd_2)+1] = '\0';
    cmd_3[strlen(cmd_3)+1] = '\0';
    cmd_4[strlen(cmd_4)+1] = '\0';
    cmd_5[strlen(cmd_5)+1] = '\0';
    //Sending the Information over UART back
    //Check the SET Command and when true start command_set function
    if(strcmp(cmd_1, "SET\0") == 0)
    {
        DACsendVoltage(get_Voltage(cmd_3));
        UARTSendArray("Set Voltage to ");
        UARTSendArray(cmd_3);
        UARTSendArray("\r\n");
    }
}


int main(void)
{
    SPI_INIT();
    UART_INIT();
    SYSCTL->RCGCGPIO |= BIT3;//0x1 = Enable and provide a clock to GPIO port D in run mode. Port D
    GPIOD->DEN |= BIT2;
    //Config DAC
    //Config1 page 31/54
    //BIT32-24
    GPIOQ->DIR |= BIT1;//set pin as output SPI_CS
    GPIOD->DIR |= BIT2;//set pin as output SPI_LDAC
    __delay_cycles(1000000);//NOP

    SPI_CS_ON;//set CS Pin High


    SPIsend4bytes(0b00000010, 0b00000000, 0b00000001, 0b00000000);//Config page 27/54

    while(1)
    {
//        GPIOQ->DATA &= ~BIT1;
//        SPISendArray(0x01);
//        SPISendArray(0x00);
//        SPISendArray(0xFF);
//        SPISendArray(0xFF & ~15);
//        __delay_cycles(500);//NOP
//        GPIOQ->DATA |= BIT1;
//        __delay_cycles(500);//NOP
//
//        GPIOD->DATA &= ~BIT2;
//        __delay_cycles(500);//NOP
//        GPIOD->DATA |= BIT2;

//        unsigned long test = 0;
//        for(test = 524285; test <= 524300; test++)
//        {
//            DAC_value = test;
//            DACsendVoltage(0);
//            __delay_cycles(50000);//NOP
//        }

         __delay_cycles(500);//NOP

    }
}
