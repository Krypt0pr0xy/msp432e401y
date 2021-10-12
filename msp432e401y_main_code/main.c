#include <stdint.h>
#include <stdio.h>
#include "msp.h"

#include "UART.h"
#include "SPI.h"
//#define DEBUGG

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


unsigned long DAC_value = 0;//Only for tests
void DACsendVoltage(double set_voltage)
{
    DAC_value = ((set_voltage + VREF)/(2*VREF)) * 1048575;//Calc DAC value from Voltage value --> in a range from -Vref to +Vref
    DAC_value = DAC_value << 4;//Shift operation to the left with 4 Bits

    uint8_t DAC_value_1 = DAC_value>>16;//Shift operation to the right with 16 Bits
    uint8_t DAC_value_2 = DAC_value>>8;//Shift operation to the right with 8 Bits
    uint8_t DAC_value_3 = DAC_value;

    SPIsend4bytes(0x01,DAC_value_1, DAC_value_2, DAC_value_3);
    __delay_cycles(100);//NOP
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
        cmd_1[cmd_counter] = input_command[counter];//Copy char to command char
        cmd_counter++;
    }

    cmd_counter=0;//Reseting counter for out but array
    for(counter = counter+1; (input_command[counter] != COMMANDCHAR) && (counter <= length); counter++)//Searching to Special Symbol
    {
        cmd_2[cmd_counter] = input_command[counter];//Copy char to command char
        cmd_counter++;
    }

    cmd_counter=0;//Reseting counter for out but array
    for(counter = counter+1; (input_command[counter] != COMMANDCHAR) && (counter <= length); counter++)//Searching to Special Symbol
    {
        cmd_3[cmd_counter] = input_command[counter];//Copy char to command char
        cmd_counter++;
    }

    cmd_counter=0;//Reseting counter for out but array
    for(counter = counter+1; (input_command[counter] != COMMANDCHAR) && (counter <= length); counter++)//Searching to Special Symbol
    {
        cmd_4[cmd_counter] = input_command[counter];//Copy char to command char
        cmd_counter++;
    }

    cmd_counter=0;//Reseting counter for out but array
    for(counter = counter+1; (input_command[counter] != '\r') && (counter <= length); counter++)//Searching to Special Symbol
    {
        cmd_5[cmd_counter] = input_command[counter];//Copy char to command char
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

    SPIsend4bytes(0x04, 0x00, 0x00, 0x40);//Config page 27/54
    SPIsend4bytes(0b00000010, 0b00000000, 0b01000001, 0b00000000);//Config page 27/54

    while(1)
    {
//        unsigned long test = 0;
//        for(test = 0x0007FFFE; test <= 0x00080002; test++)
//        {
//            DAC_value = test;
//            DACsendVoltage(0);
//            __delay_cycles(50000);//NOP
//        }

         double test = 0;
         for(test = -4; test <= 4; test+=0.5)
         {
             DACsendVoltage(test);
             __delay_cycles(50000);//NOP


         }
         __delay_cycles(500);//NOP

    }
}
