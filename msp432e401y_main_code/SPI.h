#ifndef SPI_H_
#define SPI_H_
#endif

//        Header    Pin StandardFunction    GPIO    MCU Pin     Digital Function (FPIOPCTL Bit Encoding) + Function
//        J5        7   SPI CLK             PQ0     5           SSI3Clk + BIT14
//        J6        16  SPI MOSI            PQ2     11          SSI3XDAT0 + BIT14
//        J6        17  SPI MISO            PQ3     27          SSI3XDAT1 + BIT14
//        J6        19  CSz                 PQ1     6           SSI3Fss + BIT14
//        J5        2   LDAC                PD2     3
//#define DEBUGG
#define SPI_CS_OFF (GPIOQ->DATA &= ~BIT1)
#define SPI_CS_ON (GPIOQ->DATA |= BIT1)

void SPI_INIT();
void SPISendArray(char array_to_send);
void SPIsend4bytes(char dataset1, char dataset2, char dataset3, char dataset4);

