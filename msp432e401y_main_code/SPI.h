#ifndef SPI_H_
#define SPI_H_
#endif

//#define DEBUGG
#define SPI_CS_OFF (GPIOQ->DATA &= ~BIT1)
#define SPI_CS_ON (GPIOQ->DATA |= BIT1)

void SPI_INIT();
void SPISendArray(char array_to_send);
void SPIsend4bytes(char dataset1, char dataset2, char dataset3, char dataset4);

