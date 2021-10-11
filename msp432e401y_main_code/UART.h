
#ifndef UART_H_
#define UART_H_
#endif

//UART PA0 33 – U0Rx
//UART PA1 34 – U0Tx


#define buffer_size 20
#define END_Symbol 13//Enter Key

void UART_INIT();
void UARTSendArray(char array_to_send[]);
__interrupt void UART0_IRQHandler(void);


