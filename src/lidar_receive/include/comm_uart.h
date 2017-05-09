/*
 * comm_uart.h
 *
 *  Created on: 23 Apr 2017
 *      Author: Ryan
 *
 */

#ifndef COMM_UART_H_
#define COMM_UART_H_

#ifdef __cplusplus
extern "C" {
#endif

// Functions
void comm_uart_init(char* modemDevice);
void comm_uart_close(void);
int receive_packet(void);

#ifdef __cplusplus
}
#endif
#endif /* COMM_UART_H_ */
