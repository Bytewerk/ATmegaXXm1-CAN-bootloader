
#ifndef UARTX_H_
#define UARTX_H_

#include <stdint.h>

int uart_init(void);
int uart_send_byte(uint8_t c);
int uart_write(void *string, uint8_t len);
int uart_puts(char *string);


#endif // UARTX_H_


