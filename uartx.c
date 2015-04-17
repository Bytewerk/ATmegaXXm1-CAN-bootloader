
#include <stdint.h>
#include <avr/io.h>
#include <string.h>

int uart_init(void)
{

    // F_CPU == 16MHz!
    // Baud Rate = 19200
    // --> LINBRR == 833.3333333 / 32 samples
		// --> LINBRR == 26
    LINBRR = 26;

    LINCR =  (1<<LENA); // enable UART
    LINCR |= (1<<LCMD2)|(1<<LCMD0); // enable UART TX function

    return 1;
}


int uart_send_byte(uint8_t c)
{
    do { /* nothing */ } while (LINSIR & (1<<LBUSY)); //busy waiting until ready
    LINDAT = c;

    return 1;
}


int uart_write(void *string, uint8_t len)
{
    uint8_t *data = (uint8_t*)string;
    uint8_t l = len;

    while(l--) {
	if ( uart_send_byte(*data++) < 0 ) {
	    return -1;
	}
    }

    return len;
}


int uart_puts(char *string)
{
    return uart_write(string, strlen(string));
}


