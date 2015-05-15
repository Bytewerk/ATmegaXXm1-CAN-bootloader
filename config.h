
#define	BOOTLOADER_TYPE			0
#define	BOOT_LED				C,6

// Set ENABLE and !STANDBY pins on CAN transceiver
#define BOOT_INIT \
  do { DDRB  |= (1<<PB3)|(1<<PB4); PORTB |= (1<<PB3)|(1<<PB4); } while (0);

