#ifndef __UART_H__
#define __UART_H__

#define SOH 0x01
#define STX 0X02
#define EOT 0x04
#define ACK 0x06
#define NAK 0x15
#define CAN 0x18
#define CTRLZ 0x1a


void init_uartdbg(void);

void delay(int time);

unsigned char getchr();

void putchr(unsigned char data);

int puts(const char *s);

char *gets(char *s);

int pow_w(int y, int x);

char *itoa(int integer, char *chr);

int getfile(char *buff);

#endif
