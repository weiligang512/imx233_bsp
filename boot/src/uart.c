//#include "regs.h"
#include "soc_types.h"
#include "xldr.h"
#include "regsuartdbg.h"
#include "uart.h"



unsigned char getchr()
{
	unsigned char data = 0;

	while ((HW_UARTDBGFR_RD() & (1<<4)));
	data = HW_UARTDBGDR_RD() & 0Xff;

	return data;
}

void putchr(unsigned char data)
{
	while ((HW_UARTDBGFR_RD() & (1<<5)));
	
	HW_UARTDBGDR_RD() = (unsigned int)data;
}

int puts(const char *s)
{
	const char *p;
	
	p = s;

	while(*p)
	{
		putchr(*p);
		p++;
	}
//	putchr('\n');
//	putchr('\r');

	return p-s;
}

char *gets(char *s)
{
	char *p;

	p = s;

	while(1)
	{
		*p = getchr();
		if (*p == '\b')
		{
			if (p > s)
			{
				putchr('\b');
				putchr(' ');
				putchr('\b');
				p--;
			}
			continue;
		}
		else if (*p != '\r')
		{
			putchr(*p);
		}
		if (*p == '\r')
			break;
		p++;
	}
	*p = '\0';

	return s;
}

int getfile(char *buff)
{
	int i, count = 0;
	char num, cmpnum, data, check, sum = 0;

	putchr(NAK);
	while((data = getchr()) != EOT)
	{
		if (data != SOH)
			continue;
		num = getchr();
		cmpnum = getchr();
		for (i = 0; i < 128; i++)
		{
			*buff++ = getchr();
			sum += buff[-1];
		}
		check = getchr();
		putchr(ACK);
		count++;
#if 0
		if ((sum & 0xff) == check)
		{
			putchr(ACK);
			count++;
		}
		else
		{
			putchr(NAK);
			buff -= 128;
		}
#endif
	}
	putchr(ACK);

	return count;
}
