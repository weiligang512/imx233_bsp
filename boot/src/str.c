#include "uart.h"
#include "str.h"
#include "types.h"



char *strcpy(char *dest, const char *src)
{
	char *p = dest;

	while(*src)
	{
		*p++ = *src++;
	}

	return dest;
}

void *memcpy(void *dest, const void *src, int n)
{
	char *p = (char *)dest;
	char *q = (char *)src;

	while(n > 0)
	{
		*p++ = *q++;
		n--;
	}
	
	return dest;
}

int strcmp(const char *s1, const char *s2)
{
	while(*s1 && *s2)
	{
		if (*s1 != *s2)
			return *s1 -*s2;
		else
		{
			s1++;
			s2++;
		}
	}

	return 0;
}

int strlen(const char *s)
{
	const char *p = s;

	while(*p)
	{
		p++;
	}

	return p - s;
}

int pow_w(int y, int x)
{
	int value = 1, i;

	if (x == 0)
		return 1;
	for (i = 1; i <= x; i++)
	{
		value *= y;
//		if (value < 0)
//			return -1;
	}

	return value;
}

int atoi(const char *nptr)
{
	int len, i, sum = 0;

	len = strlen(nptr);

	for (i = len; i > 0; i--)
	{
		if (nptr[i-1] < '0' && nptr[i-1] > '9')
		{
			return 0;
		}
		else
		{
			sum += (nptr[i-1]-'0') * pow_w(10, len - i);
		}
	}

	return sum;
}

int atox(const char *nptr)
{
	int len, i, sum = 0;

	len = strlen(nptr);

	for (i = len; i > 0; i--)
	{
		if (nptr[i-1] >= '0' && nptr[i-1] <= '9')
		{
			sum += (nptr[i-1]-'0') * pow_w(16, len - i);
		}
		else if (nptr[i-1] >= 'a' && nptr[i-1] <= 'f')
		{
			sum += (nptr[i-1]-'a' + 10) * pow_w(16, len - i);
		}
		else if (nptr[i-1] >= 'A' && nptr[i-1] <= 'F')
		{
			sum += (nptr[i-1]-'A' + 10) * pow_w(16, len - i);
		}
		else
		{
			return 0;
		}
	}

	return sum;
}

int atoi_x(const char *nptr)
{
	if (nptr[0] == '0' && (nptr[1] == 'x' | nptr[1] == 'X'))
	{
		return atox(&nptr[2]);
	}
	else
	{
		return atoi(nptr);
	}

	return 0;
}

char* itox( unsigned int num )
{
	static char hex[12] = "0x";
	int i, pos;
	const char lut[]="0123456789ABCDEF";
	
	for(i=0, pos=2; i<8; i++)
	{
		if( (hex[pos] = lut[ (num<<4*i)>>28 ]) != '0' ||  pos != 2 )
			pos++;
		hex[pos+1]='\0';
	}	

	return hex;
}

char *itoa(int integer, char *chr)
{
	int i, j, tem, v;

	tem = integer;

	for (i = 0; tem /= 10; i++)
	{
		;
	}

	for (j = 0; i > 0; i--, j++)
	{
		v = pow_w(10, i);
//		chr[j] = (integer / v) + ('0' - 0);
//		integer %= v;
	}
	chr[j] = '\0';

	return chr;
}


