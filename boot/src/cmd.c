#include "cmd.h"
#include "uart.h"
#include "setup.h"

char cmdbuff[32];

int cmdline(char **argv)
{

	gets(cmdbuff);

	return cmd(cmdbuff, argv);
}



int cmd(char *cmd, char **argv)
{
	int num = 0;

	while(1)
	{
		if(*cmd == ' ')
			while(*++cmd == ' ');
		if(*cmd == '\0')
			break;
		*argv++ = cmd;

		while(*cmd++ != ' ')
			if(*cmd == '\0')
			{
				num++;
				break;
			}

		if(*cmd != '\0')
		{
			cmd[-1] = '\0';
			num++;
		}
	}

	return num;
}

void cmd_get(char *cmd)
{
	int i;

	puts("\r\n");
	puts("please use xmodem");
	for (i = 0; i < 20; i++)
	{
		putchr('.');
		mdelay(1000);
	}
	getfile((char *)atoi_x(cmd));
	
	//getfile((char *)0x6000);
}

void cmd_go(char *cmd)
{
	unsigned int p = (unsigned int)0x40000000;
	void (*go)(int zero, int arch, unsigned int ) = (void (*)(int, int, unsigned int))atoi_x(cmd);
	go(0, 0xa45, p);
}


