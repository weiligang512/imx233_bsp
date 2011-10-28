//#include "regs.h"
#include "xldr.h"
#include "uart.h"
#include "cmd.h"
#include "delay.h"
#include "str.h"
#include "init.h"
#include "setup.h"


int ddrtest(void)
{
		volatile unsigned int * src= (volatile unsigned int *) 0x41f00000;
		volatile unsigned int * dst= (volatile unsigned int *) 0x40000000;
		unsigned int temp;
		unsigned int i;	

		for(i=0 ; i < 0xffffff; i++ )
		{
//			if (i & 0xf)
//				continue;

			//	dst[i] = ~0;
	//		else
	//			dst[i] = 0; 

			dst[i]= 0xffffff - i;
		}
		//verify
		for(i=0 ; i < 0xffffff; i++ )
		{
//			if (i & 0xf)
//				continue;
			if((temp = dst[i]) != 0xffffff - i)
			{
				puts("addr:");
				PrintHex(i);
				puts("should:");
				PrintHex(0xffffff - i);
				puts("bad:");
			//	PrintHex(src[i]);
			//	puts("should:");
				PrintHex(temp);
			//	puts("\r\n");
			}
			//	printf("bad add:%x, val:%x, shouldbe %x\n", &dst[i], temp, src[i%(64*1024)]);		

		}
		return 0;
}
char *help[3] = 
	{
	"h - display the help command",
	"get <address> - use xmodem load binary",
	"go <address> - jump to <address>",
	};

int Main(void)
{
	char *cmd[8];
	int cmdcnt;
	char buff[32];

	init();
	start();
	puts(help[0]);
	init_taglist(0x40000000);
	while(1)
	{
		puts("IMX233# ");
		cmdcnt = cmdline(cmd);
		if (cmdcnt == 1 && strcmp(cmd[0], "h") == 0)
		{
			int i;

			for (i = 0; i < 3; i++)
			{
				puts("\r\n");
				puts(help[i]);
			}
		}
		if (cmdcnt == 2 && strcmp(cmd[0], "get") == 0)
		{	
			cmd_get(cmd[1]);
		}
		if (cmdcnt == 2 && strcmp(cmd[0], "go") == 0)
		{
			cmd_go(cmd[1]);
		}
		if (cmdcnt == 1 && strcmp(cmd[0], "ddr") == 0)
		{
			ddrtest();
		}
		putchr('\n');
		putchr('\r');

	}

	return 0;
}


