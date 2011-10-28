//#include "regs.h"
#include "soc_types.h"
#include "xldr.h"
#include "regsdigctl.h"
#include "delay.h"


void udelay(int time)
{
	int start;
	start = HW_DIGCTL_MICROSECONDS_RD();
	while(HW_DIGCTL_MICROSECONDS_RD() < (start + time))
	{
		;
	}
}

void mdelay(int time)
{
	while(time--)
	{
		udelay(1000);
	}
}

