#include "regs.h"
//#include "regspinctrl.h"
#include "delay.h"
#include "init.h"
//#include "dm9000x.h"

#if 0
static void power_init(void);
static void pin_init(void);
static void cpu_init(void);
static void gpmi_reset(void);
static void gpmi_init(void);
static void dma_init(void);
#endif

#if 0
void init_io(void)
{
	HW_PINCTRL_MUXSEL0_CLR = 0xffff; //gpmi: d0-d7
	HW_PINCTRL_MUXSEL1_CLR = 0xf03cf;//gpmi: cle, ale, rb0, rb1, wpn(unused), wrn, rdn
	HW_PINCTRL_MUXSEL5_CLR = 0x3c00000; //gpmi: ce1, ce0
//	HW_PINCTRL_PULL0_SET = 0xff;
}

void gpmi_ini(void)
{
	HW_CLKCTRL_GPMI = 0X01; //on clock
	HW_GPMI_CTRL0_CLR = (1<<30); 
	udelay(1000);
	HW_GPMI_CTRL0_CLR = (1<<31);
	udelay(100);
	
	HW_APBH_CTRL0_CLR = (1<<30);
	udelay(1000);
	HW_APBH_CTRL0_CLR = (1<<31);
	udelay(100);
//	HW_GPMI_TIMEOUT = 0X00FF0000;
}
void init_uartdbg(void)
{
	int divisor;

	HW_PINCTRL_MUXSEL3_CLR = 0Xf00000; //choice GPIO uart
	HW_PINCTRL_MUXSEL3_SET = 0Xa00000; //choice GPIO uart
	divisor = (24 * 1000 * 1000 *4) / 115200; //baut rate 115200
	HW_UARTDBGLCR_H = 0X60;
	HW_UARTDBGFBRD = divisor & 0x3f;
	HW_UARTDBGIBRD = (divisor >> 6) & 0xffff;
	HW_UARTDBGCR |= 0X301;
}
#endif

void init(void)
{
	power_init();
	cpu_init();
//	init_uartdbg();
	pin_init();
//	init_io();
//	gpmi_ini();
//	dm9000_initialize();
//	dma_init();
//	gpmi_init();
}

void power_init(void)
{
	unsigned int val;
	//gate on power 
	HW_POWER_CTRL_CLR = HW_POWER_CTRL_CLKGATE;
	val = HW_POWER_VDDDCTRL;
	val &= HW_POWER_VDDDCTRL_TRGMASK;
	val |= (26 & HW_POWER_VDDDCTRL_TRGMASK);
	//set voltage to 26 * 0.025 + 0.8 = 1.45V, default 1.2V
	HW_POWER_VDDDCTRL |= val;
	
//	HW_POWER_VDDMEMCTRL 

}

void pin_init(void)
{
	HW_PINCTRL_MUXSEL1_SET = (3<<14); //select bank0, 23bit to GPIO
	HW_PINCTRL_MUXSEL3_SET = (3<<6);  //select bank1, 19bit to GPIO
	HW_PINCTRL_MUXSEL3_SET = (3<<24); //select bank1, 28bit to GPIO
	HW_PINCTRL_DOE0_CLR = (1<<23); //set bank0, 23bit input
	HW_PINCTRL_DOUT1_SET = (1<<19); //set bank1, 19bit output
	HW_PINCTRL_DOUT1_SET = (1<<28); //set bank0, 28bit outut
	HW_PINCTRL_DOE1_SET = (1<<19); //set bank1, 19bit output
	HW_PINCTRL_DOE1_SET = (1<<28); //set bank0, 28bit outut
	
//	HW_PINCTRL_IRQLEVEL0_CLR = (1<<23); //set bank0, bit23, level edge 
//	HW_PINCTRL_IRQPOL0_CLR = (1<<23); //set bank0, bit23, pol low
//	HW_PINCTRL_IRQSTAT0_CLR = (1<<23); 
//	HW_PINCTRL_PIN2IRQ0_SET = (1<<23); 
//	HW_PINCTRL_IRQEN0_SET = (1<<23); 
}
void cpu_init(void)
{
	u32 cpu_stable;

	//power on PLL
	HW_CLKCTRL_PLLCTRL0_SET = HW_CLKCTRL_PLLCTRL0_POWER;
	udelay(160);
	//set cpu clock to 454MHz = 480 * (18 / 19)
	HW_CLKCTRL_FRAC_CLR = HW_CLKCTRL_FRAC_DIV_MASKCPU;
	HW_CLKCTRL_FRAC_SET = 19 & HW_CLKCTRL_FRAC_DIV_MASKCPU; 
	HW_CLKCTRL_FRAC_CLR = HW_CLKCTRL_FRAC_CLKGATECPU;

	//check cpu clock stable
	cpu_stable = HW_CLKCTRL_FRAC & HW_CLKCTRL_FRAC_CPUSTABLE;
	while((HW_CLKCTRL_FRAC ^ cpu_stable) == 0);
	HW_CLKCTRL_CPU_SET = 0X01 & HW_CLKCTRL_CPU_DIV_MASK;
//	udelay(100);

	//set hbus clock to 454 / 3 = 151MHz
	HW_CLKCTRL_HBUS_SET = HW_CLKCTRL_HBUS_DIV_MASK;
	HW_CLKCTRL_HBUS_CLR = 0X1c;
	//chose PLL
	HW_CLKCTRL_CLKSEQ_CLR = HW_CLKCTRL_CLKSEQ_BYPASS_CPU;
}

#if 0
void gpmi_reset(void)
{
	HW_GPMI_CTRL0_CLR = HW_GPMI_CTRL0_STFRST;
	udelay(5000);

	HW_GPMI_CTRL0_CLR = HW_GPMI_CTRL0_CLKGATE;
	HW_GPMI_CTRL0_SET = HW_GPMI_CTRL0_STFRST;
	udelay(5000);
	
	HW_GPMI_CTRL0_CLR = HW_GPMI_CTRL0_STFRST;
	udelay(5000);

	HW_GPMI_CTRL0_CLR = HW_GPMI_CTRL0_CLKGATE;
	udelay(5000);
}

void gpmi_init(void)
{
	u32 io_stable;

	HW_PINCTRL_MUXSEL0_CLR = 0xffff; //gpmi: d0-d7
	HW_PINCTRL_MUXSEL1_CLR = 0xf03cf;//gpmi: cle, ale, rb0, rb1, wpn(unused), wrn, rdn
	HW_PINCTRL_MUXSEL5_CLR = 0x3c00000; //gpmi: ce1, ce0
//	HW_PINCTRL_PULL0_SET = 0xff;
	
	//set clockio to 454MHz = 480 * (18 / 19)
	HW_CLKCTRL_FRAC_CLR = HW_CLKCTRL_FRAC_DIV_MASKIO;
	HW_CLKCTRL_FRAC_SET = (19<<24) & HW_CLKCTRL_FRAC_DIV_MASKIO; 
	HW_CLKCTRL_FRAC_CLR = HW_CLKCTRL_FRAC_CLKGATEIO;

	//check clockio stable
	io_stable = HW_CLKCTRL_FRAC & HW_CLKCTRL_FRAC_GPMISTABLE;
	while((HW_CLKCTRL_FRAC ^ io_stable) == 0);
//	udelay(1000);
	//gate on clk_gpmi
	HW_CLKCTRL_GPMI &= ~HW_CLKCTRL_GPMI_CLKGATE;
	//set clk_gpmi to ref_io / 5 = 454 / 5 = 90MHz
	HW_CLKCTRL_GPMI |= (5) & HW_CLKCTRL_GPMI_DIV_MASK;
	HW_CLKCTRL_GPMI |= HW_CLKCTRL_GPMI_CLKGATE;
	
	gpmi_reset();

	//chose gpmi clock
//	HW_CLKCTRL_CLKSEQ_CLR = HW_CLKCTRL_CLKSEQ_BYPASS_GPMI; 
}

void dma_init(void)
{
	HW_APBH_CTRL0_CLR = HW_APBH_CTRL0_SFTRST;
	HW_APBH_CTRL0_CLR = HW_APBH_CTRL0_CLKGATE;
	udelay(100);
	HW_APBH_CTRL0_SET = HW_APBH_CTRL0_SFTRST;
	udelay(100);
	HW_APBH_CTRL0_CLR = HW_APBH_CTRL0_SFTRST;
}

#endif
