#ifndef __GPMI_H__
#define __GPMI_H__

#include "regs.h"

typedef union 
{
	u32 u;
	struct 
	{
		u32 cmd		: 2;
		u32 chain	: 1;
		u32 irqcmp	: 1;
		u32 lock	: 1;
		u32 wait4read	: 1;
		u32 sem		: 1;
		u32 wait4end	: 1;
		u32 term	: 1;
		u32 rsvd1	: 3;
		u32 cmdword	: 4;
		u32 count	: 16;
	}b;
}hw_apbh_cmd_t;

typedef union
{
	u32	u;
	struct
	{
		u32 cnt		: 16;
		u32 inc		: 1;
		u32 addr	: 3;
		u32 cs		: 2;
		u32 lckcs	: 1;
		u32 len		: 1;
		u32 mod		: 2;
		u32 udma	: 1;
		u32 timoutirqen : 1;
		u32 rsvd3 	: 1;
		u32 run		: 1;
		u32 clkgate	: 1;
		u32 sftrst	: 1;
	}b;
}hw_gpmi_ctrl0_t;

typedef struct _apbh_dma_t
{
	struct _apbh_dma_t	*nxt;
	hw_apbh_cmd_t		cmd;
	void			*buff;
}apbh_dma_t;

typedef struct _apbh_dma_gpmi1_t
{
	struct _apbh_dma_gpmi1_t *nxt;
	hw_apbh_cmd_t		 cmd;
	void			 *buff;
	union
	{
		struct 
		{
			hw_gpmi_ctrl0_t gpmi_ctrl0;
		};
		u32 pio[1];
	};
}apbh_dma_gpmi1_t;

typedef struct _apbh_dma_gpmi2_t
{
	struct _apbh_dma_gpmi2_t *nxt;
	hw_apbh_cmd_t		 cmd;
	void			 *buff;
	union
	{
		struct 
		{
			hw_gpmi_ctrl0_t gpmi_ctrl0;
			u32		gpmi_compare;
		};
		u32 pio[2];
	};
}apbh_dma_gpmi2_t;


typedef struct 
{
	apbh_dma_gpmi1_t tx_cle;
	apbh_dma_gpmi1_t tx_addr;
	apbh_dma_gpmi1_t rx_data;
}dma_readid_t;

typedef struct 
{
	apbh_dma_gpmi1_t tx_cle1;
	apbh_dma_gpmi1_t tx_cle2;
	apbh_dma_gpmi1_t wait;
	apbh_dma_gpmi1_t sense;
	apbh_dma_gpmi1_t tx_statu;
	apbh_dma_gpmi2_t rx_statu;
	apbh_dma_t branch;
	apbh_dma_t success;
	apbh_dma_t failed;
}dma_erase_block_t;

int dm9000_index(unsigned char index);

unsigned char dm9000_read(void);

int dm9000_write(unsigned char data);

void dma_start(int ch, apbh_dma_gpmi1_t *start, int sema);

/*
int flash_erase_block(u32 block);

void gpmi_init_io(void);

void gpmi_init(void);

unsigned int gpmi_read_id(void);

int gpmi_read(char *buff, int size, int addr);

int gpmi_read_page(char *buff, int block, int page);

int gpmi_erase_block(int block);

int gpmi_program_page(char *buff, int block, int page);
*/

#endif
