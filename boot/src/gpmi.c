#include "regs.h"
#include "uart.h"
#include "gpmi.h"

APBH_CH *apbh_ch = (APBH_CH *)0x80004040;

//dma_erase_block_t eraseblock;


int dm9000_index(unsigned char cmd)
{
	apbh_dma_gpmi1_t index;
	
	index.nxt = (apbh_dma_gpmi1_t *)0;
	index.cmd.u = 0x000110ca;
	index.buff = &cmd;
	index.gpmi_ctrl0.u = 0x00900001;

	dma_start(5, &index, 1);

	return 0;
}

unsigned char dm9000_read(void)
{
	unsigned char data;
	apbh_dma_gpmi1_t read;
	
	read.nxt = (apbh_dma_gpmi1_t *)0;
	read.cmd.u = 0x000110c9;
	read.buff = &data;
	read.gpmi_ctrl0.u = 0x01920001;

	dma_start(5, &read, 1);

	return data;
}

int dm9000_write(unsigned char data)
{
	apbh_dma_gpmi1_t write;
	
	write.nxt = (apbh_dma_gpmi1_t *)0;
	write.cmd.u = 0x000110ca;
	write.buff = &data;
	write.gpmi_ctrl0.u = 0x00920001;

	dma_start(5, &write, 1);
}
void dma_start(int ch, apbh_dma_gpmi1_t *start, int sema)
{
	apbh_ch[ch].nxtcmd.dat = (reg)start;
	apbh_ch[ch].sema.dat = (reg)sema;

	while(!(HW_APBH_CTRL1 & HW_APBH_CTRL1_IRQ(ch)));
	HW_APBH_CTRL1_CLR = HW_APBH_CTRL1_IRQ(ch);
}
/*
int flash_erase_block(u32 block)
{
	unsigned int statu_dat = 0;
	unsigned char cmd[6] = {
		0x60,
		((block<<5) & 0xff),
		(block>>3) & 0xff,
		(block>>11) & 0xff,
		0xd0,
		0x70
	};
	//step 1 transmit commnd and address
	eraseblock.tx_cle1.nxt = &eraseblock.tx_cle2;
	//read, chain, nandlock, wait4endcmd, haltonterminate, cmdwords(1), count(4)
	eraseblock.tx_cle1.cmd.u = 0x00041196;
	eraseblock.tx_cle1.buff = cmd;
	//count(4), inc addr, cle, cs(00), lock_cs, lenth(8-bit), cmdmod(write)
	eraseblock.tx_cle1.gpmi_ctrl0.u = 0x00c30004;

	//step 2 cle2
//	eraseblock.tx_cle2.nxt = &eraseblock.wait;
	eraseblock.tx_cle2.nxt = &eraseblock.tx_statu;
	//read, chain, nandlock, wait4endcmd, haltonterminate, cmdwords(1), count(1)
	eraseblock.tx_cle2.cmd.u = 0x00011196;
	eraseblock.tx_cle2.buff = &cmd[4];
	//count(1), not inc addr, cle, cs(00), lock_cs, lenth(8-bit), cmdmod(write)
	eraseblock.tx_cle2.gpmi_ctrl0.u = 0x00c20001;

	//step 3 wait ready
	eraseblock.wait.nxt = &eraseblock.sense;
	//NO DMA TRANSFER, chain, wait4ready, wait4encmd, cmdwords(1)
	eraseblock.wait.cmd.u = 0x00001084;
	eraseblock.wait.buff = 0x00;
	//NAND_DATA, lenth(8-bit), cmdmod: wait for ready (cs must be set to b01)
//	eraseblock.wait.gpmi_ctrl0.u = 0x03800000;
	eraseblock.wait.gpmi_ctrl0.u = 0x00000000;

	//step 4 use sense
	eraseblock.sense.nxt = &eraseblock.tx_statu;
	//SENSE, chain
	eraseblock.sense.cmd.u = 0x00000007;
	//if timeout, next chain is this buff
	eraseblock.sense.buff = &eraseblock.failed;
	eraseblock.sense.gpmi_ctrl0.u = 0x00;

	//step 5 transmit status command
	eraseblock.tx_statu.nxt = (apbh_dma_gpmi1_t *)&eraseblock.rx_statu;
	//read, chain, nandlock, wait4endcmd, haltonterminate, cmdwords(1), count(1)
	eraseblock.tx_statu.cmd.u = 0x00011196;
	eraseblock.tx_statu.buff = &cmd[5];
	//count(1), not inc addr, cle, cs(00), lock_cs, lenth(8-bit), cmdmod(write)
	eraseblock.tx_statu.gpmi_ctrl0.u = 0x00c20001;

	//step 6 receive status data and compare
	eraseblock.rx_statu.nxt = (apbh_dma_gpmi2_t *)&eraseblock.branch;
	//write, chain, wait4endcmd, haltonterminate, cmdwords(1), count(1)
	eraseblock.rx_statu.cmd.u = 0x00011185;
	eraseblock.rx_statu.buff = &statu_dat;
	//read and compare, not inc addr, lenth(8-bit), count(1)
	eraseblock.rx_statu.gpmi_ctrl0.u = 0x02800001;
	eraseblock.rx_statu.gpmi_compare |= 0x00010000;

	//step 7 branch
	eraseblock.branch.nxt = &eraseblock.success;
	//SENSE, chain
	eraseblock.sense.cmd.u = 0x00000007;
	//if timeout, next chain is this buff
	eraseblock.sense.buff = &eraseblock.failed;
	//eraseblock.sense.gpmi_ctrl0.u = 0x00;
	
	//setp 8 success
	eraseblock.success.nxt = (apbh_dma_t *)0;
	//NO DMA TRANSFER, irq, semaphore, wait4endcmd
	eraseblock.success.cmd.u = 0x000000c8;
	eraseblock.success.buff = (void *)1;

	//setp 9 failed
	eraseblock.failed.nxt = (apbh_dma_t *)0;
	//NO DMA TRANSFER, irq, semaphore, wait4endcmd
	eraseblock.failed.cmd.u = 0x000000c8;
	eraseblock.failed.buff = (void *)0;

	dma_start(4, &eraseblock.tx_cle1, 1);

	return 0;
}

int flash_erase_block(u32 block)
{
	unsigned int statu_dat = 0;
	unsigned char cmd[6] = {
		0x60,
		((block<<5) & 0xff),
		(block>>3) & 0xff,
		(block>>11) & 0xff,
		0xd0,
		0x70
	};
	//step 1 transmit commnd and address
	eraseblock.tx_cle1.nxt = &eraseblock.tx_cle2;
	//read, chain, nandlock, wait4endcmd, haltonterminate, cmdwords(1), count(4)
	eraseblock.tx_cle1.cmd.u = 0x00041196;
	eraseblock.tx_cle1.buff = cmd;
	//count(4), inc addr, cle, cs(00), lock_cs, lenth(8-bit), cmdmod(write)
	eraseblock.tx_cle1.gpmi_ctrl0.u = 0x00c30004;

	//step 2 cle2
	eraseblock.tx_cle2.nxt = &eraseblock.wait;
	//read, chain, nandlock, wait4endcmd, haltonterminate, cmdwords(1), count(1)
	eraseblock.tx_cle2.cmd.u = 0x00011196;
	eraseblock.tx_cle2.buff = &cmd[4];
	//count(1), not inc addr, cle, cs(00), lock_cs, lenth(8-bit), cmdmod(write)
	eraseblock.tx_cle2.gpmi_ctrl0.u = 0x00c20001;

	//step 3 wait ready
	eraseblock.wait.nxt = &eraseblock.sense;
	//NO DMA TRANSFER, chain, wait4ready, wait4encmd, cmdwords(1)
	eraseblock.wait.cmd.u = 0x000010a4;
	eraseblock.wait.buff = 0x00;
	//NAND_DATA, lenth(8-bit), cmdmod: wait for ready (cs must be set to b01)
	eraseblock.wait.gpmi_ctrl0.u = 0x03810000;

	//step 4 use sense
	eraseblock.sense.nxt = &eraseblock.tx_statu;
	//SENSE, chain
	eraseblock.sense.cmd.u = 0x00000007;
	//if timeout, next chain is this buff
	eraseblock.sense.buff = &eraseblock.failed;
	eraseblock.sense.gpmi_ctrl0.u = 0x00;

	//step 5 transmit status command
	eraseblock.tx_statu.nxt = (apbh_dma_gpmi1_t *)&eraseblock.rx_statu;
	//read, chain, nandlock, wait4endcmd, haltonterminate, cmdwords(1), count(1)
	eraseblock.tx_statu.cmd.u = 0x00011196;
	eraseblock.tx_statu.buff = &cmd[5];
	//count(1), not inc addr, cle, cs(00), lock_cs, lenth(8-bit), cmdmod(write)
	eraseblock.tx_statu.gpmi_ctrl0.u = 0x00c20001;

	//step 6 receive status data and compare
	eraseblock.rx_statu.nxt = (apbh_dma_gpmi2_t *)&eraseblock.branch;
	//write, chain, wait4endcmd, haltonterminate, cmdwords(1), count(1)
	eraseblock.rx_statu.cmd.u = 0x00011185;
	eraseblock.rx_statu.buff = &statu_dat;
	//read and compare, not inc addr, lenth(8-bit), count(1)
	eraseblock.rx_statu.gpmi_ctrl0.u = 0x02800001;
	eraseblock.rx_statu.gpmi_compare |= 0x00010000;

	//step 7 branch
	eraseblock.branch.nxt = &eraseblock.success;
	//SENSE, chain
	eraseblock.sense.cmd.u = 0x00000007;
	//if timeout, next chain is this buff
	eraseblock.sense.buff = &eraseblock.failed;
	//eraseblock.sense.gpmi_ctrl0.u = 0x00;
	
	//setp 8 success
	eraseblock.success.nxt = (apbh_dma_t *)0;
	//NO DMA TRANSFER, irq, semaphore, wait4endcmd
	eraseblock.success.cmd.u = 0x000000c8;
	eraseblock.success.buff = (void *)1;

	//setp 9 failed
	eraseblock.failed.nxt = (apbh_dma_t *)0;
	//NO DMA TRANSFER, irq, semaphore, wait4endcmd
	eraseblock.failed.cmd.u = 0x000000c8;
	eraseblock.failed.buff = (void *)0;

	dma_start(4, *(reg *)&eraseblock.tx_cle1, 1);

	return 0;
}


unsigned int gpmi_read_id(void)
{
	unsigned int id;
	unsigned char id_cmd[] = {0x90, 0x00};

	DMA_CMD rdidcmd[2] = {
		{
		&rdidcmd[1],
		{2, 1, 0, 0, 0, 0, 1, 0, 0, 1, 2},
		(unsigned int *)id_cmd,
		0x00c20002
		},
		{
		0,
		{1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 4},
		&id,
		0x01800004
		}
	};

	apbh_ch[4].nxtcmd.dat = (reg)rdidcmd;
	apbh_ch[4].sema.dat = 0x1;
	while(!(HW_APBH_CTRL1 & (1<<4)))
	{
		;
	}

	HW_APBH_CTRL1_CLR = (1<<4);

	return id;
}


unsigned int gpmi_read_id(void)
{
	unsigned int id;
	unsigned char id_cmd[] = {0x90, 0x00};

	DMA_CMD rdidcmd[3] = {
		{
		&rdidcmd[1],
		{2, 1, 0, 0, 0, 0, 1, 0, 0, 1, 1},
		(unsigned int *)id_cmd,
		0x00c20001
		},
		{
		&rdidcmd[2],
		{2, 1, 0, 0, 0, 0, 1, 0, 0, 1, 1},
		(unsigned int *)&id_cmd[1],
		0x00c40001
		},
		{
		0,
		{1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 4},
		&id,
		0x01800004
		}
	};

	apbh_ch[4].nxtcmd.dat = (reg)rdidcmd;
	apbh_ch[4].sema.dat = 0x1;
	while(!(HW_APBH_CTRL1 & (1<<4)))
	{
		;
	}

	HW_APBH_CTRL1_CLR = (1<<4);

	return id;
}

int gpmi_read(char *buff, int size, int addr)
{
	unsigned char cmd[5] = {
		0,
		addr & 0x7f,
		(addr>>9) & 0xff,
		(addr>>17) & 0xff,
		(addr>>25) & 0x01
	};

	if ((addr>>8) & 0x01)
	{
		cmd[0] = 0x00;
	}
	else
	{
		cmd[1] = 0x01;
	}

	DMA_CMD read[2] = {
		{
		&read[1],
		{2, 1, 0, 0, 0, 0, 1, 0, 0, 1, 5},
		(unsigned int *)cmd,
		0x00c30005
		},
		{
		0,
		{1, 0, 1, 0, 0, 1, 1, 0, 0, 1, size},
		(unsigned int *)buff,
		0x01800000 | (size & 0xffff)
		}
	};

	apbh_ch[4].nxtcmd.dat = (reg)read;
	apbh_ch[4].sema.dat = 0x1;
	while(!(HW_APBH_CTRL1 & (1<<4)))
	{
		;
	}

	HW_APBH_CTRL1_CLR = (1<<4);

	return size;
}


int gpmi_read_page(char *buff, int block, int page)
{
	unsigned char cmd[5] = {
		0,
		0,
		((block<<5) & 0xff) | page,
		(block>>3) & 0xff,
		(block>>11) & 0xff
	};

	DMA_CMD read[3] = {
		{
		&read[1],
		{2, 1, 0, 0, 0, 0, 1, 0, 0, 1, 1},
		(unsigned int *)cmd,
		0x00c20001
		},
		{
		&read[2],
		{2, 1, 0, 0, 0, 0, 1, 0, 0, 1, 4},
		(unsigned int *)&cmd[1],
		0x00c40004
		},
		{
		0,
		{1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 512},
		(unsigned int *)buff,
		0x01c00000 | 512
		}
	};

	apbh_ch[4].nxtcmd.dat = (reg)read;
	apbh_ch[4].sema.dat = 0x1;
	while(!(HW_APBH_CTRL1 & (1<<4)))
	{
		;
	}

	HW_APBH_CTRL1_CLR = (1<<4);

	return 0;
}

int gpmi_erase_block(int block)
{
	unsigned int statu_dat = 0;
//	unsigned char statu[1] = {0x70};
//	unsigned char seccmd[1] = {0xd0};
	unsigned char cmd[6] = {
		0x60,
		((block<<5) & 0xff),
		(block>>3) & 0xff,
		(block>>11) & 0xff,
		0xd0,
		0x70
	};

	DMA_CMD erase[6] = {
		{
		&erase[1],
		{2, 1, 0, 0, 0, 0, 1, 0, 0, 1, 1},
		(unsigned int *)cmd,
		0x00c20001
		},
		{
		&erase[2],
		{2, 1, 0, 0, 0, 0, 1, 0, 0, 1, 3},
		(unsigned int *)&cmd[1],
		0x00c40003
		},
		{
		&erase[3],
		{2, 1, 0, 0, 0, 0, 1, 0, 0, 1, 1},
		(unsigned int *)&cmd[4],
		0x00c20001
		},
		{
		&erase[4],
		{2, 1, 0, 0, 0, 0, 1, 0, 0, 1, 0},
		(unsigned int *)&cmd[4],
		0x03800000
		},
		{
		&erase[5],
		{2, 1, 0, 0, 0, 0, 1, 0, 0, 1, 1},
		(unsigned int *)&cmd[5],
		0x00c20001
		},
		{
		0,
		{1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 1},
		&statu_dat,
		0x01c00001
		}
	};

	apbh_ch[4].nxtcmd.dat = (reg)erase;
	apbh_ch[4].sema.dat = 0x1;
	while(!(HW_APBH_CTRL1 & (1<<4)))
	{
		;
	}

	HW_APBH_CTRL1_CLR = (1<<4);

	if (statu_dat & 0x1)
	{
		return -1;
	}

	return 0;
}

int gpmi_erase_block(int block)
{
	unsigned int statu_dat = 0;
	unsigned char statu[1] = {0x70};
	unsigned char cmd[5] = {
		0x60,
		((block<<5) & 0xff),
		(block>>3) & 0xff,
		(block>>11) & 0xff,
		0xd0
	};

	DMA_CMD erase[3] = {
		{
		&erase[1],
		{2, 1, 0, 0, 0, 0, 1, 0, 0, 1, 5},
		(unsigned int *)cmd,
		0x00c40005
		},
		{
		&erase[2],
		{2, 1, 0, 0, 0, 0, 1, 0, 0, 1, 1},
		(unsigned int *)statu,
		0x00c00001
		},
		{
		0,
		{1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 1},
		&statu_dat,
		0x01800001
		}
	};

	apbh_ch[4].nxtcmd.dat = (reg)erase;
	apbh_ch[4].sema.dat = 0x1;
	while(!(HW_APBH_CTRL1 & (1<<4)))
	{
		;
	}

	HW_APBH_CTRL1_CLR = (1<<4);

	if (statu_dat & 0x1)
	{
		return -1;
	}

	return 0;
}


int gpmi_program_page(char *buff, int block, int page)
{
	unsigned int statu_dat = 0;
//	unsigned char statu[1] = {0x70};
//	unsigned char seccmd[1] = {0xd0};
	unsigned char cmd[5] = {
		0x80,
		((block<<5) & 0xff) | page,
		(block>>3) & 0xff,
		(block>>11) & 0xff,
		0x70
	};

	DMA_CMD write[4] = {
		{
		&write[1],
		{2, 1, 0, 0, 0, 0, 1, 0, 0, 1, 1},
		(unsigned int *)cmd,
		0x00c30001
		},
		{
		&write[2],
		{2, 1, 0, 0, 0, 0, 1, 0, 0, 1, 3},
		(unsigned int *)&cmd[1],
		0x00c50003
		},
		{
		&write[3],
		{2, 1, 0, 0, 0, 0, 1, 0, 0, 1, 1},
		(unsigned int *)&cmd[4],
		0x00c30001
		},
		{
		0,
		{1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 512},
		(unsigned int *)buff,
		0x01800000 | 512
		}
	};

	apbh_ch[4].nxtcmd.dat = (reg)write;
	apbh_ch[4].sema.dat = 0x1;
	while(!(HW_APBH_CTRL1 & (1<<4)))
	{
		;
	}

	HW_APBH_CTRL1_CLR = (1<<4);

	if (statu_dat & 0x1)
	{
		return -1;
	}

	return 0;
}
*/
