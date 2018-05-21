/*
 * Copyright © 2009 Nuvoton technology corporation.
 *
 * Wan ZongShun <mcuos.com@gmail.com>
 * ZM.Song <zmsong001@gmail.com> modified - 2011.01.07
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */


#include <linux/slab.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/blkdev.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>

#include <mach/map.h>
#include <mach/regs-clock.h>
#include <mach/w55fa93_reg.h>
#include <linux/dma-mapping.h>

//#define WB_MTD_DEBUG
#define WB_MTD_DEBUG_ENTER_LEAVE

#ifdef WB_MTD_DEBUG
#define DBG(fmt, arg...)			printk(fmt, ##arg)
#else
#define DBG(fmt, arg...)
#endif

#ifdef WB_MTD_DEBUG_ENTER_LEAVE
#define ENTER()					DBG("[%-10s] : Enter\n", __FUNCTION__)
#define LEAVE()					DBG("[%-10s] : Leave\n", __FUNCTION__)
#else
#define ENTER()
#define LEAVE()
#endif


//#define REG_FMICR   	0x00
//#define REG_SMCSR    	0xa0
//#define REG_SMISR    	0xac
//#define REG_SMCMD    	0xb0
//#define REG_SMADDR   	0xb4
//#define REG_SMDATA   	0xb8
//#define REG_SMTCR     	0xa4

#define RESET_FMI	 	0x01
#define NAND_EN		 	0x08

#ifdef CONFIG_CARD_MTD		// on board NAND uased for MTD (defined in SCSI)s
	#define NANDCARD_NAND		// external NAND card
#else
	#define ONBOARD_NAND		// on-board NAND
#endif

#if defined(ONBOARD_NAND)		
	#define READYBUSY		(0x01 << 18)
//	#define READYBUSY		(0x01 << 10)
#endif		

#if defined (NANDCARD_NAND)
	#define READYBUSY		(0x01 << 19)
//	#define READYBUSY		(0x01 << 11)	
#endif		

#define OPT_HW_ECC

#define SWRST		  	 0x01
#define PSIZE		  	(0x01 << 3)
#define DMARWEN			(0x03 << 1)
#define BUSWID			(0x01 << 4)
#define ECC4EN			(0x01 << 5)
#define WP		    	(0x01 << 24)
#define NANDCS			(0x01 << 25)
#define ENDADDR			(0x01 << 31)


extern struct semaphore fmi_sem;
extern struct semaphore dmac_sem;

#define read_data_reg(dev)		\
	readl(REG_SMDATA)

#define write_data_reg(dev, val)	\
	writel((val), REG_SMDATA)

#define write_cmd_reg(dev, val)		\
	writel((val), REG_SMCMD)

#define write_addr_reg(dev, val)	\
	writel((val), REG_SMADDR)

struct w55fa93_nand {
	struct mtd_info mtd;
	struct nand_chip chip;
	void __iomem *reg;
	struct clk *clk;
    struct clk *sic_clk;
    struct clk *nand_clk;
	spinlock_t lock;
};

static int s_nand_init = 0;
volatile int g_page_addr;
static unsigned char * s_nand_vaddr = NULL;
static unsigned char * s_nand_phyaddr = NULL;
int mtd_fmiSMCheckRB(struct w55fa93_nand *nand);

volatile int g_SMCR_mtd;
	
static void dump_regs(struct w55fa93_nand *dev)
{
	printk("==========================\n");
	printk("REG_FMICR[00] : 0x%08X\n",readl(REG_FMICR));
	printk("REG_SMCSR[A0] : 0x%08X\n",readl(REG_SMCSR));
	printk("REG_SMISR[AC] : 0x%08X\n",readl(REG_SMISR));
	printk("REG_SMCMD[B0] : 0x%08X\n",readl(REG_SMCMD));
	printk("REG_SMADDR[B4] : 0x%08X\n",readl(REG_SMADDR));
	printk("REG_SMDATA[B8] : 0x%08X\n",readl(REG_SMDATA));
	printk("REG_SMTCR[A4] : 0x%08X\n",readl(REG_SMTCR));
	printk("==========================\n");
}

#ifdef CONFIG_USER_PARTITION_W55FA93
	static const struct mtd_partition partitions[] = {
		{
		 .name = "NAND FS 0",
		 .size   = CONFIG_MTD_NAND_PARTITION_SIZE,
		 .offset = CONFIG_MTD_NAND_PARTITION_OFFSET,
		},
		{
		 .name = "NAND FS 1",
		 .offset = MTDPART_OFS_APPEND,
		 .size = MTDPART_SIZ_FULL
		}
	};

#else
	static const struct mtd_partition partitions[] = {
		{
		 .name = "NAND FS 0",
		 .offset = 0,
		 .size = 8 * 1024 * 1024
		},
		{
		 .name = "NAND FS 1",
		 .offset = MTDPART_OFS_APPEND,
		 .size = MTDPART_SIZ_FULL
		}
	};
#endif	

static unsigned char w55fa93_nand_read_byte(struct mtd_info *mtd)
{
	unsigned char ret;
	struct w55fa93_nand *nand;

	ENTER();

	nand = container_of(mtd, struct w55fa93_nand, mtd);

  	if(down_interruptible(&fmi_sem)) //jhe+ 2010.12.21
	{
		printk("w55fa93 mtd nand driver read_byte sem error\n");
		return -1;
	}
#ifdef NANDCARD_NAND	
	writel(readl(REG_SMCSR)&(~0x06000000)|0x02000000, REG_SMCSR);		// CS1 is slected
#else	
	writel(readl(REG_SMCSR)&(~0x06000000)|0x04000000, REG_SMCSR);		// CS0 is slected	
#endif
	
	if(readl(REG_FMICR) != 0x8)
	   writel(0x08, REG_FMICR);
	
	ret = (unsigned char)read_data_reg(nand);
//	printk("NAND_DATA = 0x%x  !!!\n", ret);
	up(&fmi_sem);

	return ret;
	LEAVE();	
}

static void fmiSM_CorrectData_BCH(char ucFieidIndex, char ucErrorCnt, char* pDAddr)
{
	// ECC is based on 512+32, when ECC code error is encountered, we must coorect it by this base

    int uaData[16], uaAddr[16];
	int uaErrorData[4];
	unsigned char	ii, jj;
	int uPageSize;
	
	uPageSize = readl(REG_SMCSR) & SMCR_PSIZE;
	    
    jj = ucErrorCnt/4;
	jj ++;
	if (jj > 4)	jj = 4;
    
    for(ii=0; ii<jj; ii++)    
    {
		uaErrorData[ii] = readl(REG_BCH_ECC_DATA0 + ii*4);
	}		
	
    for(ii=0; ii<jj; ii++)	
    {
	    uaData[ii*4+0] = uaErrorData[ii] & 0xff;
	    uaData[ii*4+1] = (uaErrorData[ii]>>8) & 0xff;    
	    uaData[ii*4+2] = (uaErrorData[ii]>>16) & 0xff;        
	    uaData[ii*4+3] = (uaErrorData[ii]>>24) & 0xff;            
	}		    

    jj = ucErrorCnt/2;
    jj ++;
	if (jj > 8)	jj = 8;    

    for(ii=0; ii<jj; ii++)	    
    {
	    uaAddr[ii*2+0] = readl(REG_BCH_ECC_ADDR0 + ii*4) & 0x1fff;
	    uaAddr[ii*2+1] = (readl(REG_BCH_ECC_ADDR0 + ii*4)>>16) & 0x1fff;            
    }

    pDAddr += (ucFieidIndex-1)*0x200;
  
    for(ii=0; ii<ucErrorCnt; ii++)	
    {
    	if (uPageSize == PSIZE_2K)		
		{
			switch(readl(REG_SMCSR) & SMCR_BCH_TSEL)
			{
				case BCH_T4:
				default:
			    	if (uaAddr[ii] < 512)
			  	        *(pDAddr+uaAddr[ii]) ^=  uaData[ii];  
					else
					{
				    	if (uaAddr[ii] < 515)
				    	{
							uaAddr[ii] -= 512;								    	
							uaAddr[ii] += 8*(ucFieidIndex-1);	// field offset, only Field-0
				  	        *((unsigned char*)REG_SMRA_0+uaAddr[ii]) ^=  uaData[ii];  			    	
				    	}
				    	else
				    	{
							uaAddr[ii] = 543 - uaAddr[ii];					
							uaAddr[ii] = 7 - uaAddr[ii];											
							uaAddr[ii] += 8*(ucFieidIndex-1);	// field offset
				  	        *((unsigned char*)REG_SMRA_8+uaAddr[ii]) ^=  uaData[ii];  
						}
					}			  	        
					break;
			}
		}
  	}
}


int fmiSMCorrectData_2K(int uDAddr)
{
	int uStatus, ii;
    volatile int uErrorCnt;


	while(1) 
	{
		if (readl(REG_SMISR) & SMISR_ECC_FIELD_IF)
		{
            uStatus = readl(REG_SM_ECC_ST0);   		
        	for (ii=1; ii<5; ii++)	        
        	{
	    		if ((uStatus & 0x03)==0x01)  // correctable error in 1st field 
	            {
	                uErrorCnt = uStatus >> 2;
	                fmiSM_CorrectData_BCH(ii, uErrorCnt, (char*)uDAddr);	                
		//	#ifdef WB_MTD_DEBUG
					printk("Field %d have %d error!!\n", ii, uErrorCnt);
		//	#endif
	                break;
	            }    
	    		else if (((uStatus & 0x03)==0x02)  
	    		      ||((uStatus & 0x03)==0x03)) // uncorrectable error or ECC error in 1st field                   
		            {
	        #ifdef WB_MTD_DEBUG			
	        	    printk("SM uncorrectable error is encountered, %4x !!\n", uStatus);
	        #endif   
	                break;		        	
	            }            
				uStatus >>= 8;		    		            
			}		            

	   	    writel(SMISR_ECC_FIELD_IF, REG_SMISR);	   	// clear ECC_FLD_Error	        
		}		    

	#if 1
		if (!(readl(REG_SMCSR) & SMCR_DRD_EN))      // wait to finish DMAC transfer. 
		{											// don't check SMISR_DMA_IF, since it may be cleared in SCSI isr
			if ( !(readl(REG_SMISR) & SMISR_ECC_FIELD_IF) )
				break;
		}
	#else
		if (readl(REG_SMISR) & SMISR_DMA_IF)      // wait to finish DMAC transfer.   		    
		{
			if ( !(readl(REG_SMISR) & SMISR_ECC_FIELD_IF) )
				break;
		}
	#endif		
	}
    
    return 0;
}


static void w55fa93_nand_read_buf(struct mtd_info *mtd,
				 unsigned char *buf, int len)
{
	int i, column;
	struct w55fa93_nand *nand;
	volatile char s_buf[64];
	struct nand_chip *chip = mtd->priv;
	char* ptr;
	int SMCR_buf;
	
	ENTER();	

#ifdef OPT_HW_ECC

	nand = container_of(mtd, struct w55fa93_nand, mtd);
	if(down_interruptible(&fmi_sem)) //jhe+ 2010.12.21
	{
		printk("w55fa93 mtd nand driver read buf sem error\n");
		return;
	}

#ifdef NANDCARD_NAND	
	writel(readl(REG_SMCSR)&(~0x06000000)|0x02000000, REG_SMCSR);		// CS1 is slected
#else	
	writel(readl(REG_SMCSR)&(~0x06000000)|0x04000000, REG_SMCSR);		// CS0 is slected	
#endif


  	
	if(readl(REG_FMICR) != 0x8)
	   writel(0x08, REG_FMICR);

//	printk("len = 0x%x !!!!\n", len);
//	printk("mtd->oobsize = 0x%x !!!!\n", mtd->oobsize);
//	printk("mtd->writesize = 0x%x !!!!\n", mtd->writesize);	

//	dump_regs(NULL);
	if(len==mtd->oobsize)		// read oob data
	{		
		for (i = 0; i < len; i++)
			buf[i] = (unsigned char)read_data_reg(nand);

		goto read_out;
	}
	else if (len == mtd->writesize)
	{
		ptr = (char*)REG_SMRA_0;
		for (i = 0; i < 0x40; i++)		// 2K-page reduntant area is 64 bytes
		{
			ptr[i] = (unsigned char)read_data_reg(nand);
//			printk("ptr[%d] = 0x%x  !!!\n", i, ptr[i]);			
		}			

		// read main area data
		column = 0x00;		
		write_cmd_reg(nand, NAND_CMD_READ0);
		write_addr_reg(nand, (unsigned char)column);
	  	write_addr_reg(nand, (unsigned char)(column >> 8)&0x0f);	
		write_addr_reg(nand, (unsigned char)(g_page_addr )& 0xff);	

		/* One more address cycle for devices > 128MiB */
		if (chip->chipsize >= (64 << 20))		// mhkuo@20121204
		{
			write_addr_reg(nand, (unsigned char) (g_page_addr>>8 )& 0xff);
			write_addr_reg(nand, (unsigned char) ((g_page_addr>>16 )& 0xff)|0x80000000);
		}
		else
			write_addr_reg(nand, (unsigned char) ((g_page_addr>>8 )& 0xff)|0x80000000);

	#if defined(ONBOARD_NAND)		
		  	writel(0x400, REG_SMISR);//jhe
	#endif		
	#if defined (NANDCARD_NAND)
		  	writel(0x800, REG_SMISR);//jhe
	#endif		
		
		write_cmd_reg(nand, NAND_CMD_READSTART);
		if (!mtd_fmiSMCheckRB(nand))
				printk("check RB error\n");
	
		if(down_interruptible(&dmac_sem)) 
		{
			printk("w55fa93 mtd nand driver read buf dmac error\n");
			return;
		}

	  	while (readl(REG_DMACCSR) & FMI_BUSY); 	// wait DMAC ready
	  	writel(readl(REG_DMACCSR) | DMAC_EN, REG_DMACCSR);	  		  		  	
	  	writel(readl(REG_DMACCSR) | DMAC_SWRST, REG_DMACCSR);	  		  	
	  	writel(readl(REG_DMACCSR) &~DMAC_SWRST, REG_DMACCSR);	  		  		  	
	  	writel((unsigned long)s_nand_phyaddr, REG_DMACSAR);	  	

		SMCR_buf = readl(REG_SMCSR);
		writel(g_SMCR_mtd, REG_SMCSR);
		writel(readl(REG_SMISR) | SMISR_DMA_IF,REG_SMISR);  	//clear DMA finished flag	  	
		writel(SMISR_ECC_FIELD_IF, REG_SMISR);  	//clear DMA finished flag	  	
		writel(readl(REG_SMCSR) | SMCR_DRD_EN, REG_SMCSR); 		//enable DMA write	
		fmiSMCorrectData_2K((unsigned long)s_nand_vaddr);			
		writel(readl(REG_SMISR) | SMISR_DMA_IF,REG_SMISR);  	//clear DMA finished flag	  	
		
		memcpy(buf, s_nand_vaddr, len);		// copy write buffer contents to DMA buffer		
		writel(SMCR_buf, REG_SMCSR);		
		up(&dmac_sem);		
	}
	else
		printk("wrong read buffer size !!!\n");			
	
read_out:

	up(&fmi_sem);

#else	// SW ECC

	nand = container_of(mtd, struct w55fa93_nand, mtd);
	if(down_interruptible(&fmi_sem)) //jhe+ 2010.12.21
	{
		printk("w55fa93 mtd nand driver read buf sem error\n");
		return;
	}

#ifdef NANDCARD_NAND	
	writel(readl(REG_SMCSR)&(~0x06000000)|0x02000000, REG_SMCSR);		// CS1 is slected
#else	
	writel(readl(REG_SMCSR)&(~0x06000000)|0x04000000, REG_SMCSR);		// CS0 is slected	
#endif

	if(readl(REG_FMICR) != 0x8)
	   writel(0x08, REG_FMICR);

	for (i = 0; i < len; i++)
		buf[i] = (unsigned char)read_data_reg(nand);

	up(&fmi_sem);
#endif
	LEAVE();		
}

static void w55fa93_nand_write_buf(struct mtd_info *mtd,
				  const unsigned char *buf, int len)
{
#ifdef OPT_HW_ECC		// hardware ECC

	int i, ii, jj, ecc_value;
	struct w55fa93_nand *nand;
	int SMCR_buf;

	ENTER();
	
	nand = container_of(mtd, struct w55fa93_nand, mtd);
	
	if(down_interruptible(&fmi_sem)) //jhe+ 2010.12.21
	{
		printk("w55fa93 mtd nand driver write buf sem error\n");
		return;
	}

#ifdef NANDCARD_NAND	
	writel(readl(REG_SMCSR)&(~0x06000000)|0x02000000, REG_SMCSR);		// CS1 is slected
#else	
	writel(readl(REG_SMCSR)&(~0x06000000)|0x04000000, REG_SMCSR);		// CS0 is slected	
#endif

	if(readl(REG_FMICR) != 0x8)
	   writel(0x08, REG_FMICR);

//	dump_regs(NULL);
	if(len==mtd->oobsize)		// write for oob 
	{
  		if( (buf[0]!=0xFF) || (buf[1]!=0xFF))
  		{
  			for(ii=0; ii<16; ii++){
				printk("buf[%d] = 0x%x, ",4*ii+0, buf[4*ii+0]);  				
				printk("buf[%d] = 0x%x, ",4*ii+1, buf[4*ii+1]);  				
				printk("buf[%d] = 0x%x, ",4*ii+2, buf[4*ii+2]);  				
				printk("buf[%d] = 0x%x, ",4*ii+3, buf[4*ii+3]);  				
				printk(" !!!\n");												
			}
		}			
	
		for (i = 0; i < len; i++)
			write_data_reg(nand, buf[i]);
			
	    goto write_out;			
	}			
	else if (len == mtd->writesize)
	{
		if(down_interruptible(&dmac_sem)) 
		{
			printk("w55fa93 mtd nand driver read buf dmac error\n");
			return;
		}

		memcpy(s_nand_vaddr, buf, len);		// copy write buffer contents to DMA buffer
	  	while (readl(REG_DMACCSR) & FMI_BUSY); 	// wait DMAC ready
	  	
	  	writel(readl(REG_DMACCSR) | DMAC_EN, REG_DMACCSR);	  		  		  	
	  	writel(readl(REG_DMACCSR) | DMAC_SWRST, REG_DMACCSR);	  		  	
	  	writel(readl(REG_DMACCSR) &~DMAC_SWRST, REG_DMACCSR);	  		  		  	
	  	writel((unsigned long)s_nand_phyaddr, REG_DMACSAR);	 

		SMCR_buf = readl(REG_SMCSR);
		writel(g_SMCR_mtd, REG_SMCSR);
		writel(readl(REG_SMISR) | SMISR_DMA_IF,REG_SMISR);  	//clear DMA finished flag	  	
		writel(readl(REG_SMCSR) | SMCR_DWR_EN, REG_SMCSR); 		//enable DMA write	
//		while(!(readl(REG_SMISR) & SMISR_DMA_IF));				//wait for dma finished
		while(readl(REG_SMCSR) & SMCR_DWR_EN);				//wait for dma finished
		writel(readl(REG_SMISR) | SMISR_DMA_IF,REG_SMISR);  	//clear DMA finished flag	  	

		writel(SMCR_buf, REG_SMCSR);		
		up(&dmac_sem);		
	}
	else
		printk("wrong write buffer size !!!\n");		
	
write_out:
		
	up(&fmi_sem);
	LEAVE();	
	
#else		// software ECC
	int i;
	struct w55fa93_nand *nand;

	ENTER();
	
	nand = container_of(mtd, struct w55fa93_nand, mtd);
	
	if(down_interruptible(&fmi_sem)) //jhe+ 2010.12.21
	{
		printk("w55fa93 mtd nand driver write buf sem error\n");
		return;
	}


#ifdef NANDCARD_NAND	
	writel(readl(REG_SMCSR)&(~0x06000000)|0x02000000, REG_SMCSR);		// CS1 is slected
#else	
	writel(readl(REG_SMCSR)&(~0x06000000)|0x04000000, REG_SMCSR);		// CS0 is slected	
#endif

	if(readl(REG_FMICR) != 0x8)
	   writel(0x08, REG_FMICR);

	for (i = 0; i < len; i++)
		write_data_reg(nand, buf[i]);
		
	up(&fmi_sem);
	LEAVE();	
#endif	
}

static int w55fa93_verify_buf(struct mtd_info *mtd,
			     const unsigned char *buf, int len)
{
	int i;
	struct w55fa93_nand *nand;

	ENTER();
	
	nand = container_of(mtd, struct w55fa93_nand, mtd);
	if(down_interruptible(&fmi_sem)) //jhe+ 2010.12.21
	{
		printk("w55fa93 mtd nand driver verify_buf sem error\n");
		return -EFAULT;
	}

#ifdef NANDCARD_NAND	
	writel(readl(REG_SMCSR)&(~0x06000000)|0x02000000, REG_SMCSR);		// CS1 is slected
#else	
	writel(readl(REG_SMCSR)&(~0x06000000)|0x04000000, REG_SMCSR);		// CS0 is slected	
#endif

	if(readl(REG_FMICR) != 0x8)
	   writel(0x08, REG_FMICR);
	
	for (i = 0; i < len; i++) {
		if (buf[i] != (unsigned char)read_data_reg(nand))
		{
			up(&fmi_sem);
			return -EFAULT;
		}
	}

  	up(&fmi_sem);
	LEAVE();  	
	return 0;
}

/* select chip */
static void w55fa93_nand_select_chip(struct mtd_info *mtd, int chip)
{
  struct w55fa93_nand *nand;
	nand = container_of(mtd, struct w55fa93_nand, mtd);

//	ENTER();
			
	if(down_interruptible(&fmi_sem)) //jhe+ 2010.12.21
	{
		printk("w55fa93 mtd nand driver select_chip sem error\n");
		return ;
	}

#ifdef NANDCARD_NAND	
	writel(readl(REG_SMCSR)&(~0x06000000)|0x02000000, REG_SMCSR);		// CS1 is slected
#else	
	writel(readl(REG_SMCSR)&(~0x06000000)|0x04000000, REG_SMCSR);		// CS0 is slected	
#endif

	if(readl(REG_FMICR) != 0x8)
	   writel(0x08, REG_FMICR);

		// remove this segment by mhkuo @20121122
//  	if(mtd->writesize==0x800)//2K page
//  		writel((readl(REG_SMCSR)&0xfffffff0)|0x00000008, REG_SMCSR);  // ???????
  
  	up(&fmi_sem);
  	
//	LEAVE();  	
}

static int w55fa93_check_rb(struct w55fa93_nand *nand)
{
	unsigned int val;

	
	spin_lock(&nand->lock);
	val = readl(REG_SMISR);
	val &= READYBUSY;
	spin_unlock(&nand->lock);

  
	return val;
}


static int w55fa93_nand_devready(struct mtd_info *mtd)
{
	struct w55fa93_nand *nand;
	int ready;

//	ENTER();

  if(down_interruptible(&fmi_sem)) //jhe+ 2010.12.21
	{
		printk("w55fa93 mtd nand driver devready sem error\n");
		return -1;
	}

#ifdef NANDCARD_NAND	
	writel(readl(REG_SMCSR)&(~0x06000000)|0x02000000, REG_SMCSR);		// CS1 is slected
#else	
	writel(readl(REG_SMCSR)&(~0x06000000)|0x04000000, REG_SMCSR);		// CS0 is slected	
#endif

	if(readl(REG_FMICR) != 0x8)
	   writel(0x08, REG_FMICR);

	nand = container_of(mtd, struct w55fa93_nand, mtd);

	ready = (w55fa93_check_rb(nand)) ? 1 : 0;
	
	up(&fmi_sem);	
//	LEAVE();	
	return ready;
}

/* functions */
int mtd_fmiSMCheckRB(struct w55fa93_nand *nand)
{
#if defined(ONBOARD_NAND)		
	while(1)
	{
		if (readl(REG_SMISR) & 0x400)
		{
			writel(0x400,REG_SMISR);
			return 1;
		}
	}
#endif		

#if defined (NANDCARD_NAND)
	while(1)
	{
		if (readl(REG_SMISR) & 0x800)
		{
			writel(0x800,REG_SMISR);
			return 1;
		}
	}
#endif		
	return 0;
}

int ebiCheckRB(struct w55fa93_nand *nand)
{
	int ret;

	ret = readl(REG_SMISR)& 0x40000;
	
  return ret;
}

static int  w55fa93_nand_reset(struct w55fa93_nand *nand)
{
	unsigned int  volatile i;

	ENTER();
	
	writel(0xff, REG_SMCMD);	
	for (i=100; i>0; i--);
	//while(!ebiCheckRB(nand));
	
	LEAVE();	
	while(!w55fa93_check_rb(nand));
}

static unsigned char pre_nand_command;

static void w55fa93_nand_command_lp (struct mtd_info *mtd, unsigned command, int column, int page_addr)
{
	register struct nand_chip *this = mtd->priv;
	struct w55fa93_nand *nand;
	nand = container_of(mtd, struct w55fa93_nand, mtd);
	
//	ENTER();
	g_page_addr  = page_addr;
	
//	printk("===enter %s, writesize=%x, command = %x\n",__FUNCTION__,mtd->writesize, command);
//	printk("columnAddr=%x, pageAddr = %x\n", column, page_addr);	
	
	if(down_interruptible(&fmi_sem)) //jhe+ 2010.12.21
	{
		printk("w55fa93 mtd nand driver command sem error\n");
		return ;
	}

#ifdef NANDCARD_NAND	
	writel(readl(REG_SMCSR)&(~0x06000000)|0x02000000, REG_SMCSR);		// CS1 is slected
#else	
	writel(readl(REG_SMCSR)&(~0x06000000)|0x04000000, REG_SMCSR);		// CS0 is slected	
#endif

	if(readl(REG_FMICR) != 0x8)
	   writel(0x08, REG_FMICR);

	if(pre_nand_command == NAND_CMD_READOOB && mtd->writesize == 0x200)
	{	
//		pre_nand_command = 0;
//		w55fa93_nand_reset(nand);
	}
	/* Emulate NAND_CMD_READOOB */
	if (command == NAND_CMD_READOOB) {
		column += mtd->writesize;
		if(mtd->writesize == 0x200){
				//column = 5;
				column = 0;
				pre_nand_command = NAND_CMD_READOOB;
		}	
		else {
			command = NAND_CMD_READ0;
		}
	}

	if(command == NAND_CMD_SEQIN)
	{
		unsigned readcommand;
		if(mtd->writesize == 0x200)
		{
			if(column < 0x100)
  			{
				readcommand = NAND_CMD_READ0;
			}
			else if(column >= 0x200)
			{
  				column -= 0x200;
				readcommand = NAND_CMD_READOOB;
				pre_nand_command = NAND_CMD_READOOB;
			}
			else
			{
				column -= 0x100;
				readcommand = NAND_CMD_READ1;
			}
	    
			write_cmd_reg(nand, readcommand);
		}
	}

	write_cmd_reg(nand, command);

	if (column != -1 || page_addr != -1) {
#if 0
		writel(readl(REG_SMISR), REG_SMISR);	// clear R/B flag
#else
	#if defined(ONBOARD_NAND)		
	  	writel(0x400, REG_SMISR);//jhe
	#endif		
	#if defined (NANDCARD_NAND)
	  	writel(0x800, REG_SMISR);//jhe
	#endif		
#endif				
		/* Serially input address */
		if (column != -1) {
			/* Adjust columns for 16 bit buswidth */
			if (this->options & NAND_BUSWIDTH_16)
				column >>= 1;

#ifdef OPT_HW_ECC
			// check if to read full page data, if yes, change to read redundant data firstly for ECC check
			if (command ==NAND_CMD_READ0)
			{
				column = mtd->writesize;
				write_addr_reg(nand, (unsigned char)column);
				if(mtd->writesize == 0x800) //2K page	
				  	write_addr_reg(nand, (unsigned char)(column >> 8)&0x0f);	
			}				  	
			else
			{
				write_addr_reg(nand, (unsigned char)column);
				if(mtd->writesize == 0x800) //2K page	
				  	write_addr_reg(nand, (unsigned char)(column >> 8)&0x0f);	
			}				
#else
			write_addr_reg(nand, (unsigned char)column);
			if(mtd->writesize == 0x800) //2K page	
			  	write_addr_reg(nand, (unsigned char)(column >> 8)&0x0f);	
#endif			  	
	
		}
		if (page_addr != -1) 
		{
			write_addr_reg(nand, (unsigned char)(page_addr )& 0xff);	

			/* One more address cycle for devices > 128MiB */
//			if (this->chipsize > (128 << 20))
			if (this->chipsize >= (64 << 20))		// mhkuo@20121204
			{
				write_addr_reg(nand, (unsigned char) (page_addr>>8 )& 0xff);
				write_addr_reg(nand, (unsigned char) ((page_addr>>16 )& 0xff)|0x80000000);
			}
			else
				write_addr_reg(nand, (unsigned char) ((page_addr>>8 )& 0xff)|0x80000000);
		} 
		else
			write_addr_reg(nand, (unsigned char) ((page_addr>>8 )& 0xff)|0x80000000);
	}

	/*
	 * program and erase have their own busy handlers
	 * status, sequential in, and deplete1 need no delay
	 */
	switch (command) {
		case NAND_CMD_PAGEPROG:
			if (!mtd_fmiSMCheckRB(nand))
					printk("check RB error\n");

		case NAND_CMD_CACHEDPROG:		
		case NAND_CMD_ERASE1:
		case NAND_CMD_ERASE2:
		case NAND_CMD_SEQIN:
		case NAND_CMD_RNDIN:
		case NAND_CMD_STATUS:
		case NAND_CMD_DEPLETE1:
			goto out;
	
		/*
		 * read error status commands require only a short delay
		 */
		case NAND_CMD_STATUS_ERROR:
		case NAND_CMD_STATUS_ERROR0:
		case NAND_CMD_STATUS_ERROR1:
		case NAND_CMD_STATUS_ERROR2:
		case NAND_CMD_STATUS_ERROR3:
					udelay(this->chip_delay);
					goto out;
		case NAND_CMD_RESET:
					if (this->dev_ready)
						break;
					udelay(this->chip_delay);		
					w55fa93_nand_reset(nand);
					goto out;
	 
		 case NAND_CMD_RNDOUT:
	 	     if(mtd->writesize == 0x800)
		     write_cmd_reg(nand, NAND_CMD_RNDOUTSTART);

		     goto out;

		case NAND_CMD_READ0:
		case NAND_CMD_READ1:
			/* Begin command latch cycle */
			if(mtd->writesize == 0x800)		
			{
	#if defined(ONBOARD_NAND)		
			  	writel(0x400, REG_SMISR);//jhe
	#endif		
	#if defined (NANDCARD_NAND)
			  	writel(0x800, REG_SMISR);//jhe
	#endif		
				write_cmd_reg(nand, NAND_CMD_READSTART);
				if (!mtd_fmiSMCheckRB(nand))
						printk("check RB error\n");
			}						
			else if(mtd->writesize == 0x200)					
			{
				if (!mtd_fmiSMCheckRB(nand))
						printk("check RB error\n");
			}				
			
		/* This applies to read commands */
			if (!this->dev_ready) {
				udelay (this->chip_delay);
			}
				break;
		case NAND_CMD_READOOB:
			if(mtd->writesize == 0x800)		
			{		
				if (!mtd_fmiSMCheckRB(nand))
					printk("check RB error\n");
			}					
			else if(mtd->writesize == 0x200)		
			{		
				if (!mtd_fmiSMCheckRB(nand))
					printk("check RB error\n");
			}					
				
			if (!this->dev_ready) {
				udelay (this->chip_delay);
			}
			break;

		default:
			/*
			 * If we don't have access to the busy pin, we apply the given
			 * command delay
			*/
			if (!this->dev_ready) {
				udelay (this->chip_delay);
				break;
			}
		}

out:		
		up(&fmi_sem);
	
		/* Apply this short delay always to ensure that we do wait tWB in
		 * any case on any machine. */
		ndelay (100);
	
//		LEAVE();
		//nand_wait_ready(mtd);
}



static void w55fa93_nand_enable(struct w55fa93_nand *nand)
{
	unsigned int val;

	ENTER();
		
	if(down_interruptible(&fmi_sem)) //jhe+ 2010.12.21
	{
		printk("w55fa93 mtd nand driver nand_enable sem error\n");
		return;
	}
	
	spin_lock(&nand->lock);
	
	writel( readl(REG_AHBCLK)|SIC_CKE|NAND_CKE, REG_AHBCLK);

#if defined(ONBOARD_NAND)		
	writel( readl(REG_GPEFUN)|0x00F30000, REG_GPEFUN);	// enable NAND ALE/CLE/CS0 pins 	
	writel( readl(REG_GPDFUN)|0x0003CC00, REG_GPDFUN);	// enable NAND RB0 pins 		

//	printk("REG_GPDFUN = 0x%x   !!!\n", readl(REG_GPDFUN));			
//	writel( readl(REG_GPDFUN)|0x000003FF, REG_GPDFUN);	// enable ICE pins
	
#endif		

#if defined (NANDCARD_NAND)
	writel( readl(REG_GPEFUN)|0x00FF0000, REG_GPEFUN);	// enable NAND ALE/CLE/CS1 pins 	
	writel( readl(REG_GPDFUN)|0x0003F000, REG_GPDFUN);	// enable NAND RB1 pins 		
#endif		
	
	//writel(RESET_FMI, REG_FMICR);

	val = readl(REG_FMICR);

	//if (!(val & NAND_EN))
		writel(NAND_EN, REG_FMICR);

		writel(0x3050b, REG_SMTCR);
//  	writel(0x020305, REG_SMTCR);

  
	val = readl(REG_SMCSR);

	//val &= ~(SWRST|PSIZE|DMARWEN|BUSWID|ECC4EN|NANDCS);
	//val |= WP;

	//writel(val, REG_SMCSR);

#if defined(ONBOARD_NAND)		
//	writel((val&0xf8ffffc0)|0x04000020, REG_SMCSR);
	writel((val&~0x06000000)|0x04000000, REG_SMCSR);
#endif		

#if defined (NANDCARD_NAND)
//	writel((val&0xf8ffffc0)|0x02000020, REG_SMCSR);
	writel((val&~0x06000000)|0x02000000, REG_SMCSR);
#endif		

	// NAND initialize
	writel(readl(REG_SMCSR) | SMCR_SM_SWRST, REG_SMCSR); 		// software reset	
//	writel(readl(REG_SMCSR)&(~SMCR_ECC_EN), REG_SMCSR);				// ECC disable
	writel(readl(REG_SMCSR) | SMCR_ECC_EN, REG_SMCSR);				// ECC enable
	writel(readl(REG_SMCSR)&(~SMCR_BCH_TSEL)|BCH_T4, REG_SMCSR);	// BCH T4 selected
	writel(readl(REG_SMCSR)&(~SMCR_PSIZE)|PSIZE_2K, REG_SMCSR);		// page size = 2KB 
	writel(readl(REG_SMCSR)&(~SMCR_ECC_3B_PROTECT), REG_SMCSR);		// ECC doesn't protect redundant 3 bytes
	writel(readl(REG_SMCSR)|SMCR_ECC_CHK, REG_SMCSR);				// ECC parity check enable bit during read page
	writel(readl(REG_SMCSR)&(~SMCR_REDUN_AUTO_WEN), REG_SMCSR);		// Redundant auto write enable		
	writel(0x40, REG_SMREAREA_CTL);									// Redundant area size given (can be set 
																	// automatically when PSIZE given)
																	
	g_SMCR_mtd = readl(REG_SMCSR);
	
	// DMAC enable 
  	writel(readl(REG_DMACCSR) | DMAC_EN, REG_DMACCSR);	  		  		  	
  	writel(readl(REG_DMACCSR) | DMAC_SWRST, REG_DMACCSR);	  		  	
  	writel(readl(REG_DMACCSR) &~DMAC_SWRST, REG_DMACCSR);	  		  		  	

	spin_unlock(&nand->lock);
	up(&fmi_sem);
	LEAVE();	
}
static int w55fa93_nand_calculate_ecc(struct mtd_info *mtd, const u_char *dat,
				      u_char *ecc_code)
{
	struct nand_chip *chip = mtd->priv;
	unsigned int ecc_value, ii, jj;
	
//	return;
	ENTER();	
	if(down_interruptible(&fmi_sem)) //jhe+ 2010.12.21
	{
		printk("w55fa93 mtd nand driver nand_enable sem error\n");
		return 0;
	}
	
	ecc_code[0] = 0x12;		// for spare area byte-2	(see "nand_ecclayout" struct in "nand_base.c")
	ecc_code[1] = 0x34;		// for spare area byte-3 
		
	for (ii=0; ii<8; ii++)
	{
		ecc_value = readl(REG_SMRA_8 + ii*4);		
		for (jj=0; jj<4; jj++)		
			ecc_code[2+ii*4+jj] = (ecc_value >> (8*jj)) & 0xFF;			
	}		
	up(&fmi_sem);

	LEAVE();	
#if 0
	printk("w55fa93_nand_calculate_hwecc(%p,): 0x%02x 0x%02x 0x%02x\n",
	       mtd , ecc_code[0], ecc_code[1], ecc_code[2]);
	printk("w55fa93_nand_calculate_hwecc(%p,): 0x%02x 0x%02x 0x%02x\n",
	       mtd , ecc_code[3], ecc_code[4], ecc_code[5]);
	printk("w55fa93_nand_calculate_hwecc(%p,): 0x%02x 0x%02x 0x%02x\n",
	       mtd , ecc_code[6], ecc_code[7], ecc_code[8]);
	printk("w55fa93_nand_calculate_hwecc(%p,): 0x%02x 0x%02x 0x%02x\n",
	       mtd , ecc_code[9], ecc_code[10], ecc_code[11]);
	printk("w55fa93_nand_calculate_hwecc(%p,): 0x%02x 0x%02x 0x%02x\n",
	       mtd , ecc_code[12], ecc_code[13], ecc_code[14]);
	printk("w55fa93_nand_calculate_hwecc(%p,): 0x%02x 0x%02x 0x%02x\n",
	       mtd , ecc_code[15], ecc_code[16], ecc_code[17]);
	printk("w55fa93_nand_calculate_hwecc(%p,): 0x%02x 0x%02x 0x%02x\n",
	       mtd , ecc_code[18], ecc_code[19], ecc_code[20]);
	printk("w55fa93_nand_calculate_hwecc(%p,): 0x%02x 0x%02x 0x%02x\n",
	       mtd , ecc_code[21], ecc_code[22], ecc_code[23]);
	printk("w55fa93_nand_calculate_hwecc(%p,): 0x%02x 0x%02x 0x%02x\n",
	       mtd , ecc_code[24], ecc_code[25], ecc_code[26]);
	printk("w55fa93_nand_calculate_hwecc(%p,): 0x%02x 0x%02x 0x%02x\n",
	       mtd , ecc_code[27], ecc_code[28], ecc_code[29]);
	printk("w55fa93_nand_calculate_hwecc(%p,): 0x%02x 0x%02x 0x%02x\n",
	       mtd , ecc_code[30], ecc_code[31], ecc_code[32]);
	printk("w55fa93_nand_calculate_hwecc(%p,): 0x%02x 0x%02x 0x%02x\n",
	       mtd , ecc_code[33], ecc_code[34], ecc_code[36]);
	printk("w55fa93_nand_calculate_hwecc(%p,): 0x%02x 0x%02x 0x%02x\n",
	       mtd , ecc_code[37], ecc_code[38], ecc_code[39]);
	       
	       
#endif     	       	       
return 0;
}
/*
 * HW ECC Correction
 *
 * function called after a read
 *
 * mtd:        MTD block structure
 * dat:        raw data read from the chip
 * read_ecc:   ECC from the chip (unused)
 * isnull:     unused
 *
 * Detect and correct a 1 bit error for a page
 */
static int w55fa93_nand_correct_data(struct mtd_info *mtd, u_char *dat,
				     u_char *read_ecc, u_char *calc_ecc)
{
	#if 1
		return 0;
	#else
		//struct nand_chip *nand_chip = mtd->priv;
		unsigned int ecc_status;
		if(down_interruptible(&fmi_sem)) //jhe+ 2010.12.21
		{
			printk("w55fa93 mtd nand driver nand_enable sem error\n");
			return 0;
		}
		
		#ifdef DEBUG_NAND
		//printk("nuc900_nand_correct_data\n");
		#endif
		/* get the status from the Status Register */
		ecc_status = readl(REG_SMISR);
	
		/* if there's no error */
		if (likely(!(ecc_status & 0x02))){
			up(&fmi_sem);	
			return 0;
		}
			
	  writel(0x02,REG_SMISR);
	  
	  if (fmiSMCorrectData_2K((unsigned long)dat)==0){
	  		printk(KERN_WARNING "nuvoton_nand : error corrected\n");
	  		up(&fmi_sem);	
		    return 1;
	   }
	  /* 
		* We can't correct so many errors */
		printk(KERN_WARNING "nuvoton_nand : multiple errors detected."
					" Unable to correct.\n");
		up(&fmi_sem);			
		return -EIO;	
	#endif		
}

/*
 * Enable HW ECC : unused on most chips
 */
void w55fa93_nand_enable_hwecc(struct mtd_info *mtd, int mode)
{
	/*
	struct nand_chip *chip = mtd->priv;
	debug("nuc900_nand_enable_hwecc(%p, %d)\n", mtd, mode);
	
	*/
}


struct w55fa93_nand *w55fa93_nand;
static int __devinit w55fa93_nand_probe(struct platform_device *pdev)
{
	
	struct nand_chip *chip;
	struct mtd_info *mtd;	
	int retval;
	struct resource *res;

	ENTER();
	
	s_nand_vaddr = (unsigned char *) dma_alloc_writecombine(NULL, 512*8, (dma_addr_t *) &s_nand_phyaddr, GFP_KERNEL);
	if(s_nand_vaddr == NULL){
		printk(KERN_ERR "fa93_nand: failed to allocate ram for nand data.\n");
		return -ENOMEM;
	} 	
	
	retval = 0;

	w55fa93_nand = kzalloc(sizeof(struct w55fa93_nand), GFP_KERNEL);
	if (!w55fa93_nand)
		return -ENOMEM;
		
	mtd=&w55fa93_nand->mtd;		
	chip = &(w55fa93_nand->chip);

	w55fa93_nand->mtd.priv	= chip;
	w55fa93_nand->mtd.owner	= THIS_MODULE;
	spin_lock_init(&w55fa93_nand->lock);

    /*
     * Get Clock
     */
//	pdev = (struct platform_device *)&w55fa93_device_fmi;
//  w55fa93_nand->sic_clk = clk_get(&pdev->dev, NULL);
    w55fa93_nand->sic_clk = clk_get(NULL, "SIC");

    if (IS_ERR(w55fa93_nand->sic_clk)) {
    //    ret = -ENODEV;
        printk(&pdev->dev, "no sic_clk?\n");
        goto fail2;
    }
    
    w55fa93_nand->nand_clk = clk_get(NULL, "NAND");
    if (IS_ERR(w55fa93_nand->nand_clk)) {
    //        ret = -ENODEV;
            printk(&pdev->dev, "no nand_clk?\n");
            goto fail2;
    }
    
	clk_enable(w55fa93_nand->sic_clk);
	clk_enable(w55fa93_nand->nand_clk);
	
	chip->cmdfunc		= w55fa93_nand_command_lp;
	chip->dev_ready		= w55fa93_nand_devready;
	chip->read_byte		= w55fa93_nand_read_byte;
	chip->write_buf		= w55fa93_nand_write_buf;
	chip->read_buf		= w55fa93_nand_read_buf;
	chip->select_chip  	= w55fa93_nand_select_chip;
	chip->verify_buf	= w55fa93_verify_buf;
	chip->chip_delay	= 50;
	chip->options		= 0;

#ifdef OPT_HW_ECC
	chip->ecc.mode 		= NAND_ECC_HW;
#else
	chip->ecc.mode		= NAND_ECC_SOFT;
#endif	

	chip->ecc.calculate = w55fa93_nand_calculate_ecc;
	chip->ecc.correct 	= w55fa93_nand_correct_data;
	chip->ecc.hwctl 	= w55fa93_nand_enable_hwecc;

	w55fa93_nand->reg = 0x00;
//	w55fa93_nand->reg = ioremap(W90X900_PA_FMI, W90X900_SZ_FMI);
//	if (!w55fa93_nand->reg) {
//		retval = -ENOMEM;
//		goto fail2;
	//}

#ifdef OPT_HW_ECC

	w55fa93_nand_enable(w55fa93_nand);
	if (nand_scan_ident(mtd, 1, NULL)) {
		retval = -ENXIO;
		goto fail3;
	}
	 chip->ecc.size = mtd->writesize;
	
	// set ECC page size (support yaffs2 file system)
	switch (mtd->writesize) {
	case 2048:
		chip->ecc.bytes = 34;		// 2 + 32 (ECC4)
		break;
	default:
	#if 0	
		/* page size not handled by HW ECC */
		/* switching back to soft ECC */
		chip->ecc.mode = NAND_ECC_SOFT;
		chip->ecc.calculate = NULL;
		chip->ecc.correct = NULL;
		chip->ecc.hwctl = NULL;
		chip->ecc.read_page = NULL;
		chip->ecc.postpad = 0;
		chip->ecc.prepad = 0;
		chip->ecc.bytes = 0;
	#endif		
		break;
	}
	
	/* second phase scan */
	if (nand_scan_tail(mtd)) {
		retval = -ENXIO;
		goto fail3;
	}

#else
	w55fa93_nand_enable(w55fa93_nand);
	if (nand_scan(&(w55fa93_nand->mtd), 1)) {
		retval = -ENXIO;
		//dump_regs(w55fa93_nand);
		goto fail3;
	}
#endif	
	add_mtd_partitions(&(w55fa93_nand->mtd), partitions,
						ARRAY_SIZE(partitions));
	s_nand_init = 1;
	return retval;

//fail3:	iounmap(w55fa93_nand->reg);
fail3:	
fail2:	
fail1:	kfree(w55fa93_nand);
	dma_free_coherent(NULL, 512*8, s_nand_vaddr, (dma_addr_t )s_nand_phyaddr);	
printk("Nand init fail\n");
	LEAVE();
	return retval;
}

static int __devexit w55fa93_nand_remove(struct platform_device *pdev)
{
	ENTER();	
//	iounmap(w55fa93_nand->reg);
	dma_free_coherent(NULL, 512*8, s_nand_vaddr, (dma_addr_t )s_nand_phyaddr);	
	kfree(w55fa93_nand);

	return 0;
}

static struct platform_driver w55fa93_nand_driver = {
        .driver	= {
        	.name	= "w55fa93-fmi",
            .owner	= THIS_MODULE,
        },
        .probe		= w55fa93_nand_probe,
        .remove		= __devexit_p(w55fa93_nand_remove),
};

#ifdef NANDCARD_NAND
	int w55fa93_nand_init(void)
	{
		ENTER();	
		printk("MTD nand init !!!\n");
		w55fa93_nand_probe(NULL);
		return 0;
	}
	
	void w55fa93_nand_exit(void)
	{
		ENTER();	
		w55fa93_nand_remove(NULL);
	}
	
	module_init(w55fa93_nand_init);
	module_exit(w55fa93_nand_exit);
	
	//EXPORT_SYMBOL_GPL(w55fa93_nand_init);
	//EXPORT_SYMBOL_GPL(w55fa93_nand_exit);

#else
	// static int __init w55fa93_nand_init(void)
	int w55fa93_nand_init(void)
	{
		ENTER();	
		printk("MTD nand init !!!\n");
		if (!s_nand_init)
		{
		//	s_nand_init = 1;
			w55fa93_nand_probe(NULL);
		}		
		return 0;
	}
	
	// static void __exit w55fa93_nand_exit(void)
	void w55fa93_nand_exit(void)
	{
		ENTER();	
		w55fa93_nand_remove(NULL);
		s_nand_init = 0;	
	}
	
	module_init(w55fa93_nand_init);
	module_exit(w55fa93_nand_exit);
	
	//EXPORT_SYMBOL_GPL(w55fa93_nand_init);
	//EXPORT_SYMBOL_GPL(w55fa93_nand_exit);

#endif

MODULE_AUTHOR("xxx <xxx@gmail.com>");
MODULE_DESCRIPTION("w55fa93 nand driver!");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:w55fa93-fmi");
