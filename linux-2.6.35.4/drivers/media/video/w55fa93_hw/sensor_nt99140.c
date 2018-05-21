/* sensor.c
 *
 * Copyright (c) 2008 Nuvoton technology corporation
 * All rights reserved.
 * <clyu2@nuvoton.com>
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */
 
#include <linux/config.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/smp_lock.h>
#include <linux/vmalloc.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/videodev.h>
#include <linux/jiffies.h>
#include <asm/arch/videoin.h>
//#include <asm/arch/DrvVideoin.h>
#include "DrvVideoin.h"
//#include <asm/arch/DrvFsc.h>
#include <asm/arch/w55fa93_reg.h>
#include <asm/arch/fb.h>
#include <asm/arch/w55fa93_fb.h>
#include <asm/arch/w55fa93_gpio.h>

#include <asm/io.h>
#include <linux/i2c.h>

#include "videoinpriv.h"
#include "DrvI2C.h"


//#define LCDWIDTH	480
//#define LCDHEIGHT	272
//#define LCDBPP		16

//#define __STANDARD_I2C__

#define CONFIG_ARCH_W55FA93_DEMOBOARD
//IMPORT_SYMBOL(w55fa93_FB_BG_PHY_ADDR);
extern unsigned int w55fa93_FB_BG_PHY_ADDR;

#define DrvVideoIn_nt99140	1
#define CHIP_VERSION_H	0x3000
#define CHIP_VERSION_L	0x3001
#define CHIP_ID	0x14

#define _REG_TABLE_SIZE(nTableName)	sizeof(nTableName)/sizeof(struct NT_RegValue)

#define ERR_PRINTF			printk
#define outp32(addr, value)		outl(value, addr)
#define inp32(addr)			inl(addr)
//#define DBG_PRINTF	printk
#define DBG_PRINTF(...)
//extern videoIn_buf_t videoIn_preview_buf[];

#define REG_VALUE_INIT	0
#define REG_VALUE_HD720	1	//1280X720
#define REG_VALUE_SVGA	2	//720X480
#define REG_VALUE_VGA	3	//640X480

struct NT_RegValue{
	__u16	uRegAddr;
	__u8	uValue;
};

struct NT_RegTable{
	struct OV_RegValue *sRegTable;
	__u16 uTableSize;
};


struct NT_RegValue g_sNT99140_Init[] = 
{
	{0x32F0,0x01},
	{0x3028,0x05}, 
	{0x3029,0x02}, 
	{0x302a,0x00}, 

	//{0x306a,0x02},	
	{0x306a,0x01},
		
	{0x3336,0x14},
	{0x3210,0x04},
	{0x3211,0x04},
	{0x3212,0x04},
	{0x3213,0x04},
	{0x3214,0x04},
	{0x3215,0x04},
	{0x3216,0x04},
	{0x3217,0x04},
	{0x3218,0x04},
	{0x3219,0x04},
	{0x321A,0x04},
	{0x321B,0x04},
	{0x321C,0x04},
	{0x321D,0x04},
	{0x321E,0x04},
	{0x321F,0x04},
	{0x3230,0x1D},
	{0x3231,0x00},
	{0x3232,0x04},
	{0x3233,0x30},	
	{0x3234,0x42},
	{0x3270,0x00}, 
	{0x3271,0x0e},
	{0x3272,0x1a},
	{0x3273,0x31},
	{0x3274,0x4B},
	{0x3275,0x5D},
	{0x3276,0x7E},
	{0x3277,0x94},
	{0x3278,0xA6},
	{0x3279,0xB6},
	{0x327A,0xcc},
	{0x327B,0xde},
	{0x327C,0xeb},
	{0x327D,0xF6},
	{0x327E,0xFF},
	{0x3302,0x00},
	{0x3303,0x1D},
	{0x3304,0x00},
	{0x3305,0x97},
	{0x3306,0x00},
	{0x3307,0x4C},
	{0x3308,0x07},
	{0x3309,0xF2},
	{0x330A,0x06},
	{0x330B,0x8F},
	{0x330C,0x01},
	{0x330D,0x7F},
	{0x330E,0x00},
	{0x330F,0xB9},
	{0x3310,0x07},
	{0x3311,0x96},
	{0x3312,0x07},
	{0x3313,0xB2},
	{0x3300,0x70},
	{0x3301,0x40},
	{0x3024,0x00},
	{0x3040,0x04},
	{0x3041,0x02},
	{0x3042,0xFF},
	{0x3043,0x14},
	{0x3052,0xd0},
	{0x3057,0x80},
	{0x3058,0x00},
	{0x3059,0x2F},
	{0x305f,0x22},
	{0x32b0,0x00},
	{0x32b1,0x90},
	{0x32BB,0x0b},
	{0x32bd,0x10},
	{0x32be,0x05},
	{0x32bf,0x4a},
	{0x32c0,0x40},
	{0x32C3,0x08},
	{0x32c5,0x1f},
	{0x32cd,0x01},
	{0x32d3,0x00},
	{0x32f6,0x0c},
	{0x3324,0x00},
	{0x3118,0xF2},
	{0x3119,0xF2},
	{0x311A,0x13},
	{0x3106,0x01},
	{0x3108,0x55},
	{0x3105,0x41},
	{0x3112,0x21},
	{0x3113,0x55},
	{0x3114,0x05},
	{0x333b,0x20},
	{0x333c,0x28},
	{0x3320,0x20},
	{0x3335,0x01},
	{0x3200,0x3e},     
	{0x3201,0x3f},     
	{0x3344,0x28},
	{0x3345,0x30},
	{0x3346,0x30},
	{0x3348,0x00},
	{0x3349,0x40},
	{0x334a,0x30},
	{0x334b,0x00},
	{0x334d,0x15},
	{0x329b,0x01},
	{0x32a1,0x01},
	{0x32a2,0x40},
	{0x32a3,0x01},
	{0x32a4,0xc0},
	{0x32a5,0x01},
	{0x32a6,0x40},
	{0x32a7,0x02},
	{0x32a8,0x10},
	{0x32a9,0x11},
	{0x3054,0x05},
};

struct NT_RegValue g_sNT99140_HD720[] = 
{
	{0x3028, 0x0D}, 	//56MHz
	{0x3029, 0x02}, 
	{0x302a, 0x00}, 
	{0x3022, 0x24}, 
	{0x3023, 0x24}, 

	{0x3002, 0x00}, 
	{0x3003, 0x04}, // start x 
	{0x3004, 0x00}, 
	{0x3005, 0x04}, // start y
	{0x3006, 0x05}, 
	{0x3007, 0x03}, // end x 
	{0x3008, 0x02}, 
//	{0x3009, 0xd3}, // end y
	{0x3009, 0xd5}, // end y

	{0x300a, 0x07}, 
	{0x300b, 0x12}, 
	{0x300c, 0x02}, 
//	{0x300d, 0xe0}, // 736 
	{0x300d, 0xe2}, // 738
	{0x300e, 0x05}, 
	{0x300f, 0x00}, 
	{0x3010, 0x02}, 
//	{0x3011, 0xd0}, //720 
	{0x3011, 0xd2}, //722 

	{0x32b0, 0x00}, 
	{0x32b1, 0x00}, 
	{0x32b2, 0x01}, 
	{0x32b3, 0x80}, 
	{0x32b4, 0x00}, 
	{0x32b5, 0x68}, 
	{0x32b6, 0x99}, 
	{0x32bb, 0x1b}, 
	{0x32bc, 0x40}, 
	{0x32c1, 0x23}, 
	{0x32c2, 0x05}, 
	{0x32c8, 0x4d}, 
	{0x32c9, 0x40}, 
	{0x32c4, 0x00}, 
	{0x3201, 0x3f}, 
	{0x3021, 0x06}, 
	{0x3060, 0x01}


};


struct NT_RegValue g_sNT99140_SVGA[] = 
{
	{0x32e0, 0x02}, 
	{0x32e1, 0xd0}, 
	{0x32e2, 0x01}, 
	{0x32e3, 0xe2}, //482
	{0x32e4, 0x00}, 
	{0x32e5, 0x55}, 
	{0x32e6, 0x00}, 
	{0x32e7, 0x56}, 
	{0x3028, 0x0D}, 	
	{0x3029, 0x02}, 
	{0x302a, 0x00}, 
	{0x3022, 0x24}, 
	{0x3023, 0x24}, 
	// 1080x720 ---> 720x480
	{0x3002, 0x00}, 
	{0x3003, 0x68}, //start x 104 
	{0x3004, 0x00}, 
	{0x3005, 0x04}, //start y 4 
	{0x3006, 0x04}, 
	{0x3007, 0x9f}, //end x 1183
	{0x3008, 0x02}, 
	{0x3009, 0xd5}, //end y 725 
	{0x300a, 0x05},
	{0x300b, 0x70}, //line length  1080 + 312
	{0x300c, 0x02}, 
	{0x300d, 0xe2}, //frame length 720 + 16
	{0x300e, 0x04}, 
	{0x300f, 0x38}, //1080
	{0x3010, 0x02}, 
	{0x3011, 0xd2}, //722

	{0x32b0, 0x00}, 
	{0x32b1, 0x00}, 
	{0x32b2, 0x00}, 
	{0x32b3, 0xe0}, 
	{0x32b4, 0x00}, 
	{0x32b5, 0xc0}, 
	{0x32b6, 0x98}, 
	{0x32bb, 0x1b}, 
	{0x32bc, 0x40}, 
	{0x32c1, 0x22}, 
	{0x32c2, 0x94}, 
	{0x32c8, 0x6e}, 
	{0x32c9, 0x5c}, 
	{0x32c4, 0x00}, 
	{0x3201, 0x7f}, 
	{0x3021, 0x06}, 
	{0x3060, 0x01}, 
};

struct NT_RegValue g_sNT99140_VGA[] = 
{
	{0x32e0, 0x02}, 
	{0x32e1, 0x80}, 
	{0x32e2, 0x01}, 
	{0x32e3, 0xe2},  //482 
	{0x32e4, 0x00}, 
	{0x32e5, 0x80}, 
	{0x32e6, 0x00}, 
	{0x32e7, 0x80}, 
	{0x3028, 0x0D}, 	
	{0x3029, 0x02}, 
	{0x302a, 0x00}, 
	{0x3022, 0x24}, 
	{0x3023, 0x24}, 

	// 960x722 ---> 640x482
	{0x3002, 0x00}, 
	{0x3003, 0xa4}, //start x 164 
	{0x3004, 0x00}, 
	{0x3005, 0x04}, //start y 4 
	{0x3006, 0x04}, 
	{0x3007, 0x63}, //end x  1123
	{0x3008, 0x02}, 
	{0x3009, 0xd5}, //end y  725
	{0x300a, 0x04}, 
	{0x300b, 0xf8}, //line length  960 + 312
	{0x300c, 0x02}, 
	{0x300d, 0xe2}, //frame length 720 + 16 
	{0x300e, 0x03}, 
	{0x300f, 0xc0}, // 960 
	{0x3010, 0x02}, 
	{0x3011, 0xd2}, // 722 

	{0x32b0, 0x00}, 
	{0x32b1, 0x00}, 
	{0x32b2, 0x00}, 
	{0x32b3, 0xe0}, 
	{0x32b4, 0x00}, 
	{0x32b5, 0x68}, 
	{0x32b6, 0x99}, 
	{0x32bb, 0x1b}, 
	{0x32bc, 0x40}, 
	{0x32c1, 0x22}, 
	{0x32c2, 0x94}, 
	{0x32c8, 0x6e}, 
	{0x32c9, 0x5c}, 
	{0x32c4, 0x00}, 
	{0x3201, 0x7f}, 
//	{0x3201, 0x3f}, 

	{0x3021, 0x06}, 
	{0x3060, 0x01}, 	



};

struct NT_RegTable g_NT99140_InitTable[] =
{
	{g_sNT99140_Init,_REG_TABLE_SIZE(g_sNT99140_Init)},
	{g_sNT99140_HD720,_REG_TABLE_SIZE(g_sNT99140_HD720)},
	{g_sNT99140_SVGA,_REG_TABLE_SIZE(g_sNT99140_SVGA)},
	{g_sNT99140_VGA,_REG_TABLE_SIZE(g_sNT99140_VGA)},		
	{0,0}
};

__u8 g_uOvDeviceID= 0x54;	// nt99140

static struct i2c_client *save_client;
static int sensor_detected;
static unsigned short ignore[] = { I2C_CLIENT_END };
static unsigned short normal_addr[] = { 0x2a, I2C_CLIENT_END };


static struct i2c_client_address_data addr_data = {
	.normal_i2c = normal_addr,
	.probe = ignore,
	.ignore = ignore,
};

static int sensor_i2c_probe(struct i2c_adapter *adap, int addr, int kind);

static int sensor_i2c_attach(struct i2c_adapter *adap)
{	
	DBG_PRINTF("%s\n",__FUNCTION__);	
	sensor_detected = i2c_probe(adap, &addr_data, sensor_i2c_probe);	
	return sensor_detected;
}

static int sensor_i2c_detach(struct i2c_client *client)
{
	int rc;
	DBG_PRINTF("%s\n",__FUNCTION__);	
	if ((rc = i2c_detach_client(client)) == 0) {
		kfree(i2c_get_clientdata(client));		
	}
	return rc;
}

static struct i2c_driver sensor_i2c_driver = {
	.driver = {
		.name	= "Sensor I2C",
	},
	.id = 0x2a,	
	.attach_adapter = sensor_i2c_attach,
	.detach_client = sensor_i2c_detach,
};

static int sensor_i2c_probe(struct i2c_adapter *adap, int addr, int kind)
{
	struct i2c_client *client;
	int rc;
	//DBG_PRINTF("%s\n",__FUNCTION__);		
	client = kzalloc(sizeof(struct i2c_client), GFP_KERNEL);
	if (!client)
		return -ENOMEM;

	strncpy(client->name, "Sensor I2C", 6);
	client->addr = addr;
	client->adapter = adap;
	client->driver = &sensor_i2c_driver;

	if ((rc = i2c_attach_client(client)) != 0) {
		kfree(client);
		return rc;
	}
	
	save_client = client;
		
	return 1;
}

BOOL I2C_Write_8bitSlaveAddr_16bitReg_8bitData(UINT8 uAddr, UINT16 uRegAddr, UINT8 uData)
{
	// 3-Phase(ID address, regiseter address, data(8bits)) write transmission
	volatile u32Delay = 0x100;
	DrvI2C_SendStart();
	while(u32Delay--);		
	if ( (DrvI2C_WriteByte(uAddr,DrvI2C_Ack_Have,8)==FALSE) ||			// Write ID address to sensor
		 (DrvI2C_WriteByte((UINT8)(uRegAddr>>8),DrvI2C_Ack_Have,8)==FALSE) ||	// Write register address to sensor
		 (DrvI2C_WriteByte((UINT8)(uRegAddr&0xff),DrvI2C_Ack_Have,8)==FALSE) ||	// Write register address to sensor
		 (DrvI2C_WriteByte(uData,DrvI2C_Ack_Have,8)==FALSE) )		// Write data to sensor
	{
		DBG_PRINTF("wnoack \n");
		DrvI2C_SendStop();
		return FALSE;
	}
	DrvI2C_SendStop();

	if (uRegAddr==0x12 && (uData&0x80)!=0)
	{
		mdelay(20);			
	}
	return TRUE;
}

UINT8 I2C_Read_8bitSlaveAddr_16bitReg_8bitData(UINT8 uAddr, UINT16 uRegAddr)
{
	UINT8 u8Data;
	
	// 2-Phase(ID address, register address) write transmission
	DrvI2C_SendStart();
	DrvI2C_WriteByte(uAddr,DrvI2C_Ack_Have,8);		// Write ID address to sensor
	DrvI2C_WriteByte((UINT8)(uRegAddr>>8),DrvI2C_Ack_Have,8);	// Write register address to sensor
	DrvI2C_WriteByte((UINT8)(uRegAddr&0xff),DrvI2C_Ack_Have,8);	// Write register address to sensor	
	DrvI2C_SendStop();

	// 2-Phase(ID-address, data(8bits)) read transmission
	DrvI2C_SendStart();
	DrvI2C_WriteByte(uAddr|0x01,DrvI2C_Ack_Have,8);		// Write ID address to sensor
	u8Data = DrvI2C_ReadByte(DrvI2C_Ack_Have,8);		// Read data from sensor
	DrvI2C_SendStop();
	
	return u8Data;

}

s8  DrvVideoIn_I2cWriteNT(__u8 uAddr, __u16 uRegAddr, __u8 uData)
{
	//DBG_PRINTF("%s\n",__FUNCTION__);	
#ifdef __STANDARD_I2C__
	struct i2c_msg msg;
	u8 buf[3];
	int ret=-1;
	
	msg.flags=!I2C_M_RD;
	msg.addr=save_client->addr;
	msg.len=3;
	msg.buf=buf;		

	buf[0]=(u8)(uRegAddr>>8);
	buf[1]=(u8)(uRegAddr&0xff);
	buf[2]=uData;

	ret=i2c_transfer(save_client->adapter,&msg,1);
	return ret;
		
#else
	if(I2C_Write_8bitSlaveAddr_16bitReg_8bitData(uAddr, uRegAddr, uData)==TRUE)
		return (0);
	else	
		return -EBUSY;
#endif
		
}


__s8  DrvVideoIn_I2cReadNT(__u8 uAddr, __u16 uRegAddr)
{
#ifdef __STANDARD_I2C__
	struct i2c_msg msgs;
	int ret=-1;
	u8 buf[3];
	
	msgs.flags=!I2C_M_RD;
	msgs.addr=save_client->addr;
	msgs.len=2;
	msgs.buf=buf;
	buf[0]=(u8)(uRegAddr>>8);
	buf[1]=(u8)(uRegAddr&0xff);

	ret=i2c_transfer(save_client->adapter,&msgs,1);
	
	msgs.flags=I2C_M_RD;
	msgs.addr=save_client->addr;
	msgs.len=1;
	msgs.buf=buf;

	ret=i2c_transfer(save_client->adapter,&msgs,1);
	return buf[0];
#else
	return I2C_Read_8bitSlaveAddr_16bitReg_8bitData(uAddr,uRegAddr);
#endif
}

void I2C_Delay(UINT32 u32Delay)
{
	volatile UINT32 i;
	for(;u32Delay!=0;u32Delay--)
		for(i=0;i<5;i++);
}

void NTSetResolution(int index)
{
  	__u32 i;
	__u16 uTableSize;
	__u8  uDeviceID;
	__u8 id0;
	__u16 ExprosureH,ExprosureL,Exprosure;
	__u8 AEstep,Step;
	
	struct NT_RegValue *psRegValue;

	DBG_PRINTF("NTSetResolution:%d\n",index);

	if(index>REG_VALUE_VGA)
		return ;
	
	uTableSize = g_NT99140_InitTable[index].uTableSize;
	psRegValue = g_NT99140_InitTable[index].sRegTable;
	uDeviceID = g_uOvDeviceID;
	if ( psRegValue == 0 ){
		DBG_PRINTF("NTRegConfig psRegValue == 0");
		return;	
	}

	if(index!=REG_VALUE_INIT){
		//setting exporsure time
		ExprosureH = DrvVideoIn_I2cReadNT(uDeviceID,0x3012);
		ExprosureL = DrvVideoIn_I2cReadNT(uDeviceID,0x3013);
		AEstep = DrvVideoIn_I2cReadNT(uDeviceID,0x32c8);
		Exprosure = (ExprosureH<<8) | ExprosureL;
		Step = Exprosure / (AEstep*2);	
	}
	
	for(i=0;i<uTableSize; i++, psRegValue++)
	{
		udelay(10);
		DrvVideoIn_I2cWriteNT(uDeviceID, (psRegValue->uRegAddr), (psRegValue->uValue));
#if 0
		//mdelay(10);
		id0=DrvVideoIn_I2cReadNT(uDeviceID,(psRegValue->uRegAddr));
		if(id0!=(psRegValue->uValue)){
			DBG_PRINTF("reg=0x%04x w=0x%02x r=0x%02x\n",(psRegValue->uRegAddr),(psRegValue->uValue),id0);			
		}	
#endif
	}

	if(index!=REG_VALUE_INIT){
		//setting exporsure time
		DrvVideoIn_I2cWriteNT(uDeviceID, 0x3201,(0xdf&(DrvVideoIn_I2cReadNT(uDeviceID,0x3201))));

		AEstep = DrvVideoIn_I2cReadNT(uDeviceID,0x32c8);
		Exprosure = Step * (AEstep*2);
		ExprosureL = Exprosure & 0x00ff;
		ExprosureH =  Exprosure >> 8;
		DrvVideoIn_I2cWriteNT(uDeviceID, 0x3012,(UINT8)ExprosureH);
		DrvVideoIn_I2cWriteNT(uDeviceID, 0x3013,(UINT8)ExprosureL);
		DrvVideoIn_I2cWriteNT(uDeviceID, 0x3201,(0x20|(DrvVideoIn_I2cReadNT(uDeviceID,0x3201))));
		DrvVideoIn_I2cWriteNT(uDeviceID, 0x3060,0x01);
	}
	
	return ;
}

__s32 NT99140RegConfig(void)
{
  	__u32 i;
	__u16 uTableSize;
	__u8  uDeviceID;
	__u8 id0,id1;
	__s32 ret = 0;

	struct NT_RegValue *psRegValue;
	//DBG_PRINTF("%s\n",__FUNCTION__);	

	//Mark due to sensor reset and power down conflict with Normal UART
	SnrPowerDown(FALSE);
	udelay(10);
  	SnrReset();

#ifdef __STANDARD_I2C__	
	//outl(inl(REG_GPBFUN) | (MF_GPB13 | MF_GPB14), REG_GPBFUN);
	i2c_add_driver(&sensor_i2c_driver);
	if (sensor_detected)
	{
		DBG_PRINTF("Sensor I2C driver installed.\n");
	}
	else
	{
		printk("Failed to install I2C driver for sensor!!!\n");	
		return -EBUSY;
	}	
#else	
#if 0
	DBG_PRINTF("Non Standard I2C.\n");
	DBG_PRINTF("REG_GPBFUN = 0x%x\n", inp32(REG_GPBFUN));
	DrvI2C_Open(eDRVGPIO_GPIOB, 					
				eDRVGPIO_PIN13, 
				eDRVGPIO_GPIOB,
				eDRVGPIO_PIN14, 
				(PFN_DRVI2C_TIMEDELY)I2C_Delay);
#endif

	//GPE11-clk GPE10-data
	DBG_PRINTF("Non Standard I2C.\n");
	DBG_PRINTF("REG_GPEFUN = 0x%x\n", inp32(REG_GPEFUN));
	DrvI2C_Open(eDRVGPIO_GPIOE, 					
				eDRVGPIO_PIN11, 
				eDRVGPIO_GPIOE,
				eDRVGPIO_PIN10, 
				(PFN_DRVI2C_TIMEDELY)I2C_Delay);

#endif
	
	uTableSize = g_NT99140_InitTable[REG_VALUE_INIT].uTableSize;
	psRegValue = g_NT99140_InitTable[REG_VALUE_INIT].sRegTable;
	uDeviceID = g_uOvDeviceID;
	 
	DBG_PRINTF("uDeviceID = 0x%x\n", uDeviceID);
	DBG_PRINTF("REG_GPBFUN = 0x%x\n", inp32(REG_GPBFUN));

	/*check device id*/
	if(1){
		id0=(u8)DrvVideoIn_I2cReadNT(g_uOvDeviceID,CHIP_VERSION_H);
		id1=(u8)DrvVideoIn_I2cReadNT(g_uOvDeviceID,CHIP_VERSION_L);
		DBG_PRINTF("detectd sensor id0=%0x id1=%02x\n",id0,id1);
	}

	/*camera init*/
	if ( psRegValue == 0 ){
		DBG_PRINTF("NTRegConfig psRegValue == 0");
		return;	
	}
	for(i=0;i<uTableSize; i++, psRegValue++)
	{		
		udelay(10);
		ret = DrvVideoIn_I2cWriteNT(uDeviceID, (psRegValue->uRegAddr), (psRegValue->uValue));
		if(ret < 0)
			break;
	}
	return ret;
}


#if defined(CONFIG_ARCH_W55FA93) 
#if 0
void SnrReset(void)
{/* GPA11 reset:	H->L->H */	
	DBG_PRINTF("%s\n",__FUNCTION__);		
	while(1);
	w55fa93_gpio_configure(GPIO_GROUP_A, 11);
	w55fa93_gpio_set(GPIO_GROUP_A, 11, 1);
	w55fa93_gpio_set_output(GPIO_GROUP_A, 11);
	udelay(100);
	w55fa93_gpio_set(GPIO_GROUP_A, 11, 0);	//GPIOA 11 set low
	udelay(100);		
	w55fa93_gpio_set(GPIO_GROUP_A, 11, 1);	//GPIOA 11 set high
}

void SnrPowerDown(BOOL bIsEnable)
{/* GPA10 power down, Low for power down */
	DBG_PRINTF("%s\n",__FUNCTION__);	
	while(1);
	w55fa93_gpio_configure(GPIO_GROUP_A, 10);
	w55fa93_gpio_set(GPIO_GROUP_A, 10, 0);
	w55fa93_gpio_set_output(GPIO_GROUP_A, 10);
	if(bIsEnable)
		w55fa93_gpio_set(GPIO_GROUP_A, 10, 1);		//GPIOA 10 set high
	else			
		w55fa93_gpio_set(GPIO_GROUP_A, 10, 0);	//GPIOA 10 set low	
}
#endif

#if 0
void SnrReset(void)
{/* GPB4 reset:	H->L->H */	
	DBG_PRINTF("%s\n",__FUNCTION__);		
	//while(1);
	w55fa93_gpio_configure(GPIO_GROUP_B, 4);
	w55fa93_gpio_set(GPIO_GROUP_B, 4, 1);
	w55fa93_gpio_set_output(GPIO_GROUP_B, 4);
	mdelay(1);
	w55fa93_gpio_set(GPIO_GROUP_B, 4, 0);	//GPIOA 4 set low
	mdelay(10);		
	w55fa93_gpio_set(GPIO_GROUP_B, 4, 1);	//GPIOA 4 set high
}

void SnrPowerDown(BOOL bIsEnable)
{/* GPB5 power down, Low for power down */
	DBG_PRINTF("%s\n",__FUNCTION__);	
	//while(1);
	w55fa93_gpio_configure(GPIO_GROUP_B, 5);
	w55fa93_gpio_set(GPIO_GROUP_B, 5, 0);
	w55fa93_gpio_set_output(GPIO_GROUP_B, 5);
	if(bIsEnable)
		w55fa93_gpio_set(GPIO_GROUP_B, 5, 1);		//GPIOA 10 set high
	else			
		w55fa93_gpio_set(GPIO_GROUP_B, 5, 0);	//GPIOA 10 set low	
}
#endif

#if 1
void SnrReset(void)
{/* GPE9 reset:	H->L->H */	
	DBG_PRINTF("%s\n",__FUNCTION__);		
	//while(1);
	w55fa93_gpio_configure(GPIO_GROUP_E, 9);
	w55fa93_gpio_set(GPIO_GROUP_E, 9, 1);
	w55fa93_gpio_set_output(GPIO_GROUP_E, 9);
	mdelay(1);
	w55fa93_gpio_set(GPIO_GROUP_E, 9, 0);	//GPIOE 9 set low
	mdelay(10);		
	w55fa93_gpio_set(GPIO_GROUP_E, 9, 1);	//GPIOE 9 set high
}

//GPD7
void SnrPowerDown(BOOL bIsEnable)
{/* GPD7 power down, Low for power down */
	DBG_PRINTF("%s\n",__FUNCTION__);	
	//while(1);
	w55fa93_gpio_configure(GPIO_GROUP_D, 7);
	w55fa93_gpio_set(GPIO_GROUP_D, 7, 0);
	w55fa93_gpio_set_output(GPIO_GROUP_D, 7);
	if(bIsEnable)
		w55fa93_gpio_set(GPIO_GROUP_D, 7, 1);		//GPIOA 10 set high
	else			
		w55fa93_gpio_set(GPIO_GROUP_D, 7, 0);	//GPIOA 10 set low	
}
#endif

#endif
UINT16 vinGCD(UINT16 m1, UINT16 m2)
{
	UINT16 m;
	if(m1<m2)
	{
		m=m1; m1=m2; m2=m;
	}
	if(m1%m2==0)
		return m2;
	else
		return (vinGCD(m2,m1%m2));		
}

extern  unsigned int w55fa93_cpu_clock;

__s32 InitSensor(__u32 u32Sensor, void *priv)
{
	__u32 u32VideoDiv;
	__u32 u32GCD;	
	__u32 u32PacStride, u32PlaStride;
	__s32 res;
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	DBG_PRINTF("%s\n",__FUNCTION__);	

	//NT99140
	DBG_PRINTF("Init NT_99140 \n"); 								
#ifdef CONFIG_W55FA93_NT99140_VIDEOIN_PORT1	
	videoIn_Port(0);				
	DrvVideoIn_Init(TRUE, 								//BOOL bIsEnableSnrClock,
					0,								//E_VIDEOIN_SNR_SRC eSnrSrc,				/* Invalid if FA95 */
					24000,							//UINT32 u32SensorFreq,						/* KHz unit */
					eVIDEOIN_SNR_CCIR601);			//E_VIDEOIN_DEV_TYPE eDevType				
#elif defined  CONFIG_W55FA93_NT99140_PORT2_VH_GPE0_GPE1
	videoIn_Port(1);	
	DrvVideoIn_Init(TRUE, 								//BOOL bIsEnableSnrClock,
					0,								//E_VIDEOIN_SNR_SRC eSnrSrc,				/* Invalid if FA95 */
					24000,							//UINT32 u32SensorFreq,						/* KHz unit */
					eVIDEOIN_2ND_SNR_CCIR601);		//E_VIDEOIN_DEV_TYPE eDevType				
#elif defined  CONFIG_W55FA93_NT99140_PORT2_VH_GPD14_GPD15
	videoIn_Port(1);
	DrvVideoIn_Init(TRUE, 								//BOOL bIsEnableSnrClock,
					0,								//E_VIDEOIN_SNR_SRC eSnrSrc,				/* Invalid if FA95 */
					24000,							//UINT32 u32SensorFreq,						/* KHz unit */
					eeVIDEOIN_2ND_SNR_CCIR601_2);	//E_VIDEOIN_DEV_TYPE eDevType				
#endif
	//OvRegConfig();
	//DrvVideoIn_Open(12000, 12000);
	//DrvVideoIn_Open(60000, 24000);
	DrvVideoIn_Open(72000, 24000);

	res = NT99140RegConfig();
	if( res<0 )
		return res;

	NTSetResolution(REG_VALUE_HD720);

#if CONFIG_SENSOR_NT99140_IR
	// IR Led
	w55fa93_gpio_configure(GPIO_GROUP_D, 5);
	w55fa93_gpio_set(GPIO_GROUP_D, 5, 0);
	w55fa93_gpio_set_output(GPIO_GROUP_D, 5);
	w55fa93_gpio_set(GPIO_GROUP_D, 5, 0);
#endif
	vin_priv->sensor_intf->u8SensorDevID = g_uOvDeviceID;
	vin_priv->sensor_intf->u16CurImgHeight = vin_priv->sensor_intf->u16MaxImgHeight;
	vin_priv->sensor_intf->u16CurImgWidth = vin_priv->sensor_intf->u16MaxImgWidth;

	

	DrvVideoIn_SetBaseStartAddress(eVIDEOIN_PACKET, 0, vin_priv->videoIn_preview_buf[0].u32PhysAddr);	
	DrvVideoIn_EnableInt(eVIDEOIN_VINT);
	//DrvVideoIn_InstallCallback(eVIDEOIN_VINT, 
	//					(PFN_DRVVIDEOIN_CALLBACK)VideoIn_InterruptHandler,
	//					&pfnOldCallback	);	//Frame End interrupt

	//DrvVideoIn_SetSensorPolarity(TRUE, FALSE, TRUE);

	DrvVideoIn_SetSensorPolarity(FALSE, FALSE, TRUE);

	//DrvVideoIn_SetSensorPolarity(FALSE, FALSE, TRUE);

	DrvVideoIn_SetDataFormatAndOrder(eVIDEOIN_IN_UYVY, 
									eVIDEOIN_IN_YUV422, 									
									eVIDEOIN_OUT_YUV422);			
	

	DrvVideoIn_SetCropWinStartAddr(2,					//UINT16 u16VerticalStart, 	Y
								0);					//UINT16 u16HorizontalStart, 	X

	/* Sensor subsample resolution (640, 480)*/
	DrvVideoIn_SetCropWinSize(vin_priv->sensor_intf->u16CurImgHeight, vin_priv->sensor_intf->u16CurImgWidth); // height, width
	u32GCD = vinGCD(vin_priv->videowin.height, vin_priv->sensor_intf->u16CurImgHeight); 	// Preview height, Crop height
	DrvVideoIn_SetVerticalScaleFactor(eVIDEOIN_PACKET, vin_priv->videowin.height/u32GCD, vin_priv->sensor_intf->u16CurImgHeight/u32GCD); 
	u32GCD = vinGCD(vin_priv->videowin.width, vin_priv->sensor_intf->u16CurImgWidth);		// Preview width, Crop width
	DrvVideoIn_SetHorizontalScaleFactor(eVIDEOIN_PACKET,	vin_priv->videowin.width/u32GCD, vin_priv->sensor_intf->u16CurImgWidth/u32GCD);

	DrvVideoIn_SetVerticalScaleFactor(eVIDEOIN_PLANAR,
									1,
									1);
						
	DrvVideoIn_SetHorizontalScaleFactor(eVIDEOIN_PLANAR,
									1,
									1);		
	DrvVideoIn_GetStride(&u32PacStride, &u32PlaStride);
	DrvVideoIn_SetStride(vin_priv->videowin.width, u32PlaStride);
	DrvVideoIn_SetPipeEnable(FALSE,							// It means planar disable
								eVIDEOIN_PACKET);		//	

	DrvVideoIn_SetShadowRegister();

	return 0;	
}


void FunctionSwitch(int function)
{
	DBG_PRINTF("%s\n",__FUNCTION__);		
#if 0
	switch(function)
	{
#ifdef CONFIG_SENSOR_OV7670

		case SPI0_ON:
			outl(inl(REG_PINFUN) | (SPI0PIN_EN | SPI_SSOEN), REG_PINFUN);  //enable PINs SPI
			//outl(inl(PINFUN) | (SPI_SSOEN), PINFUN);  //enable PINs SPI
			outl((inl(REG_GPIOB_OMD) | 0x4000), REG_GPIOB_OMD);		//GPIOB14 high power down sensor
  			outl(inl(REG_GPIOB_DOUT) | 0x4000, REG_GPIOB_DOUT);
			break;
		case SENSOR_ON:
			outl(inl(REG_PINFUN) & (~(SPI0PIN_EN | SPI_SSOEN)), REG_PINFUN);  //disable PINs SPI
			//outl(inl(PINFUN) & (~(SPI_SSOEN)), PINFUN);  			//disable PINs SPI
			outl((inl(REG_GPIOB_OMD) | 0x4000), REG_GPIOB_OMD);    	//GPIOB14 low power on sensor
  			outl(inl(REG_GPIOB_DOUT) &(~0x4000), REG_GPIOB_DOUT);
			break;
#endif

		default:
			break;
	}
#endif
}

BOOL	
NTReadWriteBrightness(
	void *priv, 
	INT32 *pi32Value, 
	BOOL bIsRead
)
{
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	if(vin_priv->sensor_intf->u8SensorDevID != g_uOvDeviceID)
		return FALSE;
	if(bIsRead)
		*pi32Value = (DrvVideoIn_I2cReadNT(vin_priv->sensor_intf->u8SensorDevID, 0x32fc)&0xff);
	else
		DrvVideoIn_I2cWriteNT(vin_priv->sensor_intf->u8SensorDevID, 0x32fc, *pi32Value);

	return TRUE;
}

/*
BOOL	
NTReadWriteContrast(
	void *priv, 
	INT32 *pi32Value, 
	BOOL bIsRead
)
{
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	if(vin_priv->sensor_intf->u8SensorDevID != g_uOvDeviceID)
		return FALSE;
	if(bIsRead)
		*pi32Value = (DrvVideoIn_I2cReadNT(vin_priv->sensor_intf->u8SensorDevID, 0x56)&0xff);
	else
		DrvVideoIn_I2cWriteNT(vin_priv->sensor_intf->u8SensorDevID, 0x56, *pi32Value);

	return TRUE;
}
*/

BOOL	
NTReadWriteSharpness(
	void *priv, 
	INT32 *pi32Value, 
	BOOL bIsRead
)
{
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	if(vin_priv->sensor_intf->u8SensorDevID != g_uOvDeviceID)
		return FALSE;
	if(bIsRead)
		*pi32Value = (DrvVideoIn_I2cReadNT(vin_priv->sensor_intf->u8SensorDevID, 0x3301)&0xff);
	else
		DrvVideoIn_I2cWriteNT(vin_priv->sensor_intf->u8SensorDevID, 0x3301, *pi32Value);

	return TRUE;
}

/*
BOOL	
NTReadWriteWhiteBalance(
	void *priv, 
	INT32 *pi32Value, 
	BOOL bIsRead
)
{
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	if(vin_priv->sensor_intf->u8SensorDevID != g_uOvDeviceID)
		return FALSE;
	if(bIsRead)
		*pi32Value = (DrvVideoIn_I2cReadNT(vin_priv->sensor_intf->u8SensorDevID, 0x6f)&0xff);
	else
		DrvVideoIn_I2cWriteNT(vin_priv->sensor_intf->u8SensorDevID, 0x6f, *pi32Value);

	return TRUE;
}
*/

BOOL	
NTReadWriteNoiseReduction(
	void *priv, 
	INT32 *pi32Value, 
	BOOL bIsRead
)
{
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	if(vin_priv->sensor_intf->u8SensorDevID != g_uOvDeviceID)
		return FALSE;
	if(bIsRead)
		*pi32Value = (DrvVideoIn_I2cReadNT(vin_priv->sensor_intf->u8SensorDevID, 0x3300)&0x3f);
	else
		DrvVideoIn_I2cWriteNT(vin_priv->sensor_intf->u8SensorDevID, 0x3300, (*pi32Value & 0x3f));

	return TRUE;
}

/*
BOOL	
NTReadWriteColorSaturation(
	void *priv, 
	INT32 *pi32Value, 
	BOOL bIsRead
)
{
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	if(vin_priv->sensor_intf->u8SensorDevID != g_uOvDeviceID)
		return FALSE;
	if(bIsRead)
		*pi32Value = (DrvVideoIn_I2cReadNT(vin_priv->sensor_intf->u8SensorDevID, 0xc9)&0xff);
	else
		DrvVideoIn_I2cWriteNT(vin_priv->sensor_intf->u8SensorDevID, 0xc9, *pi32Value);

	return TRUE;
}

*/

BOOL	
NTSetFlickerFreq(
	void *priv, 
	UINT32 u32FlickerFreq
)
{
	UINT8 u8AECntl0; 
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	if(vin_priv->sensor_intf->u8SensorDevID != g_uOvDeviceID)
		return FALSE;
	if((u32FlickerFreq != 50) && (u32FlickerFreq != 60))
		return FALSE;
	u8AECntl0 = (DrvVideoIn_I2cReadNT(vin_priv->sensor_intf->u8SensorDevID, 0x32BB)&0xff);
	if(u32FlickerFreq == 50){
		u8AECntl0 = u8AECntl0 & ~0xa;
		u8AECntl0 = u8AECntl0 | 0xa;
	}
	else{
		u8AECntl0 = u8AECntl0 & ~0xa;
		u8AECntl0 = u8AECntl0 | 0x2;
	}
	DrvVideoIn_I2cWriteNT(vin_priv->sensor_intf->u8SensorDevID, 0x32BB, u8AECntl0);
}

typedef struct
{
	UINT16 u16ImageWidth; 
	UINT16 u16ImageHeight;
	UINT8 i8ResolIdx;
}S_NTSuppResol;

#define NT_RESOL_SUPP_CNT  3

S_NTSuppResol s_asNTSuppResolTable[NT_RESOL_SUPP_CNT] = {
	{640, 480, REG_VALUE_VGA},
	{720, 480, REG_VALUE_SVGA},
	{1280, 720, REG_VALUE_HD720}
};

BOOL	
NTChangeImageResolution(
	void *priv, 
	UINT16 u16ImageWidth, 
	UINT16 u16ImageHeight
)
{
	INT8 i;
	INT8 i8WidthIdx;
	INT8 i8HeightIdx;
	INT8 i8SensorIdx;

	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;

	for(i = 0; i < NT_RESOL_SUPP_CNT ; i ++){
		if(u16ImageWidth <= s_asNTSuppResolTable[i].u16ImageWidth)
			break;
	}
	if(i == NT_RESOL_SUPP_CNT)
		return FALSE;
	i8WidthIdx = i;
	for(i = 0; i < NT_RESOL_SUPP_CNT ; i ++){
		if(u16ImageHeight <= s_asNTSuppResolTable[i].u16ImageHeight)
			break;
	}
	if(i == NT_RESOL_SUPP_CNT)
		return FALSE;
	i8HeightIdx = i;

	if(i8HeightIdx >= i8WidthIdx){
		i8SensorIdx = i8HeightIdx;
	}
	else{
		i8SensorIdx = i8WidthIdx;
	}

	NT99140RegConfig();
	NTSetResolution(s_asNTSuppResolTable[i8SensorIdx].i8ResolIdx);
	vin_priv->sensor_intf->u16CurImgHeight = s_asNTSuppResolTable[i8SensorIdx].u16ImageHeight;
	vin_priv->sensor_intf->u16CurImgWidth = s_asNTSuppResolTable[i8SensorIdx].u16ImageWidth;
	return TRUE;
}

BOOL	
NTIRLedOnOff(
	void *priv, 
	BOOL bIsOn
)
{
	if(bIsOn){
#if CONFIG_SENSOR_NT99140_IR
			printk("IR led on \n");
			w55fa93_gpio_set(GPIO_GROUP_D, 5, 1);
#endif
	}
	else{
#if CONFIG_SENSOR_NT99140_IR
			printk("IR led off \n");
			w55fa93_gpio_set(GPIO_GROUP_D, 5, 0);
#endif

	}
	return TRUE;
}

#define LOW_LUX_GATE	0x0A00
#define HIGH_LUX_GATE	0x07C5

static int s_i32PrintCnt = 0;
static BOOL s_bIsLowLux = FALSE;

BOOL	
NTLowLuxDetect(
	void *priv
)
{
	UINT8 u8ShutterH; 
	UINT8 u8ShutterL;
	UINT16 u16Shutter;
	UINT8 u8RegGain; 
	UINT32 u32Gain;
	UINT32 u32AE;

	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	if(vin_priv->sensor_intf->u8SensorDevID != g_uOvDeviceID)
		return FALSE;
	
	s_i32PrintCnt ++;
	if((s_i32PrintCnt % 2) != 0)
		return s_bIsLowLux;


	u8ShutterH = (DrvVideoIn_I2cReadNT(vin_priv->sensor_intf->u8SensorDevID, 0x3012)&0xff);
	u8ShutterL = (DrvVideoIn_I2cReadNT(vin_priv->sensor_intf->u8SensorDevID, 0x3013)&0xff);

	u16Shutter = (uint16_t)(u8ShutterH << 8) | u8ShutterL;

	u8RegGain = (DrvVideoIn_I2cReadNT(vin_priv->sensor_intf->u8SensorDevID, 0x301d)&0xff);
	u32Gain = (((u8RegGain & 0x80) >> 7) + 1) * (((u8RegGain & 0x40) >> 6) + 1) *
			(((u8RegGain & 0x20) >> 5) + 1) * (((u8RegGain & 0x10) >> 4) + 1) * ((((u8RegGain & 0x0F) + 1) / 16) + 1);

	u32AE = u16Shutter * u32Gain;

	if(s_i32PrintCnt >= 30){
		s_i32PrintCnt = 0;
//		printk("u8ShutterH: %x \n", u8ShutterH);
//		printk("u8ShutterL: %x \n", u8ShutterL);
//		printk("u16Shutter: %x \n", u16Shutter);
//		printk("u8RegGain: %x \n", u8RegGain);
//		printk("u32Gain: %x \n", u32Gain);
//		printk("u32AE: %x \n", u32AE);
//		printk("\n \n");
	}
 
	if(u32AE >= LOW_LUX_GATE){
		if(s_bIsLowLux == FALSE){
			printk("lux detect low \n");
			s_bIsLowLux = TRUE;
			NTIRLedOnOff(priv, TRUE);
		}
	}
	else if(u32AE <= HIGH_LUX_GATE){
		if(s_bIsLowLux == TRUE){
			printk("lux detect high \n");
			s_bIsLowLux = FALSE;
			NTIRLedOnOff(priv, FALSE);
		}
	} 

	return s_bIsLowLux;
}

#if 0
/*================================================== V4L2 User Ctrl ================================================== */
struct v4l2_queryctrl
{
	__u32		     id;
	enum v4l2_ctrl_type  type;
	__u8		 name[32];	/* Whatever */
	__s32		 minimum;	/* Note signedness */
	__s32		 maximum;
	__s32		 step;
	__s32		 default_value;
	__u32               flags;
	__u32		reserved[2];
};

/*
 *	C O N T R O L S
 */
struct v4l2_control
{
	__u32		     id;
	__s32		     value;
};

#endif

static UINT16 u16SensRegAddr = 0; 
#define V4L2_CID_PRIVATE_I2C_SET_REG_ADDR     	(V4L2_CID_PRIVATE_BASE + 0)
#define V4L2_CID_PRIVATE_I2C_WRITE     			(V4L2_CID_PRIVATE_BASE + 1)
#define V4L2_CID_PRIVATE_I2C_READ     			(V4L2_CID_PRIVATE_BASE + 2)
#define V4L2_CID_PRIVATE_LASTP1     				 (V4L2_CID_PRIVATE_BASE + 3)

static const struct v4l2_queryctrl no_ctrl = {
	.name  = "42",
	.flags = V4L2_CTRL_FLAG_DISABLED,
};
static const struct v4l2_queryctrl video_ctrls[] = {
	/* --- private --- */
	{
		.id            	= V4L2_CID_PRIVATE_I2C_SET_REG_ADDR,
		.name          	= "i2c_set_addr",
		.minimum       = 0x3000,
		.maximum       = 0x3380,
		.step          	= 1,
		.type          	= V4L2_CTRL_TYPE_INTEGER,
	},{
		.id            	= V4L2_CID_PRIVATE_I2C_WRITE,
		.name          	= "i2c_write",
		.minimum       = 0,
		.maximum       = 255,
		.step          	= 1,
		.type          	= V4L2_CTRL_TYPE_INTEGER,
	},{
		.id            = V4L2_CID_PRIVATE_I2C_READ,
		.name          = "i2c_read",
		.minimum       = 0,
		.maximum       = 255,
		.step          	= 1,
		.type          = V4L2_CTRL_TYPE_INTEGER,
	}
};
static const unsigned int CTRLS = ARRAY_SIZE(video_ctrls);

static const struct v4l2_queryctrl* ctrl_by_id(unsigned int id)
{
	unsigned int i;

	for (i = 0; i < CTRLS; i++)
		if (video_ctrls[i].id == id)
			return video_ctrls+i;
	return NULL;
}
static int SensorUserPrivateCtrl(struct inode *inode, struct file *file,
		       unsigned int cmd, unsigned long arg)
{
	const struct v4l2_queryctrl *ctrl;
	struct v4l2_queryctrl *c = arg;

	if ((c->id <  V4L2_CID_BASE ||
	     c->id >= V4L2_CID_LASTP1) &&
	    (c->id <  V4L2_CID_PRIVATE_BASE ||
	     c->id >= V4L2_CID_PRIVATE_LASTP1))
		return -EINVAL;
	ctrl = ctrl_by_id(c->id);
	*c = (NULL != ctrl) ? *ctrl : no_ctrl;
	return 0;
}

BOOL	
SensorI2cWriteData(
	void *priv, 
	struct v4l2_control *c
)
{
	const struct v4l2_queryctrl* ctrl;

	ctrl = ctrl_by_id(c->id);
	if (NULL == ctrl)
		return -EINVAL;
	DBG_PRINTF("set_control name=%s val=%d\n",ctrl->name,c->value);
	switch (ctrl->type) {
	case V4L2_CTRL_TYPE_BOOLEAN:
	case V4L2_CTRL_TYPE_MENU:
	case V4L2_CTRL_TYPE_INTEGER:
		if (c->value < ctrl->minimum)
			c->value = ctrl->minimum;
		if (c->value > ctrl->maximum)
			c->value = ctrl->maximum;
		break;
	default:
		/* nothing */;
	};

	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	if(vin_priv->sensor_intf->u8SensorDevID != g_uOvDeviceID)
		return FALSE;
	DrvVideoIn_I2cWriteNT(vin_priv->sensor_intf->u8SensorDevID, u16SensRegAddr, c->value);
	return TRUE;
}
BOOL	
SensorI2cReadData(
	void *priv, 
	struct v4l2_control *c
)
{
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	if(vin_priv->sensor_intf->u8SensorDevID != g_uOvDeviceID)
		return FALSE;
	c->value = (DrvVideoIn_I2cReadNT(vin_priv->sensor_intf->u8SensorDevID, u16SensRegAddr)&0xff);
	return TRUE;
}

BOOL 
SensorI2cSetRegAddr(
	void *priv, 
	struct v4l2_control *c
)
{
	u16SensRegAddr  = c->value;
	printk("Specified sensor addr = 0x%x\n", u16SensRegAddr);
}

/* ------------------------------------------------------------------ */
static int SensorI2cReadCtrl(void *priv,
				 	struct v4l2_control *c)
{
	const struct v4l2_queryctrl* ctrl;

	ctrl = ctrl_by_id(c->id);
	DBG_PRINTF("Get_control name=%s\n",ctrl->name);
	if (NULL == ctrl)
		return -EINVAL;
	switch (c->id) {
/*
	case V4L2_CID_PRIVATE_I2C_WRITE:
		break;
*/
	case V4L2_CID_PRIVATE_I2C_READ:
		if( SensorI2cReadData(priv, c) == FALSE)
		{
			printk("i2c read faIL\n");	
			return -EINVAL;	/* I2c read fail */
		}	
		break;
	case V4L2_CID_PRIVATE_I2C_SET_REG_ADDR:
		c->value = u16SensRegAddr;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int SensorI2cWriteCtrl(void *priv, 
					struct v4l2_control *c)
{
	const struct v4l2_queryctrl* ctrl;
	unsigned long flags;
	int restart_overlay = 0;

	ctrl = ctrl_by_id(c->id);
	if (NULL == ctrl)
		return -EINVAL;
	DBG_PRINTF("set_control name=%s val=%d\n",ctrl->name,c->value);
	switch (ctrl->type) {
	case V4L2_CTRL_TYPE_BOOLEAN:
	case V4L2_CTRL_TYPE_MENU:
	case V4L2_CTRL_TYPE_INTEGER:
		if (c->value < ctrl->minimum)
			c->value = ctrl->minimum;
		if (c->value > ctrl->maximum)
			c->value = ctrl->maximum;
		break;
	default:
		/* nothing */;
	};
	switch (c->id) {
	case V4L2_CID_PRIVATE_I2C_WRITE:
		if(SensorI2cWriteData(priv, c)==FALSE)
		{
			printk("i2c write faIl\n");	
			return -EINVAL;	/* I2c write fail */
		}
		break;	
/*
	case V4L2_CID_PRIVATE_I2C_READ:
		break;	
*/
	case V4L2_CID_PRIVATE_I2C_SET_REG_ADDR:
		u16SensRegAddr  = c->value;
		printk("Specified sensor addr = 0x%x\n", u16SensRegAddr);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}


#if 0
/*================================================== V4L2 User Ctrl ================================================== */
#endif 

NVT_SENSOR_T nvt_sensor_nt99140 = {
	sensor_init:					InitSensor,
	sensor_poweron:				NULL,
	sensor_suspend:				NULL,
	sensor_reset:				NULL,
	
	read_write_brightness:			NTReadWriteBrightness,
	read_write_contrast:			NULL,
	read_write_sharpness:			NTReadWriteSharpness,
	read_write_white_balance:		NULL,
	read_write_noise_reduction:		NTReadWriteNoiseReduction,
	read_write_color_saturation:		NULL, 

	query_private_user_ctrl:			SensorUserPrivateCtrl,    /* OK */
	sensor_i2c_setRegAddr:			SensorI2cSetRegAddr, 	/* OK */
	sensor_set_ctrl:				SensorI2cWriteCtrl,
	sensor_get_ctrl:				SensorI2cReadCtrl,

	change_image_resolution: 		NTChangeImageResolution,
	set_flicker_freq:				NTSetFlickerFreq,
	low_lux_detect:				NTLowLuxDetect,
	control_IR_led:				NTIRLedOnOff,

	u16MaxImgHeight:				720,		 
	u16MaxImgWidth: 				1280,
};

