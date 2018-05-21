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
//#include <asm/arch/DrvFsc.h>
#include <asm/arch/w55fa95_reg.h>
#include <asm/arch/fb.h>
#include <asm/arch/w55fa95_fb.h>
#include <asm/arch/w55fa95_gpio.h>

#include <asm/io.h>
#include <linux/i2c.h>

#include "DrvVideoin.h"
#include "videoinpriv.h"


#define CONFIG_ARCH_W55FA95_DEMOBOARD
//IMPORT_SYMBOL(w55fa95_FB_BG_PHY_ADDR);
//extern unsigned int w55fa95_FB_BG_PHY_ADDR;



#define ERR_PRINTF			printk
#define outp32(addr, value)		outl(value, addr)
#define inp32(addr)			inl(addr)

#define ENTER(...) 
#define DBG_PRINTF(...)
//#define ENTER() 			printk("%s\n",__FUNCTION__)
//#define DBG_PRINTF		printk

//extern videoIn_buf_t videoIn_preview_buf[];


struct HI702_RegValue{
	__u8	uRegAddr;
	__u8	uValue;
};

struct HI702_RegTable{
	struct HI702_RegValue *sRegTable;
	__u16 uTableSize;
};
#define _REG_TABLE_SIZE(nTableName)	sizeof(nTableName)/sizeof(struct HI702_RegValue)

struct HI702_RegValue g_sHI702_RegValue[]=
{	
	{0x03, 0x00},
	{0x01, 0xf1},	//PWRCTL
	{0x01, 0xf3},	//PWRCTL
	{0x01, 0xf1},	//PWRCTL
	
	{0x03, 0x20},	//page20
	{0x10, 0x0c},	//AE OFF
	
	{0x03, 0x22},	//page22
	{0x10, 0x6b},	//AWB OF
	
	//Control image size, Windowing and Sync
	{0x03, 0x00},	//page0
	{0x10, 0x00},	
	{0x11, 0xb0},//younjung.park temp{0x11, 0x90},	//VDOCTL2
	{0x12, 0x24},	//0x20},	//SYNCCTL	//pclk invert
	{0x20, 0x00},	//WINROWH
	{0x21, 0x06},	//WINROWL
	{0x22, 0x00},	//WINCOLH
	{0x23, 0x06},	//WINCOLL
	{0x24, 0x01},	//WINHGTH
	{0x25, 0xe0},	//WINHGTL
	{0x26, 0x02},	//WINWIDH
	{0x27, 0x80},	//WINWIL
	
	{0x40, 0x01}, //Hblank 516
	{0x41, 0x50}, 
	{0x42, 0x00}, //Vblank 20
	{0x43, 0x14}, 

	//Black level calibration
	{0x80, 0x3e},	//BLCCTL
	{0x81, 0x96},	//_hidden_
	{0x82, 0x90},	//_hidden_
	{0x83, 0x00},	//_hidden_
	{0x84, 0x20},	//_hidden_
	
	{0x03, 0x00}, //PAGE 0
	{0x90, 0x0c}, //BLC_TIME_TH_ON
	{0x91, 0x0c}, //BLC_TIME_TH_OFF 
	{0x92, 0xd8}, //BLC_AG_TH_ON
	{0x93, 0xd0}, //BLC_AG_TH_OFF
	
	{0x94, 0x88},	//BLCDGH
	{0x95, 0x80},	//BLCDGL
	{0x98, 0x20},	//_hidden_
	{0xA0, 0x41},	//DOFSB
	{0xA2, 0x41},	//DOFSGB
	{0xA4, 0x41},	//DOFSR
	{0xA6, 0x41},	//DOFSGR
	{0xA8, 0x00},	//_hidden_
	{0xAA, 0x00},	//_hidden_
	{0xAC, 0x00},	//_hidden_
	{0xAE, 0x00},	//_hidden_
	
	//Analog Power Control
	{0x03, 0x02},	//page 2
	{0x10, 0x00},
	{0x13, 0x00},
	{0x18, 0x1c},
	{0x19, 0x00},
	{0x1A, 0x00},
	{0x1B, 0x08},
	{0x1C, 0x00},
	{0x1D, 0x00},
	{0x20, 0x33},
	{0x21, 0xaa},
	{0x22, 0xa6},	//09.04.27(from SFT)_0x76
	{0x23, 0xb0},
	{0x31, 0x99},
	{0x32, 0x00},
	{0x33, 0x00},
	{0x34, 0x3C},
	{0x50, 0x21},
	{0x54, 0x30},
	{0x56, 0xfe},
	{0x62, 0x78},
	{0x63, 0x9e},
	{0x64, 0x78},
	{0x65, 0x9e},
	{0x72, 0x7a},
	{0x73, 0x9a},
	{0x74, 0x7a},
	{0x75, 0x9a},
	{0x82, 0x09},
	{0x84, 0x09},
	{0x86, 0x09},
	{0xA0, 0x03},
	{0xA8, 0x1d},
	{0xAA, 0x49},
	{0xB9, 0x8a},
	{0xBB, 0x8a},
	{0xBC, 0x04},
	{0xBD, 0x10},
	{0xBE, 0x04},
	{0xBF, 0x10},
	
	//Control image format and Image Effect
	{0x03, 0x10},	//page 10
	{0x10, 0x01},	//ISPCTL1	// 20110525 jinkwan.kim@lge.com camsensor YCbCr order
	{0x12, 0x30},	//ISPCTL3
	{0x41, 0x00},	//DYOFS
	{0x50, 0x60},	//AGBRT
	{0x60, 0x3d},	//SATCTL
	{0x61, 0x90},	//2011 0412//SATB
	{0x62, 0x79},	//2011 0412//SATR
	{0x63, 0x50},	//AGSAT
	{0x64, 0x80},	//_hidden_
	
#ifdef __CCIR656__
	//{0x14, 0x10},	// codeword sync = 0x8010
	//{0x14, 0x14},	// codeword sync = 0x8010
	//{0x14, 0x04},	
	{0x14, 0x00},
#endif	
	
	//LPF
	{0x03, 0x11},	//page 11
	{0x10, 0x19},	//ZLPFCTL1
	{0x11, 0x0e},	//ZLPFCTL2
	{0x21, 0x30},	//ZLPFAGTH
	{0x50, 0x03},	//_hidden_
	{0x60, 0x06},	//ZLPFTH
	{0x62, 0x43},	//ZLPFLLVL
	{0x63, 0x63},	//ZLFPDYTH
	{0x74, 0x0d},	//_hidden_
	{0x75, 0x0d},	//_hidden_
	
	//YCLPF
	{0x03, 0x12},	//page 12
	{0x40, 0x23},	//YCLPFCTL1
	{0x41, 0x37},	//YCLPFCTL2
	{0x50, 0x01},	//YCLPFTH
	{0x70, 0x1d},	//BLPFCTL
	{0x74, 0x0f},	//BLPFTH1
	{0x75, 0x0f},	//BLPFTH2
	{0x91, 0x34},	//_hidden_
	{0xB0, 0xc9},	//_hidden_
	{0xD0, 0xb1},	//_hidden
	
	//Edge Enhancement
	{0x03, 0x13},	//page 13
	{0x10, 0x35},	//EDGECTL1
	{0x11, 0x01},	//EDGECTL2
	{0x12, 0x00},	//_hidden_
	{0x13, 0x02},	//_hidden_
	{0x14, 0x01},	//EDGECTL3
	{0x20, 0x03},	//EDGENGAIN
	{0x21, 0x01},	//EDGEPGAIN
	{0x23, 0x2f},	//EDGEHCLPTH
	{0x24, 0x0f},	//EDGELCLIPTH
	{0x25, 0x40},	//_hidden_
	{0x28, 0x00},	//EDGETIMETH
	{0x29, 0x48},	//EDGEAGTH
	{0x30, 0xff},	//_hidden_
	{0x80, 0x0b},	//EDGE2DCTL1
	{0x81, 0x11},	//EDGE2DCTL2
	{0x83, 0x5d},	//EDGE2DCTL3
	{0x90, 0x05},	//EDGE2DNGAIN
	{0x91, 0x05},	//EDGE2DPGAIN
	{0x93, 0x2f},	//EDGE2DHCLIPTH
	{0x94, 0x0f},	//EDGE2DLCLIPTH
	{0x95, 0x80},	//_hidden_
	
	//Lens shading
	{0x03, 0x14},	//page 14
	{0x10, 0x05},	//LENSCTL
	{0x20, 0x80},	//center x-axis
	{0x21, 0x80},	//center y-axis
	{0x22, 0x40}, //R Lens Correction	
	{0x23, 0x2c},	//G Lens Correction
	{0x24, 0x32},	//B Lens Correction
	{0x25, 0x69},	//LAGOFF
	{0x26, 0x67},	//LAGON
	
	//Color Correction
	{0x03, 0x15},
	{0x10, 0x0f},	//CMCCTL
	//Rstep 16
	{0x14, 0x2c},	//CMCOFSGM 
	{0x16, 0x1c},	//CMCOFSGL
	{0x17, 0x2d},	//CMC SIGN
	//CMC
	{0x30, 0x5c},
	{0x31, 0x1f},
	{0x32, 0x03},
	{0x33, 0x0b},
	{0x34, 0x5a},
	{0x35, 0x0f},
	{0x36, 0x04},
	{0x37, 0x2d},
	{0x38, 0x69},
	//CMC OFS
	{0x40, 0x82},
	{0x41, 0x04},
	{0x42, 0x82},
	{0x43, 0x02},
	{0x44, 0x86},
	{0x45, 0x04},
	{0x46, 0x83},
	{0x47, 0x99},
	{0x48, 0x1c},
	
	//Gamma Correction
	{0x03, 0x16},	//page 16
	{0x30, 0x00},	//GMA0
	{0x31, 0x09},	//GMA1
	{0x32, 0x1c},	//GMA2
	{0x33, 0x31},	//GMA3
	{0x34, 0x54},	//GMA4
	{0x35, 0x71},	//GMA5
	{0x36, 0x8a},	//GMA6
	{0x37, 0x9f},	//GMA7
	{0x38, 0xb1},	//GMA8
	{0x39, 0xc0},	//GMA9
	{0x3a, 0xcd},	//GMA10
	{0x3b, 0xe1},	//GMA11
	{0x3c, 0xef},	//GMA12
	{0x3d, 0xf8},	//GMA13
	{0x3e, 0xff},	//GMA14
	
	//Auto Flicker Cancellation
	{0x03, 0x17},	//page 17
	{0xC4, 0x49},	//FLK200
	{0xC5, 0x3c},	//FLK100
	
	//scaling
	{0x03, 0x18},	//page 18
	{0x10, 0x00},	//scaling off
	
	//Auto Exposure
	{0x03, 0x20},	//page 20
	{0x10, 0x0c},	//AECTL1
	{0x11, 0x00},	//AECTL2
	{0x20, 0x01},	//AEFRAMECTL
	{0x28, 0x3f},	//AEFINECTL1
	{0x29, 0xaa},	//AEFINECTL2
	
	{0x2A, 0xf0}, //for Variable fps
	{0x2B, 0x34}, //for Variable fps
	{0x30, 0x78},	//_hidden_
	
	{0x60, 0xA8}, //AEWGT
	{0x70, 0x3c}, //YLVL //yvyl is luminance level to converge in AE operation
	{0x78, 0x23}, //YTH1
	{0x79, 0x1e}, //YTH2
	{0x7A, 0x24}, //_hidden_
	
	{0x83, 0x00}, //EXP Normal 30.00 fps 
	{0x84, 0xc3},						 
	{0x85, 0x50},						 
	{0x86, 0x00}, //EXPMin 6000.00 fps	 
	{0x87, 0xfa},						 
	
	//MIN 15FPS 60HZ
	{0x88, 0x02}, //EXP Max 10.00 fps 
	{0x89, 0x49}, 
	{0x8a, 0xf0}, 
		
	{0x8B, 0x3a}, //EXP100 
	{0x8C, 0x98}, 
	{0x8D, 0x30}, //EXP120 
	{0x8E, 0xd4}, 
	
	{0x8F, 0xc4},	//EXPDPCH
	{0x90, 0x68},	//EXPDPCL
	
	{0x91, 0x02}, //EXP Fix 10.00 fps
	{0x92, 0x40}, 
	{0x93, 0x2c}, 
	{0x98, 0x8c},	//EXPOUT1
	{0x99, 0x23},	//EXPOUT2
	
	{0x9c, 0x06}, //EXP Limit 771.79 fps 
	{0x9d, 0xd6}, 
	{0x9e, 0x00}, //EXP Unit 
	{0x9f, 0xfa}, 
	 
	
	{0xB0, 0x18},	//AG
	{0xB1, 0x14},	//AGMIN 
	{0xB2, 0xe0},	//AGMAX
	{0xB3, 0x14},	//AGLVL
	{0xB4, 0x14},	//AGTH1
	{0xB5, 0x38},	//AGTH2
	{0xB6, 0x26},	//AGBTH1
	{0xB7, 0x20},	//AGBTH2
	{0xB8, 0x1d},	//AGBTH3
	{0xB9, 0x1b},	//AGBTH4
	{0xBA, 0x1a},	//AGBTH5
	{0xBB, 0x19},	//AGBTH6
	{0xBC, 0x19},	//AGBTH7
	{0xBD, 0x18},	//AGBTH8
	{0xC0, 0x10},	//AGSKY
	{0xC3, 0x60},	//AGDPCON
	{0xC4, 0x58},	//AGDPCOFF
	{0xC8, 0x90},	//DGMAX
	{0xC9, 0x80},	//DGMIN
	
	//Auto white balance
	{0x03, 0x22},	//page 22
	{0x10, 0x6a},	//AWBCTL1
	{0x11, 0x2c},	//AWBCTL2, outdoor limt[1]
	{0x20, 0x01}, //AE Weight, Enable B[0]
	{0x21, 0x40},	//_hidden_
	
	{0x30, 0x80},	//ULVL
	{0x31, 0x80},	//VLVL
	{0x38, 0x12},	//UVTH1
	{0x39, 0x66},	//UVTH2
	{0x40, 0xf3},	//YRANGE
	{0x41, 0x55},	//CDIFF
	{0x42, 0x33},	//CSUM
	{0x43, 0xf5},	///_hidden_
	{0x44, 0xaa},	//_hidden_
	{0x45, 0x66},	//_hidden_
	{0x46, 0x0a},	//WHTPXLTH
	
	{0x60, 0x95}, //AE weight

	{0x80, 0x30},	//RGAIN
	{0x81, 0x20},	//GGAIN
	{0x82, 0x30},	//BGAIN
	
	{0x83, 0x50},	//RMAX
	{0x84, 0x0e},	//RMIN
	{0x85, 0x64},	//BMAX
	{0x86, 0x18},	//BMIN
	
	{0x87, 0x48},	//RmaxB
	{0x88, 0x35},	//Rmin B
	{0x89, 0x30},	//Bmax B
	{0x8a, 0x20},	//Bmin B
	
	{0x8B, 0x05},	//BOUNDARY STEP//Rbexplmt
	{0x8d, 0x17},	//Rdelta
	{0x8e, 0x61},	//Bdelta
				   
	{0x8F, 0x53},	//BGAINPARA1
	{0x90, 0x52},	//BGAINPARA2
	{0x91, 0x4e},	//BGAINPARA3
	{0x92, 0x41},	//BGAINPARA4
	{0x93, 0x30},	//BGAINPARA5
	{0x94, 0x23},	//BGAINPARA6
	{0x95, 0x17},	//BGAINPARA7
	{0x96, 0x0c},	//BGAINPARA8
	{0x97, 0x03},	//BGAINPARA9 
	{0x98, 0x02},	//BGAINPARA10
	{0x99, 0x02},	//BGAINPARA11
	{0x9A, 0x02},	//BGAINPARA12
	{0x9B, 0x0a},	//BGAINBND
	{0x10, 0xea},	//AWBCLT1
	
	{0x03, 0x20},	//page 20
	{0x10, 0x8c},	//60hz//AECTL1
	
	{0x03, 0x00},
	{0x01, 0xf0},	//sleep 0ff

    //{SEQUENCE_WAIT_MS, 0x64},
    //{SEQUENCE_END, 0x00}
};

struct HI702_RegTable g_HI702_InitTable[] =
{
#ifdef CONFIG_SENSOR_HI702_DEV1
	{g_sHI702_RegValue, _REG_TABLE_SIZE(g_sHI702_RegValue)},
	{0,0}
#endif
};

__u8 g_HI702_DeviceID[]= 
{
#ifdef CONFIG_SENSOR_HI702_DEV1
	0x60,		// HI702
#endif
};

static struct i2c_client *save_client;
static int sensor_detected;
static unsigned short ignore[] = { I2C_CLIENT_END };

#ifdef CONFIG_SENSOR_HI702_DEV1
static unsigned short normal_addr[] = { 0x30, I2C_CLIENT_END };
#endif

static struct i2c_client_address_data addr_data = {
	.normal_i2c = normal_addr,
	.probe = ignore,
	.ignore = ignore,
};

static int sensor_i2c_probe(struct i2c_adapter *adap, int addr, int kind);

static int sensor_i2c_attach(struct i2c_adapter *adap)
{	
	ENTER();
	sensor_detected = i2c_probe(adap, &addr_data, sensor_i2c_probe);	
	return sensor_detected;
}

static int sensor_i2c_detach(struct i2c_client *client)
{
	int rc;
	ENTER();	
	if ((rc = i2c_detach_client(client)) == 0) {
		kfree(i2c_get_clientdata(client));		
	}
	return rc;
}

static struct i2c_driver sensor_i2c_driver = {
	.driver = {
		.name	= "Sensor I2C",
	},

#ifdef CONFIG_SENSOR_HI702_DEV1
	.id = 0x30,
#endif

	.attach_adapter = sensor_i2c_attach,
	.detach_client = sensor_i2c_detach,
};

static int sensor_i2c_probe(struct i2c_adapter *adap, int addr, int kind)
{
	struct i2c_client *client;
	int rc;
	ENTER();	
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

s8  DrvVideoIn_I2cWriteHI702(__u8 uAddr, __u8 uRegAddr, __u8 uData)
{
	i2c_smbus_write_byte_data(save_client, uRegAddr, uData);
	return TRUE;
}

__s8  DrvVideoIn_I2cReadHI702(__u8 uAddr, __u8 uRegAddr)
{
	i2c_smbus_write_byte(save_client, uRegAddr);
	return i2c_smbus_read_byte(save_client);
}

void I2C_Delay(UINT32 u32Delay)
{
	volatile UINT32 i;
	for(;u32Delay!=0;u32Delay--)
		for(i=0;i<5;i++);
}

__s32 HI702RegConfig(__u32 nIndex)
{
  	__u32 i;
	__u16 uTableSize;
	__u8  uDeviceID;
	__s32 res = 0; 
	struct HI702_RegValue *psRegValue;

	ENTER();
	i2c_add_driver(&sensor_i2c_driver);
	DBG_PRINTF("i2c add successful\n");
	if (sensor_detected)
	{
		DBG_PRINTF("Sensor I2C driver installed.\n");
	}
	else
	{
		printk("Failed to install I2C driver for sensor!!!\n");		
		return -EBUSY;
	}	
	
	uTableSize = g_HI702_InitTable[nIndex].uTableSize;
	psRegValue = g_HI702_InitTable[nIndex].sRegTable;
	if ( psRegValue == 0 )
		return -EBUSY;	

	for(i=0;i<uTableSize; i++, psRegValue++)
	{			
		res = i2c_smbus_write_byte_data(save_client, (psRegValue->uRegAddr), (psRegValue->uValue));
		if(res<0)
			break;
	}
	if(res>=0)
		printk("driver i2c initial done\n");
	else
		printk("driver i2c initial fail\n");	
	return res;
}

#if defined(CONFIG_ARCH_W55FA95) 
void SnrReset(void)
{/* GPA11 reset:	H->L->H */	
	ENTER();		
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
	ENTER();
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
#define outpw	outp32
#define inpw		inp32
void HI702_sensorReset(BOOL bIsReset)
{//GPD13 acts as CAM_RESET,(1: reset inactive, 0: reset) 
#ifdef CONFIG_W55FA95_HI702_BOARD_DV1
	UINT32 u32RegData;	
	outpw(REG_GPDFUN , inpw(REG_GPDFUN) & ~(0x3 << (13<<1)) );	/* GPD13 switch to GPIO */ 			
	u32RegData = inpw(REG_GPIOD_OMD);				/* GPD13 set to output mode */
	outpw(REG_GPIOD_OMD , inpw(REG_GPIOD_OMD) | (1 << 13));	        /* 1 output. 0 input */
	if(bIsReset==TRUE)
	{//0: reset
		outpw(REG_GPIOD_DOUT , inpw(REG_GPIOD_DOUT)&~(1<<13) );
	}
	else
	{//1: non-reset
		outpw(REG_GPIOD_DOUT , inpw(REG_GPIOD_DOUT)|(1<<13) );
	}		
#endif
}

void HI702_sensorPoweron(BOOL bIsPowerOn)
{
#ifdef CONFIG_W55FA95_HI702_BOARD_DV1
	UINT32 u32RegData;
	/* GPD0 acts as CAM2V8(1: on, 0: off), 
	  
	   GPB4 acts as CHIP_ENABLE(1: Enable, 0: Suspend)
	   GPD12 acts as CAM1V8 (1: on, 0: off)
	*/
	/* CAM2V8 => output mode */
	outpw(REG_GPDFUN , inpw(REG_GPDFUN) & ~(0x3 << (0<<1)) );	/* GPD0 switch to GPIO */ 	
	u32RegData = inpw(REG_GPIOD_OMD);		
	outpw(REG_GPIOD_OMD , (u32RegData | (1<<0)));			/* GPD0 set to output mode */     
		   		
	/* CAM1V8 => output mode */
	outpw(REG_GPDFUN , inpw(REG_GPDFUN) & ~(0x3 << (12<<1)) );	/* GPD12 switch to GPIO */ 	
	u32RegData = inpw(REG_GPIOD_OMD);		
	outpw(REG_GPIOD_OMD , (u32RegData | (1<<12)));			/* GPD12 set to output mode */ 

	if(bIsPowerOn==TRUE)
	{//Turn on the 2.8v first then 1.8v 
		//unsigned long j = jiffies + 20*HZ/1000; 	/* 10ms  base*/	
		outpw(REG_GPIOD_DOUT , inpw(REG_GPIOD_DOUT)|(1) );	/* Turn on 2,8v */		
		mdelay(15);	/* Dealy 15ms */
		//while( time_before(jiffies,j) )
		//	schedule();
		outpw(REG_GPIOD_DOUT , inpw(REG_GPIOD_DOUT)|(1<<12) );	/* Turn on 1,8v */	
	
	}
	else
	{//Turn off the 1.8v first the 2.8v   	
		//mdelay(5);	/* Dealy 5ms */
		//msleep(5);	/* Sleep 5ms (Inaccuracy)*/					
		unsigned long j=0;	
		j = jiffies + 30*HZ/1000; 	/* 2Xms~30ms */			
		while( time_before(jiffies,j) )
			schedule();	
		outpw(REG_GPIOD_DOUT , inpw(REG_GPIOD_DOUT)&~(1<<12) );		/* 1.8v off */				
		j = jiffies + 30*HZ/1000; 	/* 2Xms~30ms */
		while( time_before(jiffies,j) )
			schedule();		
		outpw(REG_GPIOD_DOUT , inpw(REG_GPIOD_DOUT)&~(1) );		/* 2.8v off */			
	}					
#endif
#ifdef CONFIG_W55FA95_HI702_BOARD_PCBA
	#if 0
	/* Power on GPD0 = High */
	gpio_configure(GPIO_PORTD, 
					(0));	// pin number	
	gpio_setportdir(GPIO_PORTD, 
					(1<<0), 	// Mask 
					(1<<0));	// 1 output. 0 input.			
	gpio_setportval(GPIO_PORTD, 
					(1<<0), 	//Mask
					//(1<<0));	//High				
					(1<<0));	// power on

	#else			
	outpw(REG_GPDFUN , inpw(REG_GPDFUN) &~ (0x3 << (0<<1)));
															
	outpw(REG_GPIOD_OMD , inpw(REG_GPIOD_OMD) & ~(1 & (1 ^ 1)));
	outpw(REG_GPIOD_OMD , inpw(REG_GPIOD_OMD) | (1 & 1));						
	if(bIsPowerOn==TRUE)
	{	
		outpw(REG_GPIOD_DOUT , inpw(REG_GPIOD_DOUT) & ~(1 & (1 ^ 1)));
		outpw(REG_GPIOD_DOUT , inpw(REG_GPIOD_DOUT) | (1 & 1));
	}
	else
	{
		outpw(REG_GPIOD_DOUT , inpw(REG_GPIOD_DOUT) & ~(1 & (1 ^ 1)));
		outpw(REG_GPIOD_DOUT , inpw(REG_GPIOD_DOUT) | (1 & 1));
		printk("Sensor power off\n");
	}
	#endif
#endif
}
void HI702_sensorSuspend(BOOL bIsSuspend)
{		
	/* Standby GPB4 = LOW */		
	outpw(REG_GPBFUN, inpw(REG_GPBFUN) & ~(BIT8|BIT9));
				
	outpw(REG_GPIOB_OMD, inpw(REG_GPIOB_OMD) | (1<<4));
	mdelay(1);	/* Dealy 1ms */	
	if(bIsSuspend==TRUE)
	{// suspend GPB4 = LOW. 							
		outpw(REG_GPIOB_DOUT , inpw(REG_GPIOB_DOUT) & ~(1<<4));	
		printk("Sensor suspend\n");
	}
	else
	{//Non suspend GPB4 = High. 
		outpw(REG_GPIOB_DOUT , inpw(REG_GPIOB_DOUT) | (1<<4));	
	}
}

__s32 InitSensor(__u32 u32Sensor, void *priv)
{
	__u32 u32VideoDiv;
	__u32 u32GCD;	
	__u32 u32PacStride, u32PlaStride;
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	__s32 res;

	ENTER();
	
	DBG_PRINTF("Init HI702 \n"); 			
	//sensorPoweron(TRUE);					
	//sensorSuspend(FALSE); 
	udelay(1000);
#ifdef CONFIG_W55FA95_HI702_VIDEOIN_PORT1	
	videoIn_Port(0);				
	DrvVideoIn_Init(TRUE, 								//BOOL bIsEnableSnrClock,
					0,								//E_VIDEOIN_SNR_SRC eSnrSrc,				/* Invalid if FA95 */
					24000,							//UINT32 u32SensorFreq,						/* KHz unit */
					eVIDEOIN_SNR_CCIR601);			//E_VIDEOIN_DEV_TYPE eDevType				
#elif defined  CONFIG_W55FA95_HI702_PORT2_VH_GPE0_GPE1
	videoIn_Port(1);	
	DrvVideoIn_Init(TRUE, 								//BOOL bIsEnableSnrClock,
					0,								//E_VIDEOIN_SNR_SRC eSnrSrc,				/* Invalid if FA95 */
					24000,							//UINT32 u32SensorFreq,						/* KHz unit */
					eVIDEOIN_2ND_SNR_CCIR601);		//E_VIDEOIN_DEV_TYPE eDevType				
#elif defined  CONFIG_W55FA95_HI702_PORT2_VH_GPD14_GPD15
	videoIn_Port(1);
	DrvVideoIn_Init(TRUE, 								//BOOL bIsEnableSnrClock,
					0,								//E_VIDEOIN_SNR_SRC eSnrSrc,				/* Invalid if FA95 */
					24000,							//UINT32 u32SensorFreq,						/* KHz unit */
					eeVIDEOIN_2ND_SNR_CCIR601_2);	//E_VIDEOIN_DEV_TYPE eDevType				
#endif
#ifdef CONFIG_W55FA95_HI702_BOARD_DV1		
	mdelay(30);	/* Dealy 30ms */
	HI702_sensorReset(FALSE);
#endif

	res = HI702RegConfig(0);
	if( res<0 )
		return res;		
	vin_priv->sensor_intf->u8SensorDevID = g_HI702_DeviceID[0];//g_uOvDeviceID[DrvVideoIn_ov7725];
	vin_priv->sensor_intf->u16CurImgHeight = vin_priv->sensor_intf->u16MaxImgHeight;
	vin_priv->sensor_intf->u16CurImgWidth = vin_priv->sensor_intf->u16MaxImgWidth;

	DrvVideoIn_Open(72000, 24000);		

	DrvVideoIn_SetBaseStartAddress(eVIDEOIN_PACKET, 0, vin_priv->videoIn_preview_buf[0].u32PhysAddr);

	DrvVideoIn_EnableInt(eVIDEOIN_VINT);
	
	DrvVideoIn_SetSensorPolarity(FALSE, 
								FALSE, 
								TRUE);
	DrvVideoIn_SetDataFormatAndOrder(eVIDEOIN_IN_VYUY, 
									eVIDEOIN_IN_YUV422, 									
									eVIDEOIN_OUT_YUV422);											
	DrvVideoIn_SetCropWinStartAddr(0,					//UINT16 u16VerticalStart, 		Y
								0);					//UINT16 u16HorizontalStart, 	X
	/* Sensor subsample resolution (640, 480)*/
	DrvVideoIn_SetCropWinSize(vin_priv->sensor_intf->u16MaxImgHeight,		//UINT16 u16Height, 
							 vin_priv->sensor_intf->u16MaxImgWidth);		//UINT16 u16Width;									 
	
	DrvVideoIn_SetVerticalScaleFactor(eVIDEOIN_PACKET,		
									vin_priv->videowin.height,
									vin_priv->sensor_intf->u16MaxImgHeight);																					
	DrvVideoIn_SetHorizontalScaleFactor(eVIDEOIN_PACKET,		
									vin_priv->videowin.width,
									vin_priv->sensor_intf->u16MaxImgWidth);

	DrvVideoIn_SetVerticalScaleFactor(eVIDEOIN_PLANAR,
									1,
									1);
				
	DrvVideoIn_SetHorizontalScaleFactor(eVIDEOIN_PLANAR,
									1,
									1);		
	DrvVideoIn_GetStride(&u32PacStride, &u32PlaStride);
	DrvVideoIn_SetStride(vin_priv->videowin.width, u32PlaStride);															
	DrvVideoIn_SetPipeEnable(FALSE,							// It means planar disable
								eVIDEOIN_PACKET);

	DrvVideoIn_SetShadowRegister();
	printk("Config videoin successful\n");	
	return 0;		
}

BOOL	
HI702_ReadWriteBrightness(
	void *priv, 
	INT32 *pi32Value, 
	BOOL bIsRead
)
{
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	if(vin_priv->sensor_intf->u8SensorDevID != g_HI702_DeviceID[0])
		return FALSE;
	if(bIsRead)
		*pi32Value = (DrvVideoIn_I2cReadHI702(vin_priv->sensor_intf->u8SensorDevID, 0x55)&0xff);
	else
		DrvVideoIn_I2cWriteHI702(vin_priv->sensor_intf->u8SensorDevID, 0x55, *pi32Value);

	return TRUE;
}

BOOL	
HI702_ReadWriteContrast(
	void *priv, 
	INT32 *pi32Value, 
	BOOL bIsRead
)
{
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	if(vin_priv->sensor_intf->u8SensorDevID != g_HI702_DeviceID[0])
		return FALSE;
	if(bIsRead)
		*pi32Value = (DrvVideoIn_I2cReadHI702(vin_priv->sensor_intf->u8SensorDevID, 0x56)&0xff);
	else
		DrvVideoIn_I2cWriteHI702(vin_priv->sensor_intf->u8SensorDevID, 0x56, *pi32Value);

	return TRUE;
}

BOOL	
HI702_ReadWriteSharpness(
	void *priv, 
	INT32 *pi32Value, 
	BOOL bIsRead
)
{
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	if(vin_priv->sensor_intf->u8SensorDevID != g_HI702_DeviceID[0])
		return FALSE;
	if(bIsRead)
		*pi32Value = (DrvVideoIn_I2cReadHI702(vin_priv->sensor_intf->u8SensorDevID, 0x3f)&0xff);
	else
		DrvVideoIn_I2cWriteHI702(vin_priv->sensor_intf->u8SensorDevID, 0x3f, *pi32Value);

	return TRUE;
}


BOOL	
HI702_ReadWriteWhiteBalance(
	void *priv, 
	INT32 *pi32Value, 
	BOOL bIsRead
)
{
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	if(vin_priv->sensor_intf->u8SensorDevID != g_HI702_DeviceID[0])
		return FALSE;
	if(bIsRead)
		*pi32Value = (DrvVideoIn_I2cReadHI702(vin_priv->sensor_intf->u8SensorDevID, 0x6f)&0xff);
	else
		DrvVideoIn_I2cWriteHI702(vin_priv->sensor_intf->u8SensorDevID, 0x6f, *pi32Value);

	return TRUE;
}

BOOL	
HI702_ReadWriteNoiseReduction(
	void *priv, 
	INT32 *pi32Value, 
	BOOL bIsRead
)
{
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	if(vin_priv->sensor_intf->u8SensorDevID != g_HI702_DeviceID[0])
		return FALSE;
	if(bIsRead)
		*pi32Value = (DrvVideoIn_I2cReadHI702(vin_priv->sensor_intf->u8SensorDevID, 0x4c)&0xff);
	else
		DrvVideoIn_I2cWriteHI702(vin_priv->sensor_intf->u8SensorDevID, 0x4c, *pi32Value);

	return TRUE;
}

BOOL	
HI702_ReadWriteColorSaturation(
	void *priv, 
	INT32 *pi32Value, 
	BOOL bIsRead
)
{
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	if(vin_priv->sensor_intf->u8SensorDevID != g_HI702_DeviceID[0])
		return FALSE;
	if(bIsRead)
		*pi32Value = (DrvVideoIn_I2cReadHI702(vin_priv->sensor_intf->u8SensorDevID, 0xc9)&0xff);
	else
		DrvVideoIn_I2cWriteHI702(vin_priv->sensor_intf->u8SensorDevID, 0xc9, *pi32Value);

	return TRUE;
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
		.minimum       = 0,
		.maximum       = 255,
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
	if(vin_priv->sensor_intf->u8SensorDevID != g_HI702_DeviceID[0])
		return FALSE;
	DrvVideoIn_I2cWriteHI702(vin_priv->sensor_intf->u8SensorDevID, u16SensRegAddr, c->value);
	return TRUE;
}
BOOL	
SensorI2cReadData(
	void *priv, 
	struct v4l2_control *c
)
{
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	if(vin_priv->sensor_intf->u8SensorDevID != g_HI702_DeviceID[0])
		return FALSE;
	c->value = (DrvVideoIn_I2cReadHI702(vin_priv->sensor_intf->u8SensorDevID, u16SensRegAddr)&0xff);
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



NVT_SENSOR_T nvt_sensor_hi702 = {
	sensor_init:					InitSensor,
	sensor_poweron:				HI702_sensorPoweron,
	sensor_suspend:				HI702_sensorSuspend,
	sensor_reset:				HI702_sensorReset,

	read_write_brightness:			HI702_ReadWriteBrightness,
	read_write_contrast:			HI702_ReadWriteContrast,
	read_write_sharpness:			HI702_ReadWriteSharpness,
	read_write_white_balance:		HI702_ReadWriteWhiteBalance,
	read_write_noise_reduction:		HI702_ReadWriteNoiseReduction,
	read_write_color_saturation:		HI702_ReadWriteColorSaturation,

	query_private_user_ctrl:			SensorUserPrivateCtrl,    /* OK */
	sensor_i2c_setRegAddr:			SensorI2cSetRegAddr, 	/* OK */
	sensor_set_ctrl:				SensorI2cWriteCtrl,
	sensor_get_ctrl:				SensorI2cReadCtrl,

	change_image_resolution: 		NULL,
	set_flicker_freq:				NULL,
	low_lux_detect:					NULL,
	control_IR_led:					NULL,

	u16MaxImgHeight:				480,		 
	u16MaxImgWidth: 				640,
};

//chchen59


