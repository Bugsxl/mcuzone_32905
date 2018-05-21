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


//#include <linux/config.h>
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

#if 0
#include <linux/videodev.h>
#else
#include <linux/videodev2.h>
#include <media/v4l2-device.h>
#endif 

#include <linux/jiffies.h>
#if 0
#include <asm/arch/videoin.h>
#include <asm/arch/DrvVideoin.h>
#include <asm/arch/w55fa95_reg.h>
#include <asm/arch/fb.h>
#include <asm/arch/w55fa95_fb.h>
#include <asm/arch/w55fa95_gpio.h>
#else
#include <mach/w55fa93_reg.h>
#include <mach/fb.h>
#include <mach/w55fa93_fb.h>
#include <mach/videoin.h>
//#include <mach/DrvVideoin.h>
#include "DrvVideoin.h"
#include <mach/videodev_ex.h>
#include <mach/w55fa93_gpio.h>
#endif

#include <linux/moduleparam.h>
#include <asm/io.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/i2c-id.h>
#include <linux/i2c-dev.h>

#include "videoinpriv.h"
#include "DrvI2C.h"	/* */
#include "DrvVideoin.h"

#include <media/v4l2-chip-ident.h>
#include <media/v4l2-common.h>
#include <media/soc_camera.h>
//IMPORT_SYMBOL(w55fa95_FB_BG_PHY_ADDR);
//extern unsigned int w55fa95_FB_BG_PHY_ADDR;
//#define outp32(addr, value)		writel(value, addr)
//#define inp32(addr)				readl(addr)
 

#define _REG_TABLE_SIZE(nTableName)	sizeof(nTableName)/sizeof(struct NT_RegValue)

#define ERR_PRINTF			printk


//#define DBG
#ifdef DBG
#define ENTER()		printk("ENTER : %s  : %s\n",__FILE__, __FUNCTION__)
#define LEAVE()		printk("LEAVE : %s  : %s\n",__FILE__, __FUNCTION__)
#define ERRLEAVE()	printk("ERRLEAVE : %s  : %s : %d\n",__FILE__, __FUNCTION__, __LINE__)
#define SDBG		printk	
#else
#define ENTER(...)
#define LEAVE(...)	
#define ERRLEAVE(...)	
#define SDBG(...)
#endif

//extern VINDEV_T* pDevVin;
//#define __STANDARD_I2C__

//extern videoIn_buf_t videoIn_preview_buf[];


//struct OV_RegValue{
struct NT_RegValue{
	__u16	uRegAddr;
	__u8	uValue;
};

struct NT_RegTable{
	struct NT_RegValue *sRegTable;
	__u16 uTableSize;
};


static struct NT_RegValue g_sNT99050_RegValue[] = 
{
 	//[InitialSetting]
	//MCLK: 24M hz, PCLK: 24M hz, LineLength: 806
	{0x3021, 0x01},
	{0x32F0, 0x01}, 
	{0x3024, 0x00},
	{0x3270, 0x00}, //[Gamma_MDR]
	{0x3271, 0x0D}, 
	{0x3272, 0x19}, 
	{0x3273, 0x2A}, 
	{0x3274, 0x3C}, 
	{0x3275, 0x4D}, 
	{0x3276, 0x67}, 
	{0x3277, 0x81}, 
	{0x3278, 0x98}, 
	{0x3279, 0xAD}, 
	{0x327A, 0xCE}, 
	{0x327B, 0xE0}, 
	{0x327C, 0xED}, 
	{0x327D, 0xFF}, 
	{0x327E, 0xFF}, 
	{0x3060, 0x01},
	{0x3210, 0x04}, //LSC //D
	{0x3211, 0x04}, //F
	{0x3212, 0x04}, //D
	{0x3213, 0x04}, //D
	{0x3214, 0x04},
	{0x3215, 0x05},
	{0x3216, 0x04},
	{0x3217, 0x04},
	{0x321C, 0x04},
	{0x321D, 0x05},
	{0x321E, 0x04},
	{0x321F, 0x03},
	{0x3220, 0x00},
	{0x3221, 0xA0},
	{0x3222, 0x00},
	{0x3223, 0xA0},
	{0x3224, 0x00},
	{0x3225, 0xA0},
	{0x3226, 0x80},
	{0x3227, 0x88},
	{0x3228, 0x88},
	{0x3229, 0x30},
	{0x322A, 0xCF},
	{0x322B, 0x07},
	{0x322C, 0x04},
	{0x322D, 0x02},
	{0x3302, 0x00},//[CC: Saturation:100%]
	{0x3303, 0x1C},
	{0x3304, 0x00},
	{0x3305, 0xC8},
	{0x3306, 0x00},
	{0x3307, 0x1C},
	{0x3308, 0x07},
	{0x3309, 0xE9},
	{0x330A, 0x06},
	{0x330B, 0xDF},
	{0x330C, 0x01},
	{0x330D, 0x38},
	{0x330E, 0x00},
	{0x330F, 0xC6},
	{0x3310, 0x07},
	{0x3311, 0x3F},
	{0x3312, 0x07},
	{0x3313, 0xFC},
	{0x3257, 0x50}, //CA Setting
	{0x3258, 0x10},
	{0x3251, 0x01},  
	{0x3252, 0x50},  
	{0x3253, 0x9A},  
	{0x3254, 0x00}, 
	{0x3255, 0xd8},  
	{0x3256, 0x60}, 
	{0x32C4, 0x38}, 
	{0x32F6, 0xCF},
	{0x3363, 0x37},
	{0x3331, 0x08},
	{0x3332, 0x6C}, // 60
	{0x3360, 0x10},
	{0x3361, 0x30},
	{0x3362, 0x70},
	{0x3367, 0x40},
	{0x3368, 0x32}, //20
	{0x3369, 0x24}, //1D
	{0x336A, 0x1A},
	{0x336B, 0x20},
	{0x336E, 0x1A},
	{0x336F, 0x16},
	{0x3370, 0x0c},
	{0x3371, 0x12},
	{0x3372, 0x1d},
	{0x3373, 0x24},
	{0x3374, 0x30},
	{0x3375, 0x0A},
	{0x3376, 0x18},
	{0x3377, 0x20},
	{0x3378, 0x30},
	{0x3340, 0x1C},
	{0x3326, 0x03}, //Eext_DIV
	{0x3200, 0x3E}, //1E
	{0x3201, 0x3F},
	{0x3109, 0x82}, //LDO Open
	{0x3106, 0x07},
	{0x303F, 0x02},
	{0x3040, 0xFF},
	{0x3041, 0x01},
	{0x3051, 0xE0},
	{0x3060, 0x01},
    
    //640x480 PCLK 24 M MCLK 24 FPS fixed 30f/s
	{0x32BF,0x04}, 
	{0x32C0,0x5A}, 
	{0x32C1,0x5A}, 
	{0x32C2,0x5A}, 
	{0x32C3,0x00}, 
	{0x32C4,0x20}, 
	{0x32C5,0x20}, 
	{0x32C6,0x20}, 
	{0x32C7,0x00}, 
	{0x32C8,0x95}, 
	{0x32C9,0x5A}, 
	{0x32CA,0x7A}, 
	{0x32CB,0x7A}, 
	{0x32CC,0x7A}, 
	{0x32CD,0x7A}, 
	{0x32D0,0x01}, 
	{0x3200,0x3E}, 
	{0x3201,0x0F}, 
	{0x302A,0x00}, 
	{0x302B,0x09}, 
	{0x302C,0x00}, 
	{0x302D,0x04}, 
	{0x3022,0x24}, 
	{0x3023,0x24}, 
	
	//{0x3025,0x02},  /* Color bar */
	
	{0x3002,0x00}, 
	{0x3003,0x02}, 
	{0x3004,0x00}, 
	{0x3005,0x02}, 
	{0x3006,0x02}, 
	{0x3007,0x85}, 
	{0x3008,0x01}, 
	{0x3009,0xE1}, 
	{0x300A,0x03}, 
	{0x300B,0x26}, 
	{0x300C,0x01}, 
	{0x300D,0xF0}, 
	{0x300E,0x02}, 
	{0x300F,0x84}, 
	{0x3010,0x01}, 
	{0x3011,0xE0}, 
	{0x32B8,0x3B}, 
	{0x32B9,0x2D}, 
	{0x32BB,0x87}, 
	{0x32BC,0x34}, 
	{0x32BD,0x38}, 
	{0x32BE,0x30}, 
	{0x3201,0x3F}, 
	{0x320A,0x01}, 
	{0x3021,0x06}, 
	{0x3060,0x01}, 
};

static struct NT_RegValue g_sNT99050_50Hz[] = 
{
	//Flicker Setting
	//[50Hz]
	{0x32BF,0x04}, 
	{0x32C0,0x5A}, 
	{0x32C1,0x5A}, 
	{0x32C2,0x5A}, 
	{0x32C3,0x00}, 
	{0x32C4,0x20}, 
	{0x32C5,0x20}, 
	{0x32C6,0x20}, 
	{0x32C7,0x00}, 
	{0x32C8,0x95}, 
	{0x32C9,0x5A}, 
	{0x32CA,0x7A}, 
	{0x32CB,0x7A}, 
	{0x32CC,0x7A}, 
	{0x32CD,0x7A}, 
	{0x32D0,0x01}, 
	
};
static struct NT_RegValue g_sNT99050_60Hz[] = 
{
	//Flicker Setting
	//[60Hz]   
	{0x32BF,0x04}, 
	{0x32C0,0x60}, 
	{0x32C1,0x5F}, 
	{0x32C2,0x5F}, 
	{0x32C3,0x00}, 
	{0x32C4,0x20}, 
	{0x32C5,0x20}, 
	{0x32C6,0x20}, 
	{0x32C7,0x00}, 
	{0x32C8,0x7C}, 
	{0x32C9,0x5F}, 
	{0x32CA,0x7F}, 
	{0x32CB,0x7F}, 
	{0x32CC,0x7F}, 
	{0x32CD,0x80}, 
	{0x32D0,0x01}, 
};


struct NT_RegTable g_NT_InitTable[] =
{
	{g_sNT99050_RegValue,_REG_TABLE_SIZE(g_sNT99050_RegValue)},
	{0,0}

};

__u8 g_uOvDeviceID[]= 
{
	0x42,		// nt99050	
};
#ifdef __STANDARD_I2C__
static struct i2c_client *save_client;
 
static int sensor_probe(struct i2c_client *client,
			const struct i2c_device_id *did)
{
	int ret = 0;
	
	ENTER();
	save_client = client;
	LEAVE();
	return ret;
}

static int sensor_remove(struct i2c_client *client)
{
	ENTER();
	LEAVE();
	return 0;
}

static const struct i2c_device_id sensor_id[] = {
	{ "nt99050", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sensor_id);

static struct i2c_driver sensor_i2c_driver = {
	.driver = {
		.name = "nt99050",
	},
	.probe    = sensor_probe,
	.remove   = sensor_remove,
	.id_table = sensor_id,
};
#endif 
#ifndef __STANDARD_I2C__
static void I2C_Delay(UINT32 u32Delay)
{
	volatile UINT32 i;
	for(;u32Delay!=0;u32Delay--)
		for(i=0;i<50;i++);
}

static BOOL 
I2C_Write_8bitSlaveAddr_16bitReg_8bitData(UINT8 uAddr, UINT16 uRegAddr, UINT8 uData)
{
	volatile unsigned int u32Delay = 0x100;

	ENTER();
	DrvI2C_SendStart();
	while(u32Delay--);		
	if ( (DrvI2C_WriteByte(uAddr,DrvI2C_Ack_Have,8)==FALSE) ||			// Write ID address to sensor
		 (DrvI2C_WriteByte((UINT8)(uRegAddr>>8),DrvI2C_Ack_Have,8)==FALSE) ||	// Write register address to sensor
		 (DrvI2C_WriteByte((UINT8)(uRegAddr&0xff),DrvI2C_Ack_Have,8)==FALSE) ||	// Write register address to sensor
		 (DrvI2C_WriteByte(uData,DrvI2C_Ack_Have,8)==FALSE) )		// Write data to sensor
	{
		DrvI2C_SendStop();
		
		printk("Non-standard I2C error, Slave addr: reg_addr = 0x%x : 0x%x\n", uAddr, uRegAddr);
		return FALSE;
	}
	DrvI2C_SendStop();
	if (uRegAddr==0x12 && (uData&0x80)!=0)
	{
		mdelay(20);			
	}
	LEAVE();
	return TRUE;
}


static UINT8 I2C_Read_8bitSlaveAddr_16bitReg_8bitData(UINT8 uAddr, UINT16 uRegAddr)
{

	UINT8 u8Data;

	ENTER();

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

	LEAVE();
	return u8Data;

}
#endif 
static int32_t DrvVideoIn_I2cWriteNT(__u8 uAddr, __u16 uRegAddr, __u8 uData)
{
	
	int ret=0;	
#ifdef __STANDARD_I2C__
	#if 1
		int i;
		struct i2c_msg msg;
		u8 buf[2];
		
		msg.flags=!I2C_M_RD;
		msg.addr=save_client->addr;
		msg.len=3;
		msg.buf=buf;		

		buf[0]=(u8)(uRegAddr>>8);
		buf[1]=(u8)(uRegAddr&0xff);
		buf[2]=uData;

		ret=i2c_transfer(save_client->adapter,&msg,1);
		for(i=0;i<10; i=i+1)
			udelay(500);
		return ret;
	#else
		i2c_smbus_write_byte_data(save_client, uRegAddr, uData);	
		udelay(10);
		printk("write value = 0x%x\n", uData);	
		return ret;
	#endif
#else		
		ENTER();
		if(I2C_Write_8bitSlaveAddr_16bitReg_8bitData(uAddr, uRegAddr, uData)==FALSE)
			ret = -1;
		LEAVE();
		return ret;
#endif
}
 
static UINT8 DrvVideoIn_I2cReadOV(__u8 uAddr, __u16 uRegAddr)
{
#ifdef __STANDARD_I2C__
	#if 1
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
		UINT8 val;
		//int ret;
		ENTER();
		i2c_smbus_write_byte(save_client, uRegAddr);
		val = i2c_smbus_read_byte(save_client);
		SDBG("read value = 0x%x\n", val);	
		return val;		
	#endif
#else
		int ret=0;
		ENTER();
		ret = I2C_Read_8bitSlaveAddr_16bitReg_8bitData(uAddr, uRegAddr);
		LEAVE();
		return ret;
#endif	
}






static __s32 OvRegConfig(__u32 nIndex)
{
	__u32 i;
	__u16 uTableSize;
	__u8  uDeviceID;
	__s32 res = 0; 
	struct NT_RegValue *psRegValue;
	ENTER();

#ifdef __STANDARD_I2C__
	printk("Standard I2C.\n");	
#else	
	printk("Non Standard I2C.\n");
	outp32(REG_GPBFUN, inp32(REG_GPBFUN)& ~(MF_GPB14|MF_GPB13));
	DrvI2C_Open(eDRVGPIO_GPIOB, 					
				eDRVGPIO_PIN13, 
				eDRVGPIO_GPIOB,
				eDRVGPIO_PIN14, 
				(PFN_DRVI2C_TIMEDELY)I2C_Delay);
	
#endif
	//SDBG("nIndex = %d\n", nIndex);	
	if ( nIndex >= (sizeof(g_uOvDeviceID)/sizeof(__u8)) ){
		printk("Specified sensor not exist\n");
		return -EBUSY;	
	}
	uTableSize = g_NT_InitTable[nIndex].uTableSize;
	psRegValue = g_NT_InitTable[nIndex].sRegTable;
	uDeviceID = g_uOvDeviceID[nIndex];

	if ( psRegValue == 0 ){
		printk("Specified sensor table not exist\n");
		return -EBUSY;	
	}

	for(i=0;i<uTableSize; i++, psRegValue++)
	{
		int32_t ret;
		udelay(100);	
		ret = DrvVideoIn_I2cWriteNT(uDeviceID, (psRegValue->uRegAddr), (psRegValue->uValue));
		if(ret<0)
		{
			printk("Wrong to write register addr = 0x%x, write data = 0x%x , ret = %d\n", (psRegValue->uRegAddr), (psRegValue->uValue), ret);		
		}	
	} 
	if(res>=0)
		printk("driver i2c initial done\n");
	else
		printk("driver i2c initial fail\n");	
	return res;
}
#if defined(CONFIG_HV_FROM_GPE0_GPE1_DEV1) 
	#if defined(CONFIG_SENSOR_RESET)
	static BOOL SnrReset(BOOL bIsReset)
	{/* GPB02 reset:	H->L->H 	*/		
		printk("%s\n",__FUNCTION__);						
		//gpio_open(GPIO_PORTB);					//GPIOB2 as GPIO		
		outp32(REG_GPBFUN, inp32(REG_GPBFUN) & (~MF_GPB2));
		w55fa93_gpio_configure(GPIO_GROUP_B, 2);
		w55fa93_gpio_set(GPIO_GROUP_B, 2, 1);
		w55fa93_gpio_set_output(GPIO_GROUP_B, 2);
		udelay(100);
		w55fa93_gpio_set(GPIO_GROUP_B, 2, 0);	//GPIOB 2 set low
		udelay(100);		
		w55fa93_gpio_set(GPIO_GROUP_B, 2, 1);	//GPIOB 2 set high
		return 0;
	}
	#endif 
	#if defined(CONFIG_SENSOR_POWER_DOWN)
	static BOOL SnrPowerDown(BOOL bIsEnable)
	{/* GPB3 power down, HIGH for power down */
		printk("%s\n",__FUNCTION__);
		//gpio_open(GPIO_PORTB);						//GPIOB as GPIO
		outp32(REG_GPBFUN, inp32(REG_GPBFUN) & (~MF_GPB3));
		w55fa93_gpio_configure(GPIO_GROUP_B, 3);
		w55fa93_gpio_set(GPIO_GROUP_B, 3, 0);
		w55fa93_gpio_set_output(GPIO_GROUP_B, 3);
		if(bIsEnable)
			w55fa93_gpio_set(GPIO_GROUP_B, 3, 1);		//GPIOB 3 set high
		else			
			w55fa93_gpio_set(GPIO_GROUP_B, 3, 0);		//GPIOB 3 set low	
		return 0;
	}
	#endif
#else
	#if defined(CONFIG_SENSOR_RESET)
	static BOOL SnrReset(BOOL bIsReset)
	{
		return 0;
	}
	#endif
	#if defined(CONFIG_SENSOR_POWER_DOWN)
	static BOOL SnrPowerDown(BOOL bIsEnable)
	{
		return 0;
	}
	#endif
#endif
static __s32 InitSensor(__u32 u32Sensor, void *priv)
{
	__u32 u32PacStride, u32PlaStride;
	__s32 res = 0; 
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;

	ENTER();

#ifdef CONFIG_HV_FROM_GPE0_GPE1_DEV1
	//outp32(REG_GPDFUN, inp32(REG_GPDFUN) &~(MF_GPD2));
	//w55fa93_gpio_configure(GPIO_GROUP_D, 2);
	//w55fa93_gpio_set_output(GPIO_GROUP_D, 2);
	//w55fa93_gpio_set(GPIO_GROUP_D, 2, 1);		//GPIOD2 set high
#endif

	
	printk("Init NT99050 \n"); 								
#ifdef CONFIG_HV_FROM_GPB2_GPB3_DEV1
	vin_priv->pDevVin->Init(TRUE,                              // BOOL bIsEnableSnrClock,
                        0,                                      // E_DRVVIDEOIN_SNR_SRC eSnrSrc,        
                        24000,                                  // UINT32 u32SensorFreq,
                        eDrvVideoIn_2nd_SNR_CCIR601);   		// E_DRVVIDEOIN_DEV_TYPE eDevType
#endif
#ifdef CONFIG_HV_FROM_GPE0_GPE1_DEV1
    vin_priv->pDevVin->Init(TRUE,                                         
                        0,                                    
                        24000,                                  
                        eDrvVideoIn_3rd_SNR_CCIR601);   		        
#endif
			vin_priv->pDevVin->Open(72000, 24000);	
#ifdef CONFIG_HV_FROM_GPE0_GPE1_DEV1
	/* IN's demo board has sensor reset and power down control */
	
#if defined(CONFIG_SENSOR_POWER_DOWN)	
	SnrPowerDown(FALSE);
#endif
#if defined(CONFIG_SENSOR_RESET)
  	SnrReset(TRUE);
#endif

#else	
	/* Cliff's demo board has not sensor reset and power down control */
#endif	

	res = OvRegConfig(0);
	if( res<0 ){
		printk("Sensor initial fail \n");
		return res;	
	}
	else
		printk("Sensor initial successful \n");
	vin_priv->sensor_intf->u8SensorDevID = g_uOvDeviceID[0];//g_uOvDeviceID[pDevVin->ov7725];
	vin_priv->sensor_intf->u16CurImgHeight = vin_priv->sensor_intf->u16MaxImgHeight;
	vin_priv->sensor_intf->u16CurImgWidth = vin_priv->sensor_intf->u16MaxImgWidth;

	vin_priv->videocrop.c.left = 1;	/*X*/
	vin_priv->videocrop.c.top = 0;	/*Y*/	
	vin_priv->videocrop.c.width = vin_priv->sensor_intf->u16CurImgWidth;
	vin_priv->videocrop.c.height = vin_priv->sensor_intf->u16CurImgHeight;	

	vin_priv->videocropcap.bounds.left = 1; 
	vin_priv->videocropcap.bounds.top = 0;
	vin_priv->videocropcap.bounds.width = vin_priv->sensor_intf->u16CurImgWidth; 
	vin_priv->videocropcap.bounds.height = vin_priv->sensor_intf->u16CurImgHeight;

	vin_priv->videocropcap.defrect.left = 1; 
	vin_priv->videocropcap.defrect.top = 0;
	vin_priv->videocropcap.defrect.width = vin_priv->sensor_intf->u16CurImgWidth; 
	vin_priv->videocropcap.defrect.height =  vin_priv->sensor_intf->u16CurImgHeight;

	if(vin_priv->sensor_intf->u16CurImgWidth==1280){
		vin_priv->videocropcap.pixelaspect.numerator = 18;	/* Suppose current image size HD */
		vin_priv->videocropcap.pixelaspect.denominator = 32;	/* Zoomming step */
	}else {
		vin_priv->videocropcap.pixelaspect.numerator = 12;	/* Suppose current image size VGA/SVGA */
		vin_priv->videocropcap.pixelaspect.denominator = 16;	/* Zoomming step */
	}


	//vin_priv->pDevVin->Open(72000, 24000);		
	if(vin_priv->videommap.format ==VIDEO_PALETTE_YUV420P)			
		vin_priv->pDevVin->SetPlanarFormat(TRUE);	/* Planar YUV420 */
	else	
		vin_priv->pDevVin->SetPlanarFormat(FALSE);	/* Planar YUV422 */		
	vin_priv->pDevVin->SetBaseStartAddress(eVIDEOIN_PACKET, 0, vin_priv->videoIn_preview_buf[0].u32PhysAddr);

	vin_priv->pDevVin->EnableInt(eVIDEOIN_VINT);
	
	vin_priv->pDevVin->SetSensorPolarity(FALSE, 
											FALSE, 
											TRUE);
	vin_priv->pDevVin->SetDataFormatAndOrder(eVIDEOIN_IN_UYVY, 
									eVIDEOIN_IN_YUV422, 									
									eVIDEOIN_OUT_YUV422);											
	vin_priv->pDevVin->SetCropWinStartAddr(0,					//UINT16 u16VerticalStart, 		Y
									1);					//UINT16 u16HorizontalStart, 	X
	/* Sensor subsample resolution (640, 480)*/
	vin_priv->pDevVin->SetCropWinSize(vin_priv->sensor_intf->u16MaxImgHeight,		//UINT16 u16Height, 
							 vin_priv->sensor_intf->u16MaxImgWidth);		//UINT16 u16Width;									 
	
	vin_priv->pDevVin->PreviewPipeSize(vin_priv->videowin.height, vin_priv->videowin.width);
	
	vin_priv->pDevVin->GetStride(&u32PacStride, &u32PlaStride);
	vin_priv->pDevVin->SetStride(vin_priv->videowin.width, u32PlaStride);															
	vin_priv->pDevVin->SetPipeEnable(FALSE,							// It means planar disable
								eVIDEOIN_PACKET);

	vin_priv->pDevVin->SetShadowRegister();
	printk("Sensor initial successful \n");
	
	return 0;	
}

static BOOL	
OVReadWriteBrightness(
	void *priv, 
	INT32 *pi32Value, 
	BOOL bIsRead
)
{
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	ENTER();
	if(vin_priv->sensor_intf->u8SensorDevID != g_uOvDeviceID[0])
		return FALSE;
	if(bIsRead)
		*pi32Value = (DrvVideoIn_I2cReadOV(vin_priv->sensor_intf->u8SensorDevID, 0x55)&0xff);
	else
		DrvVideoIn_I2cWriteNT(vin_priv->sensor_intf->u8SensorDevID, 0x55, *pi32Value);
	return TRUE;
}

static BOOL	
OVReadWriteContrast(
	void *priv, 
	INT32 *pi32Value, 
	BOOL bIsRead
)
{
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	ENTER();
	if(vin_priv->sensor_intf->u8SensorDevID != g_uOvDeviceID[0])
		return FALSE;
	if(bIsRead)
		*pi32Value = (DrvVideoIn_I2cReadOV(vin_priv->sensor_intf->u8SensorDevID, 0x56)&0xff);
	else
		DrvVideoIn_I2cWriteNT(vin_priv->sensor_intf->u8SensorDevID, 0x56, *pi32Value);
	return TRUE;
}

static BOOL	
OVReadWriteSharpness(
	void *priv, 
	INT32 *pi32Value, 
	BOOL bIsRead
)
{
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	ENTER();
	if(vin_priv->sensor_intf->u8SensorDevID != g_uOvDeviceID[0])
		return FALSE;
	if(bIsRead)
		*pi32Value = (DrvVideoIn_I2cReadOV(vin_priv->sensor_intf->u8SensorDevID, 0x3f)&0xff);
	else
		DrvVideoIn_I2cWriteNT(vin_priv->sensor_intf->u8SensorDevID, 0x3f, *pi32Value);

	return TRUE;
}


static BOOL	
OVReadWriteWhiteBalance(
	void *priv, 
	INT32 *pi32Value, 
	BOOL bIsRead
)
{
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	ENTER();
	if(vin_priv->sensor_intf->u8SensorDevID != g_uOvDeviceID[0])
		return FALSE;
	if(bIsRead)
		*pi32Value = (DrvVideoIn_I2cReadOV(vin_priv->sensor_intf->u8SensorDevID, 0x6f)&0xff);
	else
		DrvVideoIn_I2cWriteNT(vin_priv->sensor_intf->u8SensorDevID, 0x6f, *pi32Value);

	return TRUE;
}

static BOOL	
OVReadWriteNoiseReduction(
	void *priv, 
	INT32 *pi32Value, 
	BOOL bIsRead
)
{
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	ENTER();
	if(vin_priv->sensor_intf->u8SensorDevID != g_uOvDeviceID[0])
		return FALSE;
	if(bIsRead)
		*pi32Value = (DrvVideoIn_I2cReadOV(vin_priv->sensor_intf->u8SensorDevID, 0x4c)&0xff);
	else
		DrvVideoIn_I2cWriteNT(vin_priv->sensor_intf->u8SensorDevID, 0x4c, *pi32Value);

	return TRUE;
}

static BOOL	
OVReadWriteColorSaturation(
	void *priv, 
	INT32 *pi32Value, 
	BOOL bIsRead
)
{
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	ENTER();
	if(vin_priv->sensor_intf->u8SensorDevID != g_uOvDeviceID[0])
		return FALSE;
	if(bIsRead)
		*pi32Value = (DrvVideoIn_I2cReadOV(vin_priv->sensor_intf->u8SensorDevID, 0xc9)&0xff);
	else
		DrvVideoIn_I2cWriteNT(vin_priv->sensor_intf->u8SensorDevID, 0xc9, *pi32Value);

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
		.maximum       = 65535,
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

static int SensorUserPrivateCtrl(struct file *file,
		 							unsigned int cmd, 
									unsigned long *arg)
{
	const struct v4l2_queryctrl *ctrl;
	struct v4l2_queryctrl *c = (struct v4l2_queryctrl *)arg;

	if ((c->id <  V4L2_CID_BASE ||
	     c->id >= V4L2_CID_LASTP1) &&
	    (c->id <  V4L2_CID_PRIVATE_BASE ||
	     c->id >= V4L2_CID_PRIVATE_LASTP1))
		return -EINVAL;
	ctrl = ctrl_by_id(c->id);
	*c = (NULL != ctrl) ? *ctrl : no_ctrl;
	return 0;
}

static BOOL	
SensorI2cWriteData(
	void *priv, 
	struct v4l2_control *c
)
{
	const struct v4l2_queryctrl* ctrl;
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;

	ctrl = ctrl_by_id(c->id);
	if (NULL == ctrl)
		return -EINVAL;
	SDBG("set_control name=%s val=%d\n",ctrl->name,c->value);
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

	if(vin_priv->sensor_intf->u8SensorDevID != g_uOvDeviceID[0])
		return FALSE;
	DrvVideoIn_I2cWriteNT(vin_priv->sensor_intf->u8SensorDevID, u16SensRegAddr, c->value);
	return TRUE;
}
static BOOL	
SensorI2cReadData(
	void *priv, 
	struct v4l2_control *c
)
{
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	ENTER();	
	if(vin_priv->sensor_intf->u8SensorDevID != g_uOvDeviceID[0])
		return FALSE;
	c->value = (DrvVideoIn_I2cReadOV(vin_priv->sensor_intf->u8SensorDevID, u16SensRegAddr)&0xff);
	return TRUE;
}

static INT32 
SensorI2cSetRegAddr(
	void *priv, 
	struct v4l2_control *c
)
{
	u16SensRegAddr  = c->value;
	SDBG("Specified sensor addr = 0x%x\n", u16SensRegAddr);
	return 0;
}

/* ------------------------------------------------------------------ */
static INT32 SensorI2cReadCtrl(void *priv,
				 				struct v4l2_control *c)
{
	const struct v4l2_queryctrl* ctrl;

	ctrl = ctrl_by_id(c->id);
	SDBG("Get_control name=%s\n",ctrl->name);
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
	//unsigned long flags;
	//int restart_overlay = 0;

	ctrl = ctrl_by_id(c->id);
	if (NULL == ctrl)
		return -EINVAL;
	SDBG("set_control name=%s val=%d\n",ctrl->name,c->value);
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

static NVT_SENSOR_T nvt_sensor_ov = {
	sensor_init:				InitSensor,
	sensor_poweron:				NULL,

#ifdef CONFIG_SENSOR_POWER_DOWN
	sensor_suspend:				SnrPowerDown,
#else
	sensor_suspend:				NULL,
#endif
#ifdef CONFIG_SENSOR_RESET
	sensor_reset:				SnrReset,
#else	
	sensor_reset:				NULL,
#endif
	read_write_brightness:			OVReadWriteBrightness,
	read_write_contrast:			OVReadWriteContrast,
	read_write_sharpness:			OVReadWriteSharpness,
	read_write_white_balance:		OVReadWriteWhiteBalance,
	read_write_noise_reduction:		OVReadWriteNoiseReduction,
	read_write_color_saturation:	OVReadWriteColorSaturation,

	query_private_user_ctrl:		SensorUserPrivateCtrl,    /* OK */
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
#ifdef CONFIG_W55FA93_VIDEOIN_DEV1
INT32 register_vin_port1_Sensor(NVT_SENSOR_T **sensor_intf)
{
	*sensor_intf = (NVT_SENSOR_T*)(&nvt_sensor_ov);
	return 0;
}
#endif 
//#ifdef CONFIG_W55FA93_VIDEOIN_DEV2
//INT32 register_vin_port2_Sensor(NVT_SENSOR_T **sensor_intf)
//{
	//*sensor_intf = (NVT_SENSOR_T*)(&nvt_sensor_ov;
	//return 0;
//}
//#endif 


#ifdef __STANDARD_I2C__ 
static int __init i2c_init(void)
{
	int ret;

	ENTER();
	ret = i2c_add_driver(&sensor_i2c_driver);
	if(ret)
		ERRLEAVE();
	else
		LEAVE();
	return ret;
}
static void __exit i2c_cleanup(void)
{
	ENTER();
	i2c_del_driver(&sensor_i2c_driver);
	LEAVE();
}

MODULE_LICENSE("GPL");
module_init(i2c_init);
module_exit(i2c_cleanup);
#endif
