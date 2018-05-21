/*
 * linux/arch/arm/mach-w55fa93/cpu.c
 *
 * Copyright (c) 2009 Nuvoton corporation.
 *
 * W55FA93 CPU Support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/rtc.h>

//#include <asm/hardware.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/delay.h>
//#include <asm/rtc.h>
#include <asm/tlbflush.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <mach/w55fa93_reg.h>
#include <mach/w55fa93_sysmgr.h>
#include <mach/w55fa93_rtc.h>
#include "cpu.h"
#include "dev.h"

#if defined(CONFIG_W55FA93_SYSMGR) || defined (CONFIG_W55FA93_SYSMGR_MODULE)
extern void sysmgr_report(unsigned);
#endif
extern int w55fa93_edma_isbusy(int);
extern int w55fa93_rtc_read_time_wrap(struct rtc_time *);
extern int w55fa93_rtc_set_time_wrap(struct rtc_time *);
extern int w55fa93_rtc_read_alarm_wrap(struct rtc_wkalrm *);
extern int w55fa93_rtc_set_alarm_wrap(struct rtc_wkalrm *);

/* Initial serial platform data */
/*
struct plat_serial8250_port w55fa93_uart_data[] = {
	[0] = W55FA93_8250PORT(UART0),
	// MFSEL and CLKEN must set accordingly
	//[1] = W55FA93_8250PORT(UART1),
	{},
};

struct platform_device w55fa93_serial_device = {
	.name			= "serial8250",
	.id			= PLAT8250_DEV_PLATFORM,
	.dev			= {
		.platform_data	= w55fa93_uart_data,
	},
};
*/
/* Init W55FA93 io */

void __init cpu_map_io(struct map_desc *mach_desc, int mach_size)
{
	unsigned long idcode = 0x0;

	iotable_init(mach_desc, mach_size);

	idcode = __raw_readl(REG_CHIPID);
	if (idcode == W55FA93_CPUID)
		printk(KERN_INFO "CPU type 0x%08lx is W55FA93\n", idcode);
}


static struct platform_device *sys_clk;
extern unsigned int w55fa93_upll_clock;
extern unsigned int w55fa93_system_clock;
extern unsigned int w55fa93_cpu_clock;
extern unsigned int w55fa93_ahb_clock;
extern unsigned int w55fa93_apb_clock;
#ifdef CONFIG_W55FA93_KEYPAD
extern u32 w55fa93_key_pressing;
#endif
#ifdef CONFIG_TOUCHSCREEN_W55FA93
extern u32 w55fa93_ts_pressing;
#endif
static u32 *sram_vaddr;

void enter_clock_setting(u8, u8, u8) __attribute__ ((section ("enter_cs")));
void enter_clock_setting(u8 sys_div, u8 cpu_div, u8 apb_div)
{
	unsigned int volatile i;

	// push register pages to TLB cache entry
	__raw_readl(REG_CLKDIV0);
	__raw_readl(REG_SDEMR);

	if (sys_div > 0) {
		__raw_writel((__raw_readl(REG_CLKDIV0) & ~SYSTEM_N1) | (sys_div<<8), REG_CLKDIV0);
		__raw_writel((__raw_readl(REG_CLKDIV4) & ~(APB_N|CPU_N)) | ((apb_div<<8)|cpu_div), REG_CLKDIV4);
		// disable DLL of SDRAM device
		__raw_writel(__raw_readl(REG_SDEMR) | DLLEN, REG_SDEMR);
		// disable Low Frequency mode
		__raw_writel(__raw_readl(REG_SDOPM) | LOWFREQ, REG_SDOPM);
	} else {
		// disable DLL of SDRAM device
		__raw_writel(__raw_readl(REG_SDEMR) & ~DLLEN, REG_SDEMR);
		// disable Low Frequency mode
		__raw_writel(__raw_readl(REG_SDOPM) & ~LOWFREQ, REG_SDOPM);
		__raw_writel((__raw_readl(REG_CLKDIV0) & ~SYSTEM_N1) | (sys_div<<8), REG_CLKDIV0);
		__raw_writel((__raw_readl(REG_CLKDIV4) & ~(APB_N|CPU_N)) | ((apb_div<<8)|cpu_div), REG_CLKDIV4);
	}
	for (i = 0; i < 0x1000; i++) ;
}

int set_system_clocks(u32 sys_clock, u32 cpu_clock, u32 apb_clock)
{
	void (*cs_func)(u8, u8, u8);
	unsigned long volatile flags;
	u32 int_mask, tmp_system_clock, tmp_cpu_clock, tmp_hclk1_clock;
	u8 sys_div, cpu_div, apb_div;

	sys_div = w55fa93_upll_clock / sys_clock - 1 + (w55fa93_upll_clock%sys_clock ? 1:0);
	tmp_system_clock = w55fa93_upll_clock / (sys_div+1);
	cpu_div = tmp_system_clock / cpu_clock - 1 + (tmp_system_clock%cpu_clock ? 1:0);
	tmp_cpu_clock = tmp_system_clock / (cpu_div+1);
	tmp_hclk1_clock = (tmp_system_clock < tmp_cpu_clock*2) ? tmp_cpu_clock/2:tmp_cpu_clock;
	apb_div = tmp_hclk1_clock / apb_clock - 1 + (tmp_hclk1_clock%apb_clock ? 1:0);
	//printk("sys_div=%d, cpu_div=%d, apb_div=%d\n", sys_div, cpu_div, apb_div);

	if (cpu_div > 1) {
		printk("CPU divider must be 0 or 1 !!\n");
		return -1;
	}

	w55fa93_system_clock = tmp_system_clock;
	w55fa93_cpu_clock = tmp_cpu_clock;
	w55fa93_ahb_clock = w55fa93_system_clock / 2;
	w55fa93_apb_clock = (w55fa93_cpu_clock/2) / (apb_div+1);
#if 0
	printk("SYS clock = %d\n", w55fa93_system_clock);
	printk("CPU clock = %d\n", w55fa93_cpu_clock);
	printk("AHB clock = %d\n", w55fa93_ahb_clock);
	printk("APB clock = %d\n", w55fa93_apb_clock);
	printk("REG_CLKDIV0 = 0x%x\n", __raw_readl(REG_CLKDIV0));
	printk("REG_CLKDIV4 = 0x%x\n", __raw_readl(REG_CLKDIV4));
#endif

	local_irq_save(flags);
	int_mask = __raw_readl(REG_AIC_IMR);
	// disable all interrupts
	__raw_writel(0xFFFFFFFF, REG_AIC_MDCR);
	//restore_flags(flags);

	// put enter_clock_setting into SRAM
	memcpy(sram_vaddr, enter_clock_setting, 512);
	cs_func = (void(*)(u8, u8, u8)) (sram_vaddr);

	// flush all TLB cache entries
	local_flush_tlb_all();
	// change the system clocks
	cs_func(sys_div, cpu_div, apb_div);

	//save_flags(flags);
	//cli();
	__raw_writel(0xFFFFFFFF, REG_AIC_MDCR);
	// restore interrupt mask
	__raw_writel(int_mask, REG_AIC_MECR);
	local_irq_restore(flags);

	return 0;
}

void enter_power_saving(u8) __attribute__ ((section ("enter_ps")));
// power saving mode be 0:standby, 1:power down
void enter_power_saving(u8 stop_xtal)
{
	unsigned int volatile i;

	// push register pages to TLB cache entry
	__raw_readl(REG_CLKDIV0);

	// enable SDRAM self refresh mode
	__raw_writel((__raw_readl(REG_SDCMD)|SELF_REF) & ~AUTOEXSELFREF, REG_SDCMD);
	for (i = 0; i < 0x100; i++) ;

	if (stop_xtal == 1) {
		// change system clock source to external crystal
		__raw_writel(__raw_readl(REG_CLKDIV0) & ~SYSTEM_S, REG_CLKDIV0);
		for (i = 0; i < 0x100; i++) ;
		// stop APLL clcok
		__raw_writel(__raw_readl(REG_APLLCON) | PD, REG_APLLCON);
		// stop UPLL clock
		__raw_writel(__raw_readl(REG_UPLLCON) | PD, REG_UPLLCON);
		for (i = 0; i < 0x300; i++) ;

#if 0
		// stop both of external and CPU clock
		__asm__ __volatile__ \
		( \
			"mov	r2, %0			@ read clock register \n\
			ldmia	r2, {r0, r1}		@ load registers to r0 and r1 \n\
			bic	r0, r0, #0x01		@ \n\
			bic	r1, r1, #0x01		@ \n\
			stmia	r2, {r0, r1}		@ " \
				: /* no output registers */ \
				: "r" (REG_PWRCON) \
				: "r0", "r1", "r2" \
		);
#else
		// stop only external crystal
		__raw_writel(__raw_readl(REG_PWRCON) & ~XTAL_EN, REG_PWRCON);
#endif
	}
	else {
		// stop APLL clcok
		__raw_writel(__raw_readl(REG_APLLCON) | PD, REG_APLLCON);
		for (i = 0; i < 0x300; i++) ;
		// stop CPU clock
		__raw_writel(__raw_readl(REG_AHBCLK) & ~CPU_CKE, REG_AHBCLK);
	}
	for (i = 0; i < 0x300; i++) ;

	if (stop_xtal == 1) {
		// enable APLL clock
		__raw_writel(__raw_readl(REG_APLLCON) & ~PD, REG_APLLCON);
		// enable UPLL clock
		__raw_writel(__raw_readl(REG_UPLLCON) & ~PD, REG_UPLLCON);
		for (i = 0; i < 0x3000; i++) ;
		// restore system clock source to UPLL
		__raw_writel(__raw_readl(REG_CLKDIV0) | SYSTEM_S, REG_CLKDIV0);
		for (i = 0; i < 0x500; i++) ;
	}
	else {
		// enable APLL clock
		__raw_writel(__raw_readl(REG_APLLCON) & ~PD, REG_APLLCON);
		for (i = 0; i < 0x3000; i++) ;
	}

	// exit SDRAM self refresh mode
	__raw_writel(__raw_readl(REG_SDCMD) & ~SELF_REF, REG_SDCMD);
	for (i=0; i<0x100; i++) ;
}

int rtc_add_day(struct rtc_wkalrm *alrm)
{
	unsigned char mdays[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
	unsigned char leap_year_add_day;

	leap_year_add_day = ((__raw_readl(REG_RTC_LIR) & LEAPYEAR) && ((alrm->time).tm_mon == 1)) ? 1 : 0;
	(alrm->time).tm_mday += 1;
	if ((alrm->time).tm_mday > (mdays[(alrm->time).tm_mon] + leap_year_add_day)) {
		(alrm->time).tm_mday = 1;
		(alrm->time).tm_mon += 1;
	}
	if ((alrm->time).tm_mon > 11) {
		(alrm->time).tm_mon = 0;
		(alrm->time).tm_year += 1;
	}

	return 0;
}

#define SHUTDOWN_TIME	30					// seconds
#define SHUTDOWN_COUNT	CLOCK_TICK_RATE/HZ*SHUTDOWN_TIME	// ticks

static ssize_t
read_clk(struct device *dev, struct device_attribute *attr, char *buffer)
{
	return snprintf(buffer, PAGE_SIZE, "%d:%d:%d:%d\n", w55fa93_system_clock, w55fa93_cpu_clock,
			w55fa93_ahb_clock, w55fa93_apb_clock);
}

void w55fa93_poweroff(void)
{
	unsigned long volatile flags;
	int rtc_time_out;

	// disable LVR
	__raw_writel(__raw_readl(REG_MISCR) & ~(LVR_RDY | LVR_EN), REG_MISCR);

	// turn off speaker
#if defined(CONFIG_HANNSTARR_HSD043I9W1_480x272)
	__raw_writel(__raw_readl(REG_GPIOB_OMD) | (1 << 3), REG_GPIOB_OMD);
	__raw_writel(__raw_readl(REG_GPIOB_DOUT) & ~(1 << 3), REG_GPIOB_DOUT);
#elif defined(CONFIG_GOWORLD_GWMTF9360A_320x240)
	__raw_writel(__raw_readl(REG_GPIOE_OMD) | (1 << 1), REG_GPIOE_OMD);
	__raw_writel(__raw_readl(REG_GPIOE_DOUT) & ~(1 << 1), REG_GPIOE_DOUT);
#endif

	// turn off video out
	__raw_writel((__raw_readl(REG_LCM_TVCtl) & ~TVCtl_LCDSrc) | 0x800, REG_LCM_TVCtl);

	// disable system interrupts
	local_irq_save(flags);

#if defined(CONFIG_RTC_DRV_W55FA93)
	__raw_writel(__raw_readl(REG_APBCLK) | RTC_CKE, REG_APBCLK);
	while (1) {
		rtc_time_out = 0;
		// enable long time press power disable
		if ((__raw_readl(REG_RTC_AER) & 0x10000) == 0x0) {
			// set RTC register access enable password
			__raw_writel(0xA965, REG_RTC_AER);
			// make sure RTC register read/write enable
			while ((__raw_readl(REG_RTC_AER) & 0x10000) == 0x0) {
				rtc_time_out++;
				if (rtc_time_out > 0xFFFFFF) {
					printk("RTC Access Eanble Fail\n");
					break;
				}
			}

			// FA93 does not have REG_RTC_REG_FLAG
			//rtc_wait_ready();

			if ((__raw_readl(REG_RTC_AER) & 0x10000) == 0x10000)
				break;
		}
		else
			break;
	}

	// RTC will power off
	__raw_writel((__raw_readl(REG_RTC_PWRON) & ~0x5) | 0x2, REG_RTC_PWRON);
#else
	// turn off power
	__raw_writel(__raw_readl(REG_GPIOD_OMD) | (1<<12), REG_GPIOD_OMD);
	__raw_writel(__raw_readl(REG_GPIOD_DOUT) & ~(1<<12), REG_GPIOD_DOUT);
#endif

	// enable system interrupts
	local_irq_restore(flags);

	// stop CPU clock
	//__raw_writel(__raw_readl(REG_AHBCLK) & ~CPU_CKE, REG_AHBCLK);
	// fix RTC may wakeup fail issue
	__raw_writel(0x0, REG_AHBCLK);

	// wait system enter power off
	while (1) ;
}

void w55fa93_reboot(void)
{
	// turn off speaker
#if defined(CONFIG_HANNSTARR_HSD043I9W1_480x272)
	__raw_writel(__raw_readl(REG_GPIOB_OMD) | (1 << 3), REG_GPIOB_OMD);
	__raw_writel(__raw_readl(REG_GPIOB_DOUT) & ~(1 << 3), REG_GPIOB_DOUT);
#elif defined(CONFIG_GOWORLD_GWMTF9360A_320x240)
	__raw_writel(__raw_readl(REG_GPIOE_OMD) | (1 << 1), REG_GPIOE_OMD);
	__raw_writel(__raw_readl(REG_GPIOE_DOUT) & ~(1 << 1), REG_GPIOE_DOUT);
#endif

	// turn off video out
	__raw_writel((__raw_readl(REG_LCM_TVCtl) & ~TVCtl_LCDSrc) | 0x800, REG_LCM_TVCtl);

	// turn off power
	__raw_writel((__raw_readl(REG_WTCR) & ~(3<<4|1<<10))|0x2C2, REG_WTCR);

	// wait system enter power off
	while (1) ;
}

static ssize_t
write_clk(struct device *dev, struct device_attribute *attr,
	  const char *buffer, size_t count)
{
	void (*ps_func)(u8);
	unsigned long volatile flags;
	int i;

	if (w55fa93_upll_clock == 192000) {
		// SYS:CPU:AHB:APB = 192:192:96:48
		if (buffer[0] == '1' && buffer[1] == '9' && buffer[2] == '2') {
			set_system_clocks(192000, 192000, 48000);
		}

		// SYS:CPU:AHB:APB = 96:96:48:24
		else if (buffer[0] == '9' && buffer[1] == '6') {
			set_system_clocks(96000, 96000, 24000);
		}
	}
	else if (w55fa93_upll_clock == 240000) {
		// SYS:CPU:AHB:APB = 240:240:120:60
		if (buffer[0] == '2' && buffer[1] == '4' && buffer[2] == '0') {
			set_system_clocks(240000, 240000, 60000);
		}

		// SYS:CPU:AHB:APB = 120:120:60:30
		else if (buffer[0] == '1' && buffer[1] == '2' && buffer[2] == '0') {
			set_system_clocks(120000, 120000, 30000);
		}
	}

	// you can decide what clocks are disabled in power saving modes by yourself
	// idle mode or memory idle mode
	if ((buffer[0] == 'i' && buffer[1] == 'd') || (buffer[0] == 'm' && buffer[1] == 'i')) {
		u32 int_mask, ahbclk, apbclk, tcsr0, ticr0;
		u8 shutdown_flag, skip_check_shutdown, do_half_clock;

		if (buffer[0] == 'm' && buffer[1] == 'i') {
			// turn off back light
			__raw_writel(__raw_readl(REG_GPDFUN) & ~(MF_GPD1), REG_GPDFUN);
			__raw_writel(__raw_readl(REG_GPIOD_OMD) | (1<<1), REG_GPIOD_OMD);
			__raw_writel(__raw_readl(REG_GPIOD_DOUT) & ~(1<<1), REG_GPIOD_DOUT);

			// set VPOST to grab build in color instead of SDRAM
			__raw_writel((__raw_readl(REG_LCM_TVCtl) & ~TVCtl_LCDSrc) | 0x800, REG_LCM_TVCtl);
		}

		local_irq_save(flags);
		int_mask = __raw_readl(REG_AIC_IMR);
		// disable all interrupts
		__raw_writel(0xFFFFFFFF, REG_AIC_MDCR);
		//restore_flags(flags);

		// set all SPU channels to be pause state
		__raw_writel(0xFFFFFFFF, REG_SPU_CH_PAUSE);
		// close unused engine clocks
		ahbclk = __raw_readl(REG_AHBCLK);
		apbclk = __raw_readl(REG_APBCLK);
		if (buffer[0] == 'i' && buffer[1] == 'd') {
			// make sure no DMA transaction in progress
			mdelay(100);

			__raw_writel(ahbclk & ~(SEN_CKE|CAP_CKE|SD_CKE|NAND_CKE|SIC_CKE|JPG_CKE), REG_AHBCLK);
			__raw_writel(apbclk & ~(KPI_CKE|WDCLK_CKE|SPIMS1_CKE|SPIMS0_CKE), REG_APBCLK);
		}
		else if (buffer[0] == 'm' && buffer[1] == 'i') {
			// make sure no DMA transaction in progress
			mdelay(100);
			for (i = 0; i < 5; i++)
				while (w55fa93_edma_isbusy(i));

			__raw_writel(ahbclk & ~(SEN_CKE|CAP_CKE|VPOST_CKE|SD_CKE|NAND_CKE|SIC_CKE|EDMA4_CKE|EDMA3_CKE|EDMA2_CKE|EDMA1_CKE|EDMA0_CKE|JPG_CKE|BLT_CKE), REG_AHBCLK);
			__raw_writel(apbclk & ~(KPI_CKE|WDCLK_CKE|SPIMS1_CKE|SPIMS0_CKE|PWM_CKE), REG_APBCLK);
		}
		// divide clocks by 2, ex: drop system clock from 240MHz to 120MHz
		do_half_clock = 0;
		if ((w55fa93_system_clock == 240000) || (w55fa93_system_clock == 192000)) {
			set_system_clocks(w55fa93_system_clock/2, w55fa93_cpu_clock/2, w55fa93_apb_clock/2);
			do_half_clock = 1;
		}

		// store timer register
		tcsr0 = __raw_readl(REG_TCSR0);
		ticr0 = __raw_readl(REG_TICR0);
		// reset timer
		__raw_writel(__raw_readl(REG_APBIPRST) | TMR0RST, REG_APBIPRST);
		__raw_writel(__raw_readl(REG_APBIPRST) & ~TMR0RST, REG_APBIPRST);

		// wait reset complete
		for (i = 0; i < 0x1000; i++) ;
		__raw_writel(0x01, REG_TISR);
		__raw_writel(SHUTDOWN_COUNT, REG_TICR0);
		__raw_writel(0x60010063, REG_TCSR0);

		// enable wake up interrupts
		__raw_writel((1<<IRQ_ADC)|(1<<IRQ_GPIO0)|(1<<IRQ_TIMER0), REG_AIC_MECR);

		shutdown_flag = 0;
		skip_check_shutdown = 0;

		// close audio engine clocks
		__raw_writel(ahbclk & ~(ADO_CKE|I2S_CKE|SPU_CKE), REG_AHBCLK);
		// avoid key and touch pressing
#if defined(CONFIG_W55FA93_KEYPAD) && defined(CONFIG_TOUCHSCREEN_W55FA93)
		if ((w55fa93_key_pressing == 0) && (w55fa93_ts_pressing == 0)) {
#elif defined(CONFIG_W55FA93_KEYPAD)
		if (w55fa93_key_pressing == 0) {
#elif defined(CONFIG_TOUCHSCREEN_W55FA93)
		if (w55fa93_ts_pressing == 0) {
#else
		if (1) {
#endif
			if (buffer[0] == 'i' && buffer[1] == 'd') {
				// stop APLL clcok
				__raw_writel(__raw_readl(REG_APLLCON) | PD, REG_APLLCON);
				for (i = 0; i < 0x300; i++) ;
				// stop CPU clock
				__raw_writel(__raw_readl(REG_AHBCLK) & ~CPU_CKE, REG_AHBCLK);
				for (i = 0; i < 0x300; i++) ;
				// enable APLL clock
				__raw_writel(__raw_readl(REG_APLLCON) & ~PD, REG_APLLCON);
				for (i = 0; i < 0x3000; i++) ;
			}
			else if (buffer[0] == 'm' && buffer[1] == 'i') {
				// kick into memory idle mode, make SDRAM enter self refresh mode
				// put enter_power_saving into SRAM
				memcpy(sram_vaddr, enter_power_saving, 1024);
				ps_func = (void(*)(u8)) sram_vaddr;

				// flush all TLB cache entries
				local_flush_tlb_all();
				// enter to memory idle mode
				ps_func(0x0);
			}
		}
		else
			skip_check_shutdown = 1;

		__raw_writel(0x20000063, REG_TCSR0);
		if ((__raw_readl(REG_TDR0) == 0x1) && (!skip_check_shutdown))
			shutdown_flag = 1;

		if ((ahbclk & VPOST_CKE) && !(__raw_readl(REG_AHBCLK) & VPOST_CKE)) {
			__raw_writel(__raw_readl(REG_AHBCLK) | VPOST_CKE, REG_AHBCLK);
#if defined(CONFIG_HANNSTARR_HSD043I9W1_480x272)
			mdelay(300);
#elif defined(CONFIG_GOWORLD_GW8973_480x272)
			mdelay(200);
#endif
		}

		// restore clocks to full speed
		if (do_half_clock)
			set_system_clocks(w55fa93_system_clock*2, w55fa93_cpu_clock*2, w55fa93_apb_clock*2);
		// restore registers
		__raw_writel(ahbclk, REG_AHBCLK);
		__raw_writel(apbclk, REG_APBCLK);
		// set all SPU channels to be normal state
		__raw_writel(0x00000000, REG_SPU_CH_PAUSE);

		if (shutdown_flag == 1) {
#if defined(CONFIG_W55FA93_SYSMGR) || defined(CONFIG_W55FA93_SYSMGR_MODULE)
			sysmgr_report(SYSMGR_STATUS_RTC_POWER_OFF);
			printk("sysmgr_report(SYSMGR_STATUS_RTC_POWER_OFF)\n");
#endif
		}

		// reset timer
		__raw_writel(__raw_readl(REG_APBIPRST) | TMR0RST, REG_APBIPRST);
		__raw_writel(__raw_readl(REG_APBIPRST) & ~TMR0RST, REG_APBIPRST);
		// wait reset complete
		for (i = 0; i < 0x1000; i++) ;
		// store timer register
		__raw_writel(0x01, REG_TISR);
		__raw_writel(tcsr0, REG_TCSR0);
		__raw_writel(ticr0, REG_TICR0);

		//save_flags(flags);
		//cli();
		__raw_writel(0xFFFFFFFF, REG_AIC_MDCR);
		// restore interrupt mask
		__raw_writel(int_mask, REG_AIC_MECR);
		local_irq_restore(flags);

		if (shutdown_flag == 0) {
			// VPOST get SDRAM data
			__raw_writel((__raw_readl(REG_LCM_TVCtl) & ~TVCtl_LCDSrc) | 0x400, REG_LCM_TVCtl);

			// turn on back light
			__raw_writel(__raw_readl(REG_GPIOD_DOUT) | (1<<1), REG_GPIOD_DOUT);
		}
	}

	// power down mode
	else if (buffer[0] == 'p' && buffer[1] == 'd') {
		u32 int_mask, ahbclk, apbclk;
		u8 shutdown_flag;
		struct rtc_wkalrm alrm, alrm_bak;

		// turn off back light
		__raw_writel(__raw_readl(REG_GPDFUN) & ~(MF_GPD1), REG_GPDFUN);
		__raw_writel(__raw_readl(REG_GPIOD_OMD) | (1<<1), REG_GPIOD_OMD);
		__raw_writel(__raw_readl(REG_GPIOD_DOUT) & ~(1<<1), REG_GPIOD_DOUT);

		// set VPOST to grab build in color instead of SDRAM
		__raw_writel((__raw_readl(REG_LCM_TVCtl) & ~TVCtl_LCDSrc) | 0x800, REG_LCM_TVCtl);

		local_irq_save(flags);
		int_mask = __raw_readl(REG_AIC_IMR);
		// disable all interrupts
		__raw_writel(0xFFFFFFFF, REG_AIC_MDCR);
		//restore_flags(flags);

#if 0
		// make sure no DMA transaction in progress
		mdelay(100);
		for (i = 0; i < 5; i++)
			while (w55fa93_edma_isbusy(i));
#endif

		// set all SPU channels to be pause state
		__raw_writel(0xFFFFFFFF, REG_SPU_CH_PAUSE);
		// close VPOST engine clock
		ahbclk = __raw_readl(REG_AHBCLK);
		__raw_writel(ahbclk & ~(VPOST_CKE), REG_AHBCLK);
		// open RTC engine clock
		apbclk = __raw_readl(REG_APBCLK);
		__raw_writel(apbclk | (RTC_CKE), REG_APBCLK);

		// clear wake-up status
		__raw_writel(__raw_readl(REG_MISSR) | 0xFF000000, REG_MISSR);
#if defined(CONFIG_SND_SOC_W55FA93_ADC)
		if (__raw_readl(REG_AUDIO_CON) & AUDIO_EN) {
			// enable wake-up source
			__raw_writel((__raw_readl(REG_MISSR) & ~0x00FF0000) | (GPIO_WE|RTC_WE), REG_MISSR);
		} else
#endif
		{
			// for ADC wake up
			// enable wake-up source
			__raw_writel((__raw_readl(REG_MISSR) & ~0x00FF0000) | (ADC_WE|GPIO_WE|RTC_WE), REG_MISSR);
			// enable pull up PMOS
			//__raw_writel(__raw_readl(REG_ADC_TSC) | ADC_PU_EN, REG_ADC_TSC);
			// set ADC config for power down mode
			__raw_writel((WT_INT_EN|ADC_INT_EN|ADC_CON_ADC_EN|ADC_TSC_MODE), REG_ADC_CON);
		}

		// for GPIO wake up
		// enable IRQ0 wake up and latch
		__raw_writel(0x11, REG_IRQLHSEL);
		// clear IRQ0 interrupt status
		__raw_writel(__raw_readl(REG_IRQTGSRC0) | 0x1C, REG_IRQTGSRC0);

#if defined(CONFIG_RTC_DRV_W55FA93)
		// for RTC wake up
		// set RTC alarm time
		memset(&alrm, 0, sizeof(struct rtc_wkalrm));
		memset(&alrm_bak, 0, sizeof(struct rtc_wkalrm));
		w55fa93_rtc_read_alarm_wrap(&alrm_bak);
		w55fa93_rtc_read_time_wrap(&alrm.time);
		alrm.enabled = 1;
		alrm.pending = 0;
#if 0
		printk("YEAR=%d : ", alrm.time.tm_year);
		printk("MON=%d : ", alrm.time.tm_mon);
		printk("DAY=%d : ", alrm.time.tm_mday);
		printk("HOUR=%d : ", alrm.time.tm_hour);
		printk("MIN=%d : ", alrm.time.tm_min);
		printk("SEC=%d\n", alrm.time.tm_sec);
#endif
		alrm.time.tm_sec += SHUTDOWN_TIME;
		if (alrm.time.tm_sec > 59) {
			alrm.time.tm_sec -= 60;
			alrm.time.tm_min += 1;
		}
		if (alrm.time.tm_min > 59) {
			alrm.time.tm_min = 0;
			alrm.time.tm_hour += 1;
		}
		if (alrm.time.tm_hour > 23) {
			alrm.time.tm_hour = 0;
			rtc_add_day(&alrm);
		}
		w55fa93_rtc_set_alarm_wrap(&alrm);
#endif

		shutdown_flag = 0;

		// close audio engine clocks
		__raw_writel(__raw_readl(REG_AHBCLK) & ~(ADO_CKE|I2S_CKE|SPU_CKE), REG_AHBCLK);
		// close all edma engine clocks
		__raw_writel(__raw_readl(REG_AHBCLK) & ~(EDMA0_CKE|EDMA1_CKE|EDMA2_CKE|EDMA3_CKE|EDMA4_CKE), REG_AHBCLK);
		// kick into power down mode, make SDRAM enter self refresh mode
		// put enter_power_saving into SRAM
		memcpy(sram_vaddr, enter_power_saving, 1024);
		ps_func = (void(*)(u8)) sram_vaddr;
		// flush all TLB cache entries
		local_flush_tlb_all();
		// enter to power down mode
		ps_func(0x1);

		printk("REG_MISSR=0x%x\n", __raw_readl(REG_MISSR));
#if defined(CONFIG_W55FA93_SYSMGR) || defined(CONFIG_W55FA93_SYSMGR_MODULE)
		if (__raw_readl(REG_MISSR) & RTC_WS) {
			shutdown_flag = 1;
			sysmgr_report(SYSMGR_STATUS_RTC_POWER_OFF);
			printk("sysmgr_report(SYSMGR_STATUS_RTC_POWER_OFF)\n");
		}
#endif
		// clear wake-up status
		__raw_writel(__raw_readl(REG_MISSR) | 0xFF000000, REG_MISSR);

#if defined(CONFIG_RTC_DRV_W55FA93)
		// restore RTC alarm time
		w55fa93_rtc_set_alarm_wrap(&alrm_bak);
#endif

		if (ahbclk & VPOST_CKE) {
			__raw_writel(__raw_readl(REG_AHBCLK) | VPOST_CKE, REG_AHBCLK);
#if defined(CONFIG_HANNSTARR_HSD043I9W1_480x272)
			mdelay(300);
#elif defined(CONFIG_GOWORLD_GW8973_480x272)
			mdelay(200);
#endif
		}
		// restore registers
		__raw_writel(ahbclk, REG_AHBCLK);
		__raw_writel(apbclk, REG_APBCLK);
		// set all SPU channels to be normal state
		__raw_writel(0x00000000, REG_SPU_CH_PAUSE);

		//save_flags(flags);
		//cli();
		// restore interrupt mask
		__raw_writel(int_mask, REG_AIC_MECR);
		local_irq_restore(flags);

		if (shutdown_flag == 0) {
			// VPOST get SDRAM data
			__raw_writel((__raw_readl(REG_LCM_TVCtl) & ~TVCtl_LCDSrc) | 0x400, REG_LCM_TVCtl);

			// turn on back light
			__raw_writel(__raw_readl(REG_GPIOD_DOUT) | (1<<1), REG_GPIOD_DOUT);
		}
	}

#if defined(CONFIG_RTC_DRV_W55FA93)
	// RTC power off mode
	else if (buffer[0] == 'r' && buffer[1] == 'p' && buffer[2] == 'o') {
		w55fa93_poweroff();
	}
#else
	// power off mode
	else if (buffer[0] == 'p' && buffer[1] == 'o') {
		w55fa93_poweroff();
	}
#endif

	// power reset mode
	else if (buffer[0] == 'p' && buffer[1] == 'r') {
		w55fa93_reboot();
	}

	return count;
}

/* Attach the sysfs write method */
DEVICE_ATTR(clock, 0644, read_clk, write_clk);

/* Attribute Descriptor */
static struct attribute *clk_attrs[] = {
	&dev_attr_clock.attr,
	NULL
};

/* Attribute group */
static struct attribute_group clk_attr_group = {
	.attrs = clk_attrs,
};

static int __init w55fa93_system_clock_init(void)
{
	/* Register a platform device */
	printk("register clock device\n");

	sys_clk = platform_device_register_simple("w55fa93-clk", -1, NULL, 0);
	if (sys_clk == NULL)
		printk("register failed\n");
	sysfs_create_group(&sys_clk->dev.kobj, &clk_attr_group);
	sram_vaddr = ioremap(0xFF000000, 4*1024);

	return 0;
}

module_init(w55fa93_system_clock_init);

#if 0
static int __init w55fa93_arch_init(void)
{
	int ret;
//	struct platform_device **ptr = w55fa93_board.devices;
	struct platform_device *ptr;
	ptr= &w55fa93_lcddevice;
	ret = platform_device_register(&w55fa93_lcddevice);
	printk("### Call platform_device_register in %s \n", __FUNCTION__);
	if (ret) {
		printk(KERN_ERR "w55fa93: failed to add board device %s (%d) @%p\n", (ptr)->name, ret, ptr);
	}

	return 0;
}

arch_initcall(w55fa93_arch_init);
#endif
