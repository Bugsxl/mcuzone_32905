#ifndef __POWER_MANAGEMENT_H__
#define __POWER_MANAGEMENT_H__

#define POWER_STATE_NORMAL		0x01000000
#define POWER_STATE_DISPLAY_OFF		0x02000000
#define POWER_STATE_IDLE		0x04000000
#define POWER_STATE_MEMORY_IDLE		0x08000000
#define POWER_STATE_POWER_DOWN		0x10000000
#define POWER_STATE_RTC_POWER_OFF	0x20000000
#define POWER_STATE_POWER_OFF		0x40000000

#define SYSMGR_STATUS_SD_INSERT		0x00000010
#define SYSMGR_STATUS_SD_REMOVE		0x00000020
#define SYSMGR_STATUS_USBD_PLUG		0x00000040
#define SYSMGR_STATUS_USBD_UNPLUG	0x00000080
#define SYSMGR_STATUS_AUDIO_OPEN	0x00000100
#define SYSMGR_STATUS_AUDIO_CLOSE	0x00000200
#define SYSMGR_STATUS_NORMAL		0x01000000
#define SYSMGR_STATUS_DISPLAY_OFF	0x02000000
#define SYSMGR_STATUS_IDLE		0x04000000
#define SYSMGR_STATUS_MEMORY_IDLE	0x08000000
#define SYSMGR_STATUS_POWER_DOWN	0x10000000
#define SYSMGR_STATUS_RTC_POWER_OFF	0x20000000
#define SYSMGR_STATUS_POWER_OFF		0x40000000

#define SYSMGR_CMD_NORMAL		0x01000000
#define SYSMGR_CMD_DISPLAY_OFF		0x02000000
#define SYSMGR_CMD_IDLE			0x04000000
#define SYSMGR_CMD_MEMORY_IDLE		0x08000000
#define SYSMGR_CMD_POWER_DOWN		0x10000000
#define SYSMGR_CMD_RTC_POWER_OFF	0x20000000
#define SYSMGR_CMD_POWER_OFF		0x40000000

#define SYSMGR_IOC_MAXNR		10
#define SYSMGR_IOC_MAGIC		'S'
#define SYSMGR_IOC_SET_POWER_STATE	_IOW(SYSMGR_IOC_MAGIC, 0, unsigned int)
#define SYSMGR_IOC_GET_USBD_STATE	_IOR(SYSMGR_IOC_MAGIC, 1, unsigned int)
#define SYSMGR_IOC_GET_POWER_KEY	_IOR(SYSMGR_IOC_MAGIC, 3, unsigned int)

#define VIDEO_DISPLAY_ON		_IOW('v', 24, unsigned int)
#define VIDEO_DISPLAY_OFF		_IOW('v', 25, unsigned int)

#ifdef __cplusplus
extern "C"
{
#endif

bool PM_Init(void);
void PM_Service(void);
bool PM_ProcessPowerState(int power_state);

#ifdef __cplusplus
}
#endif

#endif
