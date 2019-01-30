/*
 * DATE		: 2014/08/13
 * Author	: shellbt
 * How to 	:
 * 	GPIO_MODEM, GPIO_PRIVATE, GPIO_PINPAD2, GPIO_DEBUG : 인자값에  0을 주면 현재 상태값을 인자에 넣어서 반환
 *
 * 	GPIO_KEY_LED, GPIO_ON_LED, GPIO_ICC_LED, GPIO_SAM_SEL, GPIO_USB_PWR : 인자값에 따라 ON/OFF을 실행(0을 주면 OFF),
 * 		-1을 주면 상태값만 읽어 온다.
 *
 * 	GPIO_USB_VBUS : 상태값만 반환
 *
 *  GPIO_DRAWER : 0 이상의 값을 주면 준 값 동안 High을 유지함.(ms 단위)
 *  GPIO_ICC_RESET : 1 - ON, 0 - OFF, -1 : 상태값 읽어오기
 */
#ifndef __GPIO_H__
#define __GPIO_H__


#define LED_CNTL_KEY	1
#define LED_CNTL_ONLINE	2
#define LED_CNTL_ICC	3
#define nLED_CNTL		4



struct LedControl {
	int type;		// LED_CNTL_LCD ~ LED_CNTL_ICC (0 ~ 3)
	int on_time;	// * 100ms	ex) 1 sec = 10
	int off_time;	// * 100ms	ex) 1 sec = 10
	int loop;	// 0 : infinite, 1~n : loop count
};


#define GPIO_IOCTL_MAGIC         'g'

//#define GPIO_INIT                   _IO  (GPIO_IOCTL_MAGIC,  1)
#define GPIO_MODEM                  _IOW(GPIO_IOCTL_MAGIC,  1, unsigned long)
#define GPIO_KEY_LED                _IOW(GPIO_IOCTL_MAGIC,  2, unsigned long)
#define GPIO_ON_LED                 _IOW(GPIO_IOCTL_MAGIC,  3, unsigned long)
#define GPIO_ICC_LED                _IOW(GPIO_IOCTL_MAGIC,  4, unsigned long)

#define GPIO_ICC_RESET				_IOW(GPIO_IOCTL_MAGIC, 5, unsigned long)
#define GPIO_SAM_SEL				_IOW(GPIO_IOCTL_MAGIC, 6, unsigned long)
#define GPIO_USB_PWR				_IOW(GPIO_IOCTL_MAGIC, 7, unsigned long)
#define GPIO_USB_VBUS				_IOR (GPIO_IOCTL_MAGIC, 8, unsigned long)

#define GPIO_MODEM_RESET			_IOW(GPIO_IOCTL_MAGIC, 9, unsigned long)
#define GPIO_ICC_DETECT             _IOR (GPIO_IOCTL_MAGIC, 10, unsigned long)
#define GPIO_READ                   _IOR (GPIO_IOCTL_MAGIC, 11, unsigned char)
#define GPIO_LED_CONTROL	     _IOW (GPIO_IOCTL_MAGIC, 12, struct LedControl)

#define GPIO_MODEM_ESC  			_IOW(GPIO_IOCTL_MAGIC, 13, unsigned long)
//20170223_sjh
#define GPIO_WAN_PWR_EN	     _IOW(GPIO_IOCTL_MAGIC, 13, unsigned char)
#define GPIO_WAN_ON_OFF	     _IOW(GPIO_IOCTL_MAGIC, 14, unsigned char)
#define GPIO_WAN_DM_EN	     _IOW(GPIO_IOCTL_MAGIC, 15, unsigned char)

//20170320_sjh re-add drawer
#define GPIO_DRAWER					_IOW (GPIO_IOCTL_MAGIC, 16 , int)

#define GPIO_PSTN_PWR_EN	       _IOW(GPIO_IOCTL_MAGIC, 17, unsigned char)

#define GPIO_VERSION                _IOR (GPIO_IOCTL_MAGIC, 99, char *)

#endif	//__GPIO_H__

