#ifndef __KEYPAD_H__
#define __KEYPAD_H__

#if 1   
#define KEYP_1                   0x31
#define KEYP_2                   0x32
#define KEYP_3                   0x33
#define KEYP_4                   0x34
#define KEYP_5                   0x35
#define KEYP_6                   0x36
#define KEYP_7                   0x37
#define KEYP_8                   0x38
#define KEYP_9                   0x39
#define KEYP_0                   0x30

#define KEYP_DZERO               '*'//0x2A
#define KEYP_TZERO               '#'//0x2E

#define KEYP_BACKSPACE           0xfa//0x68
#define KEYP_END                 0xfb//0x6A
#define KEYP_ENTER               0xfc//0x69

#define KEYP_F1                  0xf0//0x60
#define KEYP_F2                  0xf1//0x61
#define KEYP_F3                  0xf2//0x62
#define KEYP_F4                  0xf3//0x63
#define KEYP_F5                  0xf4//0x64

#define KEYP_UP                  0xf5//0x65
#define KEYP_TOTAL               0xf6//0x66
#define KEYP_DOWN                0xf7//0x67

#else   /* linux/input.h 에 있는 key code 사용 {{{1 */
    // #define KEY_DZERO               KEY_D
    // #define KEY_TZERO               KEY_T
    #define KEY_DZERO               KEY_F8
    #define KEY_TZERO               KEY_F9
    #define KEY_TOTAL               KEY_F10
#endif  /* }}}1 */

#define KEY_IOCTL_MAGIC             'k'
#define KEY_INIT                    _IO(    KEY_IOCTL_MAGIC, 0)
#define KEY_CONTINUE_CHK            _IOW(   KEY_IOCTL_MAGIC, 1, unsigned char)

#define KEY_SET_SCANTIME            _IOW(    KEY_IOCTL_MAGIC, 2, unsigned short) // add 20151001_sjh
#define KEY_GET_EXISTKEY            _IOR(   KEY_IOCTL_MAGIC, 3,  unsigned char)// add 20151001_sjh
#define KEY_GET_PRESSKEY            _IOR(   KEY_IOCTL_MAGIC, 4,  unsigned char)// add 20151001_sjh
#define KEY_GET_KEY                  _IOR(   KEY_IOCTL_MAGIC, 5,  unsigned char)// add 20151001_sjh
#define KEY_GET_BUFF_LEN             _IOR(   KEY_IOCTL_MAGIC, 6,  unsigned short)// add20151001_sjh

#define KEY_VERSION                 _IOR(   KEY_IOCTL_MAGIC, 99, unsigned char *)



#endif	//_KEYPAD_H__

