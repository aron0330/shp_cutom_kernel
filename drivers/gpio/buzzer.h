#ifndef __BUZZER_H_
#define __BUZZER_H_


struct buzzer_param2 {
	unsigned int 	type[10];
	unsigned int	count[10];    
	unsigned int 	mson[10];
	unsigned int 	msoff[10];
	unsigned int 	buzzer_seq;
    unsigned int 	melody_MaxCnt;
};

struct buzzer_param {	
    unsigned int type;
	unsigned int count;
	unsigned int mson;
	unsigned int msoff;
};
#define BUZZER_IOCTL_MAGIC      	'b'

#define BUZZER_INIT					_IO (BUZZER_IOCTL_MAGIC, 0)
#define BUZZER_ON					_IO (BUZZER_IOCTL_MAGIC, 1)
#define BUZZER_OFF					_IO (BUZZER_IOCTL_MAGIC, 2)
#define BUZZER_TONE					_IOW(BUZZER_IOCTL_MAGIC, 3, struct buzzer_param *)
//#define BUZZER_VERSION				_IOR (BUZZER_IOCTL_MAGIC, 20, unsigned char *)


#define BUZZER_STATUS		_IOR(BUZZER_IOCTL_MAGIC, 4, unsigned int)

#define BUZZER_CUSTOM_FREQ	_IOW(BUZZER_IOCTL_MAGIC, 5,  struct buzzer_param2 *)

#define BUZZER_VERSION		_IOR(BUZZER_IOCTL_MAGIC, 99, unsigned char *)


#define PWM_MR_DIVB_Pos                             16
#define PWM_MR_DIVA_Pos                             0
#define PWM_MR_PREA_Pos                             8
#define PWM_MR_PREB_Pos                             24

#define PWM_MR_PREA_MCK                             (0x0u << 8) /* (PWM_MR) Master Clock */
#define PWM_MR_PREA_MCKDIV2                         (0x1u << 8) /* (PWM_MR) Master Clock divided by 2 */
#define PWM_MR_PREA_MCKDIV4                         (0x2u << 8) /* (PWM_MR) Master Clock divided by 4 */
#define PWM_MR_PREA_MCKDIV8                         (0x3u << 8) /* (PWM_MR) Master Clock divided by 8 */
#define PWM_MR_PREA_MCKDIV16                        (0x4u << 8) /* (PWM_MR) Master Clock divided by 16 */
#define PWM_MR_PREA_MCKDIV32                        (0x5u << 8) /* (PWM_MR) Master Clock divided by 32 */
#define PWM_MR_PREA_MCKDIV64                        (0x6u << 8) /* (PWM_MR) Master Clock divided by 64 */
#define PWM_MR_PREA_MCKDIV128                       (0x7u << 8) /* (PWM_MR) Master Clock divided by 128 */
#define PWM_MR_PREA_MCKDIV256                       (0x8u << 8) /* (PWM_MR) Master Clock divided by 256 */
#define PWM_MR_PREA_MCKDIV512                       (0x9u << 8) /* (PWM_MR) Master Clock divided by 512 */
#define PWM_MR_PREA_MCKDIV1024                      (0xAu << 8) /* (PWM_MR) Master Clock divided by 1024 */

#define PWM_MR_PREB_MCK                             (0x0u << 24) /* (PWM_MR) Master Clock */
#define PWM_MR_PREB_MCKDIV2                         (0x1u << 24) /* (PWM_MR) Master Clock divided by 2 */
#define PWM_MR_PREB_MCKDIV4                         (0x2u << 24) /* (PWM_MR) Master Clock divided by 4 */
#define PWM_MR_PREB_MCKDIV8                         (0x3u << 24) /* (PWM_MR) Master Clock divided by 8 */
#define PWM_MR_PREB_MCKDIV16                        (0x4u << 24) /* (PWM_MR) Master Clock divided by 16 */
#define PWM_MR_PREB_MCKDIV32                        (0x5u << 24) /* (PWM_MR) Master Clock divided by 32 */
#define PWM_MR_PREB_MCKDIV64                        (0x6u << 24) /* (PWM_MR) Master Clock divided by 64 */
#define PWM_MR_PREB_MCKDIV128                       (0x7u << 24) /* (PWM_MR) Master Clock divided by 128 */
#define PWM_MR_PREB_MCKDIV256                       (0x8u << 24) /* (PWM_MR) Master Clock divided by 256 */
#define PWM_MR_PREB_MCKDIV512                       (0x9u << 24) /* (PWM_MR) Master Clock divided by 512 */
#define PWM_MR_PREB_MCKDIV1024                      (0xAu << 24) /* (PWM_MR) Master Clock divided by 1024 */

#define PWM_CMR_CPRE_CLKA                           (0xBu << 0) /* (PWM_CMR) Clock A */
#define PWM_CMR_CPRE_CLKB                           (0xCu << 0) /* (PWM_CMR) Clock B */
#define PWM_CMR_CALG_CENTER							( 1 << 8 )
#define PWM_CMR_CALG_LEFT							( ~(1 << 8) )

/* global PWM controller registers */
#define PWM_MR                                      0x00
#define PWM_ENA                                     0x04
#define PWM_DIS                                     0x08
#define PWM_SR                                      0x0c
#define PWM_IER                                     0x10
#define PWM_IDR                                     0x14
#define PWM_IMR                                     0x18
#define PWM_ISR                                     0x1c

/* PWM Channel Registers */
#define PWM_CMR(ch)                                 ( 0x200 + (ch*0x20) + 0x00 )
#define PWM_CDTY(ch)                                ( 0x200 + (ch*0x20) + 0x04 )

#define PWM_CPRD(ch)                                ( 0x200 + (ch*0x20) + 0x0c )

//#define PWM_CCNT(ch)                                ( 0x200 + (ch*0x20) + 0x0C )
//#define PWM_CUPD(ch)                                ( 0x200 + (ch*0x20) + 0x10 )

#define BUZZER_CHAN                                 0

/* buzzer default values */
#define PRESCALER_DEFAULT							( PWM_MR_PREA_MCK /*| 32 << PWM_MR_DIVA_Pos*/ )
#define PERIOD_DEFAULT                              1024
#define DUTY_DEFAULT                                512


/* default prescaler, (MCK / 2) / 32 */
#define BUZZER_PRESCALER_DEFAULT					pwm_writel(pwm0_base, PWM_MR, (PWM_MR_PREA_MCK | 32 << PWM_MR_DIVA_Pos) )
/* pwm0 disable */
#define BUZZER_DISABLE								pwm_writel(pwm0_base, PWM_DIS, (1 << BUZZER_CHAN) )
/* pwm0 clock source */
#define BUZZER_DEFAULT_CLKSRC	pwm_writel(pwm0_base, PWM_CMR(BUZZER_CHAN), PWM_CMR_CPRE_CLKA)
/* pwm0 period */
#define BUZZER_PERIOD(x)							pwm_writel(pwm0_base, PWM_CPRD(BUZZER_CHAN), x)
/* pwm0 duty */
#define BUZZER_DUTY(x)								pwm_writel(pwm0_base, PWM_CDTY(BUZZER_CHAN), x)
/* pwm0 enable */
#define BEEP_ON										pwm_writel(pwm0_base, PWM_ENA, (1 << BUZZER_CHAN) )
/* pwm0 disable */
#define BEEP_OFF									pwm_writel(pwm0_base, PWM_DIS, (1 << BUZZER_CHAN) )
/* pwm0 default beep */
#define BUZZER_DEFAULT								{\
														BUZZER_PERIOD(1024); \
														BUZZER_DUTY(512); \
														BEEP_ON;\
														mdelay(50); \
														BEEP_OFF; \
													}
/* pwm0 beep */
#define BUZZER(val,time)							{\
														pwm_writel(pwm0_base, PWM_CPRD(BUZZER_CHAN), x );\
														pwm_writel(pwm0_base, PWM_CPRD(BUZZER_CHAN), x/2);\
														BEEP_ON; \
														mdelay(time); \
														BEEP_OFF; \
													}


#endif	//__BUZZER_H_

