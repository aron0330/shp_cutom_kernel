/*
 *
 *
 *  Revision History
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */


#include <linux/fs.h>
#include <linux/string.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/timer.h>

#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/ioctl.h>


#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/proc_fs.h>

#include <linux/miscdevice.h>
#include <linux/gpio.h>


#include <linux/mfd/atc260x/atc260x.h>
#include <linux/export.h>
#include <linux/power_supply.h>
#include <asm/smp_scu.h>
#include <linux/of_gpio.h>

//#include <linux/clk/at91_pmc.h> /* add aron */

//#include <mach/gpio.h>
//#include <mach/at91sam9x5.h>
//#include <mach/at91_pmc.h>

#include "buzzer.h"


#define ON 	1
#define OFF 0

//#define DEBUG
#ifdef DEBUG
#define DPRINTK(format,args...)     printk(KERN_ALERT "%s: " format, __FUNCTION__, ## args)
#else
#define DPRINTK(format,args...)
#endif 

#define MODEL_NAME                                  "SHC-650"
#define DEVICE_NAME                                 "buzzer"
#define RELEASE_VERSION                             "0.2"

//0.2 : ±âº»À½À»  2093 -> 2050À¸·Î º¯°æ 

static void __iomem *pwm0_base;

#define GET_RELOADV_FROM_HZ(_hz)    (2593750UL/(_hz)) // 2.66mhz ? 166 / 64
#define GET_DUTY(_hz,_duty)         (((2593750UL/(_hz))*_duty)/100)

static struct _pwm_table_t {
	unsigned int reloadv;
	unsigned int duty;
} pwm_table[] = {
#if 0
		{ 1024, 512 },			/* default */
		{ 912,  460 },			/* 90% */
		{ 819,	409 },			/* 80% */
		{ 716,  358 },			/* 70% */
		{ 614,  307 },			/* 60% */
		{ 512,  256 }			/* 50% */
#else
//    { GET_RELOADV_FROM_HZ(2093), GET_DUTY(2093,50), },   // Default value
/*    { GET_RELOADV_FROM_HZ(2050), GET_DUTY(2050,50), },   // Default value
    { GET_RELOADV_FROM_HZ(2217), GET_DUTY(2217,50), },
    { GET_RELOADV_FROM_HZ(2349), GET_DUTY(2349,50), },
    { GET_RELOADV_FROM_HZ(2489), GET_DUTY(2489,50), },
    { GET_RELOADV_FROM_HZ(2637), GET_DUTY(2637,50), },
    { GET_RELOADV_FROM_HZ(2793), GET_DUTY(2793,50), },
    { GET_RELOADV_FROM_HZ(2959), GET_DUTY(2959,50), },
    { GET_RELOADV_FROM_HZ(3135), GET_DUTY(3135,50), },
    { GET_RELOADV_FROM_HZ(3322), GET_DUTY(3322,50), },

    { GET_RELOADV_FROM_HZ(3520), GET_DUTY(3520,50), },
    { GET_RELOADV_FROM_HZ(3729), GET_DUTY(3729,50), },
    { GET_RELOADV_FROM_HZ(3951), GET_DUTY(3951,50), },*/

//add new@
    { GET_RELOADV_FROM_HZ(2050), GET_DUTY(2050,50), }, //12
    { GET_RELOADV_FROM_HZ(912<<1), GET_DUTY(912<<1,50), },
    { GET_RELOADV_FROM_HZ(819<<1), GET_DUTY(819<<1,50), },
    { GET_RELOADV_FROM_HZ(716<<1), GET_DUTY(716<<1,50), },
    { GET_RELOADV_FROM_HZ(614<<1), GET_DUTY(614<<1,50), },
    { GET_RELOADV_FROM_HZ(512<<1), GET_DUTY(512<<1,50), },
    
    	
#endif
};

static unsigned char buzzer_flag = 0;
static unsigned char ucBuzzerOnOffflag;
static unsigned long ulDeviceOpen;
///////////////////////////////////////////////////////////////////////////////////
// proc_fs system
///////////////////////////////////////////////////////////////////////////////////
#define PROC_BUZ_DIR						"buzzer"
#define PROC_BUZ_PERIOD						"period"
#define PROC_BUZ_DUTY						"duty"
#define PROC_BUZ_ONOFF						"onoff"

#define PROC_BUZ_LEN						5

#define PROC_PERIOD							0x01
#define PROC_DUTY							0x02
#define PROC_ONOFF							0x03

static struct proc_dir_entry *buz_proc_dir, *buz_proc_period, \
							 *buz_proc_duty, *buz_proc_onoff;


static DEFINE_MUTEX(buzzer_mutex);

///////////////////////////////////////////////////////////////////////////////////
// kernel timer
///////////////////////////////////////////////////////////////////////////////////
static struct timer_list beep_timer;
//struct buzzer_param Buzzer_strct;   
struct buzzer_param2 Buzzer_strct;   
static unsigned int gBuzzer_index;
/*
 * Device Operations
 */
static inline void pwm_writel(const void __iomem *p, unsigned offset, u32 val)
{
//    printk("addr %x / data %x\n",p + offset,val);
	__raw_writel(val, p + offset);
}

static inline u32 pwm_readl(const void __iomem *p, unsigned offset)
{
	return __raw_readl(p + offset);
}

static void set_pwm(unsigned long type)
{
    type %= ARRAY_SIZE(pwm_table);
    BUZZER_PERIOD( pwm_table[type].reloadv );
    BUZZER_DUTY( pwm_table[type].duty );
   // printk("ty = %d / period : %d / duty %d\n",type,pwm_table[type].reloadv,pwm_table[type].duty );
}

static void set_pwm_custom(unsigned int freq)
{    
    BUZZER_PERIOD( GET_RELOADV_FROM_HZ(freq) );
    BUZZER_DUTY( GET_DUTY(freq,50) );
}

static int pwm_onoff(unsigned long on)
{
   
    if (on && ucBuzzerOnOffflag) 
    {
        buzzer_flag = ON;    
        BEEP_ON;
    } 
    else 
    {
        buzzer_flag = OFF;
        BEEP_OFF;
    }

    return 0;
}
/*!
 \function gpio_init_set
 \brief PIO ?€?¤ì„ ì´ˆê¸°???œí‚¨??
 \retval static void :
 \param void :
*/
static void gpio_init_set(void)
{
    //if ( at91_set_C_periph(AT91_PIN_PC18, 1) )
//        printk(KERN_ALERT "PC18 change peripheral C failed\n");
//    DPRINTK("PC18 = 0x%x\n", at91_get_gpio_value(AT91_PIN_PC18) );
}


static void beep_timer_callback(unsigned long data)
{
    
	DPRINTK("start!\n");
#if 0    

#else
    BEEP_OFF;
    if(Buzzer_strct.count[gBuzzer_index]>0)
    {
        if(Buzzer_strct.buzzer_seq==ON)
        {
            Buzzer_strct.buzzer_seq = OFF;
            set_pwm(Buzzer_strct.type[gBuzzer_index]);
            pwm_onoff(ON);
            
            mod_timer( &beep_timer, 
                jiffies + msecs_to_jiffies(Buzzer_strct.mson[gBuzzer_index]) ); // 1ms 
            DPRINTK("ON!\n");
        }
        else{
            Buzzer_strct.buzzer_seq = ON;
            set_pwm(Buzzer_strct.type[gBuzzer_index]);
            //pwm_onoff(OFF);
            mod_timer( &beep_timer, 
                jiffies + msecs_to_jiffies(Buzzer_strct.msoff[gBuzzer_index]) ); // 1ms 
            DPRINTK("OFF!\n");            
        }

        Buzzer_strct.count[gBuzzer_index]--;
        
    }
    else
    {
        //set_pwm(0);
        pwm_onoff(OFF);
        
        if(gBuzzer_index<Buzzer_strct.melody_MaxCnt)
        {
            gBuzzer_index++;

            if(gBuzzer_index != Buzzer_strct.melody_MaxCnt)
                mod_timer( &beep_timer, jiffies + msecs_to_jiffies(10) ); // 1ms 
        }
        else
        {
            //
        }
        
        
    }
#endif
}



//static void buzzer(unsigned int type, unsigned int cnt, unsigned int mson, unsigned int msoff )
static void buzzer(struct buzzer_param *data )
{
	int i;

    DPRINTK("Buzzer Set!\n");

    pwm_onoff(OFF);
    
    gBuzzer_index = 0;
    
    Buzzer_strct.buzzer_seq = ON;

    memset(Buzzer_strct.count,0,sizeof(Buzzer_strct.count));

    Buzzer_strct.count[0] = data->count*2; // on/off time is 2 
    Buzzer_strct.type[0] = data->type; 
    Buzzer_strct.mson[0] = data->mson; 
    Buzzer_strct.msoff[0] = data->msoff; 
    Buzzer_strct.melody_MaxCnt = 1;        
    
    /*for(i=0;i<10;i++)
        Buzzer_strct.count[i] = data->count[i]*2; // on/off time is 2 
    
    Buzzer_strct.melody_MaxCnt = data->melody_MaxCnt;

    if(Buzzer_strct.melody_MaxCnt>10)
        Buzzer_strct.melody_MaxCnt = 10;
    
    memcpy(Buzzer_strct.type,data->type,sizeof(Buzzer_strct.type));
    memcpy(Buzzer_strct.mson,data->mson,sizeof(Buzzer_strct.mson));
    memcpy(Buzzer_strct.msoff,data->msoff,sizeof(Buzzer_strct.msoff));
    */
    mod_timer( &beep_timer, jiffies + msecs_to_jiffies(10) ); // 1ms 

}



static int buzzer_open(struct inode *inode, struct file *file)
{
    if ( ulDeviceOpen )
    {
        DPRINTK("%s : Device has already opened\n", DEVICE_NAME);
        return -EBUSY;
    }
    ++ulDeviceOpen;
    return 0;
}

static int buzzer_release(struct inode *inode, struct file *file)
{
    if (!ulDeviceOpen)
    {
        DPRINTK("%s : Device has not opened\n", DEVICE_NAME);
        return -EINVAL;
    }
    set_pwm(0);
    pwm_onoff(OFF);
    --ulDeviceOpen;
    return 0;
}

static long buzzer_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct buzzer_param data;
    struct buzzer_param2 data2;
    unsigned int uBuzzer_status;
	
	if (_IOC_TYPE(cmd) != BUZZER_IOCTL_MAGIC) return -ENOTTY;	
	//if (_IOC_NR(cmd) >= BUZZER_IOCTL_MAGIC) return -ENOTTY;

    mutex_lock(&buzzer_mutex);
    
    switch (cmd)
    {
        case BUZZER_STATUS:
            if(gBuzzer_index<Buzzer_strct.melody_MaxCnt)
                uBuzzer_status =  1;    
            else
                uBuzzer_status =  0;

            if(copy_to_user((unsigned int*)arg, &uBuzzer_status, sizeof(unsigned int)))
            {
                mutex_unlock(&buzzer_mutex);
                DPRINTK("copy_to_user = %d\n",uBuzzer_status);
                return -EFAULT;
            }            
            DPRINTK("uBuzzer_status = %d\n",uBuzzer_status);
            break;
		case BUZZER_INIT:
			break;

		case BUZZER_ON:
            pwm_onoff(OFF);
            ucBuzzerOnOffflag = 1;
            Buzzer_strct.buzzer_seq = 0;
            Buzzer_strct.count[0] = 0;
            Buzzer_strct.type[0]  = 0;
            Buzzer_strct.mson[0]  = 0;
            Buzzer_strct.msoff[0] =  0;
            gBuzzer_index = 0;
            Buzzer_strct.melody_MaxCnt=0;
            DPRINTK("BUZZER Enable\n");
			break;

		case BUZZER_OFF:
            Buzzer_strct.count[0]=0;
            pwm_onoff(OFF);
            ucBuzzerOnOffflag = 0;
            gBuzzer_index = 0;
            Buzzer_strct.melody_MaxCnt=0;
            DPRINTK("BUZZER Disable\n");
			break;


        case BUZZER_CUSTOM_FREQ:
            if(ucBuzzerOnOffflag)
            {
                if(copy_from_user((void *)&data2, (const void *)arg, sizeof(struct buzzer_param2))) 
                {
                    mutex_unlock(&buzzer_mutex);
                    return -EFAULT;
                }
                Buzzer_strct.buzzer_seq = 0;
                Buzzer_strct.count[0] = 0;
                Buzzer_strct.type[0]  = 0;
                Buzzer_strct.mson[0]  = 0;
                Buzzer_strct.msoff[0] =  0;
                gBuzzer_index = 0;
                Buzzer_strct.melody_MaxCnt=0;

                pwm_onoff(OFF);
                set_pwm_custom(data2.type[0]);
                pwm_onoff(ON);
                mod_timer( &beep_timer, jiffies + msecs_to_jiffies(data2.mson[0])); 
            }
        break;
        
		case BUZZER_TONE:
			if(ucBuzzerOnOffflag)
            {
                //struct buzzer_param data;   

                if(copy_from_user((void *)&data, (const void *)arg, sizeof(struct buzzer_param))) 
                {
                    mutex_unlock(&buzzer_mutex);
                    return -EFAULT;
                }
                //buzzer(data.type, data.count, data.mson, data.msoff);                
                buzzer(&data);                
            }
			break;

        case BUZZER_VERSION:
            if(copy_to_user((unsigned char *)arg, RELEASE_VERSION, 3) )
            {
               mutex_unlock(&buzzer_mutex);
               goto failed;
            }
            break;

        default:
             mutex_unlock(&buzzer_mutex);
             DPRINTK(" ioctl : unknown command %02x(module)\n", cmd);
             goto failed;
             break;
    }

    mutex_unlock(&buzzer_mutex);
    return 0;

failed:
    return -EFAULT;
}

static struct file_operations buzzer_fops = {
    .owner              = THIS_MODULE,
    .open               = buzzer_open,
    .release            = buzzer_release,
    .unlocked_ioctl     = buzzer_ioctl,
};

static struct miscdevice buzzer_dev = {
    MISC_DYNAMIC_MINOR,
    DEVICE_NAME,
    &buzzer_fops
};

static int buz_file_read( char *page, char **start, off_t off, int count, int *eof, void *data )
{
    int len = 0;

    if ( off > 0 )
        len = 0;
    else
    {
        switch( (long)data )
        {
            case PROC_PERIOD:
                len = sprintf( page, "%d\n", pwm_readl(pwm0_base, PWM_CPRD(BUZZER_CHAN)) );
                break;

            case PROC_DUTY:
                len = sprintf( page, "%d\n", pwm_readl(pwm0_base, PWM_CDTY(BUZZER_CHAN)) );
                break;

            case PROC_ONOFF:
                break;

            default: 
                printk(KERN_ALERT "read : Unknown Command(%d)\n", (int)data);
                len = 0;
                break;
        }
    }

    return len;
}
static int buz_file_write( struct file *file, const char *buffer, unsigned long count, void *dat)
{
    int len = 0;
    int data = 0;
    char buf[PROC_BUZ_LEN];

    if ( count > PROC_BUZ_LEN )
        len = PROC_BUZ_LEN;
    else
        len = count;

    switch( (long)dat )
    {
        case PROC_PERIOD:
            if ( copy_from_user(buf, buffer, len) )
                goto failed;

            data = (int)simple_strtoul(buf, NULL, 10);

            BUZZER_PERIOD(data);
            break;

        case PROC_DUTY:
        	if ( copy_from_user(buf, buffer, len) )
        		goto failed;

        	data = (int)simple_strtoul(buf, NULL, 10);
        	BUZZER_DUTY(data);
            break;

        case PROC_ONOFF:
        	if ( copy_from_user(buf, buffer, len) )
        		goto failed;

        	data = (int)simple_strtoul(buf, NULL, 10);
        	DPRINTK("onoff : data = %d\n", data);
        	if ( data )
        		BEEP_ON;
        	else
        	if ( !data )
        		BEEP_OFF;
            break;

        default: 
            printk(KERN_ALERT "write : Unknown Command(%d)\n", (int)data);
            len = 0;
            break;
    }

    return len;

failed:
	return -EFAULT;
}

static void delete_proc_buzzer(void)
{
	
}

static int create_proc_buzzer(void)
{
	int err = 0;

	/*if ( (buz_proc_dir = proc_mkdir( PROC_BUZ_DIR, NULL)) == NULL )
	{
		printk(KERN_ALERT "Error : Could not make directory /proc/%s\n", PROC_BUZ_DIR );
		err = -ENOMEM;
		goto failed;
	}

	if ( (buz_proc_period = create_proc_entry( PROC_BUZ_PERIOD, 0644, buz_proc_dir)) == NULL )
	{
		printk(KERN_ALERT "error : could not make directory /proc/%s/%s\n", PROC_BUZ_DIR, PROC_BUZ_PERIOD );
		err = -ENOMEM;
		goto failed;
	}
	buz_proc_period->read_proc		= buz_file_read;
	buz_proc_period->write_proc		= buz_file_write;
	buz_proc_period->data			= (void *)PROC_PERIOD;

	if ( (buz_proc_duty = create_proc_entry( PROC_BUZ_DUTY, 0644, buz_proc_dir)) == NULL )
	{
		printk(KERN_ALERT "error : could not make directory /proc/%s/%s\n", PROC_BUZ_DIR, PROC_BUZ_DUTY );
		err = -ENOMEM;
		goto failed;
	}
	buz_proc_duty->read_proc		= buz_file_read;
	buz_proc_duty->write_proc		= buz_file_write;
	buz_proc_duty->data				= (void *)PROC_DUTY;


	if ( (buz_proc_onoff = create_proc_entry( PROC_BUZ_ONOFF, 0644, buz_proc_dir)) == NULL )
	{
		printk(KERN_ALERT "error : could not make directory /proc/%s/%s\n", PROC_BUZ_DIR, PROC_BUZ_ONOFF );
		err = -ENOMEM;
		goto failed;
	}
	buz_proc_onoff->read_proc		= buz_file_read;
	buz_proc_onoff->write_proc		= buz_file_write;
	buz_proc_onoff->data			= (void *)PROC_ONOFF;
    
	return 0;

failed:*/
//	delete_proc_buzzer();

	return err;
}



static int __init buzzer_init(void)
{
    int err = 0;
    unsigned int scsr;

    gpio_init_set();

    /* check PWM clock from PMC */
    scsr = at91_pmc_read(AT91_PMC_PCSR1);
    DPRINTK("1:PMC peripheral clock = 0x%x\n", (unsigned int )scsr );

    /* PWM PMC Enable */
	at91_pmc_write(AT91_PMC_PCER1, scsr | (1 << 6) );
	scsr = at91_pmc_read(AT91_PMC_PCSR1);
	DPRINTK("2:PMC peripheral clock = 0x%x / %x\n", (unsigned int)scsr ,SZ_16K -1 );

    if ( !(pwm0_base = ioremap(/*AT91SAM9X5_BASE_PWMC*/0xF802C000, SZ_16K -1 )) )
    {
        printk(KERN_ALERT "PWM Controller : virtual address allocate failed\n");
        err = -ENOMEM;
        goto failed;
    }

    /* pwm prescaler */
    BUZZER_PRESCALER_DEFAULT;

    /* configure PWM channel 0 */
    // PWM channel 0 disable
    BUZZER_DISABLE;
    // PWM channel mode register
    BUZZER_DEFAULT_CLKSRC;

    // Default perios & duty 


   // BUZZER_DEFAULT;
    if ( (err = misc_register(&buzzer_dev)) )
    {
        DPRINTK("Buzzer  driver : Unable to register driver\n");
        goto failed;
    }

    BEEP_OFF; 

    //create_proc_buzzer();

    ucBuzzerOnOffflag = 1; // enable!! 
    set_pwm(0);    

    Buzzer_strct.buzzer_seq = 0;
    Buzzer_strct.count[0] = 0;
    Buzzer_strct.type[0]  = 0;
    Buzzer_strct.mson[0]  = 0;
    Buzzer_strct.msoff[0] =  0;
    gBuzzer_index = 0;
    Buzzer_strct.melody_MaxCnt=0;

    setup_timer( &beep_timer, beep_timer_callback, 0);

    printk(KERN_ALERT "\n%s %s %s Load Success - %s \n", 
        MODEL_NAME, DEVICE_NAME, RELEASE_VERSION, __TIMESTAMP__);

    return err;

failed:
	if (pwm0_base)
		iounmap(pwm0_base);
    misc_deregister(&buzzer_dev);

    return err;
}

static void __exit buzzer_exit(void)
{	
    pwm_onoff(OFF);
    
    if ( pwm0_base )
        iounmap(pwm0_base);

    del_timer_sync(&beep_timer);
    //delete_proc_buzzer();
    misc_deregister(&buzzer_dev);
    printk(KERN_ALERT "\n%s %s %s Exit Success\n", MODEL_NAME, DEVICE_NAME, RELEASE_VERSION );
}

module_init(buzzer_init);
module_exit(buzzer_exit);

MODULE_AUTHOR("SHC");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("SHC-500 driver");
