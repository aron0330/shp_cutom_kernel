/*
 *
 *
 *  Revision History
 *
 *
 *
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
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <asm/page.h>
/* #include <mach/platform.h> */
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/ioctl.h>


#include <linux/device.h>
#include <linux/err.h>
#include <linux/proc_fs.h>

#include <linux/miscdevice.h>



#include <linux/timer.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>

#include <linux/sched.h>

#include "gpio-owl-user-io.h"
#include <mach/gpio.h>


//#define DEBUG
#ifdef DEBUG
#define DPRINTK(format,args...)     printk(KERN_ALERT "%s: " format, __FUNCTION__, ## args)
#else
#define DPRINTK(format,args...)
#endif 

#define MODEL_NAME                              "SHC-650"
#define DEVICE_NAME                             "gpio"

#define RELEASE_VERSION                         "0.7"
/*0.3 ver : pwr en 변경 , modem on/on add ! */
// 0.5 : init시 lte modem on루틴 remove

#define TEST_MAJOR                              251

#define PIO_NUM_IO		32

#define	AT91C_PIN_PA(io)	(0 * PIO_NUM_IO + io)
#define	AT91C_PIN_PB(io)	(1 * PIO_NUM_IO + io)
#define	AT91C_PIN_PC(io)	(2 * PIO_NUM_IO + io)
#define	AT91C_PIN_PD(io)	(3 * PIO_NUM_IO + io)
#define	AT91C_PIN_PE(io)	(4 * PIO_NUM_IO + io)





/* GPIO define ------------------------------------------------------------------------------*/
/*#define GPIO_D3_PE24	(128+24)*/
/*A = 0     B = 1      C = 2        D = 3       E = 4 */

//#define KEYIN_0                                 ((32*3)+6) /*PD06*/

//#define COM_SEL1								((32*1)+9) /*PB09_SAM_SEL1#*/
//#define COM_SEL2								((32*1)+10) /*PB10_SAM_SEL2#*/

//#define KEY_LED									AT91C_PIN_PC(7) /*PC07_KEY_LED#*/
#define KEY_LED									OWL_GPIO_PORTD(12) /*PC07_KEY_LED#*/


#define ON_LED									OWL_GPIO_PORTD(14) /*PC08_ONLINE_LED#*/
#define ICC_LED									OWL_GPIO_PORTD(16) /*PC09_ICC_LED#*/

//#define DRAWER_EN								AT91C_PIN_PA(18) /*PA18_DRAW_PWREN#*/

#define ICC_RESET								AT91C_PIN_PB(7) /*PB07_IFM_RST#*/
#define ICC_DETECT								AT91C_PIN_PB(8) /*PB08_IFM_DET#*/

#define SAM_SEL1								AT91C_PIN_PB(9) /*PB09_SAM_SEL1#*/
#define SAM_SEL2								AT91C_PIN_PB(10) /*PB10_SAM_SEL2#*/

#define USB_PWR									AT91C_PIN_PA(16) /*PA16_USBH_PWREN#*/

/* GPIO Input */
#define VBUS_SENS								AT91C_PIN_PA(31)/*PA31_USBD_VBUS//*pull-down */

#define MODEM_RESET                             AT91C_PIN_PC(10)//PC10_MODEM_RESET#
#define MODEM_ESC                               AT91C_PIN_PB(0)

#define WAN_PWR_EN                              AT91C_PIN_PB(13) 
#define WAN_ON_OFF                              AT91C_PIN_PA(20)//PA20_WAN_ON_OFF#
#define WAN_DM_EN                               AT91C_PIN_PB(6) //PB6_WAN_DM_EN#

#define PSTN_PWR_EN                             AT91C_PIN_PD(28)

/*
- WAN_PWR_EN : PA19-->PB13
- PSTN_PWR_EN 추가 : PD28
*/


//ADD WAN PINS 
/*PA14_MODEL_ID0 
PA29_MODEL_ID1
PA19_WAN_PWR_EN
PA20_WAN_ON_OFF
PB6_WAN_DM_EN
*/
/*-------------------------------------------------------------------------------------------*/

/* Macros -----------------------------------------------------------------------------------*/

#define SET_VALUE(pin, x)						gpio_set_value(pin, x)
#define GET_VALUE(x)							gpio_get_value(x)


/*-------------------------------------------------------------------------------------------*/

struct sam_slot {
	char sel1;
	char sel2;
	char num;
};
static struct sam_slot sam_slots[4] = {
		{ 0, 0, 1 },
		{ 1, 0, 2 },
		{ 0, 1, 3 },
		{ 1, 1, 4 }
};


///////////////////////////////////////////////////////////////////////////////////
// proc_fs variable
///////////////////////////////////////////////////////////////////////////////////
#define PROC_GPIO_DIR			"gpio"
#define PROC_GPIO_MODEM			"modem"
#define PROC_GPIO_DEBUG			"debug"
#define PROC_GPIO_KEY_LED		"key_led"
#define PROC_GPIO_ON_LED		"online_led"
#define PROC_GPIO_ICC_LED		"icc_led"
//#define PROC_GPIO_DRAW_PWR		"drawer"
#define PROC_GPIO_ICC_RESET		"icc_reset"
#define PROC_GPIO_SAM_SEL		"sam"
#define PROC_GPIO_USB_PWR		"usb_pwr"
#define PROC_GPIO_USB_VBUS		"usb_vbus"
//#define PROC_GPIO_BACKLIGHT		"backlight"


#define PROC_BUF_LEN			4

static struct timer_list onoff_timer;


struct led_control {
    int enable;
    int on_cnt;
    int off_cnt;
    int loop;

    // internal +/- use
    int val;
    int on;
    int off;
    int cnt;
};

static DEFINE_SPINLOCK(led_lock);
static DEFINE_MUTEX(gpio_mutex);

static volatile struct led_control led_control[nLED_CNTL];


///////////////////////////////////////////////////////////////////////////////////
// Device Operations
///////////////////////////////////////////////////////////////////////////////////

static void sam_sel(char num)
{
	int i;
	for (i = 0;i < ARRAY_SIZE(sam_slots); i++ )
	{
		if ( sam_slots[i].num == num )
		{
			SET_VALUE(SAM_SEL1, sam_slots[i].sel1 );
			SET_VALUE(SAM_SEL2, sam_slots[i].sel2 );
            /* printk(KERN_ALERT "num = %d, sel1 = %d, sel2 = %d\n", sam_slots[i].num,  */
            /*         sam_slots[i].sel1, sam_slots[i].sel2); */
		}
	}
}

static int get_sam(char sel1, char sel2)
{
	char sam = -1;

    switch(sel1)
    {
        case 0:
            if ( sel2 == 0 )
                return 1;
            else
            if ( sel2 == 1 )
                return 3;
            break;

        case 1:
            if ( sel2 == 0 )
                return 2;
            else
            if ( sel2 == 1 )
                return 4;
            break;
    }

	return sam;
}

/*! 
 \function gpio_init_set 
 \retval static void :
 \param void : 
 
#define WAN_PWR_EN                              AT91C_PIN_PA(19)//PA19_WAN_PWR_EN#
#define WAN_ON_OFF                              AT91C_PIN_PA(20)//PA20_WAN_ON_OFF#
#define WAN_DM_EN  
*/
static void gpio_init_set(void)
{
    gpio_request(KEY_LED,"sysfs");
    gpio_request(ON_LED,"sysfs");
    gpio_request(ICC_LED,"sysfs");
#if 0 

   gpio_request(MODEM_RESET,"sysfs");
    gpio_request(MODEM_ESC,"sysfs");
    gpio_request(ICC_DETECT,"sysfs");
    

    //20170223_sjh add
    gpio_request(WAN_PWR_EN,"sysfs");
    gpio_request(WAN_ON_OFF,"sysfs");
    gpio_request(WAN_DM_EN,"sysfs");
   
    gpio_request(SAM_SEL1,"sysfs");
    gpio_request(SAM_SEL2,"sysfs");
    gpio_request(USB_PWR,"sysfs");    
    gpio_request(ICC_RESET,"sysfs");    
    gpio_request(VBUS_SENS,"sysfs");
    
#endif
    gpio_direction_output(KEY_LED,false); // led off
    gpio_direction_output(ON_LED,false);
    gpio_direction_output(ICC_LED,false);
  
#if 0
  gpio_direction_output(MODEM_RESET,true);    
    gpio_direction_output(MODEM_ESC,false);    
//    gpio_direction_output(DRAWER_EN,false);
    
    gpio_direction_output(SAM_SEL1,true);
    gpio_direction_output(SAM_SEL2,true);
    gpio_direction_output(USB_PWR,true);
    gpio_direction_output(ICC_RESET,false);

    //20170223_sjh add
    gpio_direction_output(WAN_PWR_EN,false);
    gpio_direction_output(WAN_ON_OFF,false);
    gpio_direction_output(WAN_DM_EN ,false);

    gpio_direction_input(VBUS_SENS);    
    gpio_direction_input(ICC_DETECT);
    
    /* selected sam slot 1 */
    sam_sel(1);

    DPRINTK("vbus_sens = %x\n", GET_VALUE(VBUS_SENS) );
    DPRINTK("usb_pwren = %x\n", GET_VALUE(USB_PWR) );

    /* IFM POWER ON */
    SET_VALUE(ICC_RESET,1 );

    /* USB Host Mode */
    SET_VALUE(USB_PWR, 1);

    /* DRAWER Disable */
    //SET_VALUE(DRAWER_EN, 0);

    /*modem reset and on! */
    SET_VALUE(PSTN_PWR_EN, 1);
    SET_VALUE(MODEM_RESET, 1);

    /*ifm reset and on! */
    SET_VALUE(ICC_RESET, 0);

    //test
//    SET_VALUE(WAN_PWR_EN, 0);
//    SET_VALUE(WAN_ON_OFF, 0);
    
    SET_VALUE(WAN_PWR_EN, 1);
    mdelay(510);
    
    SET_VALUE(WAN_ON_OFF, 1);
    DPRINTK("WAN_ON_OFF on!\n");
    mdelay(1300);
    SET_VALUE(WAN_ON_OFF, 0);
    DPRINTK("WAN_ON_OFF off!\n");
    
#endif

    

}


static int gpio_open(struct inode *inode, struct file *filefp)
{
    if ( !try_module_get(THIS_MODULE) ) return -ENODEV;

    memset(&led_control, 0, sizeof(led_control));    
   // mod_timer( &onoff_timer, jiffies + msecs_to_jiffies(100) );
    
    DPRINTK("%s %s Open Success\n", DEVICE_NAME, RELEASE_VERSION);
    return 0;
}

static int gpio_release(struct inode *inode, struct file *filefp)
{
    module_put(THIS_MODULE);
   // del_timer_sync( &onoff_timer );
    DPRINTK("\n%s %s Release Success\n", DEVICE_NAME, RELEASE_VERSION);
    return 0;
}



static long gpio_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    unsigned long flags,val;
    int data;
    int err = -EFAULT;
    int ret = 0;
    char sam = 0;
    char brightness = 0;
    void __iomem *gpioReadReg;
    
    if (_IOC_TYPE(cmd) != GPIO_IOCTL_MAGIC) return -ENOTTY;
    //if (_IOC_NR(cmd)   >= GPIO_IOCTL_MAGIC) return -ENOTTY;

    mutex_lock(&gpio_mutex);
    
    switch (cmd) 
    {
        //case GPIO_INIT:            
        //    break;

        case GPIO_KEY_LED:
            /*if (copy_from_user((void *)&val, (unsigned long *)arg, sizeof(unsigned long))) {
                mutex_unlock(&gpio_mutex);
                return -EFAULT;
            }*/    
            //spin_lock_irqsave(&led_lock, flags);
            led_control[LED_CNTL_KEY].enable = 0;
            //spin_unlock_irqrestore(&led_lock, flags);
            DPRINTK("GPIO_KEY_LED %d\n",arg);
             if (arg)
                SET_VALUE(KEY_LED, 1);
            else
                SET_VALUE(KEY_LED, 0);
            break;

        case GPIO_ON_LED:
            /*if (copy_from_user((void *)&val, (unsigned long *)arg, sizeof(unsigned long))) {
                mutex_unlock(&gpio_mutex);
                return -EFAULT;
            } */   
            //spin_lock_irqsave(&led_lock, flags);
            led_control[LED_CNTL_ONLINE].enable = 0;
            //spin_unlock_irqrestore(&led_lock, flags);
            DPRINTK("GPIO_ON_LED %d\n",arg);
            if (arg)
                SET_VALUE(ON_LED, 1);
            else
                SET_VALUE(ON_LED, 0);            
          
            break;

        case GPIO_ICC_LED:
            /*if (copy_from_user((void *)&val, (unsigned long *)arg, sizeof(unsigned long))) {
                mutex_unlock(&gpio_mutex);
                return -EFAULT;
            }*/    
            //spin_lock_irqsave(&led_lock, flags);
            led_control[LED_CNTL_ICC].enable = 0;
            //spin_unlock_irqrestore(&led_lock, flags);
            DPRINTK("GPIO_ICC_LED %d\n",arg);
            if (arg)
                SET_VALUE(ICC_LED, 1);
            else
                SET_VALUE(ICC_LED, 0);
            break;            
      	
        case GPIO_MODEM_RESET:
            DPRINTK("GPIO_MODEM_RESET\n");
            SET_VALUE(MODEM_RESET, 0);
            mdelay(100);
            SET_VALUE(MODEM_RESET, 1);
        break;         

        case GPIO_MODEM_ESC:
            DPRINTK("GPIO_MODEM_ESC\n");
            if (arg)
                SET_VALUE(MODEM_ESC, 1);
            else // 100ms need!
                SET_VALUE(MODEM_ESC, 0);
        break;
        
        case GPIO_ICC_RESET:
            /*if (copy_from_user((void *)&val, (unsigned long *)arg, sizeof(unsigned long))) {
                mutex_unlock(&gpio_mutex);
                return -EFAULT;
            }*/    
            DPRINTK("\n @GPIO_ICC_RESET %d/%x\n",arg,arg);
        	if (arg)
                SET_VALUE(ICC_RESET,1);
            else
                SET_VALUE(ICC_RESET, 0);
            
        	break;

        case GPIO_SAM_SEL:
        	//DPRINTK("GPIO_SAM_SEL %d\n",arg);
        	if ( 1 <= arg && arg <= 4 )
        	{
        		sam_sel(arg);
        		//sam = get_sam( GET_VALUE(SAM_SEL1), GET_VALUE(SAM_SEL2) );
        	}
			
        	break;

        case GPIO_USB_PWR:
            /*if (copy_from_user((void *)&val, (unsigned long *)arg, sizeof(unsigned long))) {
                mutex_unlock(&gpio_mutex);
                return -EFAULT;
            }*/    
            if (arg)
                SET_VALUE(USB_PWR, 1);
            else
                SET_VALUE(USB_PWR, 0);
        	break;

        case GPIO_USB_VBUS:
        	data = GET_VALUE(VBUS_SENS);

            DPRINTK("usb %x\n",data);
            
        	if ( copy_to_user( (int *)arg, &data, sizeof(int)) )
        		goto failed;

        	break;


         case GPIO_ICC_DETECT:
        	data = GET_VALUE(ICC_DETECT);

            DPRINTK("ICC_DETECT %x\n",data);
            
        	if ( copy_to_user( (int *)arg, &data, sizeof(int)) )
        		goto failed;

        break;
        case GPIO_LED_CONTROL: 
        {
            /*struct LedControl LedControl;
            int type;

            if (copy_from_user((void *)&LedControl, (void __user *)arg, sizeof(struct LedControl)))
                goto failed;
            type = LedControl.type;
            if (type < 0 || type >= nLED_CNTL || (LedControl.on_time == 0 
                && LedControl.off_time == 0))
                goto failed;

            //spin_lock_irqsave(&led_lock, flags);
            memset((void*)&led_control[type], 0, sizeof(struct led_control));

            led_control[type].on_cnt = LedControl.on_time;
            led_control[type].off_cnt = LedControl.off_time;
            led_control[type].loop = LedControl.loop;
            led_control[type].enable = 1;
            //spin_unlock_irqrestore(&led_lock, flags);*/
            break;
        }
         case GPIO_READ:
            ret = -1;
            //PIO Output Data Status Register PORTC
            if(!(gpioReadReg = ioremap(0xFC038098,4)))  
        	{
        	    mutex_unlock(&gpio_mutex);
                return -EFAULT;
        	}	
            
            val = __raw_readl(gpioReadReg);
            
            //printk("%x\n",__raw_readl(gpioReadReg));
            if(arg == 1) 
               ret = (val & 0x80)>>7;
               //ret = GET_VALUE(KEY_LED);
            else if(arg == 2) 
                ret = (val & 0x100)>>8;
               //ret = GET_VALUE(ON_LED);
            else if(arg == 3) 
                 ret = (val & 0x200)>>9;
               //ret = GET_VALUE(ICC_LED);

            iounmap(gpioReadReg);
            mutex_unlock(&gpio_mutex);
            return  ret ;
            
            break;

    	case GPIO_VERSION:
            if(copy_to_user((unsigned char *)arg, RELEASE_VERSION, 3) )
            {
               mutex_unlock(&gpio_mutex);
               return -EFAULT;
            }
            break;


        //20170223_sjh add
        case GPIO_WAN_PWR_EN:           
            if (arg)
                SET_VALUE(WAN_PWR_EN, 1);
            else
                SET_VALUE(WAN_PWR_EN, 0);
        break;

        case GPIO_WAN_ON_OFF:           
            if (arg)
                SET_VALUE(WAN_ON_OFF, 1);
            else
                SET_VALUE(WAN_ON_OFF, 0);
        break;
        
        case GPIO_WAN_DM_EN:           
            if (arg)
                SET_VALUE(WAN_DM_EN, 1);
            else
                SET_VALUE(WAN_DM_EN, 0);
        break;

        case GPIO_PSTN_PWR_EN:           
            if (arg)
                SET_VALUE(PSTN_PWR_EN, 1);
            else
                SET_VALUE(PSTN_PWR_EN, 0);
        break;

        //
        default:
             DPRINTK(" ioctl : unknown command %02x(module)\n", cmd);
             mutex_unlock(&gpio_mutex);
             return -EINVAL;
    } 
    mutex_unlock(&gpio_mutex);
    return ret;

failed:
    mutex_unlock(&gpio_mutex);
    DPRINTK(" ioctl : error! %x\n", err);
    return err;

}


static struct file_operations gpio_fops = {
    .owner          = THIS_MODULE,
    .open           = gpio_open,
    .release        = gpio_release,
    .unlocked_ioctl = gpio_ioctl,
};

static struct miscdevice gpio_dev = {
    MISC_DYNAMIC_MINOR,
    DEVICE_NAME,
    &gpio_fops
};


static int gpio_proc_read( char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int len = 0;
	int status = 0;
	char sam = 0;
	char sel1,sel2;

#if 0
	
#endif
	return len;
}

static int gpio_proc_write( struct file *file, const char *buf, unsigned long count, void *dat )
{
	int len = 0;
	int data = 0;
	char sam = 0;
	char backlight= 0;
	char buffer[PROC_BUF_LEN];

	if ( count >= PROC_BUF_LEN )
		len = PROC_BUF_LEN;
	else
		len = count;
#if 0
	
#endif
	return len;
}

struct proc_dir_entry *proc_file_entry; // proc fs 
static struct proc_dir_entry    *gpio_proc_dir;
#define PROC_GPIO_DIR                            "gpio"
#define PROC_USB_PWR                            "usb_pwr"

static int gpio_usb_proc_write( struct file *file, const char *buf, unsigned long count, void *data )
{
    int len = 0;
    char cp_data[4] = {0,};
    
    
    if ( copy_from_user(cp_data, buf, 1) ) {
        return -EFAULT;
    }

    if(cp_data[0] == '1')
        SET_VALUE(USB_PWR, 1);
    else  if(cp_data[0] == '0')
        SET_VALUE(USB_PWR, 0);

    printk("%c \n",cp_data[0]);
    
    return count;
}

static const struct file_operations key_repeat_proc_fops = {
  .owner = THIS_MODULE,
  //.read = repeat_proc_read,
  .write = gpio_usb_proc_write,

};



static void delete_proc_gpio(void)
{
    remove_proc_entry(PROC_USB_PWR, gpio_proc_dir);    
    remove_proc_entry( PROC_GPIO_DIR, NULL);	
}

static int create_proc_gpio(void)
{
    if ( (gpio_proc_dir = proc_mkdir(PROC_GPIO_DIR, NULL)) == NULL)
    {
        printk(KERN_ALERT "ERROR : Could not make directory /proc/%s\n", PROC_GPIO_DIR);
        remove_proc_entry( PROC_GPIO_DIR,NULL);
        return -ENOMEM;
    }

    proc_create(PROC_USB_PWR, 0666, gpio_proc_dir, &key_repeat_proc_fops);
}



/*
 * Device Operations
 */

static void Led_onoff(int type, int value)
{
    led_control[type].val = value;

    switch(type) {
        case LED_CNTL_KEY:
            //gpio_set_value(KBD_LED_GPIO, value);
            SET_VALUE(KEY_LED, value);
            break;
        case LED_CNTL_ONLINE:
            //gpio_set_value(ONLINE_LED_GPIO, value);
            SET_VALUE(ON_LED, value);
            break;
        case LED_CNTL_ICC:
            //gpio_set_value(ICC_LED_GPIO, value);
            SET_VALUE(ICC_LED, value);
            break;
    }
}

static void Led_toggle(int type)
{
    int value = ~led_control[type].val & 0x1;

    switch(type) {
        case LED_CNTL_KEY:
            SET_VALUE(KEY_LED, value);
            break;
        case LED_CNTL_ONLINE:
            SET_VALUE(ON_LED, value);
            break;
        case LED_CNTL_ICC:
            SET_VALUE(ICC_LED, value);
            break;
    }
    led_control[type].val = value;
}

static void onoff_handler(unsigned long data)
{
    int i;
    unsigned long flags;


    //spin_lock_irqsave(&led_lock, flags);

    for (i=0;i < nLED_CNTL;i++) 
    {
        if (led_control[i].enable) 
        {
            if (led_control[i].loop == 0) 
            {   // INFINITE
                if (led_control[i].on == led_control[i].on_cnt && \
                    led_control[i].off == led_control[i].off_cnt) 
                {
                    led_control[i].on = 0;
                    led_control[i].off = 0;
                }

                if (led_control[i].on < led_control[i].on_cnt) 
                {
                    Led_onoff(i, 1);
                    led_control[i].on++;
                }
                else 
                if (led_control[i].off < led_control[i].off_cnt) 
                {
                    Led_onoff(i, 0);
                    led_control[i].off++;
                }
            }
            else 
            { 
                if (led_control[i].on == led_control[i].on_cnt && \
                    led_control[i].off == led_control[i].off_cnt) 
                {
                    led_control[i].on = 0;
                    led_control[i].off = 0;
                    led_control[i].cnt++;
                   // printk("cnt! %d / %d\n",led_control[i].cnt,led_control[i].loop);

                    if (led_control[i].loop == led_control[i].cnt) 
                    {
                        Led_onoff(i, 0);
                        led_control[i].enable = 0;
                        //printk("cnt end! %d / %d\n",led_control[i].cnt,led_control[i].loop);
                        continue;
                    }
                }
                
                if (led_control[i].on < led_control[i].on_cnt) 
                {
                    Led_onoff(i, 1);
                    led_control[i].on++;
                   // printk("on! %d / %d\n",led_control[i].on,led_control[i].on_cnt);
                }
                else 
                if (led_control[i].off < led_control[i].off_cnt) 
                {
                    Led_onoff(i, 0);
                    led_control[i].off++;
                    //printk("off! %d / %d\n",led_control[i].off,led_control[i].off_cnt);
                }

                
            }   // if (led_control[i].loop == 0) 
        }   // if (led_control[i].enable) 
    }   // for (i=0;i < nLED_CNTL;i++) 
    //spin_unlock_irqrestore(&led_lock, flags);

  //  mod_timer( &onoff_timer, jiffies + msecs_to_jiffies(100) );
}


static int __init shc350_gpio_init(void)
{
    int ret;

    gpio_init_set();

    if ( (ret = misc_register(&gpio_dev)) )
    {
        printk(KERN_ALERT "GPIO driver : Unable to register driver\n");
        goto failed;
    }

    //setup_timer( &onoff_timer, onoff_handler, 0 );
    
    printk(KERN_ALERT "\n%s %s %s Load Success - %s\n", 
        MODEL_NAME, DEVICE_NAME, RELEASE_VERSION, __TIMESTAMP__ );

    create_proc_gpio();
    
    return ret;

failed:

    misc_deregister(&gpio_dev);
    return ret;
}


static void __exit shc350_gpio_exit(void)
{

	//delete_proc_gpio();
    //del_timer_sync( &onoff_timer );
    delete_proc_gpio();
    misc_deregister(&gpio_dev);

    //gpio_free(KEY_LED); 
    gpio_free(ON_LED); 
    //gpio_free(ICC_LED); 

//    SET_VALUE(DRAWER_EN, 0);
//    gpio_free(DRAWER_EN); 

#if 0
    gpio_free(ICC_RESET); 
    gpio_free(ICC_DETECT); 
    gpio_free(SAM_SEL1); 
    gpio_free(SAM_SEL2); 
    gpio_free(USB_PWR); 
    gpio_free(VBUS_SENS); 
    gpio_free(MODEM_RESET); 

    gpio_free(WAN_PWR_EN); 
    gpio_free(WAN_ON_OFF); 
    gpio_free(WAN_DM_EN);
#endif
    printk(KERN_ALERT "\n%s %s %s Exit Success\n", MODEL_NAME, DEVICE_NAME, RELEASE_VERSION );
}

module_init(shc350_gpio_init);
module_exit(shc350_gpio_exit);

MODULE_AUTHOR("SHC");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("SMT-T281 KEY driver");
