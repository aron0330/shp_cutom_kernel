/*

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

/* #include <mach/platform.h> */
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/ioctl.h>


#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/proc_fs.h>

#include <linux/miscdevice.h>



/* #include <linux/timer.h> */
/* #include <linux/hrtimer.h> */
/* #include <linux/ktime.h> */

#include <linux/input.h>
#include <linux/input-polldev.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/spinlock.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/sysctl.h>
#include "gpio-owl-keypad.h"
#include <linux/mod_devicetable.h>
#include <linux/slab.h>
#include <linux/poll.h>

//#define DEBUG
#ifdef DEBUG
#define DPRINTK(format,args...)     printk(KERN_ALERT "%s: " format, __FUNCTION__, ## args)
#else
#define DPRINTK(format,args...)
#endif 

#define MODEL_NAME                              "SHC-500"
#define DEVICE_NAME                             "key"
#define RELEASE_VERSION                         "0.2"
#define KEY_MAJOR                               242

#define SHC_SCAN_SIZE                           5

/*#define GPIO_D3_PE24	(128+24)*/
/*A = 0     B = 1      C = 2        D = 3       E = 4 */

#if 0
#define KEYIN_0                                 ((32*3)+6) /*PD06*/
#define KEYIN_1                                 ((32*3)+7) /*PD07*/
#define KEYIN_2                                 ((32*3)+8) /*PD08*/
#define KEYIN_3                                 ((32*3)+9) /*PD09*/
#define KEYIN_4                                 ((32*3)+10) /*PD10*/
#define KEYIN_5                                 ((32*3)+11) /*PD11*/

#define KEYOUT_0                                ((32*3)+25) /*PD25*/
#define KEYOUT_1                                ((32*3)+26) /*PD26*/
#define KEYOUT_2                                ((32*2)+0) /*PD27 -> pc0*/
#define KEYOUT_3                                ((32*2)+3) /*PD28 -> pc3*/
#endif

#define KEYIN_0                                 (12) //((32*3)+6) /*PA12*/
#define KEYIN_1                                 (13) // ((32*3)+7) /*PA13*/
#define KEYIN_2                                 (28) //((32*3)+8) /*PA28*/
#define KEYIN_3                                 ((32*2)+26) /*PC26*/
#define KEYIN_4                                 ((32*3)+12) /*PD12*/
//#define KEYIN_5                                 ((32*3)+11) /*PD14*/

#define KEYOUT_0                                ((32*3)+16) /*PD16*/
#define KEYOUT_1                                ((32*3)+17) /*PD17*/
#define KEYOUT_2                                ((32*3)+25)  /*PD25*/
#define KEYOUT_3                                ((32*3)+14)  /*PD14*/





#define KEY_LED                                ((32*2)+7) /*PC07*/
#define PC08_ONLINE_LED                        ((32*2)+8) /*PC08*/
#define PC09_ICC_LED                           ((32*2)+9) /*PC09*/

/* Key Output define */
#define KEYOUT_HIGH                             0x0F
#define KEYOUT_LOW                              0x00
#define KEYOUT_1_CHK                            0x01
#define KEYOUT_2_CHK                            0x02
#define KEYOUT_3_CHK                            0x04
#define KEYOUT_4_CHK                            0x08


/* Key Input define */
#define NO_BUTTON                               0x00
#define COLUMN0                                 0x01
#define COLUMN1                                 0x02
#define COLUMN2                                 0x04
#define COLUMN3                                 0x08
#define COLUMN4                                 0x10
#define COLUMN5                                 0x20

#if 0
static const unsigned short shc350_scancodes[] = {
    KEYP_BACKSPACE, KEYP_END,     KEYP_ENTER,      KEYP_F1,      KEYP_F2,
    KEYP_F3,        KEYP_F4,      KEYP_7,          KEYP_8,       KEYP_9,
    KEYP_F5,        KEYP_4,       KEYP_5,          KEYP_6,       KEYP_TOTAL,
    KEYP_1,         KEYP_2,       KEYP_3,          KEYP_UP,      KEYP_DZERO,
    KEYP_0,         KEYP_TZERO,   KEYP_DOWN            
};
#else
static const unsigned short shc350_scancodes[] = {
    KEYP_BACKSPACE, KEYP_END,     KEYP_ENTER,      KEYP_F1,      KEYP_F2,
    KEYP_F3,        KEYP_F4,      KEYP_1,          KEYP_2,       KEYP_3,
    KEYP_F5,        KEYP_4,       KEYP_5,          KEYP_6,       KEYP_TOTAL,
    KEYP_7,         KEYP_8,       KEYP_9,          KEYP_UP,      KEYP_DZERO,
    KEYP_0,         KEYP_TZERO,   KEYP_DOWN            
};	
#endif

static const char keycode_map[6][4] = {
    { 0,    1,      2,      0xff },
    { 3,    4,      5,      6 },
    { 7,    8,      9,      10 },
    { 11,   12,     13,     14 },
    { 15,   16,     17,     18 },
    { 19,   20,     21,     22 }
};

/*struct shckbd {
    struct input_polled_dev *poll_dev;
    unsigned short keymap[ARRAY_SIZE(shc350_scancodes)];
    unsigned char length;
    unsigned char old_scan[SHC_SCAN_SIZE];
    unsigned char new_scan[SHC_SCAN_SIZE];
};*/

DECLARE_WAIT_QUEUE_HEAD(key_poll_read);

#define KEY_CHK_TIME                                1
#define KEY_MAXBUFF                                 100

static struct timer_list key_scan_timer;

static unsigned char now_keyvalue, old_keyvalue,ucLastKeyvalue,ucCurrentKeyvalue;
static unsigned char keydata[KEY_MAXBUFF];

static unsigned short keydata_count;
static unsigned short keydata_readcnt;

static unsigned char chatter;
static unsigned char continuekey_chk;

static unsigned char checked,old_tmp=-1;
static unsigned short ucKeyscan_time;
static struct class *key_class;

///////////////////////////////////////////////////////////////////////////////////
// proc_fs system
///////////////////////////////////////////////////////////////////////////////////
#define PROC_KEY_DIR                            "keypad"
#define PROC_KEY_REPEAT                         "repeat"
#define PROC_KEY_VER                            "version"

#define PROC_REPEAT_DATA                        0x01
#define PROC_VER_DATA                           0x02

#define PROC_BUF_LEN                            4

static struct timer_list key_scan_timer;
//static struct proc_dir_entry    *key_proc_dir, *key_proc_repeat, *key_proc_ver;


static void gpio_init_set(void)
{
    gpio_request(KEYIN_0,"sysfs");
    gpio_request(KEYIN_1,"sysfs");
    gpio_request(KEYIN_2,"sysfs");
    gpio_request(KEYIN_3,"sysfs");
    gpio_request(KEYIN_4,"sysfs");

    gpio_request(KEYOUT_0,"sysfs");
    gpio_request(KEYOUT_1,"sysfs");
    gpio_request(KEYOUT_2,"sysfs");
    gpio_request(KEYOUT_3,"sysfs");

    gpio_direction_input(KEYIN_0);
    gpio_direction_input(KEYIN_1);
    gpio_direction_input(KEYIN_2);
    gpio_direction_input(KEYIN_3);
    gpio_direction_input(KEYIN_4);
  //  gpio_direction_input(KEYIN_5);

    gpio_direction_output(KEYOUT_0,false);
    gpio_direction_output(KEYOUT_1,false);
    gpio_direction_output(KEYOUT_2,false);
    gpio_direction_output(KEYOUT_3,false);
    

}


static void set_col_value(unsigned int value)
{
    if ( value & KEYOUT_1_CHK ) gpio_set_value( KEYOUT_0, 1 );
    else                        gpio_set_value( KEYOUT_0, 0 );

    if ( value & KEYOUT_2_CHK ) gpio_set_value( KEYOUT_1, 1 );
    else                        gpio_set_value( KEYOUT_1, 0 );

    if ( value & KEYOUT_3_CHK ) gpio_set_value( KEYOUT_2, 1 );
    else                        gpio_set_value( KEYOUT_2, 0 );

    if ( value & KEYOUT_4_CHK ) gpio_set_value( KEYOUT_3, 1 );
    else                        gpio_set_value( KEYOUT_3, 0 );

}

static unsigned char get_input_value(void)
{
    unsigned char ret = 0;
    if ( gpio_get_value(KEYIN_0) == 0 ) ret |= 0x01; 
    if ( gpio_get_value(KEYIN_1) == 0 ) ret |= 0x02; 
    if ( gpio_get_value(KEYIN_2) == 0 ) ret |= 0x04; 
    if ( gpio_get_value(KEYIN_3) == 0 ) ret |= 0x08; 
    if ( gpio_get_value(KEYIN_4) == 0 ) ret |= 0x10; 
  //  if ( gpio_get_value(KEYIN_5) == 0 ) ret |= 0x20; 

    return ret;
}

void initVal(void)
{
    now_keyvalue = 0;
    old_keyvalue = 0;
    ucLastKeyvalue = 0; // sjh 
    ucCurrentKeyvalue = 0; //sjh
    chatter = 0;
    memset( keydata, 0, sizeof(keydata) );

    keydata_readcnt = 0;        
    keydata_count = 0;

    continuekey_chk = 0;

     DPRINTK(" called flush\n"); 
}


#define CHAT_TIME   2 //
static void key_scan_handler( unsigned long data )
{
     /* struct shckbd *shckbd = dev->private; */
    
     unsigned char tmp = 0;
    int loop = 10,cnt;//20;
    char column_step = 0;
    unsigned char output_pos = 0, input_pos = 0;
    
    int output[4] = { KEYOUT_1_CHK, KEYOUT_2_CHK, KEYOUT_3_CHK, KEYOUT_4_CHK };
    int input[6] = { COLUMN0, COLUMN1, COLUMN2, COLUMN3, COLUMN4, COLUMN5 };
    

    tmp = get_input_value();
  
    /* No Button */
    if ( tmp == NO_BUTTON )
    {    
        old_keyvalue = 0;
        now_keyvalue = 0;
        ucCurrentKeyvalue = 0;
        output_pos = 0;
        input_pos = 0;
        checked = 0;
        
        mod_timer( &key_scan_timer, jiffies + msecs_to_jiffies(ucKeyscan_time) );
        set_col_value( KEYOUT_LOW );
        return;
    }
    if ( checked == 0 )
        checked = 1;
    else if ( checked == 1 && (continuekey_chk == 0) && old_tmp == tmp)
    {        
        DPRINTK("Yet Key Pressed(%d) %d \n", continuekey_chk,tmp);
        mod_timer( &key_scan_timer, jiffies + msecs_to_jiffies(ucKeyscan_time) );
        return;
    }

    DPRINTK("old_tmp = %x , tmp = %X\n", old_tmp,tmp ); 
    old_tmp = tmp;
    
    /* all key input high */
    set_col_value(KEYOUT_HIGH);
    
    switch( tmp )
    {
        case COLUMN0:   input_pos = 0;  break;
        case COLUMN1:   input_pos = 1;  break;
        case COLUMN2:   input_pos = 2;  break;
        case COLUMN3:   input_pos = 3;  break;
        case COLUMN4:   input_pos = 4;  break;
        case COLUMN5:   input_pos = 5;  break;
    }
    
    //DPRINTK("Button pressed     : %d / %d\n", tmp,input_pos);
    
    while( loop-- )
    {
        mdelay(1);
        
        switch ( column_step )
        {
            case 0:
                set_col_value( KEYOUT_HIGH ^ output[output_pos] );
                column_step = 1;
                break;

            case 1:
                if ( (tmp = get_input_value()) == input[input_pos] )
                {
                    chatter = 0;
                    for(cnt = 0; cnt < CHAT_TIME;cnt++)
                    {
                        if(get_input_value() == input[input_pos])
                            chatter++;
                        mdelay(1);                            
                    }

                    if ( chatter >= CHAT_TIME )// 3 
                    {
                        int keycode = keycode_map[input_pos][output_pos];                        
                        //now_keyvalue = keycode_map[input_pos][output_pos];
                        now_keyvalue = shc350_scancodes[ keycode ];
                        
                        keydata[keydata_count++] = now_keyvalue;
                        if ( keydata_count >= KEY_MAXBUFF ) 
                            keydata_count = 0;  

DPRINTK("%x/loop=%d/chatter=%d/cnt=(%d:%d)\n",
    now_keyvalue,loop,chatter,keydata_count,keydata_readcnt);
                        
                        ucLastKeyvalue = now_keyvalue; // sjh                            
                        ucCurrentKeyvalue = now_keyvalue; // sjh
                        
                        chatter = 0;
                        input_pos = output_pos = 0;
                        now_keyvalue = old_keyvalue = 0;
                        wake_up_interruptible( &key_poll_read );
						mod_timer( &key_scan_timer, jiffies + msecs_to_jiffies(ucKeyscan_time) );
                        return ;
                    }

                }

                output_pos++;
                if ( output_pos >= 4 ) output_pos = 0;
                set_col_value( KEYOUT_HIGH );
                set_col_value( KEYOUT_HIGH ^ output[output_pos] );
                break;
        }   // switch ( column_step )   
    }   // while( loop-- )

    mod_timer( &key_scan_timer, jiffies + msecs_to_jiffies(ucKeyscan_time) );
    set_col_value( KEYOUT_LOW );
}
static unsigned int key_poll( struct file *filp, poll_table *wait )
{
    unsigned int mask = 0;

   
    poll_wait( filp, &key_poll_read, wait );

    if(keydata_readcnt != keydata_count)
    {        
        mask |= (POLLIN | POLLRDNORM);
    }

    return mask;
}

static int key_open(struct inode *inode, struct file *filefp)
{
    if ( !try_module_get(THIS_MODULE) ) return -ENODEV;
    
    initVal();
    
    DPRINTK("%s %s Open Success\n", DEVICE_NAME, RELEASE_VERSION );
    return 0;
}


static int key_release(struct inode *inode, struct file *filefp)
{
    module_put(THIS_MODULE);
    
    DPRINTK("%s %s Release Success\n", DEVICE_NAME, RELEASE_VERSION );
    return 0;
}


static ssize_t key_read( struct file *flip, char *buff, size_t count, loff_t *f_pos)
{
    unsigned long flag,uLeftBuff;
    int retstat,i;
    unsigned char buf_keydata[KEY_MAXBUFF];

    if ( (keydata_readcnt==keydata_count) && (flip->f_flags & O_NONBLOCK) ) return -EAGAIN;
    
    retstat = wait_event_interruptible( key_poll_read, keydata_readcnt!=keydata_count );

    if ( retstat ) return retstat;
    
    local_irq_save( flag );
    

    if(keydata_count > keydata_readcnt)
        uLeftBuff =  keydata_count - keydata_readcnt ;
    else if(keydata_count == keydata_readcnt)
        uLeftBuff = 0;
    else
        uLeftBuff = KEY_MAXBUFF-keydata_readcnt+ keydata_count ;

    if(count<uLeftBuff)    
        uLeftBuff = count;
    
//printk(KERN_ALERT "keydata_count= %d/ keydata_readcnt= %d/uLeftBuff = %d\n",keydata_count,
//keydata_readcnt,uLeftBuff);


    for(i=0;i<uLeftBuff;i++)
    {
        buf_keydata[i] = keydata[keydata_readcnt++];
        if ( keydata_readcnt >= KEY_MAXBUFF ) 
            keydata_readcnt = 0;
    }
    
    if ( copy_to_user( (unsigned char *)buff, buf_keydata, uLeftBuff) )
    {
        
        DPRINTK("key_read copy_to_user() Error\n");
        return -EFAULT;
    }

    
    local_irq_restore( flag );
    mod_timer( &key_scan_timer, jiffies + msecs_to_jiffies(ucKeyscan_time) );
    
    return uLeftBuff;
}


static long key_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    int ret = 0;
    unsigned long flag;
    unsigned char version[50],ret_value;
    unsigned short uLeftBuff;

    if (_IOC_TYPE(cmd) != KEY_IOCTL_MAGIC) return -ENOTTY;
    if (_IOC_NR(cmd) >= KEY_IOCTL_MAGIC) return -ENOTTY;

    switch(cmd)
    {
        case KEY_SET_SCANTIME:
            local_irq_save( flag );
            if ( copy_from_user( &ucKeyscan_time, (unsigned short *)arg, sizeof(unsigned short)) )
            {
                local_irq_restore( flag );
                return -EFAULT;
            }

            if(!ucKeyscan_time || ucKeyscan_time>1000) // 0 이거나 1초면 안됨 ! 
                ucKeyscan_time = KEY_CHK_TIME; // deault 1 ms 

            DPRINTK("KEY_SET_SCANTIME %d\n",ucKeyscan_time);
            
            local_irq_restore( flag );
            del_timer_sync( &key_scan_timer );
            mod_timer( &key_scan_timer, jiffies + msecs_to_jiffies(ucKeyscan_time) );    
            
            break;
            
        case KEY_INIT:
            initVal();
            break;
            
        case KEY_CONTINUE_CHK:
            if ( copy_from_user( &continuekey_chk, (unsigned char *)arg, sizeof(unsigned char)) )
            {
                return -EFAULT;
            }
            if ( !continuekey_chk ) initVal();
            break;
            
         case KEY_GET_KEY:
            local_irq_save( flag );
            if(keydata_readcnt!=keydata_count)
            {                
                ret_value = keydata[keydata_readcnt++];

                if ( keydata_readcnt >= KEY_MAXBUFF ) 
                    keydata_readcnt = 0;
            }
            else
                ret_value = 0;
            
            if ( copy_to_user( (unsigned char *)arg, &ret_value, sizeof(unsigned char)) )
            {
                local_irq_restore( flag );
                DPRINTK("KEY_GET_KEY copy_to_user() Error\n");
                return -EFAULT;
            }            
            
            local_irq_restore( flag );
            
            break;    
        case KEY_GET_PRESSKEY:

            if ( copy_to_user( (unsigned char *)arg, &ucCurrentKeyvalue, sizeof(unsigned char)) )
            {
                DPRINTK("KEY_GET_PRESSKEY copy_to_user() Error\n");
                return -EFAULT;
            }
            break;    

        case KEY_GET_BUFF_LEN:
            local_irq_save( flag );

            if(keydata_count > keydata_readcnt)
                uLeftBuff =  keydata_count - keydata_readcnt ;
            else if(keydata_count == keydata_readcnt)
                uLeftBuff = 0;
            else
                uLeftBuff = KEY_MAXBUFF-keydata_readcnt+ keydata_count ;

//            printk(KERN_ALERT "keydata_count= %d/ keydata_readcnt= %d/uLeftBuff = %d\n",
//keydata_count,keydata_readcnt,uLeftBuff);
            
            local_irq_restore( flag );
            
            if ( copy_to_user( (unsigned short *)arg, &uLeftBuff, sizeof(unsigned short)) )
            {
                
                DPRINTK("KEY_GET_PRESSKEY copy_to_user() Error\n");
                return -EFAULT;
            }

            break;                   
            
        case KEY_GET_EXISTKEY:
            if ( copy_to_user( (unsigned char *)arg, &ucLastKeyvalue, sizeof(unsigned char)) )
            {
                DPRINTK("KEY_GET_EXISTKEY copy_to_user() Error\n");
                return -EFAULT;
            }
            break;
        case KEY_VERSION:
            memset( version, 0, sizeof(version) );

            if ( copy_to_user( (unsigned char *)arg, RELEASE_VERSION, strlen(RELEASE_VERSION)) )
            {
                DPRINTK("KEY_VERSION copy_to_user() Error\n");
                return -EFAULT;
            }
            break;
    }
    
    return ret;
}

static struct file_operations key_fops = {
    .owner              = THIS_MODULE,
    .open               = key_open,
    .release            = key_release,
    .unlocked_ioctl     = key_ioctl,
    .read               = key_read,
    .poll               = key_poll,
};

static struct miscdevice key_dev = {
    MISC_DYNAMIC_MINOR,
    DEVICE_NAME,
    &key_fops
};



static int  shc350_key_init(void)
{
    int err = 0;
    
    gpio_init_set();
    set_col_value(KEYOUT_LOW);

     if ( (err = misc_register(&key_dev)) )
	{
		printk(KERN_ALERT "Key driver : Unable to register driver\n");
		err = -EINVAL;
		goto out_keydev;
	}
    
    ucKeyscan_time = KEY_CHK_TIME;

    
    setup_timer( &key_scan_timer, key_scan_handler, 0 );
    mod_timer( &key_scan_timer, jiffies + msecs_to_jiffies(ucKeyscan_time) );

    printk(KERN_ALERT "\n%s %s %s Load Success - %s \n", MODEL_NAME, DEVICE_NAME, 
            RELEASE_VERSION, __TIMESTAMP__);
    
    goto success;


out_keydev:

    gpio_free(KEYIN_0);
    gpio_free(KEYIN_1);
    gpio_free(KEYIN_2);
    gpio_free(KEYIN_3);
    gpio_free(KEYIN_4);
//    gpio_free(KEYIN_5);
    gpio_free(KEYOUT_0);
    gpio_free(KEYOUT_1);
    gpio_free(KEYOUT_2);
    gpio_free(KEYOUT_3);
    unregister_chrdev(KEY_MAJOR, DEVICE_NAME);
    return err;    
    
    
success:
    return err;    
}


static void __exit shc350_key_exit(void)
{    
    del_timer_sync( &key_scan_timer ); 

    gpio_free(KEYIN_0);
    gpio_free(KEYIN_1);
    gpio_free(KEYIN_2);
    gpio_free(KEYIN_3);
    gpio_free(KEYIN_4);
 //   gpio_free(KEYIN_5);
    gpio_free(KEYOUT_0);
    gpio_free(KEYOUT_1);
    gpio_free(KEYOUT_2);
    gpio_free(KEYOUT_3);

    misc_deregister(&key_dev);

    printk(KERN_ALERT "\n%s %s %s Exit Success\n", MODEL_NAME, DEVICE_NAME, RELEASE_VERSION );
   
}

module_init(shc350_key_init);
module_exit(shc350_key_exit);

MODULE_AUTHOR("SHC");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("SHC-500 KEY driver");
