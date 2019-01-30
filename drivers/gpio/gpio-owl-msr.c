/*
 *
 *
 *  Revision History
 * 
 * 0.2 : add when reverse swipe, improve "find ss code" algrithm.(msr2_analyze,msr3_analyze)
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
#include <linux/poll.h>

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

#include "gpio-owl-msr.h"

#include <mach/gpio.h> //add aron


//#define DEBUG
#ifdef DEBUG
#define DPRINTK(format,args...)     printk(KERN_ALERT "%s(%d): " format, __FUNCTION__, __LINE__, ## args)
#else
#define DPRINTK(format,args...)
#endif

#define MODEL_NAME                              "SHC-650"
#define DEVICE_NAME                             "msr"
#define RELEASE_VERSION                         "0.2"
//0.2 : 트랙 2,3 핀 설정 변경 

/* GPIO PIN define ---------------------------------------------------*/
/*#define GPIO_D3_PE24	(128+24)*/
/*A = 0     B = 1      C = 2        D = 3       E = 4 */
#define MSR_CPD_IQR                              0
#define MSR_RCP1_IQR                             1
#define MSR_RCP2_IQR                             2

//#define MSR_CPD                                 ((32*0)+27) /*PA27_MSR_NCPD*/
#define MSR_CPD                  OWL_GPIO_PORTC(26)  /* MSR_NCPD*/

//PB24_MSR_NRCP3
//PB25_MSR_NRDD3

/*
PD4_MSR_NRCP2
PD5_MSR_NRDD2
*/

#define PIO_NUM_IO		32

#define	AT91C_PIN_PA(io)	(0 * PIO_NUM_IO + io)
#define	AT91C_PIN_PB(io)	(1 * PIO_NUM_IO + io)
#define	AT91C_PIN_PC(io)	(2 * PIO_NUM_IO + io)
#define	AT91C_PIN_PD(io)	(3 * PIO_NUM_IO + io)
#define	AT91C_PIN_PE(io)	(4 * PIO_NUM_IO + io)


#if 0
/*track 2*/
#define MSR_RCP2                                AT91C_PIN_PD(4)  /*PD4_MSR_NRCP2*/
#define MSR_RDD2                                AT91C_PIN_PD(5)  /*PD5_MSR_NRDD2*/
/*track 3*/
#define MSR_RCP1                                AT91C_PIN_PB(24)  /*PB24_MSR_NRCP3*/
#define MSR_RDD1                                AT91C_PIN_PB(25)  /*PB25_MSR_NRDD3*/
#else
/*track 2*/
//#define MSR_RCP1                                AT91C_PIN_PD(4)  /*PD4_MSR_NRCP2*/
#define MSR_RCP1                                OWL_GPIO_PORTD(12)  /*PD4_MSR_NRCP2*/
//#define MSR_RDD1                                AT91C_PIN_PD(5)  /*PD5_MSR_NRDD2*/
#define MSR_RDD1                                OWL_GPIO_PORTD(14)  /*PD5_MSR_NRDD2*/
/*track 3*/
//#define MSR_RCP2                                AT91C_PIN_PB(24)  /*PB24_MSR_NRCP3*/
#define MSR_RCP2                                OWL_GPIO_PORTD(16)  /*PB24_MSR_NRCP3*/
//#define MSR_RDD2                                AT91C_PIN_PB(25)  /*PB25_MSR_NRDD3*/
#define MSR_RDD2                                OWL_GPIO_PORTD(17)  /*PB25_MSR_NRDD3*/
#endif

/*--------------------------------------------------------------------*/

#define CARD_REMOVED							0x00
#define CARD_INSERTED							0x01

#define M_ON									0
#define M_OFF									1

#define TRK1_DATA_SIZE                          79
#define TRK2_DATA_SIZE                          40
#define TRK3_DATA_SIZE                          107

#define TRK2_BIT_SIZE                           (TRK2_DATA_SIZE*5)
#define TRK3_BIT_SIZE                           (TRK3_DATA_SIZE*5)


static DEFINE_SPINLOCK(msr_lock);

static DECLARE_WAIT_QUEUE_HEAD(wait_queue);
static DECLARE_WAIT_QUEUE_HEAD(msr_poll_read);

static struct timer_list swiped_timer;

static struct {
    unsigned int evt_thread_stt;
    unsigned int evt_wait_cond;
    struct task_struct *evt_kthread;
 
    spinlock_t evt_thread_lock;
} hub_wait_queue_ctx;

////////////////////////////////////////////////////////////////////////
//      LRC check
///////////////////////////////////////////////////////////////////////
// read value from MSR
static unsigned char TRACK23_VALUE[16] = {
    0x10, 0x01, 0x02, 0x13, 0x04, 0x15, 0x16, 0x07,
    0x08, 0x19, 0x1a, 0x0b, 0x1c, 0x0d, 0x0e, 0x1f
};


/*********************************************************
 * Global Variables                                      *
 *********************************************************/
static char cfCardStatus = CARD_REMOVED;
static unsigned char tr2_started;
static unsigned char tr3_started;
static unsigned int tr2_index;
static unsigned int tr3_index;
static unsigned int tr2_reverse_index;
static unsigned int tr3_reverse_index;
static volatile int gidDataIsReady;
static unsigned char tr2buff[MAX_BITSTREAM_LEN];
static unsigned char tr3buff[MAX_BITSTREAM_LEN];
static unsigned char CPD2_CHK;							///< MSR로 부터 Track2 데이터를 받고, 분석이 완료 시 = 1
static unsigned char CPD3_CHK;							///< MSR로 부터 Track3 데이터를 받고, 분석이 완료 시 = 1

static volatile struct Msr_Data Msr_Data[2];
static volatile int gi_MsrOnOff = M_ON;


static u8 *pPacket = 0;
static u8 PacketData[1+2+TRK2_DATA_SIZE+1+TRK3_DATA_SIZE+1+1+1];

static unsigned int gIrqnumber[3]={0,};
static unsigned long ulDeviceOpen;

///////////////////////////////////////////////////////////////////////////////////
// Device Operations
///////////////////////////////////////////////////////////////////////////////////

static bool _hub_work_evt_check_cond(void)
{
    spin_lock(&hub_wait_queue_ctx.evt_thread_lock);
    bool ret = hub_wait_queue_ctx.evt_wait_cond;
    spin_unlock(&hub_wait_queue_ctx.evt_thread_lock);
    return ret;
}

/*!
 \function msr_init_set
 \brief PIO 핀들을 초기화 시킨다.
 \retval static void :
 \param void :
*/
static void msr_init_set(void)
{

    gpio_request(MSR_CPD,"sysfs");
    gpio_request(MSR_RCP1,"sysfs");
    gpio_request(MSR_RCP2,"sysfs");
    gpio_request(MSR_RDD1,"sysfs");
    gpio_request(MSR_RDD2,"sysfs");

    gpio_direction_input(MSR_CPD);
    gpio_direction_input(MSR_RCP1);    
    gpio_direction_input(MSR_RCP2);
    gpio_direction_input(MSR_RDD1);
    gpio_direction_input(MSR_RDD2);

/*	if ( at91_set_deglitch( MSR_CPD, 1) )
	{
		printk(KERN_ALERT " Could not set pin %i for GPIO deglitch.\n", MSR_CPD);
		return;
	}
	if ( at91_set_deglitch( MSR_RCP1, 1) )
	{
		printk(KERN_ALERT " Could not set pin %i for GPIO deglitch.\n", MSR_RCP1);
		return;
	}
	if ( at91_set_deglitch( MSR_RCP2, 1) )
	{
		printk(KERN_ALERT " Could not set pin %i for GPIO deglitch.\n", MSR_RCP2);
		return;
	}
*/

}

static void initVal(void)
{
    memset( tr2buff, 0, MAX_BITSTREAM_LEN );
    tr2_started = 0;
    tr2_index = 0;
    tr2_reverse_index = 0;
    CPD2_CHK = 0;

    memset( tr3buff, 0, MAX_BITSTREAM_LEN );
    tr3_started = 0;
    tr3_index = 0;
    tr3_reverse_index = 0;
    CPD3_CHK = 0;

    cfCardStatus = CARD_REMOVED;
}

static void swiped_handler(unsigned long data)
{
    unsigned long irqflags;
	mod_timer( &swiped_timer, jiffies + HZ/40 );

	if ( CPD2_CHK || CPD3_CHK )
	{
        spin_lock_irqsave(&hub_wait_queue_ctx.evt_thread_lock, irqflags);
        hub_wait_queue_ctx.evt_wait_cond = true;
        spin_unlock_irqrestore(&hub_wait_queue_ctx.evt_thread_lock, irqflags);

		wake_up_interruptible( &wait_queue );
		wake_up_interruptible( &msr_poll_read );
    
	}

}

static int msr2_analyze(void)
{
    int i,const_plus;
    unsigned char track2[400] = {0};
    unsigned char buff2[1024] = {0};
    unsigned char crcch = 0, tmpch = 0;
    unsigned int  ilen = 0, ix = 0;
    int err = TRACK_STATUS_TRACK2_OK;
#ifdef DEBUG
    char cf1X = 0;
    int j;
#endif

    memset( track2, 0, 400 );
    memset( buff2,  0, 1024 );


    if ( tr2_started )
    {
        ////////////////////////////////////////////////////
        // BIT Check
        ////////////////////////////////////////////////////
        for (i = 0; i < tr2_index; i+= 5)
        {
            track2[ix] = (tr2buff[ilen+i+4] << 4) |
                (tr2buff[ilen+i+3] << 3) |
                (tr2buff[ilen+i+2] << 2) |
                (tr2buff[ilen+i+1] << 1) |
                (tr2buff[ilen+i+0] << 0);
            if (track2[0] != 0x0b)
            {
                DPRINTK("track2[0] != 0x0b (0x%02X)\n", track2[0]);
                break;
            }
            if (track2[ix-2] == 0x1f)
            {
                ilen = ix;
                ix = ix -2;

#ifdef DEBUG
                if ( track2[1] == 0x1f ) cf1X = 1;
#endif
                break;
            }
            ix++;
        }
#ifdef DEBUG
        printk("RAW DATA tr2buff[%d],ix[%d] = \n", tr2_index, ix);
        i = j = 0;
        printk("%03d : ", j++ );
        for ( i = 0; i < tr2_index; i++ )
        {
            printk("%2x ", tr2buff[i]);
            if ( i && !((i+1) % 5) ) printk("\n%03d : ",j++);
        }
        printk("\n");
#endif
        ////////////////////////////////////////////////////
        // if reverse swipe
        ////////////////////////////////////////////////////
        if ( (track2[0] != 0x0b) || \
             (track2[0] == 0x0b && track2[1] == 0x1f) )
        {
            memset(track2, 0, sizeof(track2) );
            ix = 0;
            ilen = 0;
            for (i = 0; i < tr2_reverse_index + 1; i++) 
                buff2[i] = tr2buff[tr2_reverse_index - i];

            const_plus = 1;
            for (i = 0; i < tr2_index; i += const_plus)
            {
                track2[ix] = (buff2[ilen+i+4] << 4) |
                    (buff2[ilen+i+3] << 3) |
                    (buff2[ilen+i+2] << 2) |
                    (buff2[ilen+i+1] << 1) |
                    (buff2[ilen+i+0] << 0);

                if (track2[0] != 0x0b) // find start code (ss)! 
                {
                    ix = 0;
                    continue;
                }

                if(track2[0] == 0x0b) { // SS is finded ! change const_plus 1 to 5 
                    const_plus = 5;
                }
                                
                if (ix >=2 && track2[ix-2] == 0x1f) // exit when last data is end code.
                {
                    ilen = ix;
                    ix = ix -2;
                    break;
                }
                ix++;
            }
        }
#if 0//def DEBUG
        if ( buff2[0] )
        {
            printk("RAW DATA REVERSE buff2[%d],ix[%d] = \n", tr2_reverse_index, ix);
            i = j = 0;
            printk("%03d : ", j++ );
            for ( i = 0; i < tr2_reverse_index+1; i++ )
            {
                printk("%2x ", buff2[i]);
                if ( i && !((i+1) % 5) ) printk("\n%03d : ",j++);
            }
            printk("\n");
        }
#endif

#ifdef DEBUG
        DPRINTK("TRACK2\n");

        printk(KERN_ALERT "0   ");
        for(i=0;i<20 ;i++)   printk("%2x ",track2[i]); printk("\n20  ");
        for(i=20;i<40 ;i++)  printk("%2x ",track2[i]); printk("\n40  ");
        for(i=40;i<60 ;i++)  printk("%2x ",track2[i]); printk("\n60  ");
        for(i=60;i<80 ;i++)  printk("%2x ",track2[i]); printk("\n80  ");
        for(i=80;i<100 ;i++) printk("%2x ",track2[i]); printk("\n100 ");
        for(i=100;i<120 ;i++) printk("%2x ",track2[i]); printk("\n120 ");
        for(i=120;i<140 ;i++) printk("%2x ",track2[i]); printk("\n140 ");
        for(i=140;i<160 ;i++) printk("%2x ",track2[i]); printk("\n160 ");
        printk("\n");

        printk("cf1X = %d\nb", cf1X);
#endif
        ////////////////////////////////////////////////////
        // Check bit
        ////////////////////////////////////////////////////

        ////////////////////////////////
        // Check start Code(0x0b)
        ///////////////////////////////
        if (track2[0] != 0x0b)
        {
            DPRINTK("track2[0] != 0x0b (0x%02X)\n", track2[0]);
            err = TRACK_STATUS_SS_ERR;
            goto err_out;
        }

        ////////////////////////////////
        // Check End Code (0x1f)
        ///////////////////////////////
        if (track2[ix] != 0x1f)
        {
            initVal();
            DPRINTK("End Error::track2[%d] = 0x%02X\n", ix, track2[ix]);
            err = TRACK_STATUS_ES_ERR;
            goto err_out;
        }

        ////////////////////////////////
        // parity check
        ///////////////////////////////
        crcch = 0x0;
        for (i = 0, ix = 0; i < ilen -1; i++)
        {
            tmpch = track2[i] & 0x0f;
            if ( track2[i] != TRACK23_VALUE[tmpch] )
            {
                DPRINTK("Parity Error::track[%d] = 0x%02X, TRACK23_VALUE[%d] = 0x%02X\n",
                        i, track2[i], tmpch, TRACK23_VALUE[tmpch]);
                err = TRACK_STATUS_PARITY_ERR;
                goto err_out;
            }
            crcch ^= (tmpch & 0x0f);
            track2[i] = tmpch | 0x30;

            /* extract only track2 data*/
            /* if ( '0' <= track2[i] && track2[i] <= '9' ) */
            /*     Msr_Data[TRACK2].byte[ix++] = track2[i]; */
            /* if ( track2[i] == '=' ) */
            /*     Msr_Data[TRACK2].byte[ix++] = track2[i]; */
        }
        track2[i] &= 0x0f;


        ////////////////////////////////
        // CRC check
        ///////////////////////////////
        if (track2[ilen-1] != crcch)
        {
            err = TRACK_STATUS_LRC_ERR;
            goto err_out;
        }
        gidDataIsReady = 1;

        for ( i = 0; i < ilen-1; i++ )
        {
            /* extract only track2 data*/
            if ( '0' <= track2[i] && track2[i] <= '9' )
                Msr_Data[TRACK2].byte[ix++] = track2[i];
            if ( track2[i] == '=' )
                Msr_Data[TRACK2].byte[ix++] = track2[i];
            if ( track2[i] == '?' ) break;
        }

        goto success;

    }   // if ( tr2_started )

success:
    Msr_Data[TRACK2].bytelen = strlen( (const char *)Msr_Data[TRACK2].byte);
    Msr_Data[TRACK2].status = err;
#ifdef DEBUG
        DPRINTK("TRACK2 or 0x30\n");
        printk(KERN_ALERT "0   ");
        for(i=0;i<20 ;i++)   printk("%2x ",track2[i]); printk("\n20  ");
        for(i=20;i<40 ;i++)  printk("%2x ",track2[i]); printk("\n40  ");
        for(i=40;i<60 ;i++)  printk("%2x ",track2[i]); printk("\n60  ");
        for(i=60;i<80 ;i++)  printk("%2x ",track2[i]); printk("\n80  ");
        for(i=80;i<100 ;i++) printk("%2x ",track2[i]); printk("\n100 ");
        printk("\n");
#endif
    return err;

err_out:
    Msr_Data[TRACK2].status = err;
#ifdef DEBUG
    DPRINTK("TRACK2 Status = 0x%02X\n", err);
#endif
    initVal();
    return err;

}

static int msr3_analyze(void)
{
    unsigned char tmpch = 0, crcch = 0;
    unsigned int i, ix = 0, ilen = 0,const_plus;
    unsigned char track3[400] = {0};
    unsigned char buff3[1024] = {0};
    int err = TRACK_STATUS_TRACK3_OK;
#ifdef DEBUG
    int j;
#endif


    memset( track3, 0, 400 );
    memset( buff3,  0, 1024 );
    
    if ( tr3_started )
    {

        //////////////////////////////////////////////
        //      BIT check
        //////////////////////////////////////////////
        for (i=0; i < tr3_index; i+=5)
        {
            track3[ix] = (tr3buff[ilen+i+4] <<4) |
                (tr3buff[ilen+i+3] <<3) |
                (tr3buff[ilen+i+2] <<2) |
                (tr3buff[ilen+i+1] <<1) |
                (tr3buff[ilen+i+0] <<0 );

            if(track3[0] != 0x0b) break; // if reverse! 
            if(ix>=2 && track3[ix-2] == 0x1f) // finded last code 
            {
                ilen=ix;
                ix=ix-2;
                break;
            }
            ix++;
        }
#ifdef DEBUG
        printk("\nRAW DATA tr3buff[%d],ix[%d] = \n", tr3_index, ix);
        i = j = 0;
        printk("%03d : ", j++ );
        for ( i = 0; i < tr3_index; i++ )
        {
            printk("%2x ", tr3buff[i]);
            if ( i && !((i+1) % 5) ) printk("\n%03d : ",j++);
        }
        printk("\n");
#endif
        /////////////////////////////////////////////
        //      if reverse swipe
        /////////////////////////////////////////////
        if(track3[0] != 0x0b)
        {
            memset(track3      , 0 , sizeof(track3));
            ix=0;
            ilen = 0; // 20160719_sjh
            for( i = 0;i < tr3_reverse_index + 1; i++ )
                buff3[i] = tr3buff[tr3_reverse_index - i];
            const_plus = 1;
            for( i = 0; i < tr3_index; i += const_plus)
            {
                track3[ix] = (buff3[ilen+i+4] <<4) |
                    (buff3[ilen+i+3] <<3) |
                    (buff3[ilen+i+2] <<2) |
                    (buff3[ilen+i+1] <<1) |
                    (buff3[ilen+i+0] <<0 );

                if(track3[0] != 0x0b) // continue while start code is fined! 
                {
                    ix = 0;                    
                    continue;
                }
                if(track3[0] == 0x0b) // finded ! ss
                {
                    //printk("oello! %d\n",i);
                    const_plus = 5;
                }
                if(ix >=2 && track3[ix-2] == 0x1f) // find! ES code! 
                {
                    ilen=ix;
                    ix=ix-2;
                    break; 
                }
                ix++;
            }
        }
#ifdef DEBUG
        if ( buff3[0] )
        {
            printk("RAW DATA REVERSE buff3[%d],ix[%d] = \n", tr3_reverse_index, ix);
            i = j = 0;
            printk("%03d : ", j++ );
            for ( i = 0; i < tr3_reverse_index+1; i++ )
            {
                printk("%2x ", buff3[i]);
                if ( i && !((i+1) % 5) ) printk("\n%03d : ",j++);
            }
            printk("\n");
        }
#endif

#ifdef DEBUG
        DPRINTK("TRACK3\n");
        printk(KERN_ALERT "0   ");
        for(i=0;i<20 ;i++)   printk("%2x ",track3[i]); printk("\n20  ");
        for(i=20;i<40 ;i++)  printk("%2x ",track3[i]); printk("\n40  ");
        for(i=40;i<60 ;i++)  printk("%2x ",track3[i]); printk("\n60  ");
        for(i=60;i<80 ;i++)  printk("%2x ",track3[i]); printk("\n80  ");
        for(i=80;i<100 ;i++) printk("%2x ",track3[i]); printk("\n100 ");
        for(i=100;i<120 ;i++) printk("%2x ",track3[i]); printk("\n120 ");
        for(i=120;i<140 ;i++) printk("%2x ",track3[i]); printk("\n140 ");
        for(i=140;i<160 ;i++) printk("%2x ",track3[i]); printk("\n160 ");
        printk("\n");
#endif
        /////////////////////////////////////////////////
        //     Check bit
        /////////////////////////////////////////////////

        ///////////////////////////
        //   Check start Code(0x0b)
        ///////////////////////////
        if(track3[0] != 0x0b)
        {
            DPRINTK("\n3 Check start Code \n" );
            err = TRACK_STATUS_SS_ERR;
            goto err_out;
        }

        ///////////////////////////
        //   Check End Code(0x1f)
        ///////////////////////////

        if(track3[ix] != 0x1f)
        {
            DPRINTK("\n3 Check End Code\n " );
            initVal();
            err = TRACK_STATUS_ES_ERR;
            goto err_out;
        }

        ///////////////////////////
        //  Parity Check
        ///////////////////////////
        crcch = 0x0;
        for( i = 0; i < ilen; i++ )
        {
            tmpch = track3[i] & 0x0f;
            if( (track3[i]) != (TRACK23_VALUE[tmpch]))
            {
                DPRINTK("\n 3 Parity Chec1  \n"  );
                err = TRACK_STATUS_PARITY_ERR;
                goto err_out;
            }
            crcch ^= tmpch;
            track3[i] = tmpch | 0x30;
        }
        track3[i] &= 0x0f;

#ifdef DEBUG
        DPRINTK("TRACK3 or 0x30\n");
        printk(KERN_ALERT "0   ");
        for(i=0;i<20 ;i++)   printk("%2x ",track3[i]); printk("\n20  ");
        for(i=20;i<40 ;i++)  printk("%2x ",track3[i]); printk("\n40  ");
        for(i=40;i<60 ;i++)  printk("%2x ",track3[i]); printk("\n60  ");
        for(i=60;i<80 ;i++)  printk("%2x ",track3[i]); printk("\n80  ");
        for(i=80;i<100 ;i++) printk("%2x ",track3[i]); printk("\n100 ");
        for(i=100;i<120 ;i++) printk("%2x ",track3[i]); printk("\n120 ");
        for(i=120;i<140 ;i++) printk("%2x ",track3[i]); printk("\n140 ");
        for(i=140;i<160 ;i++) printk("%2x ",track3[i]); printk("\n160 ");
        printk("\n");
#endif

        ///////////////////////////
        //CRC Check
        ///////////////////////////
        if(track3[ilen] != crcch )
        {
            DPRINTK("\nCRC 3 Check1  \n "  );
            err = TRACK_STATUS_LRC_ERR;
            goto err_out;
        }

        //extract only track3 code
        for (i = 0, ix=0; i < ilen; i++)
        {
            if (track3[i] >= '0' && track3[i] <= '9')
                /* msrdata.tr3buf[ix++] = track3[i]; */
                Msr_Data[TRACK3].byte[ix++] = track3[i];
            if (track3[i] == '=')
                /* msrdata.tr3buf[ix++] = track3[i]; */
                Msr_Data[TRACK3].byte[ix++] = track3[i];
            if (track3[i] == '?') break;
        }

        goto success;
    }   // if ( tr3_started )

success:
    if ( strlen( (const char *)Msr_Data[TRACK3].byte) <= 3 ) Msr_Data[TRACK3].status = 
TRACK_STATUS_NODATA;
    else
    {
        Msr_Data[TRACK3].bytelen = strlen( (const char *)Msr_Data[TRACK3].byte);
        Msr_Data[TRACK3].status = err;
    }
    return err;

err_out:
    Msr_Data[TRACK3].status = err;
    initVal();
    return err;

}


static irqreturn_t CPD_IrqHandler(int irq, void *dev_id)
{
	unsigned long flags;

    if (gi_MsrOnOff == M_OFF) return IRQ_HANDLED; // noise


	if ( cfCardStatus == CARD_REMOVED )
	{	// Card Swap Start
		cfCardStatus = CARD_INSERTED;


		tr2_index = tr3_index = 0;
		tr2_started = tr3_started = 0;
		tr2_reverse_index = tr3_reverse_index = 0;
		gidDataIsReady = 0;
		memset( tr2buff, 0, MAX_BITSTREAM_LEN);
		memset( tr3buff, 0, MAX_BITSTREAM_LEN);


		spin_lock_irqsave(&msr_lock, flags);
		/* set card detect High Active => interrupt when card removed */
		irq_set_irq_type( gIrqnumber[MSR_CPD_IQR], IRQ_TYPE_EDGE_RISING );
        spin_unlock_irqrestore(&msr_lock, flags);
		/* track2 Clk interrupt enable */
//		enable_irq(MSR_RCP1);
		/* track3 clk interrupt enable */
//		enable_irq(MSR_RCP2);
		

		DPRINTK("card inserted\n");
//		printk(KERN_ALERT "card inserted\n");

	}
	else
	if ( cfCardStatus == CARD_INSERTED)
	{	// Card Swap End
		cfCardStatus = CARD_REMOVED;

        spin_lock_irqsave(&msr_lock, flags);
		/* set card detect Low Active => interrupt when card inserted */
		irq_set_irq_type( gIrqnumber[MSR_CPD_IQR], IRQ_TYPE_EDGE_FALLING);
        spin_unlock_irqrestore(&msr_lock, flags);
		/* track2 Clk interrupt disable */
//		disable_irq(MSR_RCP1);
		/* track3 clk interrupt disable */
//		disable_irq(MSR_RCP2);
//		spin_unlock_irqrestore(&msr_lock, flags);

		gidDataIsReady = 1;

		memset( &Msr_Data, 0, sizeof(struct Msr_Data) );

		if ( tr2_started )
		{
			msr2_analyze();
			CPD2_CHK = 1;
		}

		if ( tr3_started )
		{
			msr3_analyze();
			CPD3_CHK = 1;
		}


		DPRINTK("card removed\n");
//		printk(KERN_ALERT "card removed\n");
	}

    
	return IRQ_HANDLED;
}

static irqreturn_t RCP1_IrqHandler(int irq, void *dev_id)
{
    unsigned long flags;

    //spin_lock_irqsave(&msr_lock, flags);
    
	if ( ulDeviceOpen )
	{
        if ( cfCardStatus == CARD_INSERTED )
        {
            if ( !gpio_get_value(MSR_RDD1) )
            {
                tr2buff[tr2_index] = 1;
                tr2_started = 1;
                tr2_reverse_index = tr2_index;
            }
            else
            if ( tr2_started == 0 ) return IRQ_HANDLED;

            tr2_index++;
            tr2_index = tr2_index & 0x3FF;
        }
	}
    //spin_unlock_irqrestore(&msr_lock, flags);
	return IRQ_HANDLED;
}

static irqreturn_t RCP2_IrqHandler(int irq, void *dev_id)
{
    unsigned long flags;
    //spin_lock_irqsave(&msr_lock, flags);
    
	if ( ulDeviceOpen )
	{
		if ( cfCardStatus == CARD_INSERTED )
		{
            if ( !gpio_get_value(MSR_RDD2) )
            {
                tr3buff[tr3_index] = 1;
                tr3_started = 1;
                tr3_reverse_index = tr3_index;
            }
            else
            if ( tr3_started == 0 ) return IRQ_HANDLED;

            tr3_index++;
            tr3_index = tr3_index & 0x3FF;
		}
	}
    //spin_unlock_irqrestore(&msr_lock, flags);
	return IRQ_HANDLED;
}

static unsigned int msr_poll(struct file *filefp, poll_table *wait)
{
	unsigned mask = 0;

	DPRINTK("msr_poll_read wait..........\n");
	poll_wait(filefp, &msr_poll_read, wait);
	if ( tr2_index || tr3_index )
	{
		mask |= POLLIN | POLLRDNORM;
	}
	DPRINTK("msr_poll_read raise(0x%X)................\n", mask);

	return mask;
}


static int msr_open(struct inode *inode, struct file *filefp)
{
    DPRINTK("open!!\n\r");
    if ( !try_module_get(THIS_MODULE) ) return -ENODEV;

    if (ulDeviceOpen)
    	return -EBUSY;

    ulDeviceOpen = 1;

    initVal();

    if ( !timer_pending(&swiped_timer) )
    {
    	setup_timer( &swiped_timer, swiped_handler, 0 );
    	mod_timer( &swiped_timer, jiffies + HZ/40 );
    }

   // enable_irq(gIrqnumber[MSR_RCP1_IQR]);
   // enable_irq(gIrqnumber[MSR_RCP2_IQR]);
    DPRINTK("%s %s Open Success\n", DEVICE_NAME, RELEASE_VERSION);
    return 0;
}

static int msr_release(struct inode *inode, struct file *filefp)
{
        DPRINTK("msr_release!!\n\r");
	module_put(THIS_MODULE);

	if ( !ulDeviceOpen )
	{
		printk(KERN_ALERT "%s : Device has not opened\n", DEVICE_NAME);
		return -EINVAL;
	}
	ulDeviceOpen = 0;

	del_timer( &swiped_timer );

    //disable_irq(gIrqnumber[MSR_RCP1_IQR]);
    //disable_irq(gIrqnumber[MSR_RCP2_IQR]);
    DPRINTK("\n%s %s Release Success\n", DEVICE_NAME, RELEASE_VERSION);
    return 0;
}

static u8 CalcLRC(u8 *pBuf,int nLen)
{
	int i;
	char lrc;

	for(lrc=pBuf[0],i=1;i<nLen;i++)
		lrc ^= pBuf[i];

	return lrc;

}/* CalcLRC */

static int BuildPacket(u8 *pBuf)
{
	int len;
	int nCnt = 0;

	pBuf[nCnt++] = 0x02; // STX
	pBuf[nCnt++] = Msr_Data[TRACK2].status;
	pBuf[nCnt++] = Msr_Data[TRACK3].status;
    DPRINTK("Msr_Data[TRACK2].status = 0x%02X, Msr_Data[TRACK3].status = 0x%02X\n",
                Msr_Data[TRACK2].status, Msr_Data[TRACK3].status);

	memcpy(&pBuf[nCnt], &Msr_Data[TRACK2].byte, TRK2_DATA_SIZE);
	nCnt += TRK2_DATA_SIZE;
	pBuf[nCnt++] = Msr_Data[TRACK2].bytelen;

    if ( Msr_Data[TRACK3].status == TRACK_STATUS_TRACK3_OK )
    {
        memcpy(&pBuf[nCnt], &Msr_Data[TRACK3].byte, TRK3_DATA_SIZE);
        nCnt += TRK3_DATA_SIZE;
        pBuf[nCnt++] = Msr_Data[TRACK3].bytelen;
    }

	// everything is fail then assume invalid card data

	pBuf[nCnt++] = 0x03; // ETX
	len = nCnt-1;
	pBuf[nCnt++] = CalcLRC((u8 *)(pBuf+1),len);

	return nCnt;

}/* BuildPacket */


static ssize_t msr_read(struct file *file, char __user *buf, size_t count, loff_t *p_pos)
{
    int ch;
    int ilen23 = 0, result = 0;

    ch = (file->f_flags) & O_NONBLOCK;
    
    if ( !CPD2_CHK || !CPD3_CHK ) //return 0;
    {
        hub_wait_queue_ctx.evt_wait_cond = false;   
        hub_wait_queue_ctx.evt_thread_stt = 1;
        wait_event_interruptible(wait_queue, _hub_work_evt_check_cond());
    }
    
    if ( CPD2_CHK || CPD3_CHK )
    {
        if ( CPD2_CHK )
        {
            if ( Msr_Data[TRACK2].status == TRACK_STATUS_TRACK2_OK )
                ilen23 = BuildPacket(pPacket);
#ifdef DEBUG
            {
                int i;
                printk("Track2 Packet(%d) = \n", ilen23);
                for (i = 0; i < ilen23; i++ )
                {
                    printk("0x%02X ", pPacket[i]);
                    if ( i && !((i+1) % 16) ) printk("\n");
                }
                printk("\n");
                for (i = 3 ;i < ilen23; i++ )
                {
                    printk("%c ", pPacket[i]);
                    if ( i && !((i+1) % 16) ) printk("\n");
                }
                printk("\n");
            }

#endif
        }

        if ( CPD3_CHK )
        {
            if ( Msr_Data[TRACK3].status == TRACK_STATUS_TRACK3_OK )
                ilen23 = BuildPacket(pPacket);
#ifdef DEBUG
            {
                int i;
                printk("Track23 Packet(%d) = \n", ilen23);
                for (i = 0; i < ilen23; i++ )
                {
                    printk("0x%02X ", pPacket[i]);
                    if ( i && !((i+1) % 16) ) printk("\n");
                }
                printk("\n");
                for (i = 3; i < ilen23; i++ )
                {
                    printk("%c ", pPacket[i]);
                    if ( i && !((i+1) % 16) ) printk("\n");
                }
                printk("\n");
            }

#endif
        }

        if ( ilen23 != 0 )
        {
            if ( (result = copy_to_user(buf, pPacket, ilen23)) )
                goto err_read;
        }
    }

    DPRINTK("MSR Read Ok(%d)\n", result );
    initVal();
    return ilen23;

err_read:
    printk(KERN_ALERT "MSR Read Error!!!!(%d)\n", result );
    result = 0;

    return result;
}


static long msr_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
//    int data;
//    int err = -EFAULT;
    int ret = 0;


    // 해당 명령이 led 에 속하는 명령인지 체크
    if (_IOC_TYPE(cmd) != MSR_IOCTL_MAGIC) return -ENOTTY;
    // 구분 번호가 설정한 구분 번호에 속하는지 체크
    //if (_IOC_NR(cmd) >= MSR_IOCTL_MAGIC) return -ENOTTY;


    switch (cmd)
    {
    	case MSR_DFLUSH:
    		gidDataIsReady = 0;
    		memset( &Msr_Data, 0, sizeof(Msr_Data) );
    		memset( PacketData, 0, sizeof(PacketData) );
    		initVal();
    		break;

    	case MSR_DATARDY:
    		ret = Msr_Data[TRACK2].status | Msr_Data[TRACK3].status;
    		break;
        case MSR_OFF:
			gi_MsrOnOff = M_OFF;
			/* CPD Clk interrupt disable */
			//disable_irq(IRQ_CPD);
			break;
		case MSR_ONOFF:
			// interrupt on/off
			gi_MsrOnOff = (int)arg;

			if (gi_MsrOnOff == M_ON) 
            {
				/* CPD Clk interrupt enable */
				//enable_irq(IRQ_CPD);
			}
			else 
            {
				/* CPD Clk interrupt disable */
				//disable_irq(IRQ_CPD);
			}

			DPRINTK(" %s: In gi_MsrOnOff[%d]\n",__FUNCTION__, gi_MsrOnOff);
			break;
    	case MSR_VERSION:
            if(copy_to_user((unsigned char *)arg, RELEASE_VERSION, 3) )
            {
               return -EFAULT;
            }
            break;

        default:
             DPRINTK("MSR ioctl : unknown command %02x(module)\n", cmd);
             return -EINVAL;
    }

    return ret;

}


static struct file_operations msr_fops = {
    .owner          = THIS_MODULE,
    .open           = msr_open,
    .release        = msr_release,
    .unlocked_ioctl = msr_ioctl,
    .read           = msr_read,
    .poll			= msr_poll,
};

static struct miscdevice msr_dev = {
    MISC_DYNAMIC_MINOR,
    DEVICE_NAME,
    &msr_fops
};





static int  shc350_msr_init(void)
{
    int ret;
    int err = 0;

    spin_lock(&msr_lock);
    msr_init_set();

    spin_unlock(&msr_lock);

    spin_lock_init(&hub_wait_queue_ctx.evt_thread_lock);
    
#if 0 
    gIrqnumber[0] = gpio_to_irq(MSR_CPD);	
	printk("gpio irq %d\n",gIrqnumber[0]);
	if( request_irq (gIrqnumber[0],(irq_handler_t)CPD_IrqHandler,IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,"msr_cpd",NULL))
    {
    	printk(KERN_ALERT "Can't register IRQ %d\n", MSR_CPD );
    	err = -EIO;
    	goto failed;

    }

#endif

    gIrqnumber[1] = gpio_to_irq(MSR_RCP1);	
	printk("gpio irq %d\n",gIrqnumber[1]);
    if ( request_irq(gIrqnumber[1], (irq_handler_t)RCP1_IrqHandler, IRQF_TRIGGER_FALLING,"msr_rcp1", NULL) )
    {
    	printk(KERN_ALERT "Can't register IRQ %d\n", MSR_RCP1 );
		err = -EIO;
		goto failed;
    }
    gIrqnumber[2] = gpio_to_irq(MSR_RCP2);	
	printk("gpio irq %d\n",gIrqnumber[2]);
    if ( request_irq(gIrqnumber[2], (irq_handler_t)RCP2_IrqHandler, IRQF_TRIGGER_FALLING,"msr_rcp2", NULL) )
	{
		printk(KERN_ALERT "Can't register IRQ %d\n", MSR_RCP2 );
		err = -EIO;
		goto failed;
	}
    printk(KERN_ALERT "\n%s %s %s Load Success - %s\n", MODEL_NAME, DEVICE_NAME, RELEASE_VERSION, __TIMESTAMP__);

    if ( (ret = misc_register(&msr_dev)) )
	{
		printk(KERN_ALERT "msr driver : Unable to register driver\n");
		err = -EINVAL;
		goto failed;
	}

    irq_set_irq_type( gIrqnumber[MSR_CPD_IQR], IRQ_TYPE_EDGE_FALLING);
    
    //enable_irq(gIrqnumber[MSR_RCP1_IQR]);
    //disable_irq(gIrqnumber[MSR_RCP1_IQR]);
    //disable_irq(gIrqnumber[MSR_RCP1_IQR]);


    memset( &Msr_Data, 0, sizeof(Msr_Data) );
    pPacket = PacketData;

    return ret;

failed:
    if(gIrqnumber[0])    
    	free_irq(gIrqnumber[0], NULL);
    if(gIrqnumber[1])    
	    free_irq(gIrqnumber[1], NULL);
    if(gIrqnumber[2])    
    	free_irq(gIrqnumber[2], NULL);

    gIrqnumber[0] = gIrqnumber[1]=gIrqnumber[2] = 0;
    
    misc_deregister(&msr_dev);

    return err;
}


static void  shc350_msr_exit(void)
{
	if(gIrqnumber[0])    
    	free_irq(gIrqnumber[0], NULL);
    if(gIrqnumber[1])    
	    free_irq(gIrqnumber[1], NULL);
    if(gIrqnumber[2])    
    	free_irq(gIrqnumber[2], NULL);

    gIrqnumber[0] = gIrqnumber[1]=gIrqnumber[2] = 0;

    gpio_free(MSR_CPD);
    gpio_free(MSR_RCP1);
    gpio_free(MSR_RCP2);
    gpio_free(MSR_RDD1);
    gpio_free(MSR_RDD2);


    misc_deregister(&msr_dev);

    printk(KERN_ALERT "\n%s %s %s Exit Success\n", MODEL_NAME, DEVICE_NAME, RELEASE_VERSION );
}

module_init(shc350_msr_init);
module_exit(shc350_msr_exit);

MODULE_AUTHOR("SHC");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("SHC-500 MSR driver");
