/*
 * DATE		: 2014/08/13
 * Author	: shellbt
 */
#ifndef __MSR_H__
#define __MSR_H__


#define MAX_BITSTREAM_LEN						1428
#define MAX_BYTESTREAM_LEN						256

#define TRACK2									0
#define TRACK3									1

#define TRACK_STATUS_NODATA                     0x00       // No data
#define TRACK_STATUS_TRACK1_OK                  0x01
#define TRACK_STATUS_TRACK2_OK                  0x02       // OK
#define TRACK_STATUS_TRACK3_OK                  0x04
#define TRACK_STATUS_DATA_ERR                   0x08       // Analyze Error
#define TRACK_STATUS_SS_ERR                     0x10       // Start Sentinel
#define TRACK_STATUS_ES_ERR                     0x20       // End Sendtinel
#define TRACK_STATUS_LRC_ERR                    0x40       // Checksum
#define TRACK_STATUS_PARITY_ERR                 0x80       // Parity

#define MSR_REVERSE_DIR     					0
#define MSR_FORWARD_DIR     					1


struct Msr_Data {
    volatile unsigned int enable;
    volatile unsigned int bitlen;
    char                  bit[MAX_BITSTREAM_LEN];
    volatile unsigned int direction;

    unsigned int          bytelen;
    char                  byte[MAX_BYTESTREAM_LEN];
    volatile unsigned int valid;
    volatile int          status;
};


#define	MSR_IOCTL_MAGIC					'm'

//#define MSR_DATARDY						_IO  (MSR_IOCTL_MAGIC,  0)
//#define MSR_DFLUSH						_IO  (MSR_IOCTL_MAGIC,  1)
//#define MSR_VERSION						_IOR (MSR_IOCTL_MAGIC, 99, unsigned char *)


#define MSR_DATARDY         _IO (MSR_IOCTL_MAGIC, 0)
#define MSR_DFLUSH          _IO (MSR_IOCTL_MAGIC, 1)
#define MSR_OFF             _IO (MSR_IOCTL_MAGIC, 2)
#define MSR_ONOFF           _IOW(MSR_IOCTL_MAGIC, 3, unsigned int)
#define MSR_VERSION         _IOR(MSR_IOCTL_MAGIC,99, unsigned char *)

#endif	//__MSR_H__

