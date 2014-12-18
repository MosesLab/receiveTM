/*
 * MOSES telemetry ground station test code
 * 
 * Author: Roy Smart
 * History:
 *      Created Apr 28 2014
 *      Tested May 9 2014 successfully at White Sands Missile Range
 * 
 * Uses the Microgate USB Synclink adapter to receive 10 Mbps telemetry data from
 * TS-7600 flight computer and sendTM.c written by Jake Plovanic.
 * 
 * This program requires that ./mgslutil be run beforehand to configure the Synclink device.
 * 10 Mbps is only possible in rs422 mode.
 * 
 *      ./mgslutil rs422
 * 
 * receive HDLC/SDLC data and write received data to a file
 * 
 * Code is based off of receive-hdlc.c sample code provided by Microgate
 *
 * This sample demonstrates HDLC communications using a
 * SyncLink serial card. The companion sample send-hdlc.c sends
 * HDLC data. Use both programs to send data between two serial cards
 * connected with a NULL modem (cross over cable) or other serial link.
 *
 * The sample is simple to clearly show basic programming concepts.
 * Use this code to start development of more complex applications.
 *
 * Overview:
 *
 * 1. open serial device (syscall open)
 * 2. configure serial device (syscall ioctl)
 * 3. receive data from serial device (syscall read)
 * 4. write received data to a file
 *
 */

#include <stdio.h>
#include <memory.h>
#include <signal.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <termios.h>
#include <errno.h>
#include <sys/time.h>

#include "synclink.h"

#ifndef N_HDLC
#define N_HDLC 13
#endif

/*
 * set base clock frequency in cycles per second
 *
 * Data clocks are generated by dividing a fixed base clock by a 16-bit integer.
 * GT family base clock default is 14.7456MHz.
 *
 * Other base clocks (up to 33MHz) can be installed
 * at the factory by special order. Non default values require driver
 * configuration with the actual value so correct divisors are used for
 * a specified data rate.
 */
int set_base_clock(int fd, unsigned int freq) {
    MGSL_PARAMS params;
    int rc;

    /* fields other than mode and clock_speed are ignored */
    params.mode = MGSL_MODE_BASE_CLOCK;
    params.clock_speed = freq;
    rc = ioctl(fd, MGSL_IOCSPARAMS, &params);
    if (rc < 0) {
        printf("set base clock frequency error=%d %s\n",
                errno, strerror(errno));
    }
    return rc;
}

int * openFile(char* name) {
    
    FILE *fp = NULL;
    fp = fopen(name, "wb");
    if (fp == NULL) {
        printf("fopen error=%d %s\n", errno, strerror(errno));
        //return errno;
    }
}

/* handle SIGINT - do nothing */
void sigint_handler(int sigid) {
}

int main(int argc, char* argv[]) {
    int fd, rc;
    int ldisc = N_HDLC;
    MGSL_PARAMS params;
    FILE *fp = NULL;
    int sigs, idle;
    char * writeFiles[] = {"image0", "xml0", "image1", "xml1", "image2", "xml2", "image3", "xml3","image4","xml4","image5","xml5","image6","xml6"};

    //int numImages = sizeof(writeFiles) / sizeof(char*);
    int numImages = 14;
    int fileCount = 0;
    int count = 0;
    int totalFileSize = 0;
    unsigned char buf[4];
    int size = 4194300;
    char *devname;

    struct timeval runtime_begin, runtime_end;
    int runtime_elapsed;

    struct mgsl_icount icount;
    
    if (argc > 1)
        devname = argv[1];
    else
        devname = "/dev/ttyUSB0";

    printf("receive HDLC data on %s\n", devname);
    printf("receiving/writing %d files\n", numImages);


    /* open serial device with O_NONBLOCK to ignore DCD input */
    fd = open(devname, O_RDWR | O_NONBLOCK, 0);
    

    gettimeofday(&runtime_begin, NULL); //Timing

    if (fd < 0) {
        printf("open error=%d %s\n", errno, strerror(errno));
        return errno;
    }
    else printf("%s port opened\n", devname);

    /*
     * set N_HDLC line discipline
     *
     * A line discipline is a software layer between a tty device driver
     * and user application that performs intermediate processing,
     * formatting, and buffering of data.
     */
    rc = ioctl(fd, TIOCSETD, &ldisc);                         //Change to N_TTY?
    if (rc < 0) {
        printf("set line discipline error=%d %s\n",
                errno, strerror(errno));
        return rc;
    }
    
    /*log debugging*/
    

    /* required only if custom base clock (not 14745600) installed */
    	if (set_base_clock(fd, 32000000) < 0) {
    		return rc;
        }

    // get current device parameters
    rc = ioctl(fd, MGSL_IOCGPARAMS, &params);
    if (rc < 0) {
        printf("ioctl(MGSL_IOCGPARAMS) error=%d %s\n",
                errno, strerror(errno));
        return rc;
    }


    /*
     * modify device parameters
     *
     * HDLC/SDLC mode, loopback disabled, NRZ encoding
     * Data clocks sourced from clock input pins
     * Output 9600bps clock on auxclk output
     * Hardware generation/checking of CCITT (ITU) CRC 16
     */

    params.mode = MGSL_MODE_HDLC;                             //Change to N_TTY?
    params.loopback = 0;
    params.flags = HDLC_FLAG_RXC_RXCPIN + HDLC_FLAG_TXC_TXCPIN;
    params.encoding = HDLC_ENCODING_NRZ;
    params.clock_speed = 0;
    params.crc_type = HDLC_CRC_16_CCITT;
    params.preamble = HDLC_PREAMBLE_PATTERN_ONES;               //Remove?
    params.preamble_length = HDLC_PREAMBLE_LENGTH_16BITS;

    /* set current device parameters */
    rc = ioctl(fd, MGSL_IOCSPARAMS, &params);
    if (rc < 0) {
        printf("ioctl(MGSL_IOCSPARAMS) error=%d %s\n",
                errno, strerror(errno));
        return rc;
    }


    printf("Turn on RTS and DTR serial outputs\n");
    sigs = TIOCM_RTS | TIOCM_DTR;
    rc = ioctl(fd, TIOCMBIC, &sigs);
    if (rc < 0) {
        printf("assert DTR/RTS error=%d %s\n", errno, strerror(errno));
        return rc;
    }

    /* set device to blocking mode for reads and writes */
    fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) + ~O_NONBLOCK);

    /* set ctrl-C to interrupt syscall but not exit program */
    printf("Press Ctrl-C to stop program.\n");
    signal(SIGINT, sigint_handler);
    siginterrupt(SIGINT, 1);

    /*enable receiver*/
    int enable = 2;                                             //Change to 1?
    rc = ioctl(fd, MGSL_IOCRXENABLE, enable);

    fp = fopen(writeFiles[fileCount], "wb");
    if (fp == NULL) {
		printf("fopen error=%d %s\n", errno, strerror(errno));
		return errno;
    }
    fileCount++;
    
    /*crc check setup*/
    rc = ioctl(fd, MGSL_IOCGSTATS, &icount);
    __u32 crctemp = icount.rxcrc;

    int index = 0;      //line counter to verify data is still being sent

    for (;;) {
        
        //printf("print marker 1\n");
        /*check crc*/
/*
        rc = ioctl(fd, MGSL_IOCGSTATS, &icount);                //needed?
        if(crctemp != icount.rxcrc){
            printf("    CRC Failed!\n");
        }
        crctemp = icount.rxcrc;
*/
        /* get received data from serial device */
        rc = read(fd, buf, 4194300);
        
        if (rc < 0) {
            printf("read error=%d %s\n", errno, strerror(errno));
            break;
        }

        else if (rc == 0) {
            gettimeofday(&runtime_end, NULL); //Timing
	    runtime_elapsed = 1000000 * ((long) (runtime_end.tv_sec) - (long) (runtime_begin.tv_sec)) + (long) (runtime_end.tv_usec) - (long) (runtime_begin.tv_usec);
	    printf("program ran for %-3.2f seconds before failing\n", (float) runtime_elapsed / (float) 1000000);
	    printf("read returned with no data\n");
            break;
        }
        
        else if (rc == 5) {
            /*check if we need to start new file*/
            printf("received %d bytes       %d\n", rc, index);
	    
	    if (fileCount == numImages){
                 /*all done*/
                 printf("Finished %d files \n",fileCount);
                 gettimeofday(&runtime_end, NULL); //Timing
                 runtime_elapsed = 1000000 * ((long) (runtime_end.tv_sec) - (long) (runtime_begin.tv_sec)) + (long) (runtime_end.tv_usec) - (long) (runtime_begin.tv_usec);
                 printf("program ran for %-3.2f seconds total\n", (float) runtime_elapsed / (float) 1000000);
                  
                 fileCount = 0;
            }
            else {
            printf("creating new file %s\n", writeFiles[fileCount]);
            fclose(fp);
            fp = fopen(writeFiles[fileCount], "wb");
            fileCount++;
            index = 0;
            }
        }
        
        else {
            /* save received data to file */
            printf("received %d bytes       %d\n", rc, index);
	    count = fwrite(buf, sizeof(char), rc, fp);
            if (count != rc) {
                printf("fwrite error=%d %s\n", errno, strerror(errno));
                break;
            }
            totalFileSize += count;
            printf("wrote    %d total bytes\n", totalFileSize);
            index ++;
        }
        fflush(fp);
        //usleep(20);
        //printf("print marker 4\n");
    }



    printf("Turn off RTS and DTR serial outputs\n");
    sigs = TIOCM_RTS + TIOCM_DTR;
    rc = ioctl(fd, TIOCMBIC, &sigs);
    if (rc < 0) {
        printf("negate DTR/RTS error=%d %s\n", errno, strerror(errno));
        return rc;
    }

    close(fd);
    fclose(fp);

    return 0;
}
