/*******************************************************************************
 *
 *               MOSES telemetry ground station code
 *                     Montana State University
 *
 *
 * Filename      : receiveTM.c
 * Header(s)     : synclink.h
 * Description   : Uses the Microgate USB Synclink adapter to receive 10 Mbps 
 *                 telemetry data from VDX104 flight computer with MOSES flightSW
 *                 and save locally to ./data_output/
 *                 An index of received data will be stored at ./imageindex.xml
 *                 and archived periodically into sub-folder: /xml_archive/
 * 
 *                 This program requires synclink.h header provided by Microgate.
 *                 This program requires that ./mgslutil be run to configure 
 *                 the Synclink device*: ./mgslutil rs422 [path/to/device] 
 *                     *This is performed by ./synclink_init.sh bash scipt
 *                 
 *                 10 Mbps is only possible in rs422 mode.
 * 
 *                 Code is based off of receive-hdlc.c sample code provided 
 *                 by Microgate:
 *                     1. open serial device (syscall open)
 *                     2. configure serial device (syscall ioctl)
 *                     3. receive data from serial device (syscall read)
 *                     4. write received data to a file
 * Function(s)   : FILE* openFile(char*)    - Opens file streams/handles errors
 *                 void sigint_handler(int) - Does Nothing
 *                 int main(int, char*)     - Contains initialization/read/write
 * Authors(s)    : Jackson Remington, Roy Smart, Jake Plovanic
 * Date          : Updated 03/12/15
 ******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include <signal.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <termios.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/statvfs.h>
#include <time.h>

#include "synclink.h"

#ifndef N_HDLC
#define N_HDLC 13
#endif

#ifndef BUFSIZ
#define BUFSIZ 4096
#endif

/* Handles stream opens and errors*/
FILE * openFile(char* name) {

    FILE *file;
    file = fopen(name, "a+");
    fclose(file);
    file = fopen(name, "r+");
    if (file == NULL) {
        printf("fopen error = %d %s\n", errno, strerror(errno));
    }
    //printf("filepath = %d\n", (int)fileno(file));
    return file;
}

/* handle SIGINT - do nothing */
void sigint_handler(int sigid) {
}

int main(int argc, char* argv[]) {
/*********************************************************************************                           
*                                    VARIABLES
*********************************************************************************/   
    FILE *fp = NULL;
    FILE *outxml = NULL;
    pid_t MTV_child;
    int fd, rc;
    int sigs;
    int xmlCount       = 0;
    int xml_check      = 0;                     //Set this value to '0' if expecting ROE first; '1' if expecting XML first
    int count          = 0;
    int totalFileSize  = 0;
    int index          = 0;
    int ldisc          = N_HDLC;
    char *xml_header   = malloc(strlen("<ROEIMAGE>") + 1);
    char *archive_file = malloc(strlen("/media/moses/Data/TM_data/xml_archive/imageindex_000.xml") + 1);
    char *current_xml  = "/media/moses/Data/TM_data/imageindex.xml";
    char *image_path   = "/media/moses/Data/TM_data/image_buf.tmp";
    char *devname;
    unsigned char buf[BUFSIZ];
    MGSL_PARAMS params;
    size_t size = BUFSIZ;
    struct mgsl_icount icount;
    struct timeval runtime_begin, runtime_end;
    int runtime_elapsed;
    time_t current_time;
    struct tm ts;
    char timestamp[80];
    
    /*disk check debug stuff*/
    unsigned long free_disk;
    struct statvfs * disk_info = malloc(sizeof(statvfs));

    /* Run device with arguments to force device selection */
    if (argc > 1)
        devname = argv[1];
    else
        devname = "/dev/ttyUSB0";

/*********************************************************************************                           
*                              SYNCLINK INITIALIZATION
*********************************************************************************/   
   
    printf("receive HDLC data on device: %s\n", devname);

    /* open serial device with O_NONBLOCK to ignore DCD input */
    fd = open(devname, O_RDWR | O_NONBLOCK, 0);
    if (fd < 0) {
        printf("open error=%d %s\n", errno, strerror(errno));
        return errno;
    } else printf("%s port opened\n", devname);
    
    /* Timing */
    gettimeofday(&runtime_begin, NULL);

    /*
     * set N_HDLC line discipline
     *
     * A line discipline is a software layer between a tty device driver
     * and user application that performs intermediate processing,
     * formatting, and buffering of data.
     */
    rc = ioctl(fd, TIOCSETD, &ldisc);
    if (rc < 0) {
        printf("set line discipline error=%d %s\n",
                errno, strerror(errno));
        return rc;
    }

    /* get current device parameters */
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
    params.mode            = MGSL_MODE_HDLC;
    params.loopback        = 0;
    params.flags           = HDLC_FLAG_RXC_RXCPIN + HDLC_FLAG_TXC_TXCPIN;
    params.encoding        = HDLC_ENCODING_NRZ;
    params.clock_speed     = HDLC_FLAG_TXC_BRG;
    params.crc_type        = HDLC_CRC_16_CCITT;
    params.preamble        = HDLC_PREAMBLE_PATTERN_ONES;
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

    signal(SIGINT, sigint_handler);
    siginterrupt(SIGINT, 1);

    /*enable receiver*/
    int enable = 2;
    rc = ioctl(fd, MGSL_IOCRXENABLE, enable);
    
    /*crc check setup*/
    rc = ioctl(fd, MGSL_IOCGSTATS, &icount);
    __u32 crctemp = icount.rxcrc;
    
    /* Prepare image buffer/file pointer */
    fp = openFile(image_path);
    
    /*check if xml exists & archive*/
    struct stat st;
    if ((int)stat(current_xml, &st) == 0) {
        
        time(&current_time);
        ts = *localtime(&current_time);
        strftime(timestamp, sizeof(timestamp), "%y%m%d%H%M%S", &ts);
        
        sprintf(archive_file, "/media/moses/Data/TM_data/xml_archive/imageindex_%s%s", timestamp, ".xml");
        rename(current_xml, archive_file);
        xmlCount = 1;
        
    }
    
    /* Prepare xml buffer/file pointer */
    outxml = openFile(current_xml);
    /* Write XML declaration/header */
    fprintf(outxml, "<?xml version=\"1.0\" encoding=\"ASCII\" standalone=\"yes\"?>\n");
    fprintf(outxml, "<CATALOG>\n\n");
    fprintf(outxml, "</CATALOG>\n");
    fflush(outxml);
    /*place cursor right before '/catalog' tag*/
    if (fseek(outxml, (-11), SEEK_END) < 0) {
        printf("file seek error=%d %s\n", errno, strerror(errno));
        return;
    }
    fflush(outxml);
    
    /*Fork process to startup MOSES_TV*/
    MTV_child = fork();
    if (MTV_child < 0) {
        /* fork error */
        printf("fork error=%d %s\n", errno, strerror(errno));
        return;
    }
    else if (MTV_child == 0) {          
        system("sudo gnome-terminal -x tcsh /media/moses/Data/MTV_EGSE/start_MOSESTV.tcsh");
    }
    else {
        system("clear");
        printf("**************************************************\n");
        printf("*                    receiveTM                   *\n");
        printf("**************************************************\n\n");
        
        /* set ctrl-C to interrupt syscall but not exit program */
        printf("Press Ctrl-C to stop program.\n");
        printf("Waiting for incoming data.....\n");
    
        /*********************************************************************************                           
        *                              MAIN TELEMETRY LOOP
        *********************************************************************************/   
        for (;;) {
            /* check crc */
            rc = ioctl(fd, MGSL_IOCGSTATS, &icount);
            if (crctemp != icount.rxcrc) {
                printf("    CRC Failed!\n");
            }
            crctemp = icount.rxcrc;

            /* wait for and receive data from serial device */
            memset(buf, 0, BUFSIZ);
            rc = read(fd, buf, size);

            /* Check received packet size for expected values */
            if (rc < 0) {
                /* read error */
                if (errno == 4) {
                    printf("\nreceiveTM interrupted\n");
                    break;
                }
                else {
                    printf("read error=%d %s\n", errno, strerror(errno));
                    break;
                }
            } else if (rc == 0) {
                /* Incorrect synclink settings - set NONBLOCK mode */
                gettimeofday(&runtime_end, NULL);
                runtime_elapsed = 1000000 * ((long) (runtime_end.tv_sec) - (long) (runtime_begin.tv_sec)) + (long) (runtime_end.tv_usec) - (long) (runtime_begin.tv_usec);
                printf("program ran for %-3.2f seconds before failing\n", (float) runtime_elapsed / (float) 1000000);
                printf("read returned with no data - set NONBLOCK mode to continue\n");
                break;
            } else if (rc == 16) {
                /* Terminating characters for image */
                printf("received %d bytes       %d       [ TERM ]\n", rc, index);
                printf("%d total bytes received for file: %s\n", totalFileSize, buf);
                printf("creating new image buffer\n");

                /* Flush the stream, save the image, free up the buffer*/
                fflush(fp);
                fclose(fp);
                sprintf(archive_file, "/media/moses/Data/TM_data/%s", buf);
                rename(image_path, archive_file);

                fp = openFile(image_path);

                xml_check = 1; // next image will be an xml
                totalFileSize = 0;
                index = 0;
            } else if (rc == 14) {
                /* Terminating characters for xml */
                printf("received %d bytes       %d       [ TERM ]\n", rc, index);
                printf("%d total bytes received for updating xml\n", totalFileSize);

                /* Include footer in xml */
                fprintf(outxml, "</CATALOG>\n");

                fflush(outxml);
                fclose(outxml);
                xml_check = 0; //next packet will be an image
                totalFileSize = 0;
                index = 0;
            } else {
                /* data packet */
                if (xml_check == 1) {
                    /* check if the first few characters look like an xml */
                    strncpy(xml_header, buf, (strlen("<ROEIMAGE>")));
                    int cmp = strncmp(xml_header, "<ROEIMAGE>", 10 * sizeof (char));
                    if (cmp == 0) {
                        /* header matches xml format */
                        printf("xml_header = %s\n", xml_header);
                        printf("packet is an xml \n");

                        /*if not the first, archive current xml*/
                        if (xmlCount > 0) {
                            time(&current_time);
                            ts = *localtime(&current_time);
                            strftime(timestamp, sizeof (timestamp), "%y%m%d%H%M%S", &ts);

                            sprintf(archive_file, "/media/moses/Data/TM_data/xml_archive/imageindex_%s%s", timestamp, ".xml");
                            rename(current_xml, archive_file);

                            outxml = openFile(current_xml);

                            /* Write XML declaration/header */
                            fprintf(outxml, "<?xml version=\"1.0\" encoding=\"ASCII\" standalone=\"yes\"?>\n");
                            fprintf(outxml, "<CATALOG>\n\n");
                            fprintf(outxml, "</CATALOG>\n");
                            fflush(outxml);
                            
                            /*place cursor right before '/catalog' tag*/
                            if (fseek(outxml, (-11), SEEK_END) < 0) {
                                printf("file seek error=%d %s\n", errno, strerror(errno));
                                return;
                            }
                            fflush(outxml);
                            
                        }

                        xmlCount = 1;
                        xml_check = 2; //start saving xml data
                    } else xml_check = 0; // mistake: this file is not xml

                }
                if (xml_check == 2) { //start saving xml data
                    printf("received %d bytes       %d       [ XML ]\n", rc, index);
                    
                    /* write new received xml to disk */
                    count = fwrite(buf, sizeof (char), rc, outxml);
                    if (count != rc) {
                        printf("fwrite error=%d %s\n", errno, strerror(errno));
                        break;
                    }
                    if (fflush(fp) != 0) {
                        printf("fflush error=%d %s\n", errno, strerror(errno));
                        return errno;
                    }

                    fprintf(outxml, "\n");
                    totalFileSize += count;
                    index++;
                } else {
                    /* image packet */
                    /* save received data to image file */
                    printf("received %d bytes       %d\n", rc, index);
                    count = fwrite(buf, sizeof (char), rc, fp);
                    if (count != rc) {
                        printf("fwrite error=%d %s\n", errno, strerror(errno));
                        break;
                    }
                    if (fflush(fp) != 0) {
                        printf("fflush error=%d %s\n", errno, strerror(errno));
                        return errno;
                    }

                    totalFileSize += count;
                    index++;
                }
            }
        }
    
        /* Exit protocol; shouldn't reach here until interrupted with Ctrl-C*/
        printf("Turn off RTS and DTR serial outputs\n");
        sigs = TIOCM_RTS + TIOCM_DTR;
        rc = ioctl(fd, TIOCMBIC, &sigs);
        if (rc < 0) {
            printf("negate DTR/RTS error=%d %s\n", errno, strerror(errno));
            return rc;
        }

        close(fd);
        fclose(fp);
    }
    
    /*Child and parent join and return*/
    return 0;
}
