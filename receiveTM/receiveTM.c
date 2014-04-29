/*
 * receive HDLC/SDLC data and write received data to a file
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
 * For more information about SyncLink specific programming refer to
 * the Programming.txt file included with the SyncLink software package.
 *
 * Microgate and SyncLink are registered trademarks
 * of Microgate corporation.
 *
 * This code is released under the GNU General Public License (GPL)
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
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
int set_base_clock(int fd, unsigned int freq)
{
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

/* handle SIGINT - do nothing */
void sigint_handler(int sigid){}

int main(int argc, char* argv[])
{
	int fd;
	int rc;
	int ldisc = N_HDLC;
	MGSL_PARAMS params;
	int sigs, idle;
	FILE *fp = NULL;
	int size = 1024;
	int count;
	unsigned char buf[1024];
	char *devname;

	if (argc > 1)
		devname = argv[1];
	else
		devname = "/dev/ttySLG0";

	printf("receive HDLC data on %s\n", devname);

	/* open file to save received data */
	fp = fopen("data", "wb");
	if (fp == NULL) {
		printf("fopen error=%d %s\n", errno, strerror(errno));
		return errno;
	}

	/* open serial device with O_NONBLOCK to ignore DCD input */
	fd = open(devname, O_RDWR | O_NONBLOCK, 0);
	if (fd < 0) {
		printf("open error=%d %s\n", errno, strerror(errno));
		return errno;
	}

	/*
	 * set N_HDLC line discipline
	 *
	 * A line discipline is a software layer between a tty device driver
	 * and user application that performs intermediate processing,
	 * formatting, and buffering of data.
	 */
	rc = ioctl(fd, TIOCSETD, &ldisc);
	if(rc < 0) {
		printf("set line discipline error=%d %s\n",
		       errno, strerror(errno));
		return rc;
	}

	/* required only if custom base clock (not 14745600) installed */
//	if (set_base_clock(fd, 32000000) < 0)
//		return rc;

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

	params.mode = MGSL_MODE_HDLC;
	params.loopback = 0;
	params.flags = HDLC_FLAG_RXC_RXCPIN + HDLC_FLAG_TXC_TXCPIN;
	params.encoding = HDLC_ENCODING_NRZI_SPACE;
	params.clock_speed = HDLC_FLAG_TXC_BRG;
	params.crc_type = HDLC_CRC_NONE;

	/* set current device parameters */
	rc = ioctl(fd, MGSL_IOCSPARAMS, &params);
	if (rc < 0) {
		printf("ioctl(MGSL_IOCSPARAMS) error=%d %s\n",
		       errno, strerror(errno));
		return rc;
	}

	
        
	printf("Turn on RTS and DTR serial outputs\n");
	sigs = TIOCM_RTS + TIOCM_DTR;
	rc = ioctl(fd, TIOCMBIC, &sigs);
	if(rc < 0) {
		printf("assert DTR/RTS error=%d %s\n", errno, strerror(errno));
		return rc;
	}

	/* set device to blocking mode for reads and writes */
	fcntl(fd, F_SETFL, fcntl(fd,F_GETFL) & ~O_NONBLOCK);

	/* set ctrl-C to interrupt syscall but not exit program */
	printf("Press Ctrl-C to stop program.\n");
	signal(SIGINT, sigint_handler);
	siginterrupt(SIGINT, 1);
        
        /*enable receiver*/
        int enable = 1;
        rc = ioctl(fd, MGSL_IOCRXENABLE, enable);
        
	for (;;) {

		/* get received data from serial device */
		rc = read(fd, buf, size);
		if (rc < 0) {
			printf("read error=%d %s\n", errno, strerror(errno));
			break;
		}
		if (rc == 0) {
			printf("read returned with no data\n");
			continue;
		}
		printf("received %d bytes\n", rc);

		/* save received data to file */
		count = fwrite(buf, sizeof(char), rc, fp);
		if (count != rc) {
			printf("fwrite error=%d %s\n", errno, strerror(errno));
			break;
		}
		fflush(fp);
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