/*
 * compile w/
 *    gcc -g -O2 -Wall test.c  -o test
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or 
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 * 
 */

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/time.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <assert.h>
#include <ctype.h>


void waitms(int ms)
{
	struct timeval tv;
	tv.tv_sec=0;
	tv.tv_usec=ms*1000;
	select(0, NULL, NULL, NULL, &tv);
}


int serialRead(int fd, unsigned char * _byte)
{
    int size = read(fd, _byte, 1); // returns 0 if fcntl(serialFD, F_SETFL, FNDELAY) set
    return size;
}
   
void serialWrite(int fd, unsigned char _byte)
{
  unsigned char c[1];
  c[0] = _byte;
  write(fd, c, 1);
  printf("wrote to port: 0x%02x\n", c[0]);
 waitms(25);
  return;
}


void setline(int fd, int flags, int speed)
{
	struct termios t;

	tcgetattr(fd, &t);

	t.c_cflag = flags | CREAD | HUPCL | CLOCAL | CRTSCTS;
	t.c_iflag = IGNBRK | IGNPAR;
	t.c_oflag = 0;
//	t.c_cc[VMIN ] = 1;
//	t.c_cc[VTIME] = 0;
//	t.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  /* disable echo, choose raw input */
	
	cfsetispeed(&t, speed);
	cfsetospeed(&t, speed);

	tcsetattr(fd, TCSAFLUSH/*TCSANOW*/, &t);
}

int main(int argc, char **argv)
{
        int fd, i;
	unsigned char b, data, subaddr, devaddr;
	int speed=B19200;
	int flags=CS8;
	int rows = 0;
	long int startaddr = 0;

	if( (fd = open("/dev/ttyS0", O_RDWR | O_NOCTTY | O_NONBLOCK))  < 0 )
	{
		printf("err\n");
		perror("test");
		exit(1);
	}
	tcflush(fd, TCIOFLUSH);
	setline(fd, flags, speed);
	fcntl(fd, F_SETFL, 0);
	
        if( argc != 3 || sscanf(argv[1], "%i", &subaddr)==0 || sscanf(argv[2], "%i", &data)==0 )
	{
		printf("Usage %s <servonumber> <pos>\n", argv[0]);
		printf("where\tservonumber\t0..7\n");
		printf("\tpos\t0..255\n");
		exit(1);
	}
//	if( /*subaddr < 0 ||*/ subaddr > 7 )
//	{
//		printf("servonumber '%i' out of range!\n", subaddr);
//		exit(1);
//	}
	if( /*data < 0 ||*/ data > 255 )
	{
		printf("servopos '%i' out of range!\n", data);
		exit(1);
	}

	devaddr = 0x00;
	printf("sending 0x%X to display 0x%X on device 0x%X\n", data, subaddr, devaddr);
	b = 0x07;   // DO_A send
	serialWrite(fd, b);
	b = devaddr;  // DO_A dev address (zero for now)
	serialWrite(fd, b);
	b = subaddr;  // DO_A sub address (6bit)
	serialWrite(fd, b);
	b = data;
	printf("sending %i DO_A ", b);
	serialWrite(fd, b);
	
	close(fd);

	return 0;
}
