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


#define SEGA 0x01
#define SEGB 0x02
#define SEGC 0x04
#define SEGD 0x10
#define SEGE 0x20
#define SEGF 0x40
#define SEGG 0x80
#define SEGDP 0x08


char charto7seg(char decimal)
{
	switch(decimal)
	{
		case '0': return (SEGA|SEGB|SEGC|SEGD|SEGE|SEGF);
		case '1': return (SEGB|SEGC);
		case '2': return (SEGA|SEGB|SEGG|SEGE|SEGD);
		case '3': return (SEGA|SEGB|SEGC|SEGD|SEGG);
		case '4': return (SEGF|SEGG|SEGB|SEGC);
		case '5': return (SEGA|SEGF|SEGG|SEGC|SEGD);
		case '6': return (SEGF|SEGE|SEGD|SEGC|SEGG);
		case '7': return (SEGA|SEGB|SEGC);
		case '8': return (SEGA|SEGB|SEGC|SEGD|SEGE|SEGF|SEGG);
		case '9': return (SEGA|SEGB|SEGC|SEGF|SEGG);
		case '.': return (SEGDP);
		case '-': return (SEGG);
		case '+': return (SEGG|SEGF|SEGE);
		case 'A':
		case 'a': return (SEGA|SEGB|SEGC|SEGE|SEGF|SEGG);
		case 'C':
		case 'c': return (SEGA|SEGD|SEGE|SEGF);
		case 'E':
		case 'e': return (SEGA|SEGD|SEGE|SEGF|SEGG);
		case 'F':
		case 'f': return (SEGA|SEGE|SEGF|SEGG);
		case 'G':
		case 'g': return (SEGA|SEGC|SEGD|SEGE|SEGF);
		case 'H':
		case 'h': return (SEGB|SEGC|SEGE|SEGF|SEGG);
		case 'J':
		case 'j': return (SEGB|SEGC|SEGD|SEGE);
		case 'L':
		case 'l': return (SEGD|SEGE|SEGF);
		case 'N':
		case 'n': return (SEGC|SEGE|SEGG);
		case 'O':
		case 'o': return (SEGC|SEGD|SEGE|SEGG);
		case 'P':
		case 'p': return (SEGA|SEGB|SEGE|SEGF|SEGG);
		case 'R':
		case 'r': return (SEGE|SEGG);
		case 'T':
		case 't': return (SEGD|SEGE|SEGF|SEGG);
		case 'U':
		case 'u': return (SEGB|SEGC|SEGD|SEGE|SEGF);
		case 'Y':
		case 'y': return (SEGB|SEGE|SEGF|SEGG);
		case ' ':
		default:
			return 0x00;
	}
}

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
	
        if( argc != 3 || sscanf(argv[1], "0x%x", &subaddr)==0 || sscanf(argv[2], "%c", &data)==0)
	{
	 	printf("Error, data to send to DO_A out of range\n");
		printf("Usage %s <subaddr> <data>", argv[0]);
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
	b = charto7seg(data);
	printf("sending %i DO_A ", b);
	serialWrite(fd, b);
	
	close(fd);

	return 0;
}
