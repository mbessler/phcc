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

#include "common_serial.h"

int fd_phccdev=-1;

void waitms(int ms)
{
	struct timeval tv;
	tv.tv_sec=0;
	tv.tv_usec=ms*1000;
	select(0, NULL, NULL, NULL, &tv);
}


int serialRead(unsigned char * _byte)
{
	int size = read(fd_phccdev, _byte, 1); // returns 0 if fcntl(serialFD, F_SETFL, FNDELAY) set
	return size;
}
   
void serialWrite(unsigned char _byte)
{
	unsigned char c[1];
	c[0] = _byte;
	write(fd_phccdev, c, 1);
	printf("wrote to port: 0x%02x\n", c[0]);
	waitms(25);
	return;
}


void setline(int flags, int speed)
{
	struct termios t;

	tcgetattr(fd_phccdev, &t);

	t.c_cflag = flags | CREAD | HUPCL | CLOCAL | CRTSCTS;
	t.c_iflag = IGNBRK | IGNPAR;
	t.c_oflag = 0;
//	t.c_cc[VMIN ] = 1;
//	t.c_cc[VTIME] = 0;
//	t.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  /* disable echo, choose raw input */
	
	cfsetispeed(&t, speed);
	cfsetospeed(&t, speed);

	tcsetattr(fd_phccdev, TCSAFLUSH/*TCSANOW*/, &t);
}


int baudrate_check(int speed)
{
	switch(speed)
	{
		case 230400: return B230400;
		case 115200: return B115200;
		case 57600: return B57600;
		case 38400: return B38400;
		case 19200: return B19200;
		case 9600: return B9600;
		case 4800: return B4800;
		case 2400: return B2400;
		case 1800: return B1800;
		case 1200: return B1200;
		case 600: return B600;
		case 300: return B300;
		case 200: return B200;
		case 150: return B150;
		case 134: return B134;
		case 110: return B110;
		case 75: return B75;
		case 50: return B50;
		default: return -1;
	}
}


int opendevice(char * dev, int speed)
{
	int flags=CS8;
	int baudrate = -1;
	if( (fd_phccdev = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK))  < 0 )
	{
		printf("ERROR, could not open device %s\n", dev);
		return 0;
	}
	if( (baudrate = baudrate_check(speed)) == -1 )
	{
		printf("ERROR, invalid speed: %i\n", speed);
		return 0;
	}
	tcflush(fd_phccdev, TCIOFLUSH);
	setline(flags, baudrate);
	fcntl(fd_phccdev, F_SETFL, 0);
	return 1;
}

void closedevice()
{
	close(fd_phccdev);
	fd_phccdev = -1;
}
