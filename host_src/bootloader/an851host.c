/*
 * compile w/
 *    gcc -c -o ihexfile.o ihexfile.c
 *    gcc -g -O2 -Wall an851host.c  -o an851host ihexfile.o
 */

/*
 *  an851host.c 
 *
 * A linux-based control program for talking to PICs using a bootloader
 * of the AN851 kind.
 *
 * it uses ihexfile.[ch] from picprg which is
 *       Copyright 1994-1998 by Brian C. Lane
 * with a small bug fixed by me.
 *
 * not all commands are implemented right now. it started out as a collection of small programs
 * each doing one of the possible commands from AN851.
 * 
 * Limitation: it needs INHEX16 files. while it doesn't complain if you feed
 * it INHEX8, it'll program junk.
 * 
 * Don't forget to clear the memory area before you reprogram!!!!!
 * happened to me often enough. 
 *
 * example: 
 * ./an851host --eraseflash --rows 32 --startaddr 0x200 && \
 * ./an851host --writeflash --hexfile /pic/testloader6.hex && \
 * ./an851host --reset
 *
 *
 * Copyright (c) 2003,2004 by Manuel Bessler <m.bessler AT gmx.net>
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
#include <getopt.h>

#include "ihexfile.h"

#define STX 0x0F
#define ETX 0x04
#define DLE 0x05
#define CHK 0xff    //placeholder
#define TICKLE 0xFF

int checksum = 0;
//int fd = 0;
int speed=B19200;
int flags=CS8;
char dev[64];
int bytes = -1;
int startaddr = -1;
char hexfile[64];
int rows;

void waitms(int ms)
{
	struct timeval tv;
	tv.tv_sec=0;
	tv.tv_usec=ms*1000;
	select(0, NULL, NULL, NULL, &tv);
}

void flush()
{
	fflush(stdout);   //flush stdout   // how do you flush an int fd ???
}

int serialReadPrint(int fd, unsigned char * _byte)
{
	int size = serialRead(fd, _byte);
	printf("read from port: 0x%02hx\n", _byte[0]);
	return size;
}

int serialRead(int fd, unsigned char * _byte)
{
    int size;
    flush();
    size = read(fd, _byte, 1); // returns 0 if fcntl(serialFD, F_SETFL, FNDELAY) set
    return size;
}

int serialReadEscape(int fd, unsigned char * _byte)
{
	serialRead(fd, _byte);
	if( _byte[0] == DLE )
		serialRead(fd, _byte);
}

int expectSTX(int fd)
{
	unsigned char b;
	serialRead(fd, &b);
	if( b == STX )
	{
		printf("<STX>");
		return 1;
	}
	else
	{
		printf("\n[ERROR, expected STX, got 0x%02x]\n", b);
		close(fd);
		exit(-1);
	}
}

int expectETX(int fd)
{
	unsigned char b;
	serialRead(fd, &b);
	if( b == ETX )
	{
		printf("<ETX>\n");
		return 1;
	}
	else
	{
		printf("\n[ERROR, expected ETX, got 0x%02x]\n", b);
		close(fd);
		exit(-1);
	}
}

unsigned char expectCHKSUM(int fd)
{
	unsigned char b;
	printf("{CHKSUM");
	serialReadEscape(fd, &b);
	printf("<0x%02x>}", b);
}

unsigned char expectPacketType(int fd, unsigned char _byte)
{
	unsigned char b;
	printf("{Packettype ");
	serialReadEscape(fd, &b);
	switch( b )
	{
		case 0x00:
			printf("VERSION}");
			break;
		case 0x01:
			printf("READ Flash}");
			break;
		case 0x02:
			printf("WRITE Flash}");
			break;
		case 0x03:
			printf("ERASE Flash}");
			break;
		case 0x04:
			printf("READ EE}");
			break;
		case 0x05:
			printf("WRITE EE}");
			break;
		case 0x06:
			printf("READ CONFIG}");
			break;
		case 0x07:
			printf("WRITE CONFIG}");
			break;
		default:
			printf("UNKNOWN}");
			break;
	}
	if( b != _byte )
	{
		printf("\n[ERROR, expected PacketType 0x%02x, but got 0x%02x]\n", b, _byte);
		close(fd);
		exit(-1);
	}
}

void serialWrite(int fd, unsigned char _byte)
{
  unsigned char c[1];
  c[0] = _byte;
  write(fd, c, 1);
//  printf("wrote to port: 0x%02x\n", c[0]);
  waitms(10);
  return;
}

void sendSTX(int fd)
{
	printf("<STX>");
	serialWrite(fd, STX);
}

void sendETX(int fd)
{
	serialWrite(fd, ETX);
	printf("<ETX>\n");
}

void sendescape(int fd, unsigned char _byte)
{
	unsigned char c[1]; c[0] = _byte; 
	if(  (_byte ^ STX)==0 || (_byte ^ ETX)==0 || (_byte ^ DLE)==0 )
	{
		serialWrite(fd, DLE);
		printf("<DLE>");
	}
	serialWrite(fd, _byte);
	printf("<0x%02x>", c[0]);
}

void senddata(int fd, unsigned char _byte)
{
	checksum += _byte;
	sendescape(fd, _byte);
}

void sendchecksum(int fd)
{
	printf("{CHKSUM:");
	checksum = (~(checksum & 0x00FF))+1;
	sendescape(fd, checksum);
  	printf("}");
}


void setline(int fd, int flags, int speed)
{
	struct termios t;

	tcgetattr(fd, &t);

	t.c_cflag = flags | CREAD | HUPCL | CLOCAL;
	t.c_iflag = IGNBRK | IGNPAR;
	t.c_oflag = 0;
	t.c_lflag = 0;
	t.c_cc[VMIN ] = 1;
	t.c_cc[VTIME] = 0;

	cfsetispeed(&t, speed);
	cfsetospeed(&t, speed);

	tcsetattr(fd, TCSANOW, &t);
}

void usage(char * argv0)
{
	printf("To read from flash Memory: \n");
	printf("\tusage: %s --readflash --bytes <n> --startaddr <x>\n", argv0);
	printf("\twhere n specifies the number of bytes to read (max 251)\n");
	printf("\tand x the hexadecimal start address for the read with 0x prefix\n");
	printf("\t  eg: %s --readflash --bytes 100 --startaddr 0x01320\n\n", argv0);

	printf("\nTo write to flash Memory: \n");
	printf("\tusage: %s --writeflash --hexfile <filename>\n", argv0);
	printf("\twhere filename specifies the hexfile that contains the code to be burned to memory\n");
	printf("\t  eg: %s --writeflash --hexfile test.hex\n\n", argv0);

	printf("\nTo erase to flash Memory: \n");
	printf("\tusage: %s --eraseflash --rows <n> --startaddr <x>\n", argv0);
	printf("\twhere n specifies the number of rows (64 bytes) to delete\n");
	printf("\tand x the hexadecimal start address for the delete with 0x prefix\n");
	printf("\t  eg: %s --eraseflash --rows 3 --startaddr 0x01320\n\n", argv0);

	printf("\nTo reset PIC via bootloader: \n");
	printf("\tusage: %s --reset\n");

	printf("To read from EEPROM Memory: \n");
	printf("\tusage: %s --readeeprom --bytes <n> --startaddr <x>\n", argv0);
	printf("\twhere n specifies the number of bytes to read\n");
	printf("\tand x the hexadecimal start address for the read with 0x prefix\n");
	printf("\t  eg: %s --readeeprom ---bytes 16 --startaddr 0x80\n\n", argv0);


	
	exit(-1);
}


void main_readeeprom(int fd)
{
	int i;
	int adr = 0;
	unsigned char b;
	int maxsize = 256; // bytes of EEPROM the device has

	if( bytes <= 0 || startaddr < 0 )
	{
		printf("ERROR: you need to specify both --bytes and --startaddr\n");
		exit(-1);
	}
		
	if( startaddr > maxsize-1 )
	{
		printf("[ERROR, address out of range. Device EEPROM address range is 0-%i]\n", maxsize-1);
                close(fd);
		exit(-1);
	}
	if( bytes > maxsize )
	{
		printf("[device has only %i bytes EEPROM, truncating request]\n", maxsize);
		bytes=maxsize-startaddr;
	}
	printf("SENDing: Read EEPROM: ");
	sendSTX(fd); 
	sendSTX(fd);
	
	senddata(fd, 0x04); // type = 4 for read EEPROM data mem
	senddata(fd, bytes); // size
	senddata(fd, startaddr & 0x00FF);  // ADDRL
	senddata(fd, (startaddr >> 8 ) & 0x00FF);  // ADDRH
	senddata(fd, 0x00); // fill for upper
	
	sendchecksum(fd);
	sendETX(fd);  

	printf("RECEIVING: ");
	expectSTX(fd);
	expectSTX(fd);
	expectPacketType(fd, 0x04);
	serialReadEscape(fd, &b);
	bytes = b;
	printf("size = 0x%02hx\n", bytes);
	printf("Data Dump: \n");
	serialReadEscape(fd, &b); adr = b;
	serialReadEscape(fd, &b); adr += (b << 8);
	serialReadEscape(fd, &b); 
	while( adr - startaddr < bytes )
	{
	    serialReadEscape(fd, &b);
	    printf("0x%06lx\t", adr);
	    printf("%02hx\n", b);
//	    membuffer[adr] = b;
	    adr++;
	}
	expectCHKSUM(fd);
	if( expectETX(fd) )
	{
		printf("<ETX>\n");
		printf("[OK => Read EEPROM Data Memory]\n");
	}
	else
	{
		printf("[ERROR: transmission incomplete]\n");
		close(fd);
		exit(-1);
	}
}

void writeflash(int fd, unsigned int * membuffer, int startaddress, int words)
{
	int i;
	unsigned char b;
	if( words < 4 )
	{
		printf("[less than eight bytes for flash op, filling up with 0xFF]\n");
		for(i=words; i< 4; i++ )
			membuffer[i] = 0xFFFF;
		words=4;
	}
	if( words%4 != 0 )
	{
		printf("[not multiple of 8, filling memory with 0xFF]\n");
		for( i=words; i< words-(words%4)+4; i++)
			membuffer[i] = 0xFFFF;
		words = words-(words%4)+4;
	}
	printf("SENDing: Write Flash: ");
	sendSTX(fd); 
	sendSTX(fd);
	
	senddata(fd, 0x02); // type = 1 for read flash prog mem
	senddata(fd, words/4); // size
	senddata(fd, startaddress & 0x0000FF);  // TBLPTRL
	senddata(fd, (startaddress >> 8 ) & 0x0000FF);  // TBLPTRH
	senddata(fd, (startaddress >> 16 ) & 0x0000FF); // TBLPTRU

	for(i=0; i<words; i++)
	{
		senddata(fd, membuffer[i] & 0x0FF);
		senddata(fd, membuffer[i] >> 8);
	}
	
	sendchecksum(fd);
	sendETX(fd);  
	///// send complete, wait for answer ///////////////////////////////
	printf("RECEIVING: "); 
	expectSTX(fd);
	expectSTX(fd);
	expectPacketType(fd, 0x02);
	expectCHKSUM(fd);
	expectETX(fd);
	printf("[OK => Acknowledged Write Flash Program Memory]\n");
}

void main_readflash(int fd)
{
	unsigned char b;
	long adr;
	unsigned int membuffer[65536];

	memset(membuffer, 0, 65536);

	
	if( bytes <= 0 || startaddr < 0 )
	{
		printf("ERROR: you need to specify both --bytes and --startaddr\n");
		exit(-1);
	}

	printf("SENDing: Read Flash: ");
	sendSTX(fd); 
	sendSTX(fd);
	
	senddata(fd, 0x01); // type = 1 for read flash prog mem
	senddata(fd, bytes); // size
	senddata(fd, startaddr & 0x0000FF);  // TBLPTRL
	senddata(fd, (startaddr >> 8 ) & 0x0000FF);  // TBLPTRH
	senddata(fd, (startaddr >> 16 ) & 0x0000FF); // TBLPTRU
	
	sendchecksum(fd);
	sendETX(fd);  

	printf("RECEIVING: ");
	expectSTX(fd);
	expectSTX(fd);
	expectPacketType(fd, 0x01);
	serialReadEscape(fd, &b);
	bytes = b;
	printf("size = 0x%02hx\n", bytes);
	printf("Data Dump: \n");
	serialReadEscape(fd, &b); adr = b;
	serialReadEscape(fd, &b); adr += (b << 8);
	serialReadEscape(fd, &b); adr += (b << 16); //read TBLPTR[L,H,U]
	while( adr - startaddr < bytes )
	{
	    serialReadEscape(fd, &b);
	    printf("0x%06lx\t", adr);
	    printf("%02hx\n", b);
	    if( adr%2 == 0 )
		    membuffer[adr/2] = b;
	    else
		    membuffer[adr/2] = membuffer[adr/2] | (b << 8) ;
	    adr++;
	}
	expectCHKSUM(fd);
	if( expectETX(fd) )
	{
		int no_of_words = bytes/2;
		FILE * f = fopen("out.hex", "wb");
		printf("[no of words dumped to hex file: %i\n", no_of_words);
		printf("<ETX>\n");
		writeihex16(f, membuffer, startaddr/2, startaddr/2+no_of_words);
		writeiend16(f);
		fclose(f);
		printf("[OK => Read Flash Program Memory, INHEX16 file written to out.hex]\n");
	}
	else
	{
		printf("[ERROR: transmission incomplete]\n");
		close(fd);
		exit(-1);
	}
}

void eraseflash(int fd, long int startaddr, int rows)
{
	unsigned char b;
	printf("SENDing Erase Flash: ");
	sendSTX(fd); 
	sendSTX(fd);
	
	senddata(fd, 0x03); // type = 3 for erase flash prog mem
	senddata(fd, rows); // size
	senddata(fd, startaddr & 0x0000FF);  // TBLPTRL
	senddata(fd, (startaddr >> 8 ) & 0x0000FF);  // TBLPTRH
	senddata(fd, (startaddr >> 16 ) & 0x0000FF); // TBLPTRU
	
	sendchecksum(fd);
	sendETX(fd);  

	printf("RECEIVING: ");
	expectSTX(fd);
	expectSTX(fd);
	expectPacketType(fd, 0x03);
	expectCHKSUM(fd);
	if( ! expectETX(fd) )
	{
		printf("[ERROR: ACK for erase flash incomplete]\n");
		close(fd);
		exit(-1);
	}
	printf("[OK => Erase Flash Program Memory]\n");
}

void main_eraseflash(int fd)
{
	
	if( rows <= 0 || startaddr < 0 )
	{
		printf("ERROR: you need to specify both --rows and --startaddr\n");
		exit(-1);
	}

	eraseflash(fd, startaddr, rows);
}

void main_writeflash(int fd)
{
    int i, ret=OK;
	FILE * f;
	int words = 0;
	int xbytes;
	int xstartaddr = 0;
	unsigned int membuffer[10];

	if( strlen(hexfile) < 1 )
	{
		printf("ERROR: you need to specify --hexfile <filename>\n");
		exit -1;
	}

	if( (f = fopen(hexfile, "r") )==NULL )
	{
		printf("Error: could not open hexfile '%s'\n", hexfile);
		close(fd);
		exit(-2);
	}

	do {
		ret = readihexline(f, membuffer, &xstartaddr, &words);
		xstartaddr = xstartaddr * 2; // we need the byte counted addr not word counted
		xbytes = words *2;
		switch(ret)
		{
			case OK: printf("line read from hexfile had startaddress=0x%04X with length %ibytes\n", xstartaddr, xbytes);
			   writeflash(fd, membuffer, xstartaddr, words);
			   break;
			case ENDOFFILE: break;
			case JUNK: printf("[found junk in hexfile]\n");
		}

	} while(ret != ENDOFFILE );
}

void reset_pic(int fd)
{
	printf("SENDing RESET: ");
        sendSTX(fd);
        sendSTX(fd);
        senddata(fd, 0x00); // type does not matter for reset
        senddata(fd, 0x00); // size has to be zero (this is the distinguishing feature of a reset packet)
        sendchecksum(fd);
        sendETX(fd);
	/* nothing more to do, device should reset without an answer */
	printf("[OK => RESET Device]\n");
}


int number_arg(char * str) /* converts a string into a number, checks if decimal or hex, returns -1 if error */
{
	char * hex_prefix = "0x";
	char prfx[3] = "\0\0\0";
	int value = -1;
	if( strlen(str) > 63 )
	{
		printf("ERROR: option too long\n");
		return -1;
	}
	if( strlen(str) > 2 )
	{
		strncpy(prfx, str, 2);
		if( strcmp(hex_prefix, prfx) == 0 ) /* hex number */
		{
			if( sscanf(str, "0x%x", &value)==0 )
				value = -1;
			return value;
		}
	}
	value = atoi(str);
	return value;
}


/* main options */
enum
{
	WR_FLASH,
	RD_FLASH,
	ER_FLASH,
	WR_EEPROM,
	RD_EEPROM,
	ER_EEPROM,
	RESET,
};

int action=-1;

static struct option long_options[] =
{
	/* main options */
	{"writeflash", no_argument, &action, WR_FLASH},
	{"readflash", no_argument, &action, RD_FLASH},
	{"eraseflash", no_argument, &action, ER_FLASH},
	{"writeeeprom", no_argument, &action, WR_EEPROM},
	{"readeeprom", no_argument, &action, RD_EEPROM},
	{"eraseeeprom", no_argument, &action, ER_EEPROM},
	{"reset", no_argument, &action, RESET},
	/* normal options */
	{"device", required_argument, 0, 'd'},
	{"bytes", required_argument, 0, 'b'},
	{"startaddr", required_argument, 0, 'a'},
	{"hexfile", required_argument, 0, 'f'},
	{"rows", required_argument, 0, 'r'},
	{"help", no_argument, 0, 'h'},
	{0,0,0,0}
};
	
int main(int argc, char **argv)
{
	int c;
	int fd=-1;

	strcpy(dev, "/dev/ttyS0"); /* default device */
	strcpy(hexfile, "\0\0\0");  /* set to zero */
	fcntl(1, F_SETFL, O_SYNC);

	while (1)
	{
		/* `getopt_long' stores the option index here. */
		int option_index = 0;
		c = getopt_long (argc, argv, "d:b:a:f:r:h",
			long_options, &option_index);
	  
		/* Detect the end of the options. */
		if (c == -1)
			break;
	  
		switch (c)
		{
			case 0:
				/* If this option set a flag, do nothing else now. */
				if (long_options[option_index].flag != 0)
					break;
				printf ("option %s", long_options[option_index].name);
				if (optarg)
					printf (" with arg %s", optarg);
				printf ("\n");
				break;
			
			case 'd':
				if( strlen(optarg) > 63 )
				{
					printf("ERROR: argument for '--device' too long\n");
					exit -1;
				}
				strcpy(dev, optarg);
				printf("using device %s for serial comms\n", dev);
				break;
			
			case 'b':
				bytes = number_arg(optarg);
				if( bytes <= 0 )
				{
					printf("ERROR: malformed argument for '--bytes'\n");
					exit -1;
				}
				printf("with 0x%x (decimal: %i) bytes\n", bytes, bytes);
				break;
			
			case 'a':
				startaddr = number_arg(optarg);
				if( startaddr < 0 )
				{
					printf("ERROR: malformed argument for '--startaddr'\n");
					exit -1;
				}
				printf("startaddr=0x%x (decimal: %i)\n", startaddr, startaddr);
				break;
			
			case 'f':
				if( strlen(optarg) > 63 )
				{
					printf("ERROR: argument for '--hexfile' too long\n");
					exit -1;
				}
				strcpy(hexfile, optarg);
				printf("using hexfile '%s'\n", hexfile);
				break;
			
			case 'r':
				rows = number_arg(optarg);
				if( rows <= 0 )
				{
					printf("ERROR: malformed argument for '--rows'\n");
					exit -1;
				}
				printf("rows to delete 0x%x (decimal %i)\n", rows, rows);
				break;
				
			case 'h':
				printf("Help:\n");
				usage(argv[0]);
				break;
				
			case '?':
				/* `getopt_long' already printed an error message. */
				break;
				
			default:
				abort ();
		}
	}
  
	if( (fd = open(dev, O_RDWR | O_NOCTTY)) <= 0 )
	{
		printf("ERROR: could not open serial device '%s'\n", dev);
		exit -1;
	}
	setline(fd, flags, speed);
	serialWrite(fd, TICKLE); // send tickle byte

	/* Print any remaining command line arguments (not options). */
	if (optind < argc)
	{
		printf("unknown arguments: ");
		while (optind < argc)
			printf ("%s ", argv[optind++]);
		putchar ('\n');
		usage(argv[0]);
	}

	switch(action)
	{ 
		case WR_FLASH:
			printf("action: write flash\n");
			main_writeflash(fd);
			break;
		case RD_FLASH:
			printf("action: read flash\n");
			main_readflash(fd);
			break;
		case ER_FLASH:
			printf("action: erase flash\n");
			main_eraseflash(fd);
			break;
		case WR_EEPROM:
			printf("action: write eeprom\n");
/*			main_writeeeprom(fd); */
			printf("\n\tNOT IMPLEMETED YET\n\n");
			break;
		case RD_EEPROM:
			printf("action: read eeprom\n");
			main_readeeprom(fd);
			break;
		case ER_EEPROM:
			printf("action: erase eeprom\n");
/*			main_eraseeeprom(fd); */
			printf("\n\tNOT IMPLEMETED YET\n\n");
			break;
		case RESET:
			printf("resetting PIC\n");
			reset_pic(fd);
			break;
		default:
			printf("You need to specify the action to take\n");
			usage(argv[0]);
	}
   

	close(fd);

	return 0;
}
