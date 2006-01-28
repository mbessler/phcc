#include <stdio.h>
#include <sys/ioctl.h>
#include <linux/ppdev.h>
#include <sys/time.h>
#include <sys/io.h>
#include <fcntl.h>

#include <iostream>

#include "MB_Opt.h"

using namespace std;


int fd = -1;

void openport(const char * np)
{
	int k;
	fd = open(np, O_WRONLY);
	if( fd == -1 )
	{
		cout << "ERROR: could not open parallel port" << endl;
		exit(-1);
	}
	if( (k = ioctl(fd, PPCLAIM)) != 0)
	{
		char b[200];
		sprintf(b,"ERROR: openport (%.150s must be rw enabled)", np);
		perror(b);
		close(fd);
		fd = -1;
		exit(-2);
	}
}

void closeport()
{
	if( fd != -1)
	{
		int stat = ioctl(fd,PPRELEASE);
		close(fd);
		if( stat != 0)
			perror("closeport");
	}
	fd = -1;
}

void dly()
{
	struct timeval tv;
	tv.tv_sec=0;
	tv.tv_usec=1000*10; //10ms
	select(0,NULL,NULL,NULL,&tv);
}

void strobe_on()
{
	struct ppdev_frob_struct frob;
	frob.mask = 0x01;
	frob.val = 0x00;
	if( ioctl(fd, PPFCONTROL, &frob) != 0 )
		perror("strobe_on() ioctl failed!");
	dly();
}

void strobe_off()
{
        struct ppdev_frob_struct frob;
        frob.mask = 0x01;
        frob.val = 0xFF;
        if( ioctl(fd, PPFCONTROL, &frob) != 0 )
                perror("strobe_off() ioctl failed!");
        dly();
}

void D0_on()
{
        unsigned char b = 0xFF;
        if( ioctl(fd, PPWDATA, &b) != 0 )
                perror("D0_on() ioctl failed!");
        dly();
}

void D0_off()
{
	unsigned char b = 0x00;
	if( ioctl(fd, PPWDATA, &b) != 0 )
		perror("D0_on() ioctl failed!");
	dly();
}

void pulse_clk()
{
	strobe_on();
	strobe_off();
}

void send_devaddr(unsigned char _devaddr)
{
	int i;
	printf(" ");
	for(i=0; i<8; ++i)
	{
		if( _devaddr & 0x01 )
			D0_on();
		else
			D0_off();
		_devaddr = _devaddr >> 1;
	pulse_clk();
	}
}


void send_subaddr(unsigned char _subaddr)
{
	int i;
	printf(" ");
	for(i=0; i<6; ++i)
	{
		if( _subaddr & 0x01 )
			D0_on();
		else
			D0_off();
		_subaddr = _subaddr >> 1;
		pulse_clk();
	}
}

void send_data(unsigned char _data)
{
	int i;
	printf(" ");
	for(i=0; i<8; ++i)
	{
		if( _data & 0x01 )
			D0_on();
		else
			D0_off();
		_data = _data >> 1;
		pulse_clk();
	}
}





int main(int argc, char * argv[])
{
	MB_Opt * opt = new MB_Opt();
	
	opt->addOption(
	   "help",               // long opt name or name in configfile
	   'h',                  // short opt name (if any)
	   OPT_NONE,             // type
	   OSRC_CMDLINE,         // where allowed to occur (commandline, configfile, or both)
	   0,                    // default value
	   "this help text"      // help text
	   );
#ifdef __WIN32__
	#pragma error "This does not work on win32!!!"
#else   /* __WIN32__ */
	opt->addOption("device", 'd', OPT_STRING, OSRC_CMDLINE|OSRC_CONFFILE, "/dev/parport0", "parallel port device");
#endif  /* __WIN32__ */
	

	opt->addOption("--devaddr", '\0', OPT_INT, OSRC_CMDLINE, "NIL", "device address of DOA board");
	opt->addOption("--subaddr", '\0', OPT_INT, OSRC_CMDLINE, "NIL", "subdevice address of DOA board");
	opt->addOption("--data", '\0', OPT_INT, OSRC_CMDLINE, "NIL", "data byte to send to DOA board");

	if( ! opt->parseArgv(argc, argv) )
	   exit(-1);


	if( opt->getOpt("help") == string("1") )
	{
	   opt->printHelp(argv[0]);
	   exit(0);
	}

	if( opt->getOpt("devaddr") == string("NIL") ||
		opt->getOpt("subaddr") == string("NIL") ||
		opt->getOpt("data") == string("NIL") )
	{
		cout << "Error: you have to specify devaddr, subaddr, and data" << endl;
		opt->printHelp(argv[0]);
		exit(-1);
	}


	openport( opt->getOpt("device").c_str() );
	
	unsigned char devaddr = opt->getOptAsInt("devaddr");
	unsigned char subaddr = opt->getOptAsInt("subaddr");
	unsigned char data = opt->getOptAsInt("data");
	printf("sending to DOA device (devaddr=0x%02x subaddr=0x%02x data=0x%02x)\n", devaddr, subaddr, data);

        send_devaddr(devaddr);
        send_subaddr(subaddr);
        send_data(data);
	printf("\n");
	
	closeport();	
	delete opt;
	exit(0);
}

