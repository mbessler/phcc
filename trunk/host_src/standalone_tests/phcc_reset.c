#include <stdio.h>
#include "mb_options.h"
#include "common_serial.h"


int main(int argc, char * argv[])
{
	unsigned char b;
	int i;
	int baudrate=0;
	char * dev = NULL;
	mb_options_init(argc, argv);
	if( (mb_options_getboolopt(NULL, "help") == 1) )
	{
		mb_options_usage(argv[0]);
		exit(0);
	}
	dev = mb_options_getstropt(NULL, "device");
	baudrate = mb_options_getintopt(NULL, "speed");
	if( ! opendevice(dev, baudrate) )
	{
		exit(1);
	}

	printf("using %s at %i baud\n", dev, baudrate);
        printf("sending RESET to PHCC\n");
        b = 0x01;   // PHCC reset
        serialWrite(b);
        serialWrite(b);
        serialWrite(b);
						
	closedevice();
	mb_options_done();
	exit(0);
}
