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
	printf("requesting analog map from PHCC\n");
	b = 0x05;   // analogmap get
	serialWrite(b);
	for(i=0; i<1+35*2+2; i++)
	{
		unsigned char bret;
		serialRead(&bret);
		printf("0x%02x ", bret);
	}
	printf("\n");

	closedevice();
	mb_options_done();
	exit(0);
}
