//#include <stdio.h>

#include <iostream>

#include "MB_Opt.h"
#include "PHCC_Serial.h"

#define dly port->waitms(1);

using namespace std;

char initseq[] = {0x38, 0x0E, 0x06, 0x02, 0x00};

PHCC_Serial * port;
unsigned char doa_packet = 0x07;   // PHCC DOA packet
unsigned char devaddr = 0xF0;
int lcdnum = 0;

void send_lcd_data(unsigned char databyte)
{
	unsigned char subaddr = (lcdnum & 0x07);
	port->serialWriteBlocking(doa_packet);
		dly;
	port->serialWriteBlocking(devaddr);
		dly;
	port->serialWriteBlocking(subaddr);
		dly;
	port->serialWriteBlocking(databyte);
		dly;
}

void send_lcd_cmd(unsigned char cmdbyte)
{
	unsigned char subaddr = (lcdnum & 0x07) | 0x08;
	port->serialWriteBlocking(doa_packet);
		dly;
	port->serialWriteBlocking(devaddr);
		dly;
	port->serialWriteBlocking(subaddr);
		dly;
	port->serialWriteBlocking(cmdbyte);
		dly;
}


int main(int argc, char * argv[])
{
	MB_Opt * opt = new MB_Opt();
	port = new PHCC_Serial(&cout); // uses std::cout
	
	opt->addOption(
	   "help",               // long opt name or name in configfile
	   'h',                  // short opt name (if any)
	   OPT_NONE,             // type
	   OSRC_CMDLINE,         // where allowed to occur (commandline, configfile, or both)
	   0,                    // default value
	   "this help text"      // help text
	   );
#ifdef __WIN32__
	opt->addOption("port", 'p', OPT_STRING, OSRC_CMDLINE|OSRC_CONFFILE, "COM1", "serial port device");
#else   /* __WIN32__ */
	opt->addOption("port", 'p', OPT_STRING, OSRC_CMDLINE|OSRC_CONFFILE, "/dev/ttyS0", "serial port device");
#endif  /* __WIN32__ */
	
	opt->addOption("--speed", 's', OPT_INT, OSRC_CMDLINE|OSRC_CONFFILE, "115200", "serial port speed");

	opt->addOption("--devaddr", '\0', OPT_INT, OSRC_CMDLINE, "NIL", "device address of DOA board");
	opt->addOption("--lcdnum", '\0', OPT_INT, OSRC_CMDLINE, 0, "which lcd to talk to (0-7)");
	opt->addOption("--text", '\0', OPT_STRING, OSRC_CMDLINE, "PHCC", "what to display on the LCD");
	opt->addOption("--init", '\0', OPT_BOOL, OSRC_CMDLINE, "0", "send HD44780 LCD init sequence");

	if( ! opt->parseArgv(argc, argv) )
	   exit(-1);

//	opt->listGivenOpts();
	opt->readConfigFile("phcc.conf");


	if( opt->getOpt("help") == string("1") )
	{
	   opt->printHelp(argv[0]);
	   exit(0);
	}

	if( opt->getOpt("devaddr") == string("NIL") ||
		opt->getOpt("lcdnum") == string("NIL") ||
		opt->getOpt("text") == string("NIL") )
	{
		cout << "Error: you have to specify devaddr, subaddr, and data" << endl;
		opt->printHelp(argv[0]);
		exit(-1);
	}

	cout << "opening port " << opt->getOptAsString("port") << " with " << opt->getOptAsInt("speed") << "bps" <<  endl;
	port->openDevice(opt->getOptAsString("port"), opt->getOptAsInt("speed"), true);
	
	if( ! port->isOpen() )
	{
	   cout << "could not open serial port" << endl;
	   delete port;
	   exit(-1);
	}

	devaddr = opt->getOptAsInt("devaddr");
	lcdnum = opt->getOptAsInt("lcdnum");
	string demostr = opt->getOptAsString("text");
	printf("talking to lcd #%d on DOA_char_lcd board with devaddr=0x%02x\n", lcdnum, devaddr);

	if( opt->getOptAsInt("init") == 1 )
	for( size_t i=0; i<strlen(initseq); i++ )
	{
		send_lcd_cmd(initseq[i]);
	}
	const char * text = demostr.c_str();
	printf("sending '%s' to display", text);
	for( size_t i=0; i<strlen(text); i++ )
	{
		send_lcd_data(text[i]);
	}

cout << "done" << endl;
	port->closeDevice();
	delete port;

	exit(0);
}

