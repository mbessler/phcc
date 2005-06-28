//#include <stdio.h>

#include <iostream>

#include "MB_Opt.h"
#include "PHCC_Serial.h"

using namespace std;

int main(int argc, char * argv[])
{
	MB_Opt * opt = new MB_Opt();
	PHCC_Serial * port = new PHCC_Serial(&cout); // uses std::cout
	
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


	if( ! opt->parseArgv(argc, argv) )
	   exit(-1);

//	opt->listGivenOpts();
	opt->readConfigFile("phcc.conf");


	if( opt->getOpt("help") == string("1") )
	{
	   opt->printHelp(argv[0]);
	   exit(0);
	}


	cout << "opening port " << opt->getOptAsString("port") << " with " << opt->getOptAsInt("speed") << "bps" <<  endl;
	port->openDevice(opt->getOptAsString("port"), opt->getOptAsInt("speed"));
	
	if( ! port->isOpen() )
	{
	   cout << "could not open serial port" << endl;
	   delete port;
	   exit(-1);
	}

	unsigned char b = 0x01;   // PHCC reset

	printf("sending RESET to PHCC\n");
	port->serialWriteBlocking(b);
	port->serialWriteBlocking(b);
	port->serialWriteBlocking(b);

	port->closeDevice();
	delete port;

	exit(0);
}
