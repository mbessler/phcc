//#include <stdio.h>

#include <iostream>
#ifndef __WIN32__
#include <csignal>
#else
// win32 signal processing includes go here
#endif // __WIN32__

#include "MB_Opt.h"
#include "PHCC_Serial.h"

using namespace std;

PHCC_Serial * port = NULL;

#ifndef __WIN32__
void signal_handler(int sig)
{
	cout << "caught signal... exitting" << endl;
	switch( sig )
	{
		case SIGINT:
		case SIGTERM:
		case SIGHUP:
		{
			printf("sending STOPTALKING to PHCC\n");
			unsigned char b = 0x03;  // PHCC STOPTALKING
			port->serialWriteBlocking(b);
			port->closeDevice();
			delete port;
			exit(0);
		}
		default:
			cout << "unhandled signal!!!" << endl;
	}
}

#else
// WIN32 code goes here
#endif // __WIN32__

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

	//install signal handler
#ifndef __WIN32__
	struct sigaction sigmain;
	sigemptyset(&(sigmain.sa_mask));
	sigaddset(&(sigmain.sa_mask),SIGINT);
	sigaddset(&(sigmain.sa_mask),SIGTERM);
	sigaddset(&(sigmain.sa_mask),SIGHUP);
	sigmain.sa_handler = signal_handler;
	sigmain.sa_flags = 0;
	sigaction(SIGINT,&sigmain,NULL);
	sigaction(SIGTERM,&sigmain,NULL);
	sigaction(SIGHUP,&sigmain,NULL);
#else
//install win32 equivalent of signal handler here
#endif // __WIN32__
	
	printf("sending STARTTALKING to PHCC\n");
	unsigned char b = 0x02;   // PHCC STARTTALKING
	port->serialWriteBlocking(b);

	printf("watching PHCC analog inputs and keymatrix inputs for changes:\n");
	while( 1 )  // signal handler has to gracefully exit the program
	{
		unsigned char b1, b2, b3;
		port->serialReadBlocking(&b1);
		if( b1 & 0x28 /*0x2F*/ ) /* it's a keymatrix update packet */
		{
			port->serialReadBlocking(&b2);
			printf("key 0x%02x%02x=%d\n", ((b1>>1)&0x3), ((b1<<7)&0x80)+((b2>>1)&0x7F), b2&0x1);
		}
		else if( b1 & 0x50 /*0x57*/) /* it's a analog update packet */
		{
			port->serialReadBlocking(&b2);
			port->serialReadBlocking(&b3);
			printf("channel %d=0x%02x%02x\n", (b2>>2), (b2&0x03), b3);
		}
		else
		{
			printf("garbled update packet\n");
		}
	}
	// can only be exitted via signal handler
}

