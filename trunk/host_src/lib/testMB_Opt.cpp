// testMB_Opt.cpp
//
// TEST for
//   MB_Opt, a commandline parameter and config file parser and processor
//
//   Copyright (c) 2005 by Manuel Bessler <m.bessler AT gmx.net>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or 
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
// 
// This is for emacs
// Local variables:
// mode: c++
// c-basic-offset: 3
// tab-width: 3
// End:
//

#include <iostream>

#include "MB_Opt.h"

using namespace std;

int main(int argc, char ** argv)
{
   MB_Opt * opt = new MB_Opt();
	//             long    short type    allowed where def. val   helptext
   opt->addOption(
		"help",               // long opt name or name in configfile
		'h',                  // short opt name (if any)
		OPT_NONE,             // type
		OSRC_CMDLINE,         // where allowed to occur (commandline, configfile, or both)
		0,                    // default value
		"this help text"      // help text
		);
   opt->addOption("verbose", 'v', OPT_NONE, OSRC_CMDLINE, "", "make output verbose");
   opt->addOption("exclude", 'x', OPT_STRING, OSRC_CMDLINE, "", "exclude file");
   opt->addOption("loglevel", 'l', OPT_INT, OSRC_CMDLINE|OSRC_CONFFILE, "", "set loglevel");
	opt->addOption("configfile", 'f', OPT_STRING, OSRC_CMDLINE, "phcc.conf", "configfile to read");

#ifdef __WIN32__
	opt->addOption("port", 'p', OPT_STRING, OSRC_CMDLINE|OSRC_CONFFILE, "COM1", "serial port device");
#else   /* __WIN32__ */
	opt->addOption("port", 'p', OPT_STRING, OSRC_CMDLINE|OSRC_CONFFILE, "/dev/ttyS0", "serial port device");
#endif  /* __WIN32__ */

	opt->addOption("--baud", 's', OPT_INT, OSRC_CMDLINE|OSRC_CONFFILE, "115200", "serial port speed");
//	opt->addOption("", OPT_, "");

   opt->parseArgv(argc, argv);
	opt->printHelp(argv[0]);
	opt->listGivenOpts();

	cout << "================= from config file ==================================" << endl;

	opt->readConfigFile("phcc.conf");

	cout << "opening port " << opt->getOpt("port") << " with " << opt->getOpt("baud") << "bps" <<  endl;

   delete opt;
}
