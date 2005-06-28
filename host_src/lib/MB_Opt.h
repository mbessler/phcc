// MB_Option.h
//
//   A commandline parameter and config file parser and processor
//
//   Copyright (c) 2003-2005 by Manuel Bessler <m.bessler AT gmx.net>
//
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
// 
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
// This is for emacs
// Local variables:
// mode: c++
// c-basic-offset: 3
// tab-width: 3
// End:
//

#ifndef _MB_OPTION_H
#define _MB_OPTION_H

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <map>
#include <vector>
#include <string>
#include <iostream>
#include <iomanip>
#include <fstream>

#define MB_OPTIONS_LINELENGTH 1024

#ifdef __WIN32__
// #include <windows.h>

#else   /* __WIN32__ */

#endif  /* __WIN32__ */

enum OptionTypes
{
	OPT_NONE    =  0,
	OPT_INT     =  1,
	OPT_STRING  =  2,
	OPT_HEX     =  4,
	OPT_CHAR    =  8,
	OPT_BOOL    = 16,
	OPT_FLOAT   = 32
};

enum OptionSource
{
	OSRC_CMDLINE = 1,
	OSRC_CONFFILE = 2
};

struct OptionDef
{
		std::string longOpt;
		char shortOpt;
		int validTypes;
		int optionSources;
		std::string helpText;
		std::string defaultValue;
//		std::string min;
//		std::string min;
//step (maybe a functor ?)
};


class MB_Opt
{
	public:
		MB_Opt();
		void addOption(const char * _longOpt,
							char _shortOpt,
							int _optionTypes, 
							int _optionSource,
							const char * _defaultvalue,
							const char * _helpText);
// 		void addOption(std::string _longOpt, 
// 							char _shortOpt,
// 							int _optionTypes, 
// 							int _optionSource,
// 							std::string _defaultvalue,
// 							std::string _helpText = "");
		bool readConfigFile(const char * _filename); 

		std::string getOpt(const std::string & _optname);
		const char * getOptAsString(const std::string & _optname);
		int getOptAsInt(const std::string & _optname);

		const char * getNonOptParm(unsigned int num);

		bool parseArgv(int argc, char ** argv);
 		void printHelp(const char * progname);
		void listGivenOpts();

	protected:
		const OptionDef * isValidOpt(const std::string & _opt);
		const OptionDef * isValidShortOpt(const char _c);

	private:
		std::vector<OptionDef> validOptions;
		std::vector<OptionDef>::iterator validOptionsIter;

		std::map<std::string,std::string> longOpts;
		std::map<std::string,std::string>::iterator longOptsIter;

		std::vector<std::string> nonParamOpts;
		std::vector<std::string>::iterator nonParamOptsIter;

};


#endif /* _MB_OPTION_H */
