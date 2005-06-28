// MB_Option.cpp
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
// c-basic-offset: 4
// tab-width: 4
// End:
//

#include "MB_Opt.h"

using namespace std;

MB_Opt::MB_Opt()
{
}


string MB_Opt::getOpt(const string & _optname)
{
   longOptsIter = longOpts.find(_optname);
   if( longOptsIter == longOpts.end() )  // not found
   {
	  // check if this option exists in the validoptions list and possibly return default value 
	  const OptionDef * optdef = isValidOpt(_optname);
	  if( optdef == NULL )
	  {
		 cerr << "unknown Option ('" << _optname << "') requested. Implementation error please fix!" << endl;
		 exit(-2);
	  }
	  else
	  {
		 if( optdef->defaultValue.size() > 0 )
			return optdef->defaultValue;
		 else
			return string("");
	  }
   }
   else // found
   {
	  if( (*longOptsIter).second.size() > 0 )
		 return (*longOptsIter).second;
	  else
		 return string("");
   }
}

const char * MB_Opt::getOptAsString(const std::string & _optname)
{
   return getOpt(_optname).c_str();
}

int MB_Opt::getOptAsInt(const std::string & _optname)
{
   int value=0;
   string str = getOpt(_optname);
   const char * s = str.c_str();
   if( str == "" || strlen(s) < 1 )
	  return -1;

   if( strlen(s) > 2 && s[0] == '0' && s[1] == 'x' ) // looks like a hex number
   {
	  value = strtol(s, (char **)NULL, 16);
   }
   else
   {
	  value = strtol(s, (char **)NULL, 10);
   }
   return value;
}

const char * MB_Opt::getNonOptParm(unsigned int num)
{
   int counter = 0;
   nonParamOptsIter = nonParamOpts.begin();
   while( nonParamOptsIter != nonParamOpts.end() )
   {
	  if( counter == num )
		 return (*nonParamOptsIter).c_str();
	  nonParamOptsIter++;
	  counter++;
   }
   return NULL;
}

bool MB_Opt::parseArgv(int argc, char ** argv)
{
   int i = 1;
   enum parse_state { PSTART, PDASH, PNODASH, PERROR, PSHORTOPT, PLONGOPT, PDASHDASH, PNONOPTPARAMS };
   parse_state state = PSTART;
   while( i < argc )
   {
	  char shortopt = '\0';
	  string longopt;
	  switch( state )
	  {
		 case PSTART:
		 {
			if( strlen(argv[i]) <= 1 )
			{
			   state = PERROR;
			   break;
			}
			// strlen(argv[i]) must now be >= 2
			if( argv[i][0] == '-' )
			{
			   state = PDASH;
			   break;
			}
			else
			{
			   state = PNODASH;
			   break;
			}
			break;
		 }
		 case PDASH:
		 {
			// at this point we know that argv[i] has at least 2 chars
			if( strlen(argv[i]) == 2 ) 
			{
			   if( argv[i][1] == '-' ) // "--" end of options 
			   {
				  state = PDASHDASH;
				  break;
			   }
			   else // must be short opt
			   {
				  state = PSHORTOPT;
				  break;
			   }
			}
			// at this point we konw that argv[i] has at least 3 chars
			if( argv[i][1] == '-' ) // is it a long opt ?
			{
			   state = PLONGOPT;
			   break;
			}
			break;
		 }
		 case PSHORTOPT:
		 {
			shortopt = argv[i][1];
			// check if valid shortopt, check if it can have parameter, check parameter type

			const OptionDef * optdef = NULL;
			if( (optdef = isValidShortOpt(shortopt)) != NULL && ( optdef->optionSources & OSRC_CMDLINE ) )
			{
			   if( optdef->validTypes == 0 )
			   {
				  longOpts.insert(make_pair(optdef->longOpt, string("1")));
				  state = PSTART; // next round
				  i++;
				  break;
			   }
			}
			else
			{
			   cout << "unknown option: \"-" << shortopt << "\"" << endl;
			   cout << "try --help" << endl;
			   return false;
			}
			// when we get here, the option needs to have a parameter
			if( argc <= i+1 ) // no parameter
			{
			   cout << "option \"-" << shortopt << "\" requires a parameter" << endl;
			   cout << "try --help" << endl;
			   return false;
			}
			else
			{
			   longOpts.insert(make_pair(optdef->longOpt, argv[i+1]));
			   i += 2;
			   state = PSTART;
			}
			break;
		 }
		 case PLONGOPT:
		 {
			if( argv[i][2] == '-' ) // no third dash allowed here
			{
			   state = PERROR;
			   cout << "Three dashes not allowed in option!" << endl;
			   break;
			}
			bool hasEqual = false;
			string afterEqual;
			for( unsigned int idx=2; idx<strlen(argv[i]); idx++ )
			   if( argv[i][idx] == '=' && strlen(argv[i]) >= idx+2 )
			   {
				  hasEqual = true;
				  afterEqual =((argv[i])+idx+1);
				  argv[i][idx] = '\0'; //terminate the option part
			   }

			char * longname = argv[i] + 2;
			longopt = longname;
			// check if valid longopt, check if it can have parameter, check parameter type
			const OptionDef * optdef = NULL;
			if( (optdef = isValidOpt(longopt)) != NULL && ( optdef->optionSources & OSRC_CMDLINE ) )
			{
			   if( optdef->validTypes == 0 )
			   {
				  longOpts.insert(make_pair(longopt,string("1")));
				  state = PSTART; // next round
				  i++;
				  break;
			   }
			}
			else
			{
			   cout << "unknown option: \"--" << longopt << "\"" << endl;
			   cout << "try --help" << endl;
			   return false;
			}
			// when we get here, the option needs to have a parameter
			if( argc <= i+1 && ! hasEqual ) // no parameter
			{
			   cout << "option \"--" << longopt << "\" requires a parameter" << endl;
			   cout << "try --help" << endl;
			   return false;
			}
			else
			{
			   if( hasEqual )
			   {
				  longOpts.insert(make_pair(longopt,afterEqual));
				  i++;
			   }
			   else
			   {
				  longOpts.insert(make_pair(longopt,argv[i+1]));
				  i += 2;
			   }
			   state = PSTART;
			}
			break;
		 }
		 case PDASHDASH:
			//from now on all options are literal (file) params
			i++;
			state = PNONOPTPARAMS;
			break;
		 case PNODASH:
			state = PSTART;
			nonParamOpts.push_back(string(argv[i]));
			i++;
			break;
		 case PNONOPTPARAMS:
			nonParamOpts.push_back(string(argv[i]));
			i++;
			break;
		 case PERROR:
			cout << "Error while parsing command line: token #" << i << 
			   " <" << argv[i] << "> could not be parsed!"<< endl;
			cout << "try --help" << endl;
			return false;
		 default:
			cout << "Error while parsing command line: token #" << i << 
			   " <" << argv[i] << "> parser fell on its nose!"<< endl;
			cout << "go and fix the parser or report to author" << endl;
			return false;
	  }
   }
   return true;
}

void MB_Opt::printHelp(const char * progname)
{
   cout << "usage " << progname << " [options] ..." << endl;
   const int maxwidth = 24;
   for(validOptionsIter = validOptions.begin(); validOptionsIter != validOptions.end(); ++validOptionsIter)
   {
	  OptionDef od = (*validOptionsIter);
	  if( od.optionSources & OSRC_CMDLINE )
	  {
		 string longopt = od.longOpt;
		 int padding = maxwidth - od.longOpt.length();
		 if( padding < 0 ) 
			padding = 0;
		 string padstr(padding, ' ');
		 if( od.shortOpt != '\0' )
			cout << "  -" << od.shortOpt << ", ";
		 else
			cout << "      ";
		 cout << "--" << od.longOpt << padstr << od.helpText;
		 if( od.defaultValue != "" )
			cout << "(default: " << od.defaultValue << ")";
		 cout << endl;
	  }
   }
}


void MB_Opt::addOption(const char * _longOpt, char _shortOpt, int _optionTypes, 
					   int _optionSource, const char * _defaultvalue,
					   const char * _helpText)
{ 
   OptionDef od;

   if( _longOpt == NULL )
	  cerr << "BAD: NULL pointer passed as longopt" << endl, exit(-1);

   if( _longOpt[0] == '-' )
   {
	  if( _longOpt[1] != '-')
	  {
		 cerr << "ERROR in longopt, exitting!" << endl;
		 exit(-1);
	  }
	  if( strlen(_longOpt) < 4 )
	  {
		 cerr << "ERROR in longopt, exitting!" << endl;
		 exit(-1);
	  }
   }
   else if( strlen(_longOpt) < 2 )
   {
	  cerr << "ERROR in longopt, exitting!" << endl;
	  exit(-1);
   }
   // TODO checks for invalid chars in (long|short)-opts

   if( strlen(_longOpt) > 3 && _longOpt[0] == '-' && _longOpt[1] == '-' ) // longopt
   {
	  _longOpt += 2;
   }
   else if( strlen(_longOpt) > 1 && _longOpt[0] != '-' )  // long opt, w/o '--' given
   {
	  //opt is good as is
   }
   else
   {
	  cerr << "error in " << __PRETTY_FUNCTION__ << " option not valid: \"" << _longOpt << "\"" << endl;
	  exit(-1);
   }
   od.longOpt = _longOpt;
   od.shortOpt = _shortOpt;
   od.validTypes = _optionTypes;
   od.optionSources = _optionSource;

   if( _defaultvalue == NULL )
	  od.defaultValue = string("");
   else
	  od.defaultValue = _defaultvalue;
   if( _helpText == NULL )
	  od.helpText = string("");
   else
	  od.helpText = _helpText;
   validOptions.push_back(od);
}





bool MB_Opt::readConfigFile(const char * _filename)
{
   const int bufsize = 1024;
   char buf[bufsize];
   int lineno = 0;
   ifstream conf(_filename);
   while( conf.getline(buf, bufsize) ) // get reads a line, including \n
   {
	  lineno++;
	  cout << "read config line #" << lineno << ": <" << buf << ">" << endl;
	  char * charptr = buf;
	  char * startptr = NULL;
	  char * endptr = NULL;
	  int len = strlen(buf);
	  enum { LINE_PARSE_START, LINE_PARSE_COMMENT, LINE_PARSE_NAME_START, LINE_PARSE_NAME, 
			 LINE_PARSE_NAME_EQ, LINE_PARSE_VALUE_START, LINE_PARSE_VALUE_QUOTED, LINE_PARSE_VALUE, 
			 LINE_PARSE_EOL, LINE_PARSE_ERR };
	  int parse_state = LINE_PARSE_START;
	  string optname;
	  string optval;
	  bool escaped=false;
	  do
	  {
		 switch( parse_state )
		 {
			case LINE_PARSE_START: // allowed are space,tab,commentstarter and alnum
			   if( len == 0 )
			   {
				  parse_state = LINE_PARSE_START; // empty line
				  charptr = buf + len + 1; // cause loop to exit for reading next line					  
			   }
			   else if( *charptr == ' ' || *charptr == '\t' )
				  charptr++;
			   else if( *charptr == '#' )
				  parse_state = LINE_PARSE_COMMENT;
			   else if( (*charptr >= 'a' && *charptr <= 'z') || (*charptr >= 'A' && *charptr <= 'Z') )
				  parse_state = LINE_PARSE_NAME_START;
			   else if( (*charptr == '\n') || (*charptr == '\r') ) // newline
			   {
				  parse_state = LINE_PARSE_START;
				  charptr = buf + len + 1; // cause loop to exit for reading next line
			   }
			   else
			   {
				  parse_state = LINE_PARSE_ERR;
				  cerr << "Parse Error in configfile '" << _filename << "', line #" << lineno 
					   << " garbled line." << endl;
				  return false;
				  break;
			   }
			   break;
			case LINE_PARSE_NAME_START:
			   startptr = charptr;
			   charptr++;
			   parse_state = LINE_PARSE_NAME;
			   break;
			case LINE_PARSE_NAME:
			   if( (*charptr >= 'a' && *charptr <= 'z') || 
				   (*charptr >= 'A' && *charptr <= 'Z') ||
				   (*charptr >= '0' && *charptr <= '9') ||
				   *charptr == '-' )
				  charptr++;
			   else
			   {
				  endptr = charptr-1;
				  optname = startptr;
				  optname.erase(endptr-startptr+1);
				  parse_state = LINE_PARSE_NAME_EQ;
			   }
			   break;
			case LINE_PARSE_NAME_EQ:
			   if( *charptr == '=' )
			   {
				  charptr++;
				  parse_state = LINE_PARSE_VALUE_START;
			   }
			   else if( *charptr == ' ' || *charptr == '\t' )
				  charptr++;
			   else
			   {
				  parse_state = LINE_PARSE_ERR;
				  cerr << "Parse Error in configfile '" << _filename << "', line #" << lineno 
					   << " invalid chars found." << endl;
				  return false;
			   }
			   break;
			case LINE_PARSE_VALUE_START:
			   if( *charptr == ' ' || *charptr == '\t' )
				  charptr++;
			   else if( *charptr == '"' )
			   {
				  parse_state = LINE_PARSE_VALUE_QUOTED;
				  charptr++;
				  startptr = charptr;
			   }
			   else if( *charptr == '#' )
			   {
				  parse_state = LINE_PARSE_ERR;
				  cerr << "Parse Error in configfile '" << _filename << "', line #" << lineno 
					   << " comment char found where name/value pair expected." << endl;			
				  return false;
			   }
			   else
			   {
				  parse_state = LINE_PARSE_VALUE;
				  startptr = charptr;
				  charptr++;
			   }
			   break;
			case LINE_PARSE_VALUE_QUOTED:
			   if( charptr+1 >= buf+len )
			   {
				  parse_state = LINE_PARSE_EOL;
				  endptr = charptr-1;
				  optval = startptr;
				  optval.erase(endptr-startptr+1);
				  const OptionDef * od = NULL;
				  if( (od = isValidOpt(optname)) != NULL && ( od->optionSources & OSRC_CONFFILE ) )
					 longOpts.insert(make_pair(optname, optval));
//					 opts.insert(make_pair(optname,optval));
				  else
				  {
					 parse_state = LINE_PARSE_ERR;
					 cerr << "unknown option found in configfile '" << _filename << "', line #" << lineno 
						  << endl;
					 return false;
				  }
				  charptr++; // this will end the do {} while() loop for this line
			   }
			   else if( *charptr == '\\' )
			   {
				  escaped = ! escaped;
				  charptr++;
			   }
			   else if( *charptr == '"' &&  ! escaped )
			   {
				  parse_state = LINE_PARSE_EOL;
				  endptr = charptr-1;
				  optval = startptr;
				  optval.erase(endptr-startptr+1);
				  charptr++;
			   }
			   else
				  charptr++;
			   break;
			case LINE_PARSE_VALUE:
			   if( charptr+1 >= buf+len )
			   {
				  parse_state = LINE_PARSE_EOL;
				  endptr = charptr;
				  optval = startptr;
				  optval.erase(endptr-startptr+1);

				  const OptionDef * od = NULL;
				  if( (od = isValidOpt(optname)) != NULL && ( od->optionSources & OSRC_CONFFILE ) )
					 longOpts.insert(make_pair(optname, optval));
//					 opts.insert(make_pair(optname,optval));
				  else
				  {
					 parse_state = LINE_PARSE_ERR;
					 cerr << "unknown option found in configfile '" << _filename << "', line #" << lineno 
						  << endl;
					 return false;
				  }

				  charptr++; // this will end the do {} while() loop for this line
			   }
			   else if( *charptr == ' ' || *charptr == '\t' || *charptr == '#' )
			   {
				  parse_state = LINE_PARSE_EOL;
				  endptr = charptr-1;
				  optval = startptr;
				  optval.erase(endptr-startptr+1);
			   }
			   else
				  charptr++;
			   break;
			case LINE_PARSE_EOL:
			   if( charptr+1 >= buf+len ) // last char in line (before \0)
			   {
				  const OptionDef * od = NULL;
				  if( (od = isValidOpt(optname)) != NULL && ( od->optionSources & OSRC_CONFFILE ) )
					 longOpts.insert(make_pair(optname, optval));
//					 opts.insert(make_pair(optname,optval));
				  else
				  {
					 parse_state = LINE_PARSE_ERR;
					 cerr << "unknown option found in configfile '" << _filename << "', line #" << lineno 
						  << endl;
					 return false;
				  }
				  charptr++; // this will end the do {} while() loop for this line
			   }
			   else if( *charptr == ' ' || *charptr == '\t' )
				  charptr++;
			   else if( *charptr == '#' ) // the rest is comment
			   {
				  const OptionDef * od = NULL;
				  if( (od = isValidOpt(optname)) != NULL && ( od->optionSources & OSRC_CONFFILE ) )
					 longOpts.insert(make_pair(optname, optval));
//					 opts.insert(make_pair(optname,optval));
				  else
				  {
					 parse_state = LINE_PARSE_ERR;
					 cerr << "unknown option found in configfile '" << _filename << "', line #" << lineno 
						  << endl;
					 return false;
				  }
				  charptr = buf + len + 1;
				  charptr++; // this will end the do {} while() loop for this line						
			   }
			   else
			   { // garbage on line
				  parse_state = LINE_PARSE_ERR;
				  cerr << "Parse Error in configfile '" << _filename << "', line #" << lineno 
					   << " invalid chars found (garbage after name/value pair. Forgotten '#'?)." << endl;
				  return false;
			   }
			   break;					
			case LINE_PARSE_COMMENT:
			   // exit the do {} while() loop by setting charptr beyond buf+len
			   charptr = buf + len + 1;
			   break;
			default:
			   break;
		 };
	  } while( charptr < buf+len );	
   }
   return true;
}


const OptionDef * MB_Opt::isValidOpt(const string & _opt)
{
   // check if valid longopt, check if it can have parameter, check parameter type
   for(validOptionsIter = validOptions.begin(); validOptionsIter != validOptions.end(); ++validOptionsIter)
	  if( (*validOptionsIter).longOpt == _opt )
		 return &(*validOptionsIter);
   return NULL;
}


const OptionDef * MB_Opt::isValidShortOpt(const char _c)
{
   // check if valid longopt, check if it can have parameter, check parameter type
   for(validOptionsIter = validOptions.begin(); validOptionsIter != validOptions.end(); ++validOptionsIter)
	  if( (*validOptionsIter).shortOpt == _c )
		 return &(*validOptionsIter);
   return NULL;
}

void MB_Opt::listGivenOpts()
{
//    cout << "short opts: " << endl;
//    for(shortOptsIter = shortOpts.begin(); shortOptsIter != shortOpts.end(); ++shortOptsIter)
//    {
// 	  cout << "\t -" << (*shortOptsIter).first << " " << (*shortOptsIter).second << endl;	
//    }

   cout << "long opts: " << endl;
   for(longOptsIter = longOpts.begin(); longOptsIter != longOpts.end(); ++longOptsIter)
   {
	  cout << "\t --" << (*longOptsIter).first << " " << (*longOptsIter).second << endl;	
   }

//    cout << "opts from configfile" << endl;
//    for( fileOptsIter = fileOpts.begin(); fileOptsIter != fileOpts.end(); ++fileOptsIter )
//    {
// 	  cout << "<" << (*fileOptsIter).first << ">=<" << (*fileOptsIter).second << ">" << endl;	
//    }

   cout << "other parameters: " << endl;
   for(nonParamOptsIter = nonParamOpts.begin(); nonParamOptsIter != nonParamOpts.end(); ++nonParamOptsIter)
   {
	  cout << "\t " << (*nonParamOptsIter) << endl;	
   }
}




// string MB_Opt::getNextParam() // non-option parameters
// {
   
// }
