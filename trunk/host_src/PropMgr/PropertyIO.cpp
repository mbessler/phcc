//PropertyIO.cpp - select()able IO that provides access to PropertyMgr
//
// This file is part of X11GC.
//
//  X11GC is a Glass Cockpit Software Suite for X11,
//  which does NOT use OpenGL but relies only on xlib.
//  Copyright (C) 2003 Manuel Bessler
//
//  The full text of the legal notices is contained in the file called
//  COPYING, included with this distribution.
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the GNU General Public License
//  as published by the Free Software Foundation; either version 2
//  of the License, or (at your option) any later version.
// 
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program; if not, write to the Free Software
//  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
//
// This is for emacs
// Local variables:
// mode: c++
// c-basic-offset: 3
// tab-width: 3
// End:
//
//  $Id: $
////////////////////////////////////////////////////////////////////////

#include "PropertyIO.h"
#include "PropertyMgr.h"
#include "PropertyValue.h"
#include "FDstream.h"

using namespace std;

PropertyIO::PropertyIO(PropertyMgr * _pmgr)
   : pmgr(_pmgr)
{

}

PropertyIO::~PropertyIO()
{
}

///////////////////////////////////////

PropertyIOcmdline::PropertyIOcmdline(PropertyMgr * _pmgr)
   : PropertyIO(_pmgr),
	 fd(open("/dev/stdin", O_RDWR)),
	 fdin(FDistream(fd, 128)),
	 fdout(FDostream(fd, 128)),
	 in(&fdin),
	 out(&fdout)
{
   putPrompt();
}

PropertyIOcmdline::~PropertyIOcmdline()
{
   close(fd);
}

int PropertyIOcmdline::getFD(int ** _fdarray)
{
//   cout << __PRETTY_FUNCTION__ << " my FD is: " << inout.rdbuf()->fd() << endl;
//   return inout.rdbuf()->fd();
   *_fdarray = &fd;     // let the _fdarray parameter point to fd
   return 1;            // and return 1 as in "returning one fd in array"
//   return fd;
}

bool PropertyIOcmdline::processInput()
{
   if( ! CmdLineParse() )
    return false;
   putPrompt();
   return true;
}


void PropertyIOcmdline::onChange(PropertyValue * _pv)
{
   cout << __PRETTY_FUNCTION__ << " notification onChange of " << _pv->getPath() << " = " << _pv << endl;
}

void PropertyIOcmdline::putPrompt()
{
   string prompt = "PropertyMgr";
   out << prompt << "# " << flush;
}

bool PropertyIOcmdline::CmdLineParse()
{
#define bufferlen 100
   char buffer[bufferlen];
	if( ! in.getline(buffer, bufferlen) || in.eof() )
		return false;
#undef bufferlen
	string strbuffer = string(buffer);
	vector<string> input;
	unsigned int start = strbuffer.find(' '); // find first
	unsigned int end;
	if( start == strbuffer.npos )
		input.push_back(strbuffer);
	else
		input.push_back(string(strbuffer.substr(0, start)));
	while( start != strbuffer.npos )
	{
		++start;                 // move past first occurence
		end = strbuffer.find(' ', start); // find next
		input.push_back(string(strbuffer.substr(start, end-start)));
		start = end;
	}
	//////////////////////////////////////////////
	if( input[0] == "stop" )
	{
		out << "+OK FOUND STOP, quitting app!!!" << endl;
		PropertyValue * pvrun = pmgr->get("/running");
		pvrun->set(false);
	}
	else if( input[0] == "get" && input.size() == 2 )
	{
		PropertyValue * pv = pmgr->get(input[1]);
		if( pv )
			out << "+OK " << pv->getPath() << " = " << pv << endl;
		else
			out << "-ERR node not found!" << endl;
	}
	else if( input[0] == "set" && input.size() == 3 )
	{
		PropertyValue * pv = pmgr->get(input[1]);
		if( ! pv )
		{
			out << "-ERR path does not exist" << endl;
			return true;
		}
		enum PropType t = pv->getType();
		switch( t )
		{
			case PROPTYPE_NOTDEF:
			   out << "-ERR type not yet set, try using the 'new' command" << endl;
			   break;
			default:
				if( pv->set(input[2], t) )
				   out << "+OK set successful" << endl;
				else
				   out << "-ERR value specified cannot be represented with this type" << endl;
		}
	}
	else if( input[0] == "new" && input.size() == 4 )
	{
		enum PropType newtype = PROPTYPE_NOTDEF;
		// first check type flag
		if( input[3] == "i" )
			newtype = PROPTYPE_INT;
		else if( input[3] == "f" )
			newtype = PROPTYPE_FLOAT;
		else if( input[3] == "s" )
			newtype = PROPTYPE_STRING;
		else if( input[3] == "b" )
			newtype = PROPTYPE_BOOL;
		else
		{
			out << "-ERR unknown type specified. try one of 'i', 'f', 's', or 'b'" << endl;
			return true;
		}
		PropertyValue * pv = pmgr->get(input[1]);
		if( pv ) // leaf exists, check if its type is NOTDEF, otherwise inform to use set instead
		{
			if( pv->getType() != PROPTYPE_NOTDEF )
			{
				out << "-ERR path exists and type is already set, use the 'set' command instead" << endl;
				return true;
			}
		}
		else
		{
			pv = pmgr->addLeaf(input[1]);
		}
		if( pv->set(input[2], newtype) )
			out << "+OK set successful" << endl;
		else
			out << "-ERR type error" << endl;
	}
	else if( input[0] == "list" )
	{
		out << pmgr;
	}
#ifdef WITH_LUA
	else if( input[0] == "lua" && input.size() == 2 )
	{
	   if( lua_dofile(LuaState, input[1].c_str()) != 0 )
	   {
		  out << "-ERR error while loading lua script" << endl;
	   }
	}
#endif // WITH_LUA
	else if( input[0] == "help" )
	{
	   out << " Command/parameters           Description" << endl
		   << " list                         dump all nodes of the property tree" << endl
		   << " get <path>                   show node 'path'" << endl
		   << " set <path> <value>           sets node 'path' to new value 'value'" << endl
		   << " new <path> <value> <type>    adds a new node 'path' with 'value' of 'type'" << endl
#ifdef WITH_LUA
		   << " lua <file>                   load and run Lua code in file" << endl
#endif // WITH_LUA
		   << " stop                         quit the app, equivalent to 'set /running false'" << endl
		   << " ---- Notes:" << endl
		   << "      types: 'i' for int, 'f' for float, 's' for string, 'b' for bool" << endl;
	}
	else
	{
	   out << "-ERR unknown command and/or parameter error, use 'help' for help" << endl;
	}
	return true;
}


//////////////////////////////////////////////////////////////////////


PropertyIO_PicKeyMatrix::PropertyIO_PicKeyMatrix(PropertyMgr * _pmgr)
   : PropertyIO(_pmgr)
{
   bytecounter = 0;
   for(int i=0; i< 16; ++i )
	  matrix_state[i] = 0;
   serialFD = open("/dev/ttyS0", O_RDWR | O_NOCTTY | O_NONBLOCK);
   if( serialFD == -1 )
   {
	  cerr << "Could not open serial port for connection to PicKeyMatrix!" << endl;
   }
   setLineDiscipline();
   fcntl(serialFD, F_SETFL, 0);
}

PropertyIO_PicKeyMatrix::~PropertyIO_PicKeyMatrix()
{
   close(serialFD);
}

int PropertyIO_PicKeyMatrix::getFD(int ** _fdarray)
{
   *_fdarray = &serialFD;
   return 1;
//   return serialFD;
}

bool PropertyIO_PicKeyMatrix::processInput()
{  // format: row/column:[0|1]CRLF  eg: 3/0:1\r\n
   cout << __PRETTY_FUNCTION__ << endl;
   unsigned char byte;
   int row=0, column=0, state_on=0;
   bool validdata = true;
   fcntl(serialFD, F_SETFL, FNDELAY);
   while(serialRead(&byte) >= 0 )
   {
//	  cout << __PRETTY_FUNCTION__ << " returned " << byte << "  validdata=" << validdata << endl;
	  switch( bytecounter )
	  {
		 case 0:
			if( byte >= '0' && byte <= '7' )
			   row = byte-48;
			else 
			   validdata = false;
			break;
		 case 1:
			if( byte != '/' )
			   validdata = false;
			break;
		 case 2:
			if( byte >= '0' && byte <= '7' )
			   column = byte-48;
			else
			   validdata = false;
			break;
		 case 3:
			if( byte != ':' )
			   validdata = false;
			break;
		 case 4:
			if( byte == '0' || byte == '1' )
			   state_on = byte-48;
			else
			   validdata = false;
			break;
		 case 5:
			if( byte != '\r' )
			   validdata = false;
			break;
		 case 6:
			if( byte != '\n' )
			   validdata = false;
			else
			   bytecounter = -1; // ready for next packet/line
			break;
		 default:
			validdata = false;
	  }
	  ++bytecounter;
   }
   if( validdata )
   {
	  cout << "switch " << row << "/" << column << " turned ";
	  if( state_on )
	  {
		 matrix_state[row] |= ( 1 << column ); // OR to turn bit on
		 cout << "on" << endl;
	  }
	  else
	  {
		 matrix_state[row] ^= (0 << column ); //XOR to turn bit off
		 cout << "off" << endl;
	  }
   }
   else
   {
	  cout << "invalid data" << endl;
	  bytecounter = 0; // reset after error for next packet/line
   }
   fcntl(serialFD, F_SETFL, 0);
   
   return true;  //TODO: needs EOF/connection closed handling to return false
}

void PropertyIO_PicKeyMatrix::onChange(PropertyValue * _pv)
{// does nothing here, since we cannot set the keys :)
}

void PropertyIO_PicKeyMatrix::setLineDiscipline()
{
   struct termios t;

   tcgetattr(serialFD, &t);
   t.c_cflag = CS8 | CREAD | HUPCL | CLOCAL;
   t.c_iflag = IGNBRK | IGNPAR;
   t.c_oflag = 0;
   t.c_lflag = 0;
//   t.c_cc[VMIN ] = 1;
//   t.c_cc[VTIME] = 0;

   cfsetispeed(&t, B19200);
   cfsetospeed(&t, B19200);

   tcsetattr(serialFD, TCSANOW, &t);
}

int PropertyIO_PicKeyMatrix::serialRead(unsigned char * _byte)
{
   return read(serialFD, _byte, 1); // returns 0 if fcntl(serialFD, F_SETFL, FNDELAY) set
}

void PropertyIO_PicKeyMatrix::serialWrite(unsigned char _byte)
{
   write(serialFD, &_byte, 1);
   return;
}


