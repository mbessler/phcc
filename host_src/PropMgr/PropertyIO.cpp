//PropertyIO.cpp - select()able IO that provides access to PropertyMgr
//
// This file is part of X11GC.
//
//  X11GC is a Glass Cockpit Software Suite for X11,
//  which does NOT use OpenGL but relies only on xlib.
//  Copyright (C) 2003-2005 by Manuel Bessler
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

PropertyIO_PHCC::PropertyIO_PHCC(PropertyMgr * _pmgr)
   : PropertyIO(_pmgr), package_type(0), input_state(WAITFOR_FIRSTBYTE)
{
   int flags=CS8;
   int baudrate = -1;
   int speed = 115200;
   char * dev = "/dev/ttyS0";

   serialFD = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
   if( serialFD < 0 )
   {
	  cerr << "Could not open serial port for connection to PHCC!" << endl;
//	  return -1;
	  exit(-1);
   }

   if( (baudrate = baudrate_check(speed)) == -1 )
   {
	  cerr << "ERROR, invalid speed for PHCC: " << speed << endl;
//	  return -1;
	  exit(-1);
   }

   tcflush(serialFD, TCIOFLUSH);
   setLineDiscipline(flags, baudrate);
   fcntl(serialFD, F_SETFL, 0);

   //// setup datastructures
   int i;
   for(i=0; i<KEYMATRIX_ARRAYSIZE; ++i)
	  matrix_state[i] = 0;
   for(i=0; i<ANALOG_ARRAYSIZE; ++i)
	  an_state[i] = 0;
   inbufidx=0;
   for(i=0; i<256; ++i)
	  inbuf[i] = 0;
}

PropertyIO_PHCC::~PropertyIO_PHCC()
{
   close(serialFD);
}
int PropertyIO_PHCC::getFD(int ** _fdarray)
{
   *_fdarray = &serialFD;
   return 1;
//   return serialFD;
}

bool PropertyIO_PHCC::processInput()
{
   unsigned char byte;
   fcntl(serialFD, F_SETFL, FNDELAY);  // set filedescriptor to non-blocking
   while(serialRead(&byte) >= 0 )
   {
	  // what state are we in ?
	  switch(input_state)
	  {
		 case WAITFOR_FIRSTBYTE:
			// extract the three MSBs. these indicate the package type
			package_type = ((byte>>5)&0x03);
			switch( package_type)
			{
			   case 0x0: // all-bits-zero-byte
				  break;
			   case 0x1: // key matrix update
				  inbuf[inbufidx++]=byte;
				  input_state = WAITFOR_NEXTBYTE;
				  break;
			   case 0x2: // analog update
				  inbuf[inbufidx++]=byte;
				  input_state = WAITFOR_NEXTBYTE;
				  break;
			   case 0x3: // I2C packet
				  // not implemented yet
				  break;
			   case 0x4: // keymatrix full bitmap
				  inbuf[inbufidx++]=byte;
				  input_state = WAITFOR_NEXTBYTE;
				  break;
			   case 0x5: // analog all axes dump
				  inbuf[inbufidx++]=byte;
				  input_state = WAITFOR_NEXTBYTE;
				  break;
			   case 0x6: // [unused]
				  break;
			   case 0x7: // all-bits-one-byte
				  break;
			   default:  // nothing
				  break;
			}
			break;
		 case WAITFOR_NEXTBYTE:
			switch( package_type)
			{
			   case 0x1: // key matrix update
				  inbuf[inbufidx++]=byte;
				  input_state = WAITFOR_FIRSTBYTE;
				  process_key_update();
				  inbufidx = 0;
				  break;
			   case 0x2: // analog update
				  inbuf[inbufidx++]=byte;
				  if( inbufidx == 3)
				  {
					 input_state = WAITFOR_FIRSTBYTE;
					 process_analog_update();
					 inbufidx = 0;
				  }
				  break;
			   case 0x4: // keymatrix full bitmap
				  inbuf[inbufidx++]=byte;
				  if( inbufidx >= 131)
				  {
					 input_state = WAITFOR_FIRSTBYTE;
					 process_keymatrix_map();
					 inbufidx = 0;
				  }
				  break;
			   case 0x5: // analog all axes dump
				  inbuf[inbufidx++]=byte;
				  if( inbufidx >= 48)
				  {
					 input_state = WAITFOR_FIRSTBYTE;
					 process_analog_map();
					 inbufidx = 0;
				  }
				  break;
			   default:  // something else will reset to WAITFOR_FIRSTBYTE
				  input_state = WAITFOR_FIRSTBYTE;
				  inbufidx = 0;
			}
			break;
		 case WAITFOR_FF_BYTE:
			break;
		 case WAITFOR_00_BYTE:
			break;
		 default:
			break;
	  }
   }
   fcntl(serialFD, F_SETFL, 0);  // set filedescriptor back to blocking
   return true;  //TODO: needs EOF/connection closed handling to return false
}

void PropertyIO_PHCC::process_key_update()
{
   cout << "received an key update: 0x" << 
	  hex << setw(2) << ((inbuf[0]>>1)&0x03) <<
	  hex << setw(2) << ((inbuf[0]<<7)&0x80)+((inbuf[1]>>1)&0x7F) <<
	  "=" << (inbuf[1]&0x01) << endl;
}

void PropertyIO_PHCC::process_analog_update()
{
//   printf("channel %d=0x%02x%02x\n", (inbuf[1]>>2), (inbuf[1]&0x03), (inbuf[2]));
  cout << "received an analog update: channel " << 
	 dec << setw(2) << ((inbuf[1]>>2)&0x3F) << "=0x" <<
	  hex << setw(2) << (inbuf[1]&0x03) <<
	  hex << setw(2) << (inbuf[2]&0xFF) << endl;
}

void PropertyIO_PHCC::process_keymatrix_map()
{
   cout << "received full keymatrix" << endl;
   for(int i=0; i<KEYMATRIX_ARRAYSIZE; ++i)
   {
	  matrix_state[i] = ((int(inbuf[4+i*4]) << 24) | (int(inbuf[3+i*4]) << 16) |
						 (int(inbuf[2+i*4]) << 8) | (int(inbuf[1+i*4])));
   }
}

void PropertyIO_PHCC::process_analog_map()
{
   cout << "received full analog map" << endl;
   for(int i=0; i<ANALOG_INPUTS; ++i)
   {
	  an_state[i] = int(inbuf[1+i*4+i*1]);
	  an_state[i] |= (int(inbuf[((i/4)+1)*5]) >> ((i%4)*2));
   }
}


void PropertyIO_PHCC::onChange(PropertyValue * _pv)
{// does nothing here, since we cannot set the keys :)
}

void PropertyIO_PHCC::setLineDiscipline(int flags, int speed)
{
   struct termios t;

   tcgetattr(serialFD, &t);

   t.c_cflag = flags | CS8 | CREAD | HUPCL | CLOCAL | CRTSCTS;
   t.c_iflag = IGNBRK | IGNPAR;
   t.c_oflag = 0;
   t.c_lflag = 0;

   cfsetispeed(&t, speed);
   cfsetospeed(&t, speed);

   tcsetattr(serialFD, TCSAFLUSH, &t);
}

int PropertyIO_PHCC::serialRead(unsigned char * _byte)
{
   return read(serialFD, _byte, 1); // returns 0 if fcntl(serialFD, F_SETFL, FNDELAY) set
}

void PropertyIO_PHCC::serialWrite(unsigned char _byte)
{
   write(serialFD, &_byte, 1);
   return;
}

int PropertyIO_PHCC::baudrate_check(int speed)
{
        switch(speed)
        {
                case 230400: return B230400;
                case 115200: return B115200;
                case 57600: return B57600;
                case 38400: return B38400;
                case 19200: return B19200;
                case 9600: return B9600;
                case 4800: return B4800;
                case 2400: return B2400;
                case 1800: return B1800;
                case 1200: return B1200;
                case 600: return B600;
                case 300: return B300;
                case 200: return B200;
                case 150: return B150;
                case 134: return B134;
                case 110: return B110;
                case 75: return B75;
                case 50: return B50;
                default: return -1;
        }
}


//////////////////////////////////////////////////////////////////////

