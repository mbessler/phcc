//PropertyIO.h - select()able IO that provides access to PropertyMgr
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

#ifndef _PROPERTYIO_H
#define _PROPERTYIO_H

#include <iosfwd>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
//#include <istream>
//#include <ostream>
#include "FDstream.h"

#include <unistd.h>     // write/read
#include <stdio.h>
//#include <cstring> /* String function definitions */
#include <unistd.h>
#include <fcntl.h>
//#include <errno.h> /* Error number definitions */ 
#include <termios.h> // terminal/serial stuff
#include <stdlib.h>

#include "PropertyMgrCommon.h"
#include "OnChangeObj.h"

#ifdef WITH_LUA
extern "C" 
{
#include "lua.h"
#include "lauxlib.h"
#include "lualib.h"
}

extern lua_State * LuaState;
#endif // WITH_LUA


class PropertyMgr;
class PropertyValue;

// base class for cmdline/socket/XML,... accessors of the property tree
class PropertyIO : public OnChangeObj
{
	public:
		PropertyIO(PropertyMgr * _pmgr);
		virtual ~PropertyIO();
		virtual int getFD(int ** _fdarray)=0;
		virtual bool processInput()=0;
		virtual void onChange(PropertyValue * _pv)=0;
	protected:
		PropertyMgr * pmgr;
};


class PropertyIOcmdline : public PropertyIO
{
	public:
		PropertyIOcmdline(PropertyMgr * _pmgr);
		~PropertyIOcmdline();
		int getFD(int ** _fdarray);
		bool processInput();
		void onChange(PropertyValue * _pv);
	private:
		bool CmdLineParse();
		void putPrompt();
	protected:
		int fd;
		FDistream fdin;
		FDostream fdout;
		std::istream in;
		std::ostream out;
};


class PropertyIO_PHCC : public PropertyIO
{
#define KEYMATRIX_INPUTS 1024
// up to 1024 keys (assuming 32bit ints, we need an array of 32 ints)
#define KEYMATRIX_ARRAYSIZE KEYMATRIX_INPUTS/32

// 33 channels of analog inputs
#define ANALOG_INPUTS 33
#define ANALOG_ARRAYSIZE ANALOG_INPUTS
	public:
		PropertyIO_PHCC(PropertyMgr * _pmgr);
		~PropertyIO_PHCC();
		int getFD(int ** _fdarray);
		bool processInput();
		void onChange(PropertyValue * _pv); // does nothing here, since we cannot set the keys :)
		void serialWrite(unsigned char _byte);
	protected:
		void setLineDiscipline(int flags, int speed);
		int serialRead(unsigned char * _byte);
		int baudrate_check(int speed);
		void process_key_update();
		void process_analog_update();
		void process_keymatrix_map();
		void process_analog_map();
	protected:
		int bytecounter;
		int serialFD;
		int matrix_state[KEYMATRIX_ARRAYSIZE]; // digital inputs buffer
		int an_state[ANALOG_ARRAYSIZE];  // analog inputs buffer
		unsigned char inbuf[256];
		int inbufidx;
		unsigned char package_type;
		enum InStates { WAITFOR_FIRSTBYTE, WAITFOR_NEXTBYTE, WAITFOR_FF_BYTE, WAITFOR_00_BYTE };
		InStates input_state;
};

// class PropertyIO_XMLStream : public PropertyIO
// {
// 	public:
// 		PropertyIO_XMLStream(PropertyMgr * _pmgr);
// 		~PropertyIO_XMLStream();
// 		int getFD(int ** _fdarray);
// 		void processInput();
// 		void onChange(PropertyValue * _pv); // can send back events in form of XML stream data
// 	protected:
// 		void XMLparse();
		
// };

#endif // _PROPERTYIO_H
