//Switches.h - Classes representing hardware switches in software in C++
//
// This file is part of X11GC.
//
//  X11GC is a Glass Cockpit Software Suite for X11,
//  which does NOT use OpenGL but relies only on xlib.
//  Copyright (C) 2001 Manuel Bessler
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
#ifndef _SWITCHES_H
#define _SWITCHES_H

//using namespace std;
#include <iostream>
#include <string>

class PICKeyMatrixSwitch
{
	public:
		PICKeyMatrixSwitch(int _type, std::string * _path, int _startrow, int _startcol, 
								 int _endrow=-1, int _endcol=-1)
			: type(_type),
			  path(_path),
  			  startrow(_startrow),
			  endrow(_endrow),
			  startcolumn(_startcol),
			  endcolumn(_endcol)
		{
			std::cout << __PRETTY_FUNCTION__ 
				  << "type=" << type << " "
				  << "path=<" << *path << "> "
				  << startrow << "/" << startcolumn << " "
				  << endrow << "/" << endcolumn
				  << endl; 
		}
		~PICKeyMatrixSwitch()
		{
			delete path;
		}
	private:
		int type;
		std::string * path;
		int startrow;
		int endrow;
		int startcolumn;
		int endcolumn;
};

#endif /* _SWITCHES_H */
