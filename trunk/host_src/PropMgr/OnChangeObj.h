//OnChangeObj.h - a pure virtual base class used for callbacks
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

#ifndef _ONCHANGEOBJ_H
#define _ONCHANGEOBJ_H

class PropertyValue;

class OnChangeObj
{
	public:
		virtual void onChange(PropertyValue * _pv)=0;
};

#endif // _ONCHANGEOBJ_H
