//Timer.cpp - A countdown Timer for PropertyMgr
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

#include "Timer.h"
#include "PropertyMgr.h"

using namespace std;


Timer::Timer(PropertyMgr * _pmgr, long int _seconds, long int _usec, string _modifypath, int _modifyvalue,
				 bool _periodic)
	: pmgr(_pmgr),
	  ts(_seconds, _usec),
	  modifypath(_modifypath),
	  modifyvalue_int(_modifyvalue),
	  modifytype(PROPTYPE_INT),
	  periodic(_periodic)
{}

Timer::Timer(PropertyMgr * _pmgr, long int _seconds, long int _usec, string _modifypath, float _modifyvalue,
				 bool _periodic)
	: pmgr(_pmgr),
	  ts(_seconds, _usec),
	  modifypath(_modifypath),
	  modifyvalue_float(_modifyvalue),
	  modifytype(PROPTYPE_FLOAT),
	  periodic(_periodic)
{}

Timer::Timer(PropertyMgr * _pmgr, long int _seconds, long int _usec, string _modifypath, string _modifyvalue,
				 bool _periodic)
	: pmgr(_pmgr),
	  ts(_seconds, _usec),
	  modifypath(_modifypath),
	  modifyvalue_string(_modifyvalue),
	  modifytype(PROPTYPE_STRING),
	  periodic(_periodic)
{}

Timer::Timer(PropertyMgr * _pmgr, long int _seconds, long int _usec, string _modifypath, bool _modifyvalue,
				 bool _periodic)
	: pmgr(_pmgr),
	  ts(_seconds, _usec),
	  modifypath(_modifypath),
	  modifyvalue_bool(_modifyvalue),
	  modifytype(PROPTYPE_BOOL),
	  periodic(_periodic)
{}

Timer::~Timer()
{
//	cout << __PRETTY_FUNCTION__ << endl;
}

void Timer::action()
{
	PropertyValue * pv = pmgr->get(modifypath);
	if( periodic )
	{
		switch(modifytype)
		{
			case PROPTYPE_INT:
				pv->set(pv->getInt() + modifyvalue_int);
				break;
			case PROPTYPE_FLOAT:
				pv->set(pv->getFloat() + modifyvalue_float);
				break;
			case PROPTYPE_STRING:
				pv->set(modifyvalue_string); // dunno what would be the best to do with strings...
				break;
			case PROPTYPE_BOOL:
				pv->set( ! pv->getBool() ); // special case: bool gets inverted
				break;
			case PROPTYPE_NOTDEF:  // make -Wall happy
			default:
				//nothing
				break;
		}
	}
	else
	{
		switch(modifytype)
		{
			case PROPTYPE_INT:
				pv->set(modifyvalue_int);
				break;
			case PROPTYPE_FLOAT:
				pv->set(modifyvalue_float);
				break;
			case PROPTYPE_STRING:
				pv->set(modifyvalue_string);
				break;
			case PROPTYPE_BOOL:
				pv->set(modifyvalue_bool);
				break;
			case PROPTYPE_NOTDEF:  // make -Wall happy
			default:
				//nothing
				break;
		}
	}
}

bool Timer::isPeriodic()
{
	return periodic;
}

TimeStamp * Timer::getTimeStamp()
{
	return &ts;
}



