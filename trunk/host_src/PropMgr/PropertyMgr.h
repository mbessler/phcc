//PropertyMgr.h - the primary include for PropertyMgr
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

#ifndef _PROPERTYMGR_H
#define _PROPERTYMGR_H

#include <ctime>    // for select()
//#include <sys/types.h>   // for select()
#include <unistd.h>      // for select()

#include "PropertyMgrCommon.h"
#include "PropertyBranch.h"
#include "PropertyLeaf.h"
#include "PropertyValue.h"
#include "OnChangeObj.h"
#include "PropertyIO.h"
#include "Timer.h"


class RunningFlagHandler : public OnChangeObj  // only purpose for this class is to change PropertyMgr
{
	public:
		void onChange(PropertyValue * _pv);
};

class PropertyMgr
{
	public:
		PropertyMgr();
		~PropertyMgr();
		PropertyValue * get(std::string path);  // returns NULL if it does not exist
		PropertyValue * addLeaf(std::string path); // returns created PropertyValue
		void addIOHandler(PropertyIO * _propiohandler);
		void addTimerOneshot(Timer * _timer);
		void addPeriodicTimer(Timer * _timer);
		void run();  // this is where we wait for something happen, poll for events in DisplayMgr and PropertyAccess* modules
		void print() const;
		void printOn(std::ostream& o) const;
		friend std::ostream & operator<< (std::ostream & o, const PropertyMgr & _mgr);
		friend std::ostream & operator<< (std::ostream & o, const PropertyMgr * _mgr);
	protected:
		std::vector<std::string> parsePath(std::string path) const;
	protected:
		PropertyBranch * root;
		std::list<PropertyIO *> IOHandlers;
		std::list<PropertyIO *>::iterator ioiter;
		static bool running;
		RunningFlagHandler runningflaghandler;
		TimerQueue tq;
		friend class RunningFlagHandler;
};


#endif // _PROPERTYMGR_H
