//Timer.h - A countdown Timer for PropertyMgr
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

#ifndef _TIMER_H
#define _TIMER_H

#include <iostream>
#include <queue>
#include <string>
#include <unistd.h>

#include "PropertyMgrCommon.h"
#include "TimeStamp.h"

class PropertyMgr;

class Timer
{
	public:
		Timer(PropertyMgr * _pmgr, long int _seconds, long int _usec, 
				std::string _modifypath, int _modifyvalue, bool _periodic = false);
		Timer(PropertyMgr * _pmgr, long int _seconds, long int _usec, 
				std::string _modifypath, float _modifyvalue, bool _periodic = false);
		Timer(PropertyMgr * _pmgr, long int _seconds, long int _usec, 
				std::string _modifypath, std::string _modifyvalue, bool _periodic = false);
		Timer(PropertyMgr * _pmgr, long int _seconds, long int _usec, 
				std::string _modifypath, bool _modifyvalue, bool _periodic = false);
		~Timer();
		TimeStamp * getTimeStamp();
		void action();
		bool isPeriodic();
		long int getSecs();
		long int getuSecs();
	protected:
	private:
		PropertyMgr * pmgr;
		TimeStamp ts;
		std::string modifypath;
		int modifyvalue_int;
		float modifyvalue_float;
		std::string modifyvalue_string;
		bool modifyvalue_bool;
		enum PropType modifytype;
		bool periodic;
};

struct TimerSort
{ 
	  bool operator()(Timer * t1, Timer * t2)
	  {
		  TimeStamp * ts1 = t1->getTimeStamp();
		  TimeStamp * ts2 = t2->getTimeStamp();
		  if( ts2->getEpoch() == ts1->getEpoch() )
			  return ts2->getMicroSecond() < ts1->getMicroSecond();
		  else
			  return ts2->getEpoch() < ts1->getEpoch(); 
	  } 
};

class TimerQueue : public std::priority_queue<Timer *, std::vector<Timer*>, TimerSort>
{
   public: 
		std::vector<Timer *> & impl()
		{
			return c; 
		}
		void getNextEvent(long int * sec, long int * usec)
		{
			if( ! empty() )
			{
				TimeStamp now;
				const TimeStamp * ts = top()->getTimeStamp();
				*sec = ts->getEpoch() - now.getEpoch(); 
				*usec = ts->getMicroSecond() - now.getMicroSecond();
				// adjust for carry
				if( *usec < 0 )
				{
					*sec -= 1;
					*usec = 1000000L + *usec;
				}
				if( *sec < 0 )
					*sec = 0;
			}
			else
			{
					*sec -= 0;
					*usec = 100 MILLISEC;
			}
		}
};



#endif // _TIMER_H
