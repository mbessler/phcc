//TimeStamp.h - as the name says: a timestamp class
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

#ifndef _TIMESTAMP_H
#define _TIMESTAMP_H

#include <iostream>
#include <queue>
#include <ctime>
#include <sys/time.h>

#define MILLISEC * 1000L

class TimeStamp
{
	public:
		TimeStamp();
		TimeStamp(const TimeStamp & _ts); // copy constructor
		TimeStamp(long int sec, long int usec); // sets stamp to a future time, ie. now+sec+usec
		~TimeStamp();
		friend std::ostream & operator<< (std::ostream & o, const TimeStamp & _ts);
		friend std::ostream & operator<< (std::ostream & o, const TimeStamp * _ts);
		void ResetTime();
		int getYear() const;
		int getMonth() const;
		int getDay() const;
		int getHour() const;
		int getMinute() const;
		int getSecond() const;
		long int getMicroSecond() const;
		long int getEpoch() const;
		void setNow() ;
		long int getDifferenceSec(TimeStamp * _ts);
		long int getDifferenceMicroSec(TimeStamp * _ts);
		void print();
	protected:
		void printOn(std::ostream& o) const;
	private:
		long int epoch_seconds;
		long int usec;
		struct tm * zulu;
		int year;
		int month;
		int day;
		int hour;
		int minute;
		int second;
		long int duration_sec;
		long int duration_usec;
};

struct TimeStampSort
{ 
	  bool operator()(const TimeStamp * ts1, const TimeStamp * ts2)
	  {
		  if( ts2->getEpoch() == ts1->getEpoch() )
			  return ts2->getMicroSecond() < ts1->getMicroSecond();
		  else
			  return ts2->getEpoch() < ts1->getEpoch(); 
	  } 
};

class TimeStampQueue : public std::priority_queue<TimeStamp *, std::vector<TimeStamp*>, TimeStampSort>
{
   public: 
		std::vector<TimeStamp *> & impl()
		{
			return c; 
		} 
};


#endif // _TIMESTAMP_H
