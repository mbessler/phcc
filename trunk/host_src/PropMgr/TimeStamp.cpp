//TimeStamp.cpp - as the name says: a timestamp class
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

#include "TimeStamp.h"
#include <cstdlib>

using namespace std;

TimeStamp::TimeStamp() 
   : zulu(NULL),
	 duration_sec(0),
	 duration_usec(0)     // epoch_seconds(time(NULL)),
{
   struct timeval now;
   gettimeofday(&now, NULL);
   epoch_seconds = now.tv_sec;
   usec = now.tv_usec;
   struct tm * tmptm = gmtime(&epoch_seconds);  // DO NOT free() pointer returned by gmtime()
   if( (zulu = (struct tm *)malloc(sizeof(struct tm)) ) == NULL )
	  abort();
   memcpy(zulu, tmptm, sizeof(struct tm));
}

TimeStamp::TimeStamp(const TimeStamp & _ts) // copy constructor
   : 	epoch_seconds(_ts.epoch_seconds),
		usec(_ts.usec),
		year(_ts.year),
		month(_ts.month),
		day(_ts.day),
		hour(_ts.hour),
		minute(_ts.minute),
		second(_ts.second),
		duration_sec(0),
		duration_usec(0)
{
	if( (zulu = (struct tm *)malloc(sizeof(struct tm)) ) == NULL )
	   abort();
	memcpy(zulu, _ts.zulu, sizeof(struct tm));
}

TimeStamp::TimeStamp(long int _sec, long int _usec)  // set time relative to now
{
   struct timeval now;
   gettimeofday(&now, NULL);
   epoch_seconds = now.tv_sec + _sec;
   usec = now.tv_usec + _usec;
   // for periodic, save original values:
   duration_sec = _sec;
   duration_usec = _usec;
   if( usec >= 1000000L ) // wrap to next second ?
   {
	  epoch_seconds += usec / 1000000L;
	  usec = usec % 1000000L;
   }
   else
	  usec = now.tv_usec + _usec;
   struct tm * tmptm = gmtime(&epoch_seconds);  // DO NOT free() pointer returned by gmtime()
   if( (zulu = (struct tm *)malloc(sizeof(struct tm)) ) == NULL )
	  abort();
   memcpy(zulu, tmptm, sizeof(struct tm));
}


TimeStamp::~TimeStamp()
{
	if( zulu != NULL )
		free(zulu);
}

void TimeStamp::ResetTime() // allows making periodic timers
{
   if( duration_sec == 0 && duration_usec == 0)
	  return; // zero time, no use in rescheduling timer
   
   struct timeval now;
   gettimeofday(&now, NULL);
   epoch_seconds = now.tv_sec + duration_sec;
   usec = now.tv_usec + duration_usec;
   if( usec >= 1000000L ) // wrap to next second ?
	{
		epoch_seconds += usec / 1000000L;
		usec = usec % 1000000L;
	}
	else
		usec = now.tv_usec + duration_usec;
 	struct tm * tmptm = gmtime(&epoch_seconds);  // DO NOT free() pointer returned by gmtime()
	if( zulu == NULL )
	   if( (zulu = (struct tm *)malloc(sizeof(struct tm)) ) == NULL )
		  abort();
 	memcpy(zulu, tmptm, sizeof(struct tm));
}



int TimeStamp::getYear() const { return 1900+zulu->tm_year; }

int TimeStamp::getMonth() const { return zulu->tm_mon; }

int TimeStamp::getDay() const { return zulu->tm_mday; }

int TimeStamp::getHour() const { return zulu->tm_hour; }

int TimeStamp::getMinute() const { return zulu->tm_min; }

int TimeStamp::getSecond() const { return zulu->tm_sec; }

long int TimeStamp::getMicroSecond() const { return usec; }

long int TimeStamp::getEpoch() const { return epoch_seconds; }

void TimeStamp::setNow() 
{ 
//	epoch_seconds = time(NULL);
	struct timeval now;
	gettimeofday(&now, NULL);
	epoch_seconds = now.tv_sec;
	usec = now.tv_usec;
	struct tm * tmptm = gmtime(&epoch_seconds);
	if( zulu == NULL )
		if( (zulu = (struct tm *)malloc(sizeof(struct tm)) ) == NULL )
			abort();
	memcpy(zulu, tmptm, sizeof(struct tm));
}

long int TimeStamp::getDifferenceSec(TimeStamp * _ts)
{
	long int difference = _ts->epoch_seconds - epoch_seconds;
	if( difference < 0 )
		difference = -difference;
	return difference;
}

long int TimeStamp::getDifferenceMicroSec(TimeStamp * _ts)
{
	long int difference = (/*_ts->epoch_seconds*1000000L +*/ _ts->usec) - (/*epoch_seconds*1000000L +*/ usec);
	if( difference < 0 )
		difference = -difference;
	return difference;
}

void TimeStamp::print()
{
	printOn(cout);
}

void TimeStamp::printOn(ostream& o) const
{
   o << 1900+zulu->tm_year << "-" ;
	o.width(2);
	o.fill('0');
	o << zulu->tm_mon << "-";
	o.width(2);
	o << zulu->tm_mday << " ";
	o.width(2);
	o << zulu->tm_hour << ":" ;
	o.width(2);
	o << zulu->tm_min << ":" ;
	o.width(2);
	o << zulu->tm_sec << ".";
	o.width(6);
	o << usec;
}

ostream & operator<< (ostream & o, const TimeStamp & _ts)
{
	_ts.printOn(o);
	return o;
}

ostream & operator<< (ostream & o, const TimeStamp * _ts)
{
	_ts->printOn(o);
	return o;
}
