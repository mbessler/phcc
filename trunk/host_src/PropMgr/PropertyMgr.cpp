//PropertyMgr.cpp - Access methods to the PropertyMgr
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

#include "PropertyMgr.h"

using namespace std;

bool PropertyMgr::running = true;

PropertyMgr::PropertyMgr() 
   : root(new PropertyBranch("[root]", NULL, true)), runningflaghandler(RunningFlagHandler()),
	 tq(TimerQueue())
{
	PropertyValue * pv;
	(pv = addLeaf("/running"))->set(true);
	pv->registerOnChange(&runningflaghandler);  // this guy waits for someone to set /running=false 
	                                            // when this happens, he sets PropertyMgr::running=false;
}

PropertyValue * PropertyMgr::get(string path)  // returns NULL if it does not exist
{
	vector<string> l = parsePath(path);
	int size = l.size();
	PropertyBranch * curbranch = root;
	for( int i=0; i < size-1; ++i )
	{
		curbranch = curbranch->findBranch(l[i]);
		if( curbranch == NULL )
			return NULL;  // not found
	}
	PropertyLeaf * leaf = curbranch->findLeaf(l[size-1]);
	if( leaf == NULL )
		return NULL;
	return leaf->getValue();
}

PropertyMgr::~PropertyMgr()
{
	delete root;
}


void PropertyMgr::addIOHandler(PropertyIO * _propiohandler)
{
   IOHandlers.push_back(_propiohandler);
}

void PropertyMgr::addTimerOneshot(Timer * _timer)
{
   tq.push(_timer);
}

void PropertyMgr::addPeriodicTimer(Timer * _timer)
{
   tq.push(_timer);
}


void PropertyMgr::run()
{
	running = true;  // running has to be manipulated somehow via a property in the tree
	TimeStamp now;
	fd_set FDset;
	struct timeval tv;
	tq.push( new Timer(this, 99999999, 0, "/running", true) );
	while( running )
	{
		int maxfd=0;
		FD_ZERO(&FDset);
		for( ioiter=IOHandlers.begin(); ioiter!=IOHandlers.end(); ++ioiter )
		{
			int fdtemp; // needed since FD_SET is a macro
			int * fdarray;
			int no_of_desc = (*ioiter)->getFD(&fdarray);
			for( int i=0; i<no_of_desc; i++ )
			{
			   fdtemp = fdarray[i];
			   FD_SET(fdtemp, &FDset);
			   if( fdtemp > maxfd )
				  maxfd = fdtemp;
			}
		}
		tq.getNextEvent(&tv.tv_sec, &tv.tv_usec);
		if( select(maxfd+1, &FDset, NULL, NULL, &tv) )
		{
			for( ioiter=IOHandlers.begin(); ioiter!=IOHandlers.end(); ++ioiter )
			{
			   int * fdarray;
			   int no_of_desc = (*ioiter)->getFD(&fdarray);
			   for( int i=0; i<no_of_desc; i++ )
			   {
				  int fdtemp = fdarray[i];
				  if( FD_ISSET(fdtemp, &FDset) )
					 if( ! (*ioiter)->processInput() )  
					 {  // processInput methods return false on EOF/connection closed/etc
					    cout << "[connection closed to " << *ioiter << "]" << endl;
					    delete(*ioiter);
					    IOHandlers.erase(ioiter);
					}
			   }
			}
		}
		while( ! tq.empty() )
		{
		   const TimeStamp * tstmp = tq.top()->getTimeStamp();
		   now.setNow();
		   long int secdelta = tstmp->getEpoch() - now.getEpoch(); 
		   long int usecdelta = tstmp->getMicroSecond() - now.getMicroSecond(); // negative delta if late
		   if( secdelta == 0 )
		   {
			  if( usecdelta <= 50 MILLISEC && usecdelta >= -50 MILLISEC) // +/-50msec is considered "on-time"
			  {
				 Timer * toptimer = tq.top();
				 toptimer->action();
				 if( toptimer->isPeriodic() )
					toptimer->getTimeStamp()->ResetTime();  // reset timer, if periodic
				 else
				 {
					tq.pop();
					delete(toptimer);
				 }
			  }
			  else if( usecdelta < -50 MILLISEC )  // late more than 50 msec but less than 1 sec
			  {
				 Timer * toptimer = tq.top();
				 cout << "removing 'late' timer " << toptimer << endl;
				 toptimer->action();
				 if( toptimer->isPeriodic() )
					toptimer->getTimeStamp()->ResetTime();  // reset timer, if periodic
				 else
				 {
					tq.pop();
					delete(toptimer);
				 }
			  }
			  else  // > 50 msec but < 1sec => reschedule
				 break;
		   }
		   else if ( secdelta < 0 )  // negative seconds means at least one second LATE
		   {
			  Timer * toptimer = tq.top();
			  cout << "removing 'LATE' timer " << toptimer << endl;
			  toptimer->action();
			  if( toptimer->isPeriodic() )
				 toptimer->getTimeStamp()->ResetTime();  // reset timer, if periodic
			  else
			  {
				 tq.pop();
				 delete(toptimer);
			  }
		   }
		   else
			  break;
		}
	}
	for( ioiter=IOHandlers.begin(); ioiter!=IOHandlers.end(); ++ioiter )
		delete (*ioiter);
}

PropertyValue * PropertyMgr::addLeaf(string path)
{
	vector<string> l = parsePath(path);
	int size = l.size();
	PropertyBranch * curbranch = root;
	PropertyBranch * lastbranch = root;
	for( int i=0; i < size-1; ++i )
	{
		lastbranch = curbranch->findBranch(l[i]);
		if( lastbranch == NULL )
		{ // add new branch
			curbranch->add(new PropertyBranch(l[i], curbranch));
			lastbranch = curbranch->findBranch(l[i]); // set new branch to current branch
		}
		curbranch = lastbranch;
	}
	PropertyLeaf * tmpleaf = new PropertyLeaf(l[size-1], curbranch, PROPTYPE_NOTDEF);
	curbranch->add(tmpleaf);
	return tmpleaf->getValue();
}

void PropertyMgr::print() const
{
	cout << "=== BEGIN Property Manager Tree dump=====" << endl;
	root->print("/");
	cout << "==== END Property Manager Tree dump======" << endl;
}

vector<string> PropertyMgr::parsePath(string path) const
{
	vector<string> lst;
	unsigned int start = path.find('/'); // find first
	unsigned int end;
	while( start != path.npos )
	{
		++start;                 // move past first occurence
		end = path.find('/', start); // find next
		lst.push_back(string(path.substr(start, end-start)));
		start = end;
	}
	return lst;
}



void PropertyMgr::printOn(ostream& o) const
{
	o << root;
}

ostream & operator<< (ostream & o, const PropertyMgr & _mgr)
{
	_mgr.printOn(o);
	return o;
}

ostream & operator<< (ostream & o, const PropertyMgr * _mgr)
{
	_mgr->printOn(o);
	return o;
}


//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////

void RunningFlagHandler::onChange(PropertyValue * _pv)
{
	cout << __PRETTY_FUNCTION__ << endl;
	PropertyMgr::running = _pv->getBool();//false;  // I'm a friend of PropertyMgr, 
	// this will cause PropertyMgr::run to stop and quit the application(toplevel permitting)
}

