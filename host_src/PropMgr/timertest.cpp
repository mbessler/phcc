// This is for emacs
// Local variables:
// mode: c++
// c-basic-offset: 3
// tab-width: 3
// End:

#include <iostream>
#include "TimeStamp.h"


int main()
{
	{
	struct timeval tv;
   TimeStampQueue tsq;
	TimeStamp now;
   cout << "Start Time: " << now << endl;
 	for(int i=100; i<=900; i+=100)
 		tsq.push( new TimeStamp(6, i MILLISEC) );
   tsq.push( new TimeStamp(10, 0) );
   tsq.push( new TimeStamp(3, 00 MILLISEC) );
   tsq.push( new TimeStamp(14, 950 MILLISEC) );
	cout << "      Expected time       \t         Real Time        \t  Delta  \tFlag" << endl;
	cout << "==========================\t==========================\t=========\t====" << endl;
	while( 1 )
	{
	   now.setNow();
	   if( ! tsq.empty() )
	   {
			// schedule next event
			tv.tv_sec = tsq.top()->getEpoch() - now.getEpoch(); 
			tv.tv_usec = tsq.top()->getMicroSecond() - now.getMicroSecond();
			// adjust for carry
			if( tv.tv_usec < 0 )
			{
				tv.tv_sec -= 1;
				tv.tv_usec = 1000000L + tv.tv_usec;
			}
			if( tv.tv_sec < 0 )
				tv.tv_sec = 0;
			cout << "wakeup set in " << tv.tv_sec << " seconds " << tv.tv_usec/1000 << " msec" << endl;
	   }
	   else
	   {
	break;
			tv.tv_sec = 1;
			tv.tv_usec = 50 MILLISEC;
	   }
	   select(0, NULL, NULL, NULL, &tv);  // used as sleep
	   while( ! tsq.empty() )
	   {
			TimeStamp * tstmp = tsq.top();
			now.setNow();
			long int secdelta = tstmp->getEpoch() - now.getEpoch(); 
			long int usecdelta = tstmp->getMicroSecond() - now.getMicroSecond(); // negative delta if late
			if( secdelta == 0 )
			{
				if( usecdelta <= 50 MILLISEC && usecdelta >= -50 MILLISEC) // +/-50msec is considered "on-time"
				{
					cout << tstmp << "\t" << now << "\t" << secdelta << "." << usecdelta << "\t\t----" << endl;
					tsq.pop();
					delete(tstmp);
				}
				else if( usecdelta < -50 MILLISEC )  // late more than 50 msec but less than 1 sec
				{
					cout << tstmp << "\t" << now << "\t" << secdelta << "." << usecdelta << "\tlate" << endl;
					tsq.pop();
					delete(tstmp);					
				}
				else  // > 50 msec but < 1sec => reschedule
					break;
			}
			else if ( secdelta < 0 )  // negative seconds means at least one second LATE
			{
				cout << tstmp << "\t" << now << "\t" << secdelta << "." << usecdelta << "\tLATE" << endl;
				tsq.pop();
				delete(tstmp);					
			}
			else
				break;
	   }
	}
	}
}
