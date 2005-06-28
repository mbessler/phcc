// testPHCC_Serial.cpp
//
// TEST for
//   PHCC_Serial, a serial Port access library for PHCC
//
//   Copyright (c) 2005 by Manuel Bessler <m.bessler AT gmx.net>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or 
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
// 
// This is for emacs
// Local variables:
// mode: c++
// c-basic-offset: 3
// tab-width: 3
// End:
//

#include <iostream>

#include "PHCC_Serial.h"

using namespace std;

int main()
{
//   PHCC_Serial * port = new PHCC_Serial(); // uses std::cerr (as per default parameter)
   PHCC_Serial * port = new PHCC_Serial(&cout); // uses std::cout
#ifdef __WIN32__
   port->openDevice("COM1", 115200);
#else
   port->openDevice("/dev/ttyS0", 115200);
#endif // __WIN32__

   if( port->isOpen() )
	   cout << "serial port successfully opened" << endl;
   else
   {
	   cout << "could not open serial port" << endl;
	   delete port;
	   exit(-1);
   }
   port->serialWrite('!');
   port->waitms(2000);
   port->closeDevice();
   delete port;
}
