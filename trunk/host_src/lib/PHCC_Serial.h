// PHCC_Serial.h
//
//   A serial Port access class for PHCC
//
//   Copyright (c) 2003-2005 by Manuel Bessler <m.bessler AT gmx.net>
//
//
//   The windows code used in this class was stolen from 
//     ctb, the "Platform Independant Communication Toolbox"
//   aka. wxSerialPort, a Serial port class for wxWidgets found here:
//     http://www.iftools.com/ctb.en.html
//  which is 
//    Copyright (C) 1999-2004 Joachim Buermann
//  and was released under the LGPL:
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
// 
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
// This is for emacs
// Local variables:
// mode: c++
// c-basic-offset: 3
// tab-width: 3
// End:
//

#ifndef _PHCC_SERIAL_H
#define _PHCC_SERIAL_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <ctype.h>
#include <cerrno>

#include <iostream>

#ifdef __WIN32__
#include <windows.h>
#include <process.h>  // under win instead of unistd.h
                      // thanks to Tobias Muehlhuber for that tip :)
#define xSERIALPORT_BUFSIZE 4096
#else   /* __WIN32__ */
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#endif  /* __WIN32__ */


#ifdef __WIN32__
typedef struct _EINFO EINFO;

struct _EINFO
{
    /*! number of breaks */
    int brk;
    /*! number of framing errors */
    int frame;
    /*! number of overrun errors */
    int overrun;
    /*! number of parity errors */
    int parity;
};
#endif  /* __WIN32__ */


class PHCC_Serial
{
	public:
		PHCC_Serial(std::ostream * _errstr = NULL);
		~PHCC_Serial();
		
		void waitms(int ms);
		int serialRead(unsigned char * _byte);
		int serialReadBlocking(unsigned char * _byte);
		int serialWrite(unsigned char _byte);
		int serialWriteBlocking(unsigned char _byte);
#ifdef __WIN32__
		HANDLE getfd();
#else   /* __WIN32__ */
		int getfd();
#endif  /* __WIN32__ */
		bool isOpen();
		int openDevice(const char * dev, int speed, bool useRTSCTS=true);
		void closeDevice();
		
	private:
		void setline(int speed, bool useRTSCTS=true);
		int baudrate_check(int speed);
	private:
#ifdef __WIN32__
		HANDLE fd_phccdev; 
		OVERLAPPED ov_phccdev;
#else   /* __WIN32__ */
		int fd_phccdev;
#endif  /* __WIN32__ */
		std::ostream * errstr;
};

#ifdef __WIN32__
#else   /* __WIN32__ */
#endif  /* __WIN32__ */


#endif /* _PHCC_SERIAL_H */
