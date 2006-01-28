// PHCC_Serial.cpp
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

#include "PHCC_Serial.h"

using namespace std;

#ifndef __WIN32__

///////////////////////////////////////////////////////
//////////// START UNIX implementation ////////////////
///////////////////////////////////////////////////////
   ///// win32 implemenation at end of file /////

PHCC_Serial::PHCC_Serial(std::ostream * _errstr)
	: fd_phccdev(-1)
{
	if( _errstr == NULL )
		errstr = &cerr;
	else
		errstr = _errstr;
}

PHCC_Serial::~PHCC_Serial()
{
	closeDevice();
	fd_phccdev = -1;
}

void PHCC_Serial::waitms(int ms)
{
	struct timeval tv;
	tv.tv_sec=0;
	tv.tv_usec=ms*1000;
	select(0, NULL, NULL, NULL, &tv);
}

int PHCC_Serial::serialRead(unsigned char * _byte)
{
	if( fd_phccdev == -1 )
	{
		*errstr << "ERROR use PHCC_Serial::openDevice() first" << endl;
		return 0;
	}
	int size = read(fd_phccdev, _byte, 1); // returns 0 if fcntl(serialFD, F_SETFL, FNDELAY) set
	return size;
}

int PHCC_Serial::serialReadBlocking(unsigned char * _byte)
{
	int ret;
	do
	{
		ret = serialRead(_byte);
		if( ret >= 0 )
			break;
		if(ret == -1 && errno != EAGAIN )
			break;
		waitms(1);
	} while( 1 );
	return ret;
}

int PHCC_Serial::serialWrite(unsigned char _byte)
{
	unsigned char c[1];
	if( fd_phccdev == -1 )
	{
		*errstr << "ERROR use PHCC_Serial::openDevice() first" << endl;
		return 0;
	}
	c[0] = _byte;
	return write(fd_phccdev, c, 1);
}

int PHCC_Serial::serialWriteBlocking(unsigned char _byte)
{
	int ret;

	do
	{
		ret = serialWrite(_byte);
		*errstr << "blkwrt=" << ret << endl;
		if( ret >= 0 )
			break;
		if( ret== -1 && errno != EAGAIN )
			break;
		waitms(1);
	} while(1);
//	while( (ret = serialWrite(_byte)) == 0 )
//	{ waitms(1); }
	return ret;
}

int PHCC_Serial::getfd()
{
	return fd_phccdev;
}

bool PHCC_Serial::isOpen()
{
	return (fd_phccdev != -1) ? true : false;
}

int PHCC_Serial::openDevice(const char * dev, int speed, bool useRTSCTS)
{
	int baudrate = -1;
	if( fd_phccdev != -1 )
	{
		// already open, don't open a second time
		return 1;
	}

	if( (fd_phccdev = open(dev, O_RDWR | O_NOCTTY | O_NDELAY /*| O_NONBLOCK*/))  < 0 ) /* when O_NDELAY is set, DCD is ignored  */ 
	{
		*errstr << "ERROR, could not open device " << dev << endl;
		fd_phccdev = -1;
		return 0;
	}
	if( (baudrate = baudrate_check(speed)) == -1 )
	{
		*errstr << "ERROR, invalid speed " << speed << endl;
		fd_phccdev = -1;
		return 0;
	}
	tcflush(fd_phccdev, TCIOFLUSH);
	setline(baudrate, useRTSCTS);
/*	fcntl(fd_phccdev, F_SETFL, 0);*/
	fcntl(fd_phccdev, F_SETFL, FNDELAY);  // standart ops is non-blocking since windows port can't do anything else
	return 1;
}

void PHCC_Serial::closeDevice()
{
	if( isOpen() )
		close(fd_phccdev);
	fd_phccdev = -1;
}

int PHCC_Serial::baudrate_check(int speed)
{
	switch(speed)
	{
		case 230400: return B230400;
		case 115200: return B115200;
		case 57600: return B57600;
		case 38400: return B38400;
		case 19200: return B19200;
		case 9600: return B9600;
		case 4800: return B4800;
		case 2400: return B2400;
		case 1800: return B1800;
		case 1200: return B1200;
		case 600: return B600;
		case 300: return B300;
		case 200: return B200;
		case 150: return B150;
		case 134: return B134;
		case 110: return B110;
		case 75: return B75;
		case 50: return B50;
		default: return -1;
	}
}

void PHCC_Serial::setline(int speed, bool useRTSCTS)
{
	struct termios t;

	tcgetattr(fd_phccdev, &t);

	t.c_cflag |= (CLOCAL | CREAD);
	/* 8N1 */
	t.c_cflag &= ~PARENB;
	t.c_cflag &= ~CSTOPB;
	t.c_cflag &= ~CSIZE;
	t.c_cflag |= CS8;

	if( useRTSCTS )
		t.c_cflag |= CRTSCTS; /* RTS/CTS hardware flow control */
	else
		t.c_cflag &= ~CRTSCTS;  /* NO RTS/CTS hardware flow control */
	
	t.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  /* raw input (not line oriented) */


	t.c_iflag |= (IGNPAR | IGNBRK /*| IGNCR*/ );  /* ignore parity errors, ignore Break */
	t.c_iflag &= ~(IGNCR | INLCR | ICRNL);
	t.c_iflag &= ~(IXON | IXOFF | IXANY);  /* no software flow control */
	
	t.c_oflag &= ~OPOST;  /* raw output */
	
	t.c_cc[VMIN ] = 1;
	t.c_cc[VTIME] = 0;
	
	cfsetispeed(&t, speed);
	cfsetospeed(&t, speed);

	tcsetattr(fd_phccdev, /*TCSAFLUSH*/TCSANOW, &t);
}

///////////////////////////////////////////////////////
//////////// END UNIX implementation //////////////////
///////////////////////////////////////////////////////

#else  /* __WIN32__ */

///////////////////////////////////////////////////////
//////////// START WIN32 implementation ///////////////
///////////////////////////////////////////////////////


PHCC_Serial::PHCC_Serial(std::ostream * _errstr)
	: fd_phccdev(INVALID_HANDLE_VALUE)
{
	if( _errstr == NULL )
		errstr = &cerr;
	else
		errstr = _errstr;
}

PHCC_Serial::~PHCC_Serial()
{
	closeDevice();
	fd_phccdev = INVALID_HANDLE_VALUE;
}

void PHCC_Serial::waitms(int ms)
{
	Sleep(ms);
}

int PHCC_Serial::serialRead(unsigned char * _byte)
{//non-blocking
	if( fd_phccdev == INVALID_HANDLE_VALUE )
	{
		*errstr << "ERROR use PHCC_Serial::openDevice() first" << endl;
		return 0;
	}
	DWORD dw_read;
	if(!ReadFile(fd_phccdev,_byte,1,&dw_read,&ov_phccdev))
	{
		// if we use a asynchrone reading, ReadFile gives always
		// FALSE
		// ERROR_IO_PENDING means ok, other values show an error
		if(GetLastError() != ERROR_IO_PENDING)
		{
			*errstr << "ERROR while reading from port: Error no=" << GetLastError() << endl; // oops..., error in communication
			return -1;
		}
	}
	else
	{
		// ok, we have read all wanted bytes
		return (int)dw_read;
	}
	return 0; // gets here only after ERROR_IO_PENDING, ie. when read found nothing readable yet
}

int PHCC_Serial::serialReadBlocking(unsigned char * _byte)
{
	int ret;
	while( (ret = serialRead(_byte)) == 0 )
	{ waitms(1); }
	return ret;
}

int PHCC_Serial::serialWrite(unsigned char _byte)
{//non-blocking
	unsigned char c[1];
	if( fd_phccdev == INVALID_HANDLE_VALUE )
	{
		*errstr << "ERROR use PHCC_Serial::openDevice() first" << endl;
		return 0;
	}
	c[0] = _byte;
	DWORD dw_write;
	if(!WriteFile(fd_phccdev,c,1,&dw_write,&ov_phccdev))
	{
		if(GetLastError() != ERROR_IO_PENDING)
		{
			*errstr << "ERROR while writing to port: Error no=" << GetLastError() << endl;
			return -1;
		}
		else
		{
			// VERY IMPORTANT to flush the data out of the internal
			// buffer
			FlushFileBuffers(fd_phccdev);
			// first you must call GetOverlappedResult, then you
			// get the REALLY transmitted count of bytes
			if(!GetOverlappedResult(fd_phccdev,&ov_phccdev,&dw_write,TRUE))
			{
				// ooops... sonething going wrong
				return (int)dw_write;
			}
		}
	}
	return (int)dw_write;
}

int PHCC_Serial::serialWriteBlocking(unsigned char _byte)
{
	int ret;
	while( (ret = serialWrite(_byte)) == 0 )
	{ waitms(1); }
	return ret;
}

HANDLE PHCC_Serial::getfd()
{
	return fd_phccdev;
}

bool PHCC_Serial::isOpen()
{
	return (fd_phccdev != INVALID_HANDLE_VALUE) ? true : false;
}

int PHCC_Serial::openDevice(const char * dev, int speed, bool useRTSCTS)
{
	int baudrate = -1;
	if( fd_phccdev != INVALID_HANDLE_VALUE )
	{
		// already open, don't open a second time
		return 1;
	}

	COMMTIMEOUTS cto = {MAXDWORD,0,0,0,0};
	EINFO einfo = {0,0,0,0};
	DCB dcb;
	memset(&ov_phccdev,0,sizeof(ov_phccdev));
	memset(&dcb,0,sizeof(dcb));

	fd_phccdev = CreateFile(dev,	// device name
									GENERIC_READ | GENERIC_WRITE,	// O_RDWR
									0,		// not shared
									NULL,	// default value for object security ?!?
									OPEN_EXISTING, // file (device) exists
									FILE_FLAG_OVERLAPPED,	// asynchron handling
									NULL); // no more handle flags
	
	if(fd_phccdev == INVALID_HANDLE_VALUE)
	{
		*errstr << "ERROR, could not open device " << dev << endl;
		closeDevice();
		return 0;    
	}

	// device control block
	dcb.DCBlength = sizeof(dcb);
	dcb.fBinary = 1;

	if( (baudrate = baudrate_check(speed)) == -1 )
	{
		*errstr << "ERROR, invalid speed: " << speed << endl;
		closeDevice();
		return 0;
	}
	dcb.BaudRate = baudrate;

	// Specifies whether the CTS (clear-to-send) signal is monitored 
	// for output flow control. If this member is TRUE and CTS is turned
	// off, output is suspended until CTS is sent again.
	if( useRTSCTS )
		dcb.fOutxCtsFlow = 1;
	else
		dcb.fOutxCtsFlow = 0;
	dcb.fOutxDsrFlow = 0;
	dcb.fDsrSensitivity = 0;
	dcb.fNull = 0;

	// Specifies the DTR (data-terminal-ready) flow control. 
	// This member can be one of the following values:
	// DTR_CONTROL_DISABLE   Disables the DTR line when the device is 
	//                       opened and leaves it disabled. 
	// DTR_CONTROL_ENABLE    Enables the DTR line when the device is 
	//                       opened and leaves it on. 
	// DTR_CONTROL_HANDSHAKE Enables DTR handshaking. If handshaking is 
	//                       enabled, it is an error for the application
	//                       to adjust the line by using the 
	//                       EscapeCommFunction function.  
	dcb.fDtrControl = DTR_CONTROL_ENABLE;

	// Specifies the RTS flow control. If this value is zero, the
	// default is RTS_CONTROL_HANDSHAKE. This member can be one of
	// the following values:
	// RTS_CONTROL_DISABLE   Disables the RTS line when device is
	//                       opened and leaves it disabled.
	// RTS_CONTROL_ENABLE    Enables the RTS line when device is
	//                       opened and leaves it on.
	// RTS_CONTROL_HANDSHAKE Enables RTS handshaking. The driver
	//                       raises the RTS line when the
	//                       "type-ahead" (input)buffer is less than
	//                       one-half full and lowers the RTS line
	//                       when the buffer is more than three-quarters
	//                       full. If handshaking is enabled, it is an
	//                       error for the application to adjust the
	//                       line by using the EscapeCommFunction function.
	// RTS_CONTROL_TOGGLE    Specifies that the RTS line will be high if 
	//                       bytes are available for transmission. After
	//                       all buffered bytes have been send, the RTS
	//                       line will be low.
	if( useRTSCTS )
		dcb.fRtsControl = RTS_CONTROL_HANDSHAKE;
	else
		dcb.fRtsControl = RTS_CONTROL_ENABLE;
    
	// Specifies the XON/XOFF flow control.
	// If fOutX is true (the default is false), transmission stops when the
	// XOFF character is received and starts again, when the XON character
	// is received.
	dcb.fOutX = 0;
	// If fInX is true (default is false), the XOFF character is sent when
	// the input buffer comes within XoffLim bytes of being full, and the
	// XON character is sent, when the input buffer comes within XonLim
	// bytes of being empty.
	dcb.fInX = 0;
	// default character for XOFF is 0x13 (hex 13)
	dcb.XoffChar = 0x13;
	// default character for XON is 0x11 (hex 11)
	dcb.XonChar = 0x11;
	// set the minimum number of bytes allowed in the input buffer before
	// the XON character is sent (3/4 of full size)
	dcb.XonLim = (xSERIALPORT_BUFSIZE >> 2) * 3;
	// set the maximum number of free bytes in the input buffer, before the
	// XOFF character is sent (3/4 of full size)
	dcb.XoffLim = (xSERIALPORT_BUFSIZE >> 2) * 3;

	// parity
	dcb.Parity = NOPARITY;
	// stopbits
	dcb.StopBits = ONESTOPBIT;
	// wordlen, valid values are 5,6,7,8
	dcb.ByteSize = 8;

	if(!SetCommState(fd_phccdev,&dcb))
	{
		*errstr << "ERROR, SetCommState failed" << endl;
		closeDevice();
	   return -2;
	}

	// create event for overlapped I/O
	// we need a event object, which inform us about the
	// end of an operation (here reading device)
	ov_phccdev.hEvent = CreateEvent(NULL,// LPSECURITY_ATTRIBUTES lpsa
					TRUE, // BOOL fManualReset 
					TRUE, // BOOL fInitialState
					NULL); // LPTSTR lpszEventName
	if(ov_phccdev.hEvent == INVALID_HANDLE_VALUE)
	{
		*errstr << "ERROR, CreateEvent failed" << endl;
		closeDevice();
		return -3;
	}

    /* THIS IS OBSOLETE!!!
    // event should be triggered, if there are some received data
    if(!SetCommMask(fd,EV_RXCHAR))
    return -4;
    */

	if(!SetCommTimeouts(fd_phccdev,&cto))
	{
		*errstr << "ERROR, SetCommTimeouts failed" << endl;
		closeDevice();
		return -5;
	}

	// for a better performance with win95/98 I increased the internal
	// buffer to xSERIALPORT_BUFSIZE (normal size is 1024, but this can 
	// be a little bit to small, if you use a higher baudrate like 115200)
	if(!SetupComm(fd_phccdev,xSERIALPORT_BUFSIZE,xSERIALPORT_BUFSIZE))
	{
		*errstr << "ERROR, SetupComm failed" << endl;
		closeDevice();
		return -6;
	}

	// clear the internal error struct
	memset(&einfo,0,sizeof(einfo));
	return 1;
}

void PHCC_Serial::closeDevice()
{
	if( isOpen() )
	{
		CloseHandle(ov_phccdev.hEvent);
		CloseHandle(fd_phccdev);
		fd_phccdev = INVALID_HANDLE_VALUE;
	}
}

int PHCC_Serial::baudrate_check(int speed)
{
	switch(speed)
	{
		case 115200: return CBR_115200;
		case 57600: return CBR_57600;
		case 38400: return CBR_38400;
		case 19200: return CBR_19200;
		case 9600: return CBR_9600;
		case 4800: return CBR_4800;
		case 2400: return CBR_2400;
		case 1200: return CBR_1200;
		default: return -1;
	}
}

void PHCC_Serial::setline(int speed, bool useRTSCTS)
{
	/* dummy for the win32 implementation, everything is done inside opendevice()  */
}

///////////////////////////////////////////////////////
//////////// END WIN32 implementation /////////////////
///////////////////////////////////////////////////////

#endif /* __WIN32__ */
