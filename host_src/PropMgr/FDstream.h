//FDstream.h - a stream class that is based on filedescriptors
//
// This file is part of X11GC.
// 
//    This was NOT written by me, I found it on the web, 
//    can't remeber where.
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

#ifndef _FDSTREAM_H
#define _FDSTREAM_H

#include <iosfwd>
//#include <streambuf>
//#include <string>
#include <unistd.h>


class FDostream : public std::streambuf
{
	public:
		FDostream() : BufferSize(0), StreamBuffer(NULL)
		{}
		FDostream(int _fd, unsigned int _bufsize = 1)
		{
			open(_fd, _bufsize);
		}
		FDostream(const FDostream & _fdostream) // copy constructor
		{
			StreamFD = _fdostream.StreamFD;
			BufferSize= _fdostream.BufferSize;
			if( _fdostream.StreamBuffer != NULL )
			{
				StreamBuffer = new char[BufferSize];
				memcpy(StreamBuffer, _fdostream.StreamBuffer, BufferSize);
			}
			else
				StreamBuffer = NULL;
		}
		~FDostream()
		{
			if( StreamBuffer )
			{
				sync();
				delete[] StreamBuffer;
			}
		}
		void open(int _fd, unsigned int _bufsize = 1)
		{
			StreamFD = _fd;
			BufferSize = _bufsize == 0 ? 1 : _bufsize;
			StreamBuffer = new char[BufferSize];
			setp(StreamBuffer, StreamBuffer + BufferSize);
		}
		int sync()
		{
			if (pptr() > pbase())
			{
				write(StreamFD, StreamBuffer, pptr() - pbase());
				setp(StreamBuffer, StreamBuffer + BufferSize); 
			}
			return 0;
		}
		int overflow(int c)
		{
			sync();
			if( c != EOF )
			{
				*pptr() = static_cast<char>(c);
				pbump(1);
			}
			return c;
		}
		int getFD()
		{
			return StreamFD;
		}
	protected:
	private:
		int StreamFD;
		unsigned int BufferSize;
		char * StreamBuffer;
};

class FDistream : public std::streambuf
{
	public:
		FDistream() : BufferSize(0), StreamBuffer(NULL)
		{}
		FDistream(int _fd, unsigned int _bufsize = 1)
		{
			open(_fd, _bufsize);
		}
		FDistream(const FDistream & _fdistream) // copy constructor
		{
			StreamFD = _fdistream.StreamFD;
			BufferSize= _fdistream.BufferSize;
			if( _fdistream.StreamBuffer != NULL )
			{
				StreamBuffer = new char[BufferSize];
				memcpy(StreamBuffer, _fdistream.StreamBuffer, BufferSize);
			}
			else
				StreamBuffer = NULL;
		}
		~FDistream()
		{
			if( StreamBuffer )
			{
				close(StreamFD);
				delete[] StreamBuffer;
			}
		}
		void open(int _fd, unsigned int _bufsize = 1)
		{
			StreamFD = _fd;
			BufferSize = _bufsize == 0 ? 1 : _bufsize;
			StreamBuffer = new char[BufferSize];
			setg(StreamBuffer, StreamBuffer + BufferSize, StreamBuffer + BufferSize);
		}
		int underflow()
		{
			if( gptr() < egptr() )
				return *gptr();
			int nread = read(StreamFD, StreamBuffer, BufferSize);
			if( nread <= 0 )
				return EOF;
			
			setg(StreamBuffer, StreamBuffer, StreamBuffer + nread);
			return *gptr();
		}
		std::streamsize xsgetn(char *dest, std::streamsize n)
		{
			int nread = 0;
			while( n )
			{
				if( ! in_avail() )
					if( underflow() == EOF )
						break;
				int avail = in_avail();
				if( avail > n )
					avail = n;
				memcpy(dest + nread, gptr(), avail);            // 5
				gbump(avail);                                   // 6
				nread += avail;
				n -= avail;
			}
			return nread;
		}
		int getFD()
		{
			return StreamFD;
		}
	protected:
	private:
		int StreamFD;
		unsigned int BufferSize;
		char * StreamBuffer;
};



#endif // _FDSTREAM_H
