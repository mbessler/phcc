/* stdoptions.h.in
 * This is a config file reader and commandline option processor.
 * 
 * this file defines the options that a program will accept
 *
 * Copyright (c) 2002,2004 by Manuel Bessler <m.bessler@gmx.net>
 *
 *  The full text of the legal notices is contained in the file called
 *  COPYING, included with this distribution.
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version 2
 *  of the License, or (at your option) any later version.
 * 
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 *
 * $Id:$
 */

#ifndef _STDOPTIONS_H
#define _STDOPTIONS_H

#ifdef __cplusplus
extern "C" {
#endif


/* this is included by mb_options.h/c */
/* define your valid options for commandline and configfile here ! */

char * mb_options_configfile = "phcc.conf";

knownoptions_t known_options[] = 
{
   { "config", "where to find the configfile", 
	 MB_OPTIONS_VARTYPE_STRING, 
	 { var_str: "phcc.conf"}
   },
   { "device", "device to talk to PHCC", 
	 MB_OPTIONS_VARTYPE_STRING, 
	 { var_str: "/dev/ttyS0" }
   },
   { "speed", "speed used for communications via device", 
	 MB_OPTIONS_VARTYPE_INT, 
	 { var_int: 115200 }
   },
   { "help", "this helptext", 
	 MB_OPTIONS_VARTYPE_BOOL, 
	 { var_bool: 0 }
   },

   { NULL, NULL, 0 , { var_bool: 0} } /* this has to stay as a termination sequence */
};

#ifdef __cplusplus
}
#endif


#endif  /* _STDOPTIONS_H */
