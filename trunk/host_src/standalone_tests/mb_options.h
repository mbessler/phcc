/* mb_options.h
 * This is a config file reader and commandline option processor.
 *
 * Copyright (c) 2002,2003 by Manuel Bessler <m.bessler@gmx.net>
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

#ifndef _MB_OPTIONS_H
#define _MB_OPTIONS_H

#ifdef __cplusplus
extern "C" {
#endif


#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <regex.h>

#define MB_OPTIONS_MAXOPTIONS  64
#define MB_OPTIONS_LINELENGTH 1024

#define MB_OPTIONS_VARTYPE_BOOL       1
#define MB_OPTIONS_VARTYPE_INT        2
#define MB_OPTIONS_VARTYPE_FLOAT      4
#define MB_OPTIONS_VARTYPE_STRING     8
#define MB_OPTIONS_VARTYPE_MULTIPLE   128

typedef struct _options_t options_t;
struct _options_t {
	  char * varname;
	  int vartype;
	  union
	  {
			int var_bool;
			int var_int;
			float var_float;
			char * var_str;
	  } variable;
};

typedef struct _knownoptions_t knownoptions_t;
struct _knownoptions_t {
	  char * varname;
	  char * help;
	  int vartype;
	  union
	  {
			int var_bool;
			int var_int;
			float var_float;
			char * var_str;
	  } vardefault;
};


/* application (which this code is linked against) has to define these */
extern knownoptions_t known_options[];
extern char * mb_options_configfile;


/* public functions */
extern int mb_options_init(int argc, char * argv[]);
extern void mb_options_done();
extern void mb_options_usage(const char * calledname);
extern int mb_options_getintopt(int * start, const char * optionname);
extern int mb_options_getboolopt(int * start, const char * optionname);
extern float mb_options_getfloatopt(int * start, const char * optionname);
extern char * mb_options_getstropt(int * start, const char * optionname);
extern void mb_options_dump();


/* internal functions */
extern void mb_options_init_array();
extern void mb_options_regexerr(int errcode, regex_t *compiled);
extern int mb_options_boolvar(const char * str);
extern int mb_options_intvar(const char * str);
extern int mb_options_find_freeoptionsslot();
extern int mb_options_store(char * optionname, char * optionval);
extern void mb_options_process_cmdline(int argc, char * argv[]);
extern int mb_options_process_configline(int lineno, char * line);
extern void mb_options_read_configfile(char * filename);

#ifdef __cplusplus
}
#endif

#endif  /* _MB_OPTIONS_H */
