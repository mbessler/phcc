/* mb_options.c
 * This is a config file reader and commandline option processor.
 *
 * Copyright (c) 2002 by Manuel Bessler <m.bessler@gmx.net>
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

#include "mb_options.h"
#include "stdoptions.h"

options_t options[MB_OPTIONS_MAXOPTIONS];


void mb_options_regexerr(int errcode, regex_t *compiled)
{
   size_t length = regerror (errcode, compiled, NULL, 0);
   char *buffer = malloc (length);
   (void) regerror (errcode, compiled, buffer, length);
   fprintf(stderr, " %s\n", buffer);
   free(buffer);
}

void mb_options_init_array()
{
   int i;
   for(i=0; i<MB_OPTIONS_MAXOPTIONS; ++i)
   {
	  options[i].varname = NULL;
	  options[i].vartype = 0;
	  options[i].variable.var_int = 0;
   }
}

void mb_options_dump()
{
   int i;
   for(i=0; i<MB_OPTIONS_MAXOPTIONS; ++i)
   {
	  if( options[i].varname == NULL )
		 continue;
	  fprintf(stderr, "dump: %s=", options[i].varname);
	  if( options[i].vartype == MB_OPTIONS_VARTYPE_BOOL || 
		  options[i].vartype == MB_OPTIONS_VARTYPE_BOOL+MB_OPTIONS_VARTYPE_MULTIPLE )
		 fprintf(stderr, "%i type=VARTYPE_BOOL\n", options[i].variable.var_bool);
	  if( options[i].vartype == MB_OPTIONS_VARTYPE_INT || 
		  options[i].vartype == MB_OPTIONS_VARTYPE_INT+MB_OPTIONS_VARTYPE_MULTIPLE )
		 fprintf(stderr, "%i type=VARTYPE_INT\n", options[i].variable.var_int);
	  if( options[i].vartype == MB_OPTIONS_VARTYPE_FLOAT || 
		  options[i].vartype == MB_OPTIONS_VARTYPE_FLOAT+MB_OPTIONS_VARTYPE_MULTIPLE )
		 fprintf(stderr, "%f type=VARTYPE_FLOAT\n", options[i].variable.var_float);
	  if( options[i].vartype == MB_OPTIONS_VARTYPE_STRING || 
		  options[i].vartype == MB_OPTIONS_VARTYPE_STRING+MB_OPTIONS_VARTYPE_MULTIPLE )
		 fprintf(stderr, "%s type=VARTYPE_STRING\n", options[i].variable.var_str);
   }
}

/* start for MB_OPTIONS_VARTYPE_MULTIPLE, if non-NULL, points to next search pos'n (-1 if no more) */
int mb_options_getintopt(int * start, const char * optionname)
{
   int i, begin=0;
   if( start != NULL )
	  if( *start > -1 && *start < MB_OPTIONS_MAXOPTIONS )
		 begin = *start;
   for(i=begin; i<MB_OPTIONS_MAXOPTIONS; ++i)
   {
	  if( options[i].varname == NULL )
		 continue;
	  if( strcasecmp(options[i].varname, optionname) == 0 &&
		  (options[i].vartype == MB_OPTIONS_VARTYPE_INT || 
		   options[i].vartype == MB_OPTIONS_VARTYPE_INT+MB_OPTIONS_VARTYPE_MULTIPLE) )
	  {
		 if( start != NULL )
		 {
			*start = i+1;
			if( options[i].vartype < MB_OPTIONS_VARTYPE_MULTIPLE )
			   *start = -1;
		 }
		 return options[i].variable.var_int;
	  }
   }
   if( start != NULL )
	  *start = -1;
   return 0;
}

/* start for MB_OPTIONS_VARTYPE_MULTIPLE, if non-NULL, points to next search pos'n (-1 if no more) */
int mb_options_getboolopt(int * start, const char * optionname)
{
   int i, begin=0;
   if( start != NULL )
	  if( *start > -1 && *start < MB_OPTIONS_MAXOPTIONS )
		 begin = *start;
   for(i=begin; i<MB_OPTIONS_MAXOPTIONS; ++i)
   {
	  if( options[i].varname == NULL )
		 continue;
	  if( strcasecmp(options[i].varname, optionname) == 0 &&
		  (options[i].vartype == MB_OPTIONS_VARTYPE_BOOL || 
		   options[i].vartype == MB_OPTIONS_VARTYPE_BOOL+MB_OPTIONS_VARTYPE_MULTIPLE) )
	  {
		 if( start != NULL )
		 {
			*start = i+1;
			if( options[i].vartype < MB_OPTIONS_VARTYPE_MULTIPLE )
			   *start = -1;
		 }
		 return options[i].variable.var_bool;
	  }
   }
   if( start != NULL )
	  *start = -1;
   return 0;
}

/* start for MB_OPTIONS_VARTYPE_MULTIPLE, if non-NULL, points to next search pos'n (-1 if no more) */
float mb_options_getfloatopt(int * start, const char * optionname)
{
   int i, begin=0;
   if( start != NULL )
	  if( *start > -1 && *start < MB_OPTIONS_MAXOPTIONS )
		 begin = *start;
   for(i=begin; i<MB_OPTIONS_MAXOPTIONS; ++i)
   {
	  if( options[i].varname == NULL )
		 continue;
	  if( strcasecmp(options[i].varname, optionname) == 0 &&
		  (options[i].vartype == MB_OPTIONS_VARTYPE_FLOAT || 
		   options[i].vartype == MB_OPTIONS_VARTYPE_FLOAT+MB_OPTIONS_VARTYPE_MULTIPLE) )
	  {
		 if( start != NULL )
		 {
			*start = i+1;
			if( options[i].vartype < MB_OPTIONS_VARTYPE_MULTIPLE )
			   *start = -1;
		 }
		 return options[i].variable.var_float;
	  }
   }
   if( start != NULL )
	  *start = -1;
   return -1.0;
}

/* start for MB_OPTIONS_VARTYPE_MULTIPLE, if non-NULL, points to next search pos'n (-1 if no more) */
char * mb_options_getstropt(int * start, const char * optionname)
{
   int i, begin=0;
   if( start != NULL )
	  if( *start > -1 && *start < MB_OPTIONS_MAXOPTIONS )
		 begin = *start;
   for(i=begin; i<MB_OPTIONS_MAXOPTIONS; ++i)
   {
	  if( options[i].varname == NULL )
		 continue;
	  if( strcasecmp(options[i].varname, optionname) == 0 &&
		  (options[i].vartype == MB_OPTIONS_VARTYPE_STRING || 
		   options[i].vartype == MB_OPTIONS_VARTYPE_STRING+MB_OPTIONS_VARTYPE_MULTIPLE) )
	  {
		 if( start != NULL )
		 {
			*start = i+1;
			if( options[i].vartype < MB_OPTIONS_VARTYPE_MULTIPLE )
			   *start = -1;
		 }
		 return options[i].variable.var_str;
	  }
   }
   if( start != NULL )
	  *start = -1;
   return NULL;
}


int mb_options_boolvar(const char * str)
{
   if( str == NULL )
	  return 1;  /* empty booleans mean 'enable' */
   /* returns 0 for false, 1 for true and -1 for invalid */
   if( strcasecmp(str, "1")     == 0 || 
	   strcasecmp(str, "true")  == 0 || 
	   strcasecmp(str, "yes")   == 0 || 
	   strcasecmp(str, "on")    == 0 || 
	   strcasecmp(str, "y")     == 0 )
	  return 1;
   if( strcasecmp(str, "0")     == 0 || 
	   strcasecmp(str, "false") == 0 || 
	   strcasecmp(str, "no")    == 0 || 
	   strcasecmp(str, "off")   == 0 || 
	   strcasecmp(str, "n")     == 0 )
	  return 0;
   return -1;   
}

int mb_options_intvar(const char * str)
{
   int i;
   int v;
   if( str == NULL )
	  return 0;
   v = atoi(str);
   for(i=0; i < strlen(str); ++i)
   {
	  if( str[i] < '0' || str[i] > '9' )
	  {
		 fprintf(stderr, "ERROR: wrong value format for option\n");
		 return -1;
	  }
   }
   return v;
}

int mb_options_find_freeoptionsslot()
{
   int i;
   for(i=0; i<MB_OPTIONS_MAXOPTIONS; ++i)
	  if( options[i].varname == NULL )
		 return i;
   fprintf(stderr, "ERROR: --------------------------------------------------------------\n");
   fprintf(stderr, "ERROR: ------could not find free slot in options: -------------------\n");
   fprintf(stderr, "ERROR: --------increase MB_OPTIONS_MAXOPTIONS in mb_options.h--------\n");
   fprintf(stderr, "ERROR: --------and RECOMPILE-----------------------------------------\n");
   fprintf(stderr, "ERROR: --------------------------------------------------------------\n");
   exit(1000); /* no free slot */
}

int mb_options_store(char * optionname, char * optionval)
{
   int j=0; /* counter for know_options */
   int found = 0;
   while( known_options[j].varname != NULL )
   {
	  if( strcasecmp(known_options[j].varname, optionname) == 0 )
	  {
		 int l;
		 int k = mb_options_find_freeoptionsslot(); /* slot in options array */
		 if( known_options[j].vartype < MB_OPTIONS_VARTYPE_MULTIPLE )
			for(l=0; l<MB_OPTIONS_MAXOPTIONS; ++l)
			{  /* if an entry w/ the same varname exists, overwrite 
				  (this means that commandline options overrides configfile) */
               if(  options[l].varname != NULL )
				  if( strcasecmp( options[l].varname, optionname) == 0 )
				  {
					 k = l;
				  }
			}
		 options[k].varname = known_options[j].varname;
		 options[k].vartype = known_options[j].vartype;
		 if( options[k].vartype == MB_OPTIONS_VARTYPE_BOOL ||
			 options[k].vartype == MB_OPTIONS_VARTYPE_BOOL+MB_OPTIONS_VARTYPE_MULTIPLE )
		 {
			options[k].variable.var_bool = mb_options_boolvar(optionval);
			if( options[k].variable.var_bool == -1 )
			{
			   fprintf(stderr, "ERROR: wrong value format for option '%s'\n", 
					   options[k].varname);
			   exit(1);
			}
		 }
		 if( options[k].vartype == MB_OPTIONS_VARTYPE_INT ||
			 options[k].vartype == MB_OPTIONS_VARTYPE_INT+MB_OPTIONS_VARTYPE_MULTIPLE )
			options[k].variable.var_int = mb_options_intvar(optionval);
		 if( options[k].vartype == MB_OPTIONS_VARTYPE_FLOAT ||
			 options[k].vartype == MB_OPTIONS_VARTYPE_FLOAT+MB_OPTIONS_VARTYPE_MULTIPLE )
			options[k].variable.var_float = atof(optionval);
		 if( options[k].vartype == MB_OPTIONS_VARTYPE_STRING ||
			 options[k].vartype == MB_OPTIONS_VARTYPE_STRING+MB_OPTIONS_VARTYPE_MULTIPLE )
		 {
			options[k].variable.var_str = (char *)malloc(strlen(optionval)+1);
			strcpy(options[k].variable.var_str, optionval);
		 }
		 found = 1;
		 break;
	  }
	  ++j;
   }
   if( ! found )
	  return 0;
   return 1;
}

void mb_options_process_cmdline(int argc, char * argv[])
{
   int i;  /* counter for argv array */
   for(i=1; i < argc; ++i)
   {
	  char * optionname = argv[i] + 2;       /* skip over "--" */
	  char * equalsign = strchr(optionname, '=');
	  char * optionval = (equalsign == NULL) ? argv[++i] : &(equalsign[1]);
	  if( equalsign != NULL ) 
		 *equalsign = '\0';
	  mb_options_store(optionname, optionval);
   }
}

int mb_options_process_configline(int lineno, char * line)
{
   char * pat = "^([ \t]*[a-zA-Z_\\-]+)[ \t]*=[ \t]*([a-zA-Z0-9_/\\.\\-]+)[ \t]*(#.*)*\r*\n*$";
   int err;
   regex_t *COMPILED;
   regmatch_t matches[30];

   COMPILED = (regex_t *)calloc(1, sizeof(regex_t));
   err = regcomp(COMPILED, pat, REG_EXTENDED);
   if( err != 0 )
   {	  
	  fprintf(stderr, "ERROR in option processing: regcomp() failed: ");
	  mb_options_regexerr(err, COMPILED);
   }
   err = regexec(COMPILED, line, 30, matches, 0);
   if( err == 0 )
   {
	  char * optionname = NULL;
	  char * optionvalue = NULL; 
	  line[matches[1].rm_eo] = '\0';
	  line[matches[2].rm_eo] = '\0';
	  
	  optionname = &(line[matches[1].rm_so]);
	  optionvalue = &(line[matches[2].rm_so]);
/*	  printf("found option <%s>=<%s>\n", optionname, optionvalue);*/
	  if( mb_options_store(optionname, optionvalue) == 0 )
	  {
		 fprintf(stderr, "ERROR while parsing configfile: line #%i: unknown option '%s'\n", lineno, optionname);
		 regfree(COMPILED);
		 free(COMPILED);
		 return 0;
	  }
   }
   else if( err == REG_NOMATCH )
   {
	  fprintf(stderr, "ERROR while parsing configfile: line #%i: ", lineno);
	  mb_options_regexerr(err, COMPILED);
   }
   else
   {
	  fprintf(stderr, "ERROR in option processing: regexec() failed: ");
	  mb_options_regexerr(err, COMPILED);
   }
   regfree(COMPILED);
   free(COMPILED);
   return 1;
}

void mb_options_read_configfile(char * filename)
{
   int lineno=0;
   FILE * conf;
   conf = fopen(filename, "r");
   if( conf == NULL )
   {
	  fprintf(stderr, "ERROR: could not open configfile '%s': %s\n", filename, strerror(errno));
	  exit(3);
   }
   while( ! feof(conf) )
   {
	  char * line = NULL;
	  int rv;
	  line = malloc(MB_OPTIONS_LINELENGTH);
	  ++lineno;
	  if( fgets(line, MB_OPTIONS_LINELENGTH, conf) != NULL )
	  {
		 int i=0;
		 int comment_or_emtyline = 1;
		 while( i <= strlen(line) )
		 {
			if( line[i] == '#' || line[i] == '\r' || line[i] == '\n')
			   break;
			if( line[i] != ' ' || line[i] != '\t' )
			{
			   comment_or_emtyline = 0;
			   break;
			}
			++i;
		 }
		 if( ! comment_or_emtyline )
		 {
			rv = mb_options_process_configline(lineno, line);
			if( rv == 0 ) /* error occurred */
			{
			   free(line);
			   fclose(conf);
			   exit(1);
			}
		 }
	  }	  
	  free(line);
   }
   fclose(conf);
}

void mb_options_usage(const char * calledname)
{
   int i=0;
   fprintf(stderr, "Usage: %s [OPTION]...\n", calledname);
   fprintf(stderr, "Note: Options on commandline override those in the configfile\n");
   fprintf(stderr, "Known options:\n");
   while( known_options[i].varname != NULL )
   {
	  char str[64];
	  if( known_options[i].vartype == MB_OPTIONS_VARTYPE_BOOL || 
		  known_options[i].vartype == MB_OPTIONS_VARTYPE_BOOL+MB_OPTIONS_VARTYPE_MULTIPLE )
	  {
		 sprintf(str, "   --%s=<0/1>", known_options[i].varname);
		 fprintf(stderr, "%-30s default=%i\n", str, known_options[i].vardefault.var_bool);
	  }
	  if( known_options[i].vartype == MB_OPTIONS_VARTYPE_INT || 
		  known_options[i].vartype == MB_OPTIONS_VARTYPE_INT+MB_OPTIONS_VARTYPE_MULTIPLE )
	  {
		 sprintf(str, "   --%s=<int>", known_options[i].varname);
		 fprintf(stderr, "%-30s default=%i\n", str, known_options[i].vardefault.var_int);
	  }
	  if( known_options[i].vartype == MB_OPTIONS_VARTYPE_FLOAT || 
		  known_options[i].vartype == MB_OPTIONS_VARTYPE_FLOAT+MB_OPTIONS_VARTYPE_MULTIPLE )
	  {
		 sprintf(str, "   --%s=<float>", known_options[i].varname);
		 fprintf(stderr, "%-30s default=%f\n", str, known_options[i].vardefault.var_float);
	  }
	  if( known_options[i].vartype == MB_OPTIONS_VARTYPE_STRING || 
		  known_options[i].vartype == MB_OPTIONS_VARTYPE_STRING+MB_OPTIONS_VARTYPE_MULTIPLE )
	  {
		 sprintf(str, "   --%s=<string>", known_options[i].varname);
		 fprintf(stderr, "%-30s default=%s\n", str, known_options[i].vardefault.var_str);
	  }
	  fprintf(stderr, "\t\t%s\n", known_options[i].help);
	  i++;
   }
}

/*int main(int argc, char * argv[])*/
int mb_options_init(int argc, char * argv[])
{
   int i;
   char * filename;
   filename = mb_options_configfile;
   mb_options_init_array();
   /* first of all see if there is a --config option for an alternate configfile */
   for(i=0; i < argc; ++i)
	  if( (strlen(argv[i]) >= 8) && (strncasecmp(argv[i], "--config", 8) == 0) )
	  {
		 if( strlen(argv[i]) > 9 )
		 {
			if( argv[i][8] == '=' )
			   filename = argv[i] + 9;
		 }
		 else if( strlen(argv[i]) == 8 )
			filename = argv[++i];
		 else if( argv[i][8] == '=' ) /* we know that it must be 9 chars long */
			printf("configfile name not specified, using default!\n");
	  }
   printf("reading configfile '%s'\n", filename);
   mb_options_read_configfile(filename);
   mb_options_process_cmdline(argc, argv);
   
/*   mb_options_usage(argv[0]);*/
   return 0;
}


void mb_options_done()
{
   int i;
   for(i=0; i<MB_OPTIONS_MAXOPTIONS; ++i)
	  if(  options[i].vartype == MB_OPTIONS_VARTYPE_STRING ||
		   options[i].vartype == MB_OPTIONS_VARTYPE_STRING+MB_OPTIONS_VARTYPE_MULTIPLE )
		 free(options[i].variable.var_str);		 
}
