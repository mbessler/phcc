#ifndef _IHEXFILE_H
#define _IHEXFILE_H

#include <stdio.h>
#include <string.h>

#define OK 0
#define ENDOFFILE 1
#define CHECKSUMERROR -1
#define JUNK -2

#define TRUE 1
#define MAXPICSIZE 65535

extern int readihexline(FILE *, unsigned int *, int *, int * );
extern int readihex( FILE *, unsigned int *, int *, int * );
extern int writeihex16( FILE *, unsigned int *, int, int );
extern int writeiend16( FILE * );

#endif /* _IHEXFILE_H */
