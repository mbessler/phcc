/* ------------------------------------------------------------------------
   File Manipulation Routines for Linux PIC programmer
   
   Copyright 1994-1998 by Brian C. Lane
   ---------------------------------------------------------------------
   01/01/98	Writing the writeihex16 routine

   12/31/97	The confusion of the Intel hex file format is between the
   		Intel HEX 16 format (which makes sense) and the Intel Hex
   		8 (8m) format. The HEX16 format works with 16 bit words
   		in normal byte ordering (MSB first, like God meant it to
   		be!). The HEX8 format goes in 8 bit bytes and reverses
   		the byte order of each word! Makes no sense since it is
   		supposed to be operating on bytes, why would it reverse
   		the order?

		Also, the addresses in INHX8M files are twice as large
		as they should be (because its counting by bytes)
   		
   		So, I am writing new file save and recall routines that
   		will be able to handle either type, on the fly.

   ========================================================================
   
   
   ------------------------------------------------------------------------ */

#include "ihexfile.h"

#define OK 0
#define ENDOFFILE 1
#define CHECKSUMERROR -1
#define JUNK -2

int line=0;

/* -------------------------------------------------------------------------
   Read a line from hexfile of either INHX16 or INHX8M format type, detecting
   the format automagicly by the bytecount and length of the line
   ------------------------------------------------------------------------ */
int readihexline(FILE *fp, unsigned int *mem, int * startaddress, int * words)
{
	char  buf[81];
	char  *bptr;
	int   cksum, x;
	unsigned int count, addr, type, data;

	if( fgets( buf, 81, fp ) == NULL )
		return ENDOFFILE;
	bptr = buf;
	if( *bptr != ':' )
		return JUNK;
	/* Strip off any trailing CR/LF if present, check twice. */
	if( ( buf[strlen(buf)-1] == 0x0D ) || ( buf[strlen(buf)-1] == 0x0A) )
		buf[strlen(buf)-1] = 0x00;
	if( ( buf[strlen(buf)-1] == 0x0D ) || ( buf[strlen(buf)-1] == 0x0A) )
		buf[strlen(buf)-1] = 0x00;
	/* Get the byte count for this line */
	bptr++;
	if( !sscanf( bptr, "%02X", &count ) )
		return JUNK;
     
	/* Get the address */
	bptr += 2;
	if( !sscanf( bptr, "%04X", &addr ) )
		return JUNK;
        *startaddress = addr;
	
	bptr += 4;
	if( !sscanf( bptr, "%02X", &type ) )
	  return JUNK;

	/* End of file field, exit now */      
	if( type == 0x01 )
	  return ENDOFFILE; /* no more content available */

	/* Start calculating the checksum */
	cksum = (count & 0xFF);
	cksum += ((addr >> 8) & 0xFF);
	cksum += (addr & 0xFF);
	cksum += (type & 0xFF);

	/* Point to the first data byte */
	bptr += 2; 
	  
	/* Figure out if its INHX16 or INHX8M
	   If the length of the string - 11 == count then its INHX8M
	   If it == count*2 then its INHX16
	 */
	if( (strlen(buf) - 11) == (count * 2) )
	{
		/* Processing a INHX8 line */
		for( x = 0; x < (count/2); x++ )
		{
			if( !sscanf( bptr, "%02X", &data ) )
				return JUNK;
			*startaddress = addr/2;
	    		mem[x] = data;
	    		cksum += (data & 0xFF);
	    		bptr += 2;
	    		if( !sscanf( bptr, "%02X", &data ) )
				return JUNK;
			mem[x] |= (data << 8);
			cksum += (data & 0xFF);
			bptr += 2;
	  	}
		*words = count/2;
	}
	else if( (strlen(buf) - 11) == (count * 4) ) 
	{
		/* Processing a INHX16 line */
		for( x = 0; x < count; x++ )
		{
			if( !sscanf( bptr, "%04X", &data ) )
				return JUNK;
			mem[x] = data;
			cksum += ((data >> 8) & 0xFF);
			cksum += (data & 0xFF);
			bptr += 4;
		}
		*words = count;
	}
	else
		return JUNK;

	/* Process the checksum */
	if( !sscanf( bptr, "%02X", &data ) )
		return JUNK;
	    
	if( ((-cksum) & 0xFF) != (data & 0xFF) )
	{
		printf("Checksum mismatch on line %d (%02X != %02X\n", line, data & 0xFF, cksum & 0xFF );
		return CHECKSUMERROR;
	}
	return OK;
}


/* -------------------------------------------------------------------------
   Write a Intel INHX16 formatted file
   ------------------------------------------------------------------------ */
int writeihex16( FILE *fp, unsigned int *mem, int min, int max )
{
  int x, i, cksum;

  x = min;			/* Starting Address */  
  while(1)
  {
    cksum = 0;
    /* Can we print a line of 8? */
    if( x + 7 < max )
    {
      /* Print count and address and type */
      fprintf( fp, ":08%04X00", x );
      cksum = 8;
      cksum += (x >> 8) & 0xFF;
      cksum += x & 0xFF;

      /* Print the 8 data words	    */      
      for( i = x; i < x+8; i++ )
      {
        fprintf( fp, "%04X", mem[i] );
        cksum += (mem[i] >> 8) & 0xFF;
        cksum += mem[i] & 0xFF;
      }
      x += 8;
    } else {
      fprintf( fp, ":%02X%04X00", max - x, x );
      cksum = (max - x) & 0xFF;
      cksum += (x >> 8) & 0xFF;
      cksum += x & 0xFF;

      /* Print the data words	    */      
      for( i = x; i < max; i++ )
      {
        fprintf( fp, "%04X", mem[i] );
        cksum += (mem[i] >> 8) & 0xFF;
        cksum += mem[i] & 0xFF;
      }
      x = max;
    }
    /* Print the checksum (2's complement) */
    fprintf( fp, "%02X\n", (-cksum) & 0xFF );
    
    if( x >= max )
      return TRUE;
  }
  return TRUE;
}


/* -------------------------------------------------------------------------
   Write the end of the INHX16 file
   ------------------------------------------------------------------------ */
int writeiend16( FILE *fp )
{
  return( fputs( ":00000001FF\n", fp ) );
}

   
