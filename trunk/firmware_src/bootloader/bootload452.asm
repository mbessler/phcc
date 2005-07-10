;;=================================================
;; bootload452.asm
;; 
;; part of cockpit.varxec.de, a home cockpit building site dedicated to
;; free software and the open source/free software, multiplatform
;; flight simulator FlightGear.
;;
;; This is a bootloader for the 18F series of Microchip's PIC microcontrollers.
;; It uses the RS-232 serial port to communicate with a host to send/receive
;; Flash programs/EEProm data
;; 
;; The protocol is described in Application Note 851.
;; 
;; currently not implemented is the checksum checking.
;; 
;; upon power-up, the bootloader waits a few seconds, and if nothing received
;; from the host, it boots the user program at 0x200.
;; if it receives something before the timeout, it will go into "bootloader mode",
;; waiting for commands from host.
;;   [this is different from the original, with used a byte in EEPROM to determine
;;    what to do]
;;
;;
;;  The core of this bootloader is from Microchip's AN851... 
;;  so most of the credit goes to them. The comments in this file are my own.
;;  I think they are more clear compared to mchip's original.
;;  
;;  Some restrictions apply to this code since it is basically copied
;;  from AN851:
;;  ================= Original Microchip License Agreement ==============
;;	                Software License Agreement
;;  The software supplied herewith by Microchip Technology Incorporated (the  Company )
;;  is intended and supplied to you, the Company s customer, for use solely and
;;  exclusively on Microchip products. The software is owned by the Company and/or
;;  its supplier, and is protected under applicable copyright laws. All rights are 
;;  reserved. Any use in violation of the foregoing restrictions may subject the
;;  user to criminal sanctions under applicable laws, as well as to civil liability
;;  for the breach of the terms and conditions of this license.
;;
;;  THIS SOFTWARE IS PROVIDED IN AN  AS IS  CONDITION. NO WARRANTIES,
;;  WHETHER EXPRESS, IMPLIED OR STATU-TORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED 
;;  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICU-LAR PURPOSE APPLY TO THIS
;;  SOFTWARE. THE COMPANY SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL,
;;  INCIDENTAL OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
;;  ======================================================================
;;  
;;  However, my changes and comments are available under:
;;
;; Copyright (c) 2003-2005 by Manuel Bessler <m.bessler AT gmx.net>
;; 
;;  The full text of the legal notices is contained in the file called
;;  COPYING, included with this distribution.
;;
;;  This program is free software; you can redistribute it and/or
;;  modify it under the terms of the GNU General Public License
;;  as published by the Free Software Foundation; either version 2
;;  of the License, or (at your option) any later version.
;; 
;;  This program is distributed in the hope that it will be useful,
;;  but WITHOUT ANY WARRANTY; without even the implied warranty of
;;  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
;;  GNU General Public License for more details.
;;
;;  You should have received a copy of the GNU General Public License
;;  along with this program; if not, write to the Free Software
;;  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
;; 
;;=================================================

;;=================================================
;; Circuit setup:
;;   two LEDs on RB7 (one to GND, one to Vcc)
;; 
;; Program layout:
;;
;; Changes:
;;    20050118  LED blink seq.
;;    20050429  (real-)bootblock write protect
;;=================================================

;;;;; uncomment the processor you use
;;		LIST P=18F442, R=DEC    ; what the PIC we use, decimal system as default
		LIST P=18F452, R=DEC    ; what the PIC we use, decimal system as default

;;;;;;; if you want the bootblock (first 512 bytes of program mem) to be write protected, define BOOTLDR_WRPROT
#define BOOTLDR_WRPROT

;;;; @40Mhz (10MHz x 4)

	IFDEF __18F452
        #include "p18f452.inc"  ; include appropriate processor definitions
		; set config bits:
		__CONFIG  _CONFIG1H, _OSCS_OFF_1H & _HSPLL_OSC_1H  ;;; _HS_OSC_1H
		__CONFIG  _CONFIG2L, _PWRT_ON_2L & _BOR_ON_2L & _BORV_20_2L
		__CONFIG  _CONFIG2H, _WDT_OFF_2H & _WDTPS_32_2H
		__CONFIG  _CONFIG3H, _CCP2MX_OFF_3H
		__CONFIG  _CONFIG4L, _STVR_ON_4L & _LVP_OFF_4L & _DEBUG_OFF_4L
		__CONFIG  _CONFIG5L, _CP0_OFF_5L & _CP1_OFF_5L & _CP2_OFF_5L & _CP3_OFF_5L
		__CONFIG  _CONFIG5H, _CPB_OFF_5H & _CPD_OFF_5H

		__CONFIG  _CONFIG6L, _WRT0_OFF_6L & _WRT1_OFF_6L & _WRT2_OFF_6L & _WRT3_OFF_6L
#ifdef BOOTLDR_WRPROT
		__CONFIG  _CONFIG6H, _WRTC_OFF_6H & _WRTB_ON_6H & _WRTD_OFF_6H
#else
		__CONFIG  _CONFIG6H, _WRTC_OFF_6H & _WRTB_OFF_6H & _WRTD_OFF_6H
#endif  ;; BOOTLDR_WRPROT

		
		__CONFIG  _CONFIG7L, _EBTR0_OFF_7L & _EBTR1_OFF_7L & _EBTR2_OFF_7L & _EBTR3_OFF_7L
		__CONFIG  _CONFIG7H, _EBTRB_OFF_7H

	ENDIF

	IFDEF __18F442
        #include "p18f442.inc"  ; include appropriate processor definitions
		; set config bits:
		__CONFIG  _CONFIG1H, _OSCS_OFF_1H & _HSPLL_OSC_1H  ;;; _HS_OSC_1H
		__CONFIG  _CONFIG2L, _PWRT_ON_2L & _BOR_ON_2L & _BORV_20_2L
		__CONFIG  _CONFIG2H, _WDT_OFF_2H & _WDTPS_32_2H
		__CONFIG  _CONFIG3H, _CCP2MX_OFF_3H
		__CONFIG  _CONFIG4L, _STVR_ON_4L & _LVP_OFF_4L & _DEBUG_OFF_4L
		__CONFIG  _CONFIG5L, _CP0_OFF_5L & _CP1_OFF_5L & _CP2_OFF_5L & _CP3_OFF_5L
		__CONFIG  _CONFIG5H, _CPB_OFF_5H & _CPD_OFF_5H
		__CONFIG  _CONFIG6L, _WRT0_OFF_6L & _WRT1_OFF_6L & _WRT2_OFF_6L & _WRT3_OFF_6L
#ifdef BOOTLDR_WRPROT
		__CONFIG  _CONFIG6H, _WRTC_OFF_6H & _WRTB_ON_6H & _WRTD_OFF_6H
#else
		__CONFIG  _CONFIG6H, _WRTC_OFF_6H & _WRTB_OFF_6H & _WRTD_OFF_6H
#endif  ;; BOOTLDR_WRPROT
		__CONFIG  _CONFIG7L, _EBTR0_OFF_7L & _EBTR1_OFF_7L & _EBTR2_OFF_7L & _EBTR3_OFF_7L
		__CONFIG  _CONFIG7H, _EBTRB_OFF_7H

	ENDIF

;;		ERRORLEVEL -302			; disable 302 assembler warning messages


#define FREQTIMESFOUR 10    ;; for delays, specifies the multiple of 4 MHz we're running at.
							;;  eg. for 16MHz this would be 4, for 40MHz it would be 10

#define MAJOR_VERSION 0
#define MINOR_VERSION 1

#define LED				PORTB, 7
#define LED_TRIS		TRISB, 7


#define STX 0x0F
#define ETX 0x04
#define DLE 0x05

; Frame Format
;
;  <STX><STX>[<COMMAND><DATALEN><ADDRL><ADDRH><ADDRU><...DATA...>]<CHKSUM><ETX>



		; define SRAM data areas/variables/file registers
		CBLOCK 0x100
			secondstowait
			overrunflag
			
		ENDC

		CBLOCK 0x000
			checksum, counter 
			abtime_h, abtime_L
			rxbuf, txbuf
		ENDC
		CBLOCK 0x008
			cmd, payloaddatalen, adr_l, adr_h, adr_u, payloaddata
		ENDC
#define BUFFER cmd


;;-------------------------------
;; program start point/reset vector
;;-------------------------------

		ORG		0x0000
		GOTO	init

;;-------------------------------
;; (high priority) interrupt vector
;;-------------------------------
		ORG		0x0008
		BRA		interrupt


;;-------------------------------
;; (low priority) interrupt vector
;;-------------------------------
		ORG		0x0018
		BRA		lowprio_interrupt


;;-------------------------------
;; generic initialization
;;-------------------------------
init:
		;;use RB<0:3> for debugging with LEDs in the 74HCT154 socket
		BCF		LED_TRIS			; LED pin is output
		BCF		LED					; LED off
		;; SETtting a bit in the TRISx registers -> input
		;; CLEARing a bit in the TRISx registers -> output

;;-------------------------------
;; application specific initialization
;;-------------------------------
		MOVLW	5
		MOVWF	secondstowait	; set secondstowait to 5 secons for timer interrupts


		;; enable USART for RS-232
		BCF		TRISC,6				; RS-232 Tx
		BSF		TRISC,7				; RS-232 Rx
		CLRF	TXSTA
		CLRF	RCSTA
		BSF		RCSTA,SPEN
		BSF		RCSTA,CREN
		BSF		TXSTA,TXEN
		BCF		TXSTA,SYNC
		BSF		TXSTA,BRGH
		MOVLW	129					; BRGH=1, 19200 baud **** @40MHz ****
		MOVWF	SPBRG
;;-------------------------------
;; let serial port stabilize
;;-------------------------------
		MOVF	RCREG,W
		MOVF	RCREG,W				; flush receive register
		CLRF	overrunflag
;;;;; serial port init end ;;;

		RCALL	send_bootprompt		; send alive string
		RCALL	reinit_timer0

		CLRF	INTCON				; zero out INTCON, disable all interrupt stuff
		MOVLW	b'10000111'			; Timer0 enabled, 16bit, int. source, edge whatever, prescaler on,
									; prescaler set to 1:256
		MOVWF	T0CON				; 5 seconds timer should be started now

		
waitforhost:
		RCALL	rs232_recv_with_timeout
					; if rs232_recv returns, we have received something. 
					; then disable interrupts and prepare to receive new userprogram
	
; Frame Format
;
;  <STX><STX>[<COMMAND><DATALEN><ADDRL><ADDRH><ADDRU><...DATA...>]<CHKSUM><ETX>

startpacket:
		RCALL	rs232_recv
		XORLW	STX
		BNZ		startpacket			; every packet starts w/ STX
		RCALL	rs232_recv
		XORLW	STX
		BNZ		startpacket			; look for 2nd STX

		LFSR	FSR0, BUFFER		; get buffer ready for data
		CLRF	checksum
		CLRF	counter

nextbyte:
		RCALL	rs232_recv			; get whatever's next

		XORLW	STX					; if it was another STX, discard packet
		BZ		startpacket			; and start over

		MOVF	rxbuf, W			; restore received byte into W
		XORLW	ETX					; was it a ETX
		BZ		check_checksum		; check integrity of packet

		MOVF	rxbuf, W			; restore received byte into W
		XORLW	DLE					; was it a DLE, then we have to get the next byte and treat it raw.
		BNZ		noescaped

		RCALL	rs232_recv			; get next byte, this is the escaped byte

noescaped:
		MOVF	rxbuf, W			; restore received byte into W
		ADDWF	checksum,F			; update checksum
		MOVWF	POSTINC0			; and also store (via indirect addr) the received byte to buffer area
		DCFSNZ	counter,F			; make sure we don't buffer more that 256 bytes
		BRA		startpacket			; if we ran over the buffer size of 256, start over for new packet
		BRA		nextbyte			; else get next byte for this packet

check_checksum:
		; I don't really understand how AN851's checksum checking at this point works.
;;		MOVF	checksum
;;		BNZ		startpacket
		; for now we'll just ignore it.


;; do the common read/write setup

		MOVF	adr_l, W			; move adr_l
		MOVWF	TBLPTRL				; to TBLPTR low (for ops on program memory)
		MOVWF	EEADR				; and to EEADR  (for ops on eeprom data memory)
		
		MOVF	adr_h, W			; then similarly move adr_h
		MOVWF	TBLPTRH				; to table pointer high
		
		MOVFF	adr_u, TBLPTRU		; the same with upper

		LFSR	FSR0, payloaddata	; setup indirect addressing to data-payload from packet

		MOVF	payloaddatalen, W	; copy length of payload data
		MOVWF	counter				; into counter
		BTFSC	STATUS, Z			; if length was zero
		RESET						; then it must have been a reset packet <STX><STX>[<any><0x00>]<CHKSUM><ETX>
;;;;;;;;;		BRA		startpacket			; then it was an invalid packet


		; check what command was requested from host
		MOVF	cmd, W				; load command into W
		SUBLW	7					; to check if its within the range of valid commands
		BNC		startpacket			; if it wasn't, start over


		; now we set up a jumptable based on the command code
		CLRF	PCLATH
		CLRF	PCLATU
		RLNCF	cmd, W
		ADDWF	PCL, F

		; here's the jumptable
		BRA		cmd_version
		BRA		cmd_readprogmem
		BRA		cmd_writeprogmem
		BRA		cmd_eraseprogmem
		BRA		cmd_readdataeeprom
		BRA		cmd_writedataeeprom
		BRA		cmd_readprogmem       ;; cmd_readconfig
		BRA		cmd_writeconfig


;;--------------------------------------------
; In:	<STX><STX>[<0x00><0x02>]<0xFF><ETX>
; OUT:	<STX><STX>[<0x00><DATALEN><VERL><VERH>]<CHKSUM><ETX>
;;;;;;;;;;;;;;;;;;;;;;;;; ^^^^^^^ missing in orig docs !!!
cmd_version:
		MOVLW	MINOR_VERSION
		MOVWF	BUFFER + 2
		MOVLW	MAJOR_VERSION
		MOVWF	BUFFER + 3
		MOVLW	0x04
		BRA		sendpacket

;;--------------------------------------------
; In:	<STX><STX>[<0x01><DLEN><ADDRL><ADDRH><ADDRU>]<CHKSUM><ETX>
; OUT:	<STX><STX>[<0x01><DLEN><ADDRL><ADDRH><ADDRU><DATA>...]<CHKSUM><ETX>
cmd_readprogmem:
		TBLRD	*+					; reads byte from program memory pointed at TBLPTRx into TABLAT
		MOVFF	TABLAT, POSTINC0	; store memory byte into send buffer
		DECFSZ	counter,F			; read only as many bytes from prg mem as DLEN in request specified
		BRA		cmd_readprogmem		; as DLEN field in request from host specified
		MOVF	payloaddatalen, W	; prepare length field for reply packet
		ADDLW	0x05				; add 5 to it
		BRA		sendpacket			; and send packet

;;--------------------------------------------
; In:	<STX><STX>[<0x02><DLENBLOCK><ADDRL><ADDRH><ADDRU><DATA>...]<CHKSUM><ETX>
; OUT:	<STX><STX>[<0x02>]<CHKSUM><ETX>
cmd_writeprogmem:
		MOVLW	b'11111000'			; each write does 8 bytes at once, 
		ANDWF	TBLPTRL, F			; therefore zero out the lower 3 bits for the address

		MOVLW	0x08				; setup to count each eight bytes to go into holding registers 
									; before a 8byte packet gets written to flash prog. memory
load_holding_reg:
		MOVFF	POSTINC0, TABLAT	; get data from indirect addressing and put into table latch 
		TBLWT	*+					; write to data in table latch to holding register (#1-8)
		DECFSZ	WREG, F				; have we looped over all eight bytes ?
		BRA		load_holding_reg	; no? then next

		TBLRD	*-					; let TBLPTR point back into the 8 byte area we're working on.
									; here, we are not really interested in the Read from flash, but
									; rather in the decrementing of TBLPTR[L,H,U]. see Note 1 on the
									; page with "Flash Program Write Sequence" in the datasheet
		MOVLW	b'10000100'			; set bits: flash program memory, write enable
		MOVWF	EECON1				; in EECON1      EEPGD               WREN
		RCALL	unlock_and_start_write	; command sequence init for write to eeprom/flash
		
		TBLRD	*+					; let TBLPTR point into next 8 byte area
		DECFSZ	counter, F			; have we written everything ?  (in this case, counter counts 
									; the number of 8bytes packes, not bytes !!!)
		BRA		cmd_writeprogmem	; no: next 8 bytes
		BRA		sendack				; yes, send acknowledge

;;--------------------------------------------
; In:	<STX><STX>[<0x03><DLENROW><ADDRL><ADDRH><ADDRU>]<CHKSUM><ETX>
; OUT:	<STX><STX>[<0x03>]<CHKSUM><ETX>
cmd_eraseprogmem:
		MOVLW	b'10010100'			; access to flash memory, erase mode, write enable, 
		MOVWF	EECON1				; set        EEPGD          FREE         WREN
		RCALL	unlock_and_start_write	; command sequence init for write to eeprom/flash
		MOVLW	0x40				; set erase block size to 64bytes
		ADDWF	TBLPTRL, F			; add block size to start address ???
		CLRF	WREG				; zero out W (does not affect carry)
		ADDWFC	TBLPTRH, F			; add carry(+W) to TBLPTRH
		ADDWFC	TBLPTRU, F			; add carry(+W) to TBLPTRU
		DECFSZ	counter, F			; are we done yet ?
		BRA		cmd_eraseprogmem	; if not, next row

		BRA		sendack				; else send acknowldge to host

;;--------------------------------------------
; In:	<STX><STX>[<0x04><DLEN><ADDRL><ADDRH><0x00>]<CHKSUM><ETX>
; OUT:	<STX><STX>[<0x04><DLEN><ADDRL><ADDRH><0x00><DATA>...]<CHKSUM><ETX>
cmd_readdataeeprom:
		CLRF	EECON1				; start with a clean, defined EECON1
		BSF		EECON1, RD			; set the read bit, this leaves the read byte in EEDATA
		MOVFF	EEDATA, POSTINC0	; copy EEDATA into memory pointed to by indirect adressing
		INCF	EEADR, F			; point to next byte in eeprom data memory
		DECFSZ	counter, F			; are we done reading ?
		BRA		cmd_readdataeeprom	; no, then next byte
	
		MOVF	payloaddatalen, W	; get packet length
		ADDLW	0x05				; adjust packet size

		BRA		sendpacket

;;--------------------------------------------
; In:	<STX><STX>[<0x05><DLEN><ADDRL><ADDRH><0x00><DATA>...]<CHKSUM><ETX>
; OUT:	<STX><STX>[<0x05>]<CHKSUM><ETX>
cmd_writedataeeprom:
		MOVFF	POSTINC0, EEDATA	; 
		MOVLW	b'00000100'			; setup eeprom data memory write
		MOVWF	EECON1				; by setting bit WREN
		RCALL	unlock_and_start_write	; command sequence init for write to eeprom/flash
		BTFSC	EECON1, WR			; wait until write is completed
		BRA		$-2					; loop to prev. instruction if not yet completed
		INCF	EEADR, F			; point to next byte in eeprom data memory
		DECFSZ	counter, F			; are we done writing ?
		BRA		cmd_writedataeeprom	; no, next byte

		BRA		sendack				; yes, send acknowledge to host

;;--------------------------------------------
; In:	<STX><STX>[<0x07><DLEN><ADDRL><ADDRH><ADDRU><DATA>...]<CHKSUM><ETX>
; OUT:	<STX><STX>[<0x07>]<CHKSUM><ETX>
cmd_writeconfig:
		BRA		sendpacket


;;--------------------------------------------
;;--------------------------------------------
unlock_and_start_write:
		MOVLW	0x55				; magic sequence
		MOVWF	EECON2				; for 
		MOVLW	0xAA				; write
		MOVWF	EECON2				; to eeprom/flash
		BSF		EECON1, WR			; start write
		NOP
		RETURN


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

; Send the data buffer back.
;
; <STX><STX>[<DATA>...]<CHKSUM><ETX>
sendack:
		MOVLW	0x01
sendpacket:							; sendpacket expects the number of bytes(the <DATA> part) to send in W
		MOVWF	counter				; save counter value
		MOVLW	STX
		RCALL	rs232_send			; directly send STX for start of packet
		RCALL	rs232_send			; actually send two of them
		CLRF	checksum			; prepare checksum for new use
		LFSR	FSR0, BUFFER		; set up indirect addressing to buffer area
sendnextbyte:
		MOVF	POSTINC0, W			; load indirect from buffer, post-incrementing the indir. addr. register
		ADDWF	checksum,F			; update checksum
		RCALL	rs232_sendesc		; and send out via RS232
		DECFSZ	counter, F			; update counter
		BRA		sendnextbyte		; while not zero, send next

		NEGF	checksum			; finalize checksum by negating (twos-complement)
		MOVF	checksum, W			; prepare for send
		RCALL	rs232_sendesc		; and send out

		MOVLW	ETX
		RCALL	rs232_send			; directly send ETX
		
		BRA		startpacket			; DONE, get ready for next request from host


reinit_timer0:
		; initialize TMR0L/H for 1 second (prescaler 1:256) with 39062 (decimal) => 0x9896
		MOVLW	0x98
		MOVWF	TMR0H
		MOVLW	0x96
		MOVWF	TMR0L				; TMR0L should come after TMR0H (see datasheet)
		BCF		INTCON, TMR0IF		; clear Timer0 interrupt flag
		BTG		LED_TRIS			; toggle LED in/output. this will toggle between: "both LEDs on"/"only red LED on"
		RETURN


;;-------------------------------
;; RS-232 receive routine with TIMEOUT
;;-------------------------------
;;  received char is left in W
;;  loops until a char received or timeout
;;-------------------------------
rs232_recv_with_timeout:
		BTFSC	PIR1,RCIF					; poll for new received data
		BRA		rs232_recvd_before_timeout	; we received something before timeout
		BTFSC	INTCON, TMR0IF				; poll timer0 overflow flag
		BRA		check_timeout				; timeout ?
		BRA		rs232_recv_with_timeout		; else do it again

check_timeout:
		DCFSNZ secondstowait,F		; do we have more seconds to wait ? 
		BRA		bootuserprogram		; no, boot user program
		RCALL	reinit_timer0		; yes, reinit timeout
		BRA		rs232_recv_with_timeout		; do it again

rs232_recvd_before_timeout:
        MOVF	RCREG,W				; serially received byte -> W
		BTFSC	RCSTA,OERR			; overrun error ?
		GOTO	receiveoverrun_to
		RETURN
receiveoverrun_to:
		BCF		RCSTA,CREN			; clear CREN to clear OERR flag
		BSF		RCSTA,CREN			; set CREN to reenable continuous receive
		BSF		overrunflag,0
		MOVF	RCREG,W				; flush receive register
		RETURN

bootuserprogram:
		; LED pin to output, off
		BCF		LED_TRIS			; LED pin is output
		BCF		LED					; green LED off, red LED on
		; adjust stack
		CLRF	STKPTR				; clear stackpointer, leaving an empty stack ready for user program
		; hand control over to userprogram
		BRA		userprogram


;;-------------------------------
;; RS-232 receive routine
;;-------------------------------
;;  received char is left in W
;;  loops until a char is actually received !
;;-------------------------------
rs232_recv:
		BTFSS	PIR1,RCIF			; poll for new received data
		GOTO	rs232_recv
        MOVF	RCREG,W				; serially received byte -> W
		MOVWF	rxbuf				; save a copy of the received byte for convenience
		BTFSC	RCSTA,OERR			; overrun error ?
		GOTO	receiveoverrun
		RETURN
receiveoverrun:
		BCF		RCSTA,CREN			; clear CREN to clear OERR flag
		BSF		RCSTA,CREN			; set CREN to reenable continuous receive
		BSF		overrunflag,0
		MOVF	RCREG,W				; flush receive register		
		RETURN

;;-------------------------------
;; RS-232 send routine
;;-------------------------------
;;  send char in W via RS-232
;;  waits until sending is done !
;;-------------------------------
rs232_sendesc:  ; first part does check if escaping with DLE is necessary
		MOVWF	txbuf				; make working copy of data to send

		XORLW	STX					; is it the same as a STX
		BZ		escapeDLE			; escape it

		MOVF	txbuf, W			; restore working copy
		XORLW	ETX					; check if it looks same as ETX
		BZ		escapeDLE			; if yes, escape it
		
		MOVF	txbuf, W			; restore working copy
		XORLW	DLE					; check if it looks same as DLE
		BNZ		noDLE				; if no, jump over escaping routine

escapeDLE:
		MOVLW	DLE
		RCALL	rs232_send
		
noDLE:
		MOVF	txbuf, W			; restore working copy and let rs232_send handle the rest

rs232_send:
		MOVWF	TXREG				; send out data in W via serial port

rs232_send_wait:
		BTFSS	TXSTA,TRMT
		GOTO	rs232_send_wait		; wait till byte sent
		RETURN


;;-------------------------------
;; send a "alive" message to PC
;;-------------------------------
;;   sends string 
;;-------------------------------
send_bootprompt:
                MOVLW   'B'
                RCALL    rs232_send
                RETURN

send_str_rdy:
                MOVLW   'R'
                RCALL    rs232_send
				RETURN

send_str_up:
                MOVLW   'U'
                RCALL    rs232_send
				RETURN


;;-------------------------------
;; user program start at 0x0200
;;-------------------------------
		ORG		0x0200
userprogram:
		CALL	send_str_up
		RESET		

;;-------------------------------
;; via bootloader remapped interrupt vector (high priority)
;;-------------------------------
		ORG		0x0208
interrupt:							; global interrupts automatically disabled on entry!

done_int:
		RETFIE						; return from interrupt and set GIE (global interrupt enable bit)

;;-------------------------------
;; via bootloader remapped interrupt vector (low priority)
;;-------------------------------
		ORG		0x0218
lowprio_interrupt:					; global interrupts automatically disabled on entry!

done_int_lo:
		RETFIE						; return from interrupt and set GIE (global interrupt enable bit)


;;=================================================
;; End of program
;;=================================================
	END
