;;=================================================
;; .asm
;; 
;; 
;; PHCC - PIC HomeCockpit Controller
;;  a Microcontroller based Interface Solution for Homecockpits
;;  featuring an open modular and extensible design
;;  
;;  This solution (both hardware designs and software)
;;  is licensed under the GNU Public License (GPL)
;;  and therefore is Open Source, available as Free Software.
;;
;;
;; part of cockpit.varxec.de, a home cockpit building site dedicated to
;; free software and the open source/free software, multiplatform
;; flight simulator FlightGear.
;;
;;
;; Copyright (c) 2003, 2004 by Manuel Bessler <m.bessler AT gmx DOT net>
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
;;   see http://cockpit.varxec.net/electronic/PHCC.html
;; 
;; Program layout:
;;	FSR0 is used for keymatrix buffer
;;  FSR1 is used for send/receive buffer
;;  FSR2 is used for analog in buffer stuff
;;
;;=================================================

        LIST P=18F452, R=DEC    ; what the PIC we use, decimal system as default
        #include "p18f452.inc"  ; include appropriate processor definitions

;;;; @40Mhz (10MHz x 4)

;;; #define REV3_BOARD			;; define the board/hardware revision you have

#define FREQTIMESFOUR 10    ;; for delays, specifies the multiple of 4 MHz we're running at.
							;;  eg. for 16MHz this would be 4, for 40MHz it would be 10



#define TXD 			PORTC, 6		;; data outgoing
#define RXD 			PORTC, 7		;; data incoming
#define RTS 			PORTC, 5		;; outgoing hardware flow control
#define CTS 			PORTC, 2		;; incoming hardware flow control
#define RTSTRIS			TRISC, 5		;; direction register for RTS
#define CTSTRIS			TRISC, 2		;; direction register for CTS


#define	DO_A_CLK 		PORTC, 1		;; Digital-Out Type A Clock line
#define DO_A_DATA 		PORTC, 0		;; Digital-Out Type A Data line
#define	DO_A_CLK_TRIS 	TRISC, 1		;; direction register setup
#define	DO_A_DATA_TRIS 	TRISC, 0		;; direction register setup


#define DO_B_CLK        PORTB, 5		;; Digital-Out Type B Clock line
#define DO_B_DATA       PORTB, 4		;; Digital-Out Type B Data line
#define DO_B_STO        PORTB, 6		;; Digital-Out Type B Store line
#define DO_B_CLK_TRIS   TRISB, 5		;; direction register setup
#define DO_B_DATA_TRIS  TRISB, 4		;; direction register setup
#define DO_B_STO_TRIS   TRISB, 6		;; direction register setup


#define ANP1 			PORTA, 5		;; AN4
#define ANP2 			PORTA, 3		;; AN3
#define ANP3 			PORTA, 2		;; AN2
#define AN4067A 		PORTA, 0		;; AN0
#define AN4067B 		PORTA, 1		;; AN1

#define ADR0 			PORTA, 4		;; 4-bit address to 
#define ADR1 			PORTE, 0		;; the two 4067s  and 
#define ADR2 			PORTE, 1		;; 3-bit address for all 74x138
#define ADR3 			PORTE, 2		;;

#define KEYMATRIX_IN_PORT PORTD		;; port to read in a row of the keymatrix
#define KEYMATRIX_IN_DIRREG	TRISD	;; direction register for above port


#define DOA_BITTIMING_US	500			;; bitlength in microseconds 

#define	NO_KEYMATRIX_ROWS 128		;; number of rows in matrix (ie. 128 at max: 1024inputs / 8bits per row)

#define NO_ANALOG_CHANNELS 35
#define ANALOG_CHANNEL_STORAGE_SIZE 2*NO_ANALOG_CHANNELS

;;;;;; commands from host
#define HOSTCMD_IDLE					0x00
#define HOSTCMD_RESET					0x01
#define HOSTCMD_STARTTALKING			0x02
#define HOSTCMD_STOPTALKING				0x03
#define HOSTCMD_KEYMATRIXMAP			0x04
#define HOSTCMD_ANALOGMAP				0x05
#define HOSTCMD_I2CSEND					0x06
#define HOSTCMD_DOASEND					0x07
#define HOSTCMD_DOBSEND					0x08
#define	MAXCMD						0x08		; always the last/highest value of HOSTCMD_...

;;; for receive state machine
#define RECVSTATE_IDLE					0x00
#define RECVSTATE_RESET1				0x01    ; have to send reset byte 3 times to reset device
#define RECVSTATE_RESET2				0x02
#define RECVSTATE_DOASEND1				0x03
#define RECVSTATE_DOASEND2				0x04
#define RECVSTATE_DOASEND3				0x05
#define RECVSTATE_DOBSEND1				0x06
#define RECVSTATE_DOBSEND2				0x07
#define RECVSTATE_I2CSEND1				0x08
#define RECVSTATE_I2CSEND2				0x09
#define RECVSTATE_I2CSEND3				0x0a
#define RECVSTATE_I2CSEND4				0x0b



#define BANK_global 0x0
		CBLOCK 0x0					;; 0x0 - 0x80 is part of the accessbank
			Wsave, BSRsave, STATUSsave
			delaycounter_freq
			delaycounter_inner,delaycounter_outer
delaycounterdoa
			overrunflag

			tmpFIFO, FIFOrdy
			inFIFO232first, inFIFO232last
			outFIFO232first, outFIFO232last
			recv_byte
			recv_state
			talk, gie_flag, an_changed

			keyinput_old, keyinput_new, keyinput_delta, col_addr
			doa_devaddr, doa_subaddr, doa_data
			dob_addr, dob_data
			i2cout_addr, i2cout_subaddr, i2cout_data
			anbufH, anbufL, an_addr
			
			tmp, tmp2, antmp, keytmp
			counter, counter2, counterkmx, counteran, ledcounter
			adr4067
			adr154
		ENDC

#define BANK_analog 0x1
		CBLOCK 0x100				;; bank 1
			analogin:ANALOG_CHANNEL_STORAGE_SIZE			
;;;;;;;;;;;;;			analogin_last:ANALOG_CHANNEL_STORAGE_SIZE			
	;; + analog values are 10bit.
	;; + LSB aligned.
	;; + two bytes RAM used per value, first the High byte, then the Low byte
	;; + analogin_last holds the previous value (used for filtering)
		ENDC

#define BANK_matrix 0x2
		CBLOCK 0x200				;; bank 2
			keymatrixstate:128
			keymatrixdebounce:128
		ENDC

#define FIFOsize 256
#define BANK_fifo_in 0x3
		CBLOCK 0x300				;; bank 3
			inFIFO232:FIFOsize
		ENDC

#define BANK_fifo_out 0x4
		CBLOCK 0x400				;; bank 4
			outFIFO232:FIFOsize
		ENDC		

;;-------------------------------
;; via bootloader remapped program start point/reset vector
;;-------------------------------
		ORG		0x0200
startuserprogram:
		BRA		init
;;-------------------------------
;; via bootloader remapped interrupt vector (high priority)
;;-------------------------------
		ORG		0x0208
interrupt:
		BRA		isr					; not enough space here jump to isr for processing interrupts

;;-------------------------------
;; via bootloader remapped interrupt vector (low priority)
;;-------------------------------
		ORG		0x0218
		RETFIE

;;-------------------------------
;; real program starts here
;;-------------------------------
init:
		CLRF	BSR					; set bank select register to default
		BCF		INTCON, GIE			; disable all interrupts that may be left over from bootloader

		MOVLW	0x00
		MOVWF	PORTB		; all leds off test end
		;; SETtting a bit in the TRISx registers -> input
		;; CLEARing a bit in the TRISx registers -> output

		BCF		PORTB, 7		;; LED off
		CALL		delay1sec
		BSF		PORTB, 7		;; LED on
		CALL		delay1sec
		BCF		PORTB, 7		;; LED off
		CALL		delay1sec
		BSF		PORTB, 7		;; LED on
		CALL		delay1sec

		MOVLW	RECVSTATE_IDLE		; initialize state machine
		MOVWF	recv_state			; variable state holds the current state of the state machine

		CALL	setup_everything	; call the setup routines for the different subsystems

		BSF		INTCON,	GIE			; GLOBALLY ENABLE INTERRUPTS
	
;;-------------------------------
;; main loop start
;;-------------------------------

	CLRF	ledcounter
main:
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		BCF		INTCON, GIE
		CALL	rs232inFIFOprocess
		BSF		INTCON, GIE
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		BCF		INTCON, GIE
		CALL	rs232fifo2send
		BSF		INTCON, GIE
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
;	DCFSNZ	ledcounter, F

		NOP
		NOP

		BRA		main



;;===============================
;; main loop end
;;===============================


;;===============================
;;       --== ISR ==--
;;  Interrupt Service Routine
;;===============================
isr:
		;; save registers
		MOVWF	Wsave
		MOVFF	STATUS, STATUSsave
		MOVFF	BSR, BSRsave
		;; set up addressing to bank 0 for data memory

		;; start interrupt processing
		BTFSC	PIR1, RCIF			; was it a serial port receive int?
		RCALL	rs232recv2fifo		; handle it

;;		BTFSC	PIR1, TMR1IF		 	; TODO: make sure that not only the IF is set but also the IntEnable bit
;;		BRA		t1_overflow

		BTFSC	INTCON, TMR0IF		; was it a timer0 overflow ?
		RCALL	timer0_int

		BRA		end_interrupt		; non-handled interrupt? then go to end
;;;;;;;;;;;;;;;;;;;;


;t1_overflow:	
;;	BTG		PORTB, 7
;		CLRF	TMR1H
;		CLRF	TMR1L
;		BCF		PIR1, TMR1IF		; clear int flag
;		BRA		end_interrupt		; done, go to end of int


end_interrupt:
		;; restore addessing
		MOVFF	BSRsave, BSR
		MOVF	Wsave, W
		MOVFF	STATUSsave, STATUS
		RETFIE
;;===============================
;; --== ISR ==-- end
;;===============================

timer0_int:
	BTG		PORTB, 7
		CALL	keymatrix_in            ; read in one column of the keymatrix
		CALL   read_analog_pri1        ; read ANP1
		CALL   read_analog_pri2        ; read ANP2
		CALL   read_analog_pri3        ; read ANP3
		CALL   read_analog_secA        ; read analog value of first 4067 
		CALL   read_analog_secB        ; read analog value of second 4067 
		CALL	inc_addr                        ; increment address to 4067 mux

		MOVF    adr4067, W                      ; load adr4067 into W
		ANDLW   b'00000111'                     ; mask out unused upper 5 bits
		XORLW   0x00                            ; will eval to zero if inc_addr rolled lower 
		BTFSC   STATUS, Z                       ; 3 bits of adr4067 over to zero
		CALL   inc_154                         ; yes, then increment adr154            

		MOVLW	0x01
		MOVWF	TMR0H
		MOVLW	0x04				; 0x0104 equals about 1 sec
		MOVWF	TMR0L				; TMR0L should come after TMR0H (see datasheet)
		BCF		INTCON, TMR0IF		; clear Timer0 interrupt flag
		
		RETURN



;;--------------------------------
;; rs232recv2fifo
;;   gets called by ISR
;;   puts received byte in buffer
;;   adjusts buffer pointers
;;--------------------------------
rs232recv2fifo:
		BSF		RTS					; RTS/CTS hardware handshake: NOT ready to receive while processing
		LFSR	FSR1, inFIFO232		; pointer to buffer
		MOVF	inFIFO232last, W	; last position
		ADDWF	FSR1L, F			; is new position in buffer for received byte
		MOVF	RCREG, W			; serially received byte -> W
		MOVWF	INDF1				; store at last position

		INCF	inFIFO232last, F	; point to new (empty) position
		; check for wrap-around
		MOVLW	FIFOsize-1			; buffersize -> for comparison
		CPFSEQ	inFIFO232last		; compare F with W, skip if eqal
		BRA		nowrap_inFIFOlast	; not equal, then move on
		CLRF	inFIFO232last		; otherwise we need to adjust inFIFO232last
nowrap_inFIFOlast:
		BSF		FIFOrdy, 0			; set first bit in FIFOrdy => data avail for processing
		; check for transmission errors
		BTFSC	RCSTA,OERR			; overrun error ?
		BRA		receiveoverrun
		BRA		receive_done		; else done
receiveoverrun:
		BCF		RCSTA,CREN			; clear CREN to clear OERR flag
		BSF		RCSTA,CREN			; set CREN to reenable continuous receive
		BSF		overrunflag,0
		MOVF	RCREG,W				; flush receive register
receive_done:
		BCF		RTS					; RTS/CTS hardware handshake: we're ready to receive
		BCF		PIR1, RCIF			; clear int flag
		RETURN

;;--------------------------------
;; rs232inFIFOprocess
;;   processes data in inFIFO232
;;--------------------------------
rs232inFIFOprocess:
		; check if data is available
		MOVF	inFIFO232first, W
		CPFSEQ	inFIFO232last		; if( inFIFOfirst == inFIFOlast )
		BRA		inFIFOproc_do		; not equal, then data is available for processing
		BRA		inFIFOproc_done		; equal, the FIFO is empty, we can stop processing
inFIFOproc_do:
		; else process the byte		
		LFSR	FSR1, inFIFO232
		ADDWF	FSR1L, F			; inFIFOfirst + base address of inFIFO
		MOVF	INDF1, W
		MOVWF	recv_byte			; save received byte where it is expected by some routines
		CALL	recvstatemachine	; let the statemachine decide what to do with the byte
;;	CALL	send2host
		INCF	inFIFO232first, F	; byte in buffer processed, move pointer to next
		; check for wrap-around
		MOVLW	FIFOsize-1			; buffersize -> for comparison
		CPFSEQ	inFIFO232first		; compare F with W, skip if eqal
		BRA		inFIFOproc_done		; not equal, then move on
		CLRF	inFIFO232first		; otherwise we need to adjust inFIFO232first
inFIFOproc_done:		
		RETURN


;;--------------------------------
;; send2host
;;   user callable function to send
;;   a byte to host
;; does NOT actually send, just puts byte in buffer
;;--------------------------------
send2host:
rs232_send:
		BSF		gie_flag, 0			; save GIE bit
		BTFSS	INTCON, GIE			; to differentiate if we were called from inside an interrupt handler
		BCF		gie_flag, 0
		BCF	INTCON, GIE				; disable interrupts

		MOVWF	tmpFIFO				; need to temporarily store byte
		LFSR	FSR1, outFIFO232	; load base address for indirect adressing
		MOVF	outFIFO232last, W	; last position
		ADDWF	FSR1L, F			; is new position in buffer for received byte
		MOVF	tmpFIFO, W			; get byte to send
		MOVWF	INDF1				; put in buffer
		INCF	outFIFO232last, F	; point to new (empty) position

		BTFSC	gie_flag, 0			; were we called from outside an interrupt handler ?
		BSF	INTCON, GIE				; yes, then re-enable interrupts

		RETURN

;;--------------------------------
;; rs232fifo2send
;;   gets called by ISR
;;   sends out byte via RS-232 (if outFIFO232 not empty)
;;   adjusts buffer pointers
;;--------------------------------
rs232fifo2send:
		MOVF	outFIFO232first, W	; load pointer to first unsent byte in buffer
		CPFSEQ	outFIFO232last		; check if( first unsent==last unsent) 
		BRA		fifo2send_hostrdy	; not equal, that means we do have something to send
		BRA		fifo2send_no		; they are equal, fifo empty, nothing to send

fifo2send_hostrdy:
		BTFSS	TXSTA,TRMT			; send shift register empty? (prev. send done?)
		BRA		fifo2send_no		; no, not yet, maybe next cycle
		BTFSC	CTS					; RTS/CTS hardware handshake: is the host ready?
		BRA		fifo2send_no		; no, host not ready, maybe next time
		; else (host IS ready):
		LFSR	FSR1, outFIFO232	; load base address for indirect adressing
		ADDWF	FSR1L, F			; compute position in buffer of byte to send
		MOVF	INDF1, W			; get byte from buffer
		MOVWF	TXREG				; and send it out via serial port

		INCF	outFIFO232first, F	; adjust pointer to next byte in buffer
		; check for overflow
		MOVLW	0x00				; did the pointer wrap to zero ?
		CPFSEQ	outFIFO232first		; if( outFIFOfirst == 0 )
		BRA		fifo2send_done		; not zero, did not wrap around, then we're done
		MOVLW	LOW(outFIFO232)		; yes, equal, take lower 8 bits and
		MOVWF	outFIFO232first		; set to start pos of buffer (relative to memory page)

fifo2send_done:
fifo2send_no:
		RETURN


;;===========================================

recvstatemachine:
		;; jumptable
		MOVF	recv_state, W		; need temp copy of state
		MOVWF	tmp					; for shifting for address calc
		BCF		STATUS, C			; clean carry
		RLCF	tmp, F				; shift state left
		MOVLW	HIGH(statejmptbl)	; load high part of jump table address
		BTFSC	STATUS, C			; 
		INCF	WREG, W				; if carry then w=w+1
		MOVWF	PCLATH				; move into high byte of program counter
		MOVLW	LOW(statejmptbl)	; load low part of jump table address
		ADDWF	tmp, W
		BTFSC	STATUS, C
		INCF	PCLATH, F
		MOVWF	PCL
		
statejmptbl:
		BRA		process_hostcmd		; RECVSTATE_IDLE processes the received byte from serial port to determine next state
		BRA		state_reset1		; RECVSTATE_RESET1
		BRA		state_reset2		; RECVSTATE_RESET2
		BRA		state_doasend1		; RECVSTATE_DOASEND1
		BRA		state_doasend2		; RECVSTATE_DOASEND2
		BRA		state_doasend3		; RECVSTATE_DOASEND3
		BRA		state_dobsend1		; RECVSTATE_DOBSEND1
		BRA		state_dobsend2		; RECVSTATE_DOBSEND2
		BRA		state_i2csend1		; RECVSTATE_I2CSEND1
		BRA		state_i2csend2		; RECVSTATE_I2CSEND2
		BRA		state_i2csend3		; RECVSTATE_I2CSEND3
		BRA		state_i2csend4		; RECVSTATE_I2CSEND4
		RETURN
		RETURN
		RETURN
		RETURN
		RETURN
		RETURN



process_hostcmd:
		MOVF	recv_byte, W		; check if byte received is in 
		SUBLW	MAXCMD				; valid range for hostcmds by doing W=MAXCMD-recv_byte
		BN		hostcmd_outofrange	; result negative, out of range ?
		MOVF	recv_byte, W		; else: get temp received byte
		MOVWF	tmp					; for shifting for address calc
		BCF		STATUS, C			; clean carry
		RLCF	tmp, F				; shift state left
		MOVLW	HIGH(hostcmd_jmptbl)	; load high part of jump table address
		BTFSC	STATUS, C			; 
		INCF	WREG, W				; if carry then w=w+1
		MOVWF	PCLATH				; move into high byte of program counter
		MOVLW	LOW(hostcmd_jmptbl)	; load low part of jump table address
		ADDWF	tmp, W				; add offset 
		BTFSC	STATUS, C			; did add set carry
		INCF	PCLATH, F			; possibly adjust high byte of program counter
		MOVWF	PCL					; make the jump

hostcmd_jmptbl:
		RETURN						; HOSTCMD_IDLE   do nothing
		BRA		hostcmd_reset		; HOSTCMD_RESET  need total of 3 reset commands to reset, handled by state machine
		BRA		hostcmd_starttalking; HOSTCMD_STARTTALKING
		BRA		hostcmd_stoptalking	; HOSTCMD_STOPTALKING
		BRA		hostcmd_matrixmap	; HOSTCMD_KEYMATRIXMAP
		BRA		hostcmd_analogmap	; HOSTCMD_ANALOGMAP
		RETURN						; HOSTCMD_I2CSEND
		BRA		hostcmd_doasend		; HOSTCMD_DOASEND
		BRA		hostcmd_dobsend		; HOSTCMD_DOBSEND
		RETURN
		RETURN

;;;;;;;;;;;;;;;; hostcommands further handling
hostcmd_starttalking:
		BSF		talk, 0
		RETURN

hostcmd_stoptalking:
		BCF		talk, 0
		RETURN

hostcmd_reset:
		MOVLW	RECVSTATE_RESET1	; go to state RESET1
		MOVWF	recv_state
		RETURN

hostcmd_matrixmap:

		MOVLW	b'10000000'			; keymatrix full map packet
		RCALL	rs232_send			; send to host
		MOVLW	128					; number of bytes in keymatrixstate (128x8=1024 keys)
		MOVWF	counterkmx
		LFSR	FSR0, keymatrixstate	; load base address of keymatrix[]
matrixmap_next:
		MOVF	POSTINC0, W			; load byte in keymatrixstate[]
		RCALL	rs232_send			; send to host
		DECFSZ	counterkmx, F		; are we done with all bytes ?
		BRA		matrixmap_next		; no, send next
			; send term sequence to host
		MOVLW	b'11111111'			; 'all-bits-one-byte'
		RCALL	rs232_send			; send to host
		MOVLW	b'00000000'			; 'all-bits-zero-byte'
		RCALL	rs232_send			; send to host
		RETURN

hostcmd_analogmap:
		MOVLW	b'10100000'			; analog all axes dump packet
		RCALL	rs232_send			; send to host
		MOVLW	NO_ANALOG_CHANNELS	; number of axes
		MOVWF	counteran
		LFSR	FSR2, analogin		; load base address of analogin[]
analogmap_next:
		MOVF	POSTINC2, W			; get the high byte of 10bit analog value, incrementing FSR0
		ANDLW	b'00000011'			; clip out the unwanted part (lower 2bits are used only)
		RCALL	rs232_send			; send to host  ---> NOT as defined in PHCC2HostProtcol Rev.0

		MOVF	POSTINC2, W			; lower 8 bits of analog value
		RCALL	rs232_send			; send to host

		DECFSZ	counteran, F		; are we done with all bytes ?
		BRA		analogmap_next		; no, send next
			; send term sequence to host
		MOVLW	b'11111111'			; 'all-bits-one-byte'
		RCALL	rs232_send			; send to host
		MOVLW	b'00000000'			; 'all-bits-zero-byte'
		RCALL	rs232_send			; send to host
		RETURN


hostcmd_doasend:
		MOVLW	RECVSTATE_DOASEND1	; go to state DOASEND1
		MOVWF	recv_state		
		RETURN

hostcmd_dobsend:
		MOVLW	RECVSTATE_DOBSEND1	; go to state DOBSEND1
		MOVWF	recv_state		
		RETURN

hostcmd_outofrange:
		MOVLW	0x00				; just to be sure set
		MOVWF	recv_byte			; receive byte to zero
		RETURN	


;;;;;;;;;;;;;; states futher handling
state_reset1:
		MOVF	recv_byte, W		; load copy of received byte
		XORLW	HOSTCMD_RESET		; did we receive another reset (#2)
		BZ		reset2				; zero flag set if command was reset
		MOVLW	RECVSTATE_IDLE		; else (no reset): go back to idle
		MOVWF	recv_state
		RETURN
reset2:
		MOVLW	RECVSTATE_RESET2
		MOVWF	recv_state
		RETURN

state_reset2:
		MOVLW	RECVSTATE_IDLE		; next state will be idle, independent of received byte
		MOVWF	recv_state
		MOVF	recv_byte, W		; load copy of received byte
		XORLW	HOSTCMD_RESET		; did we receive another reset (#3)
		BTFSC	STATUS, Z			; zero flag set if command was reset
		RESET						; then reset the PIC, which should invoke the bootloader
		RETURN						; else return


state_doasend1:
		MOVLW	RECVSTATE_DOASEND2	; transition to next state
		MOVWF	recv_state			; save to state variable
		MOVF	recv_byte, W		; get received byte
		MOVWF	doa_devaddr			; put byte into appropriate place
		RETURN						; done

state_doasend2:
		MOVLW	RECVSTATE_DOASEND3	; transition to next state
		MOVWF	recv_state			; save to state variable
		MOVF	recv_byte, W		; get received byte
		MOVWF	doa_subaddr			; put byte into appropriate place
		RETURN						; done

state_doasend3:
		MOVLW	RECVSTATE_IDLE		; transition to idle
		MOVWF	recv_state			; save to state variable
		MOVF	recv_byte, W		; get received byte
		MOVWF	doa_data			; put byte into appropriate place
		CALL	send_DO_A			; data complete, call handler
		RETURN						; done

state_dobsend1:
		MOVLW	RECVSTATE_DOBSEND2	; transition to next state
		MOVWF	recv_state			; save to state variable
		MOVF	recv_byte, W		; get received byte
		MOVWF	dob_addr			; put byte into appropriate place
		RETURN						; done

state_dobsend2:
		MOVLW	RECVSTATE_IDLE		; transition to idle
		MOVWF	recv_state			; save to state variable
		MOVF	recv_byte, W		; get received byte
		MOVWF	dob_data			; put byte into appropriate place
		CALL	send_DO_B			; data complete, call handler
		RETURN						; done

state_i2csend1:
		MOVLW	RECVSTATE_I2CSEND2	; transition to next state
		MOVWF	recv_state			; save to state variable
		RETURN						; done

state_i2csend2:
		MOVLW	RECVSTATE_I2CSEND3	; transition to next state
		MOVWF	recv_state			; save to state variable
		MOVF	recv_byte, W		; get received byte
		MOVWF	i2cout_addr			; put byte into appropriate place
		RETURN						; done

state_i2csend3:
		MOVLW	RECVSTATE_I2CSEND4	; transition to next state
		MOVWF	recv_state			; save to state variable
		MOVF	recv_byte, W		; get received byte
		MOVWF	i2cout_subaddr		; put byte into appropriate place
		RETURN						; done

state_i2csend4:
		MOVLW	RECVSTATE_IDLE		; transition to idle
		MOVWF	recv_state			; save to state variable
		MOVF	recv_byte, W		; get received byte
		MOVWF	i2cout_data			; put byte into appropriate place
;;;		CALL	send_i2c			; data complete, call handler
		RETURN						; done

;;-------------------------------
;; Digital Out Type A
;;-------------------------------
;; simple serial sync protocol
;; receiving end PIC should clock-in data on positive going transition (PGT) of the clk line
;; addressing by device address (8bit, each receiving PIC has a unique devaddr)
;;         then by subaddr (tells receiving PIC what to do with it, eg. which port. implementation dependent)
;; protocol timing: 
;;   
;;-------------------------------
send_DO_A:
send_DO_A_start:
		BCF		DO_A_DATA			; start condition indicated by data & clk lines low for 3 cycles
		BCF		DO_A_CLK			; 
		CALL	delayDOA			; wait 3*  specified time
		CALL	delayDOA			; 
		CALL	delayDOA			; 
		CALL	delayDOA			; 
		CALL	delayDOA			; 
send_DO_A_devaddr:
		MOVLW	0x08				; dev addr is 8 bits wide
		MOVWF	counter				; setup bit counter
		MOVF	doa_devaddr, W		; load devaddr
		MOVWF	tmp					; load copy into tmp for shifting ops
next_devaddr:
		RRCF	tmp, F				; shift right, LSB into Carry
		RCALL	send_DO_A_out		; send out bit in carry (RCALL does not affect carry)
		DECFSZ	counter, F			; decrement bit counter
		BRA		next_devaddr		; loop for all 8 bits
		
send_DO_A_subaddr:
		MOVLW	0x06				; subaddr is 6 bits wide
		MOVWF	counter				; setup bit counter
		MOVF	doa_subaddr, W		; load subaddr
		MOVWF	tmp					; load copy into tmp for shifting ops
next_subaddr:
		RRCF	tmp, F				; shift right, LSB into Carry
		RCALL	send_DO_A_out		; send out bit in carry (RCALL does not affect carry)
		DECFSZ	counter, F			; decrement bit counter
		BRA		next_subaddr		; loop for all bits

send_DO_A_data:
		MOVLW	0x08				; data is 8 bits wide
		MOVWF	counter				; setup bit counter
		MOVF	doa_data, W			; load data
		MOVWF	tmp					; load copy into tmp for shifting ops
next_databit:
		RRCF	tmp, F				; shift right, LSB into Carry
		RCALL	send_DO_A_out		; send out bit in carry (RCALL does not affect carry)
		DECFSZ	counter, F			; decrement bit counter
		BRA		next_databit		; loop for all 8 bits


		BCF		DO_A_DATA			; set data and 
		BCF		DO_A_CLK			; clock lines low after done sending
		RETURN

; sends out bit in Carry
send_DO_A_out:
		BCF		DO_A_DATA
		BTFSC	STATUS, C
		BSF		DO_A_DATA
		BSF		DO_A_CLK			; clock line high, signal receiving PICs to shift in the data bit
		CALL	delayDOA			; wait a specified time
		BCF		DO_A_CLK			; clock line low after done sending, ready for next bit
		CALL	delayDOA			; wait a specified time
		RETURN

;;-------------------------------
;; Digital Out Type B
;;-------------------------------
send_DO_B:
		;; 74x595 CLK and STORE latch on positive (low-to-high) going transition
		MOVF	dob_data, W			; get byte to send out via DO_B
		MOVWF	tmp					; make temp copy of byte to send
		MOVLW	8
		MOVWF	counter				; bit counter for 8 bits
nextbit_DO_B:
		BSF		DO_B_DATA			; preset condition for data output pin
		RLCF	tmp, F				; left rotate into carry, MSB gets shifted into 74x595 first
		BTFSS	STATUS, C			; bit to send now in carry
		BCF		DO_B_DATA			; if carry not set: then send a 0 bit, else use preset condition
		CALL	delayDOB			; wait a specified time
		BSF		DO_B_CLK			; raise clock line to clock bit into 74x595
		CALL	delayDOB			; wait a specified time
		BCF		DO_B_CLK			; lower clock line again
		DECFSZ	counter, F			; decrement bit counter
		BRA		nextbit_DO_B		; loop for all 8 bits

		; now that all bits are shifted into the 74x595 we can latch them into the parallel output register
		BSF		DO_B_STO			; raise the store line
		CALL	delayDOB			; wait a specified time
		BCF		DO_B_STO			; lower the store line

		RETURN

;;-------------------------------
;; convert binary nibble to hexadecimal
;;-------------------------------
convert2hex:
		MOVWF	tmp
		SUBLW	9					; see if less than 10
		BTFSC	STATUS,C			; carry bit is set if (k-W)>=0
		BRA		less_than_10		; <10: 
		BRA		greater_eq_10		; >= 10: hex uses letters a-f for 10-16

less_than_10:
		MOVF	tmp,W				; get saved nibble
		ADDLW	'0'					; add ascii value of "0"
		BRA		convert2hex_end
greater_eq_10:
		MOVF	tmp,w				; get saved nibble
		ADDLW	'a'-10				; add ascii value of "a"
convert2hex_end:
		RETURN



;;==============================================
;; serial port routines
;;==============================================

;;-------------------------------
;; utility routines
;;-------------------------------
inc_addr:
	; possibly disable interrupts (re-entrancy problems might arise if an interrupt handler uses adr4067)
		INCF	adr4067, F
		MOVLW	b'00001111'			; AND mask for adr4067 overflow prevention
		ANDWF	adr4067, F			; and mask out. that way, adr4067 wraps from 00001111 to 00000000
									; (other parts of code depend on it)
	; reenable interrupts
out_4067:
		BCF		PORTA, 4			; prepare bit 0 of adr4067  ->  RA4
		BTFSC	adr4067, 0
		BSF		PORTA, 4			; bit 0 of adr4067  ->  RA4
		RRNCF	adr4067, W			; put left shifted version of adr4067 into W
		MOVWF	PORTE				; out on RE<0:2>
		RETURN

inc_154:
		INCF	adr154, F			; next
		MOVLW	b'00001111'			; mask out the unused bits
		ANDWF	adr154, F			; by ANDing, this bounds the possible values for adr154 to 0000-1111

out_154:
#ifdef REV3_BOARD
		; mirror the bits in the lower nibble: bit0->bit3, bit1->bit2, bit2->bit1, bit3->bit0
		; mirror, since the REV3 boards' outputs to
		; the 74HCT154 are in opposite order.
		MOVLW	0x00				; preset W with zero
		BTFSC	adr154, 0			; xchange bit 0 -> bit 3
		BSF		WREG, 3
		BTFSC	adr154, 1			; xchange bit 1 -> bit 2
		BSF		WREG, 2
		BTFSC	adr154, 2			; xchange bit 2 -> bit 1
		BSF		WREG, 1
		BTFSC	adr154, 3			; xchange bit 3 -> bit 0
		BSF		WREG, 0
		MOVWF	tmp					; put in temp. storage
		
		MOVF	PORTB, W			; load PortB into W for manipulation
		ANDLW	b'11110000'			; mask to keep upper 4 bits (that might have a different purpose)
		IORWF	tmp, W				; OR together with reversed-bit version of adr154
		MOVWF	PORTB				; and send out on PORTB the combined adr154 for the 4 lower bits 
									; and the 4 higher bits from prev. PortB
#else
		MOVF	PORTB, W			; load PortB into W for manipulation
		ANDLW	b'11110000'			; mask to keep upper 4 bits (that might have a different purpose)
		IORWF	adr154, W			; OR together with adr154
		MOVWF	PORTB				; and send out on PORTB the combined adr154 for the 4 lower bits 
									; and the 4 higher bits from prev. PortB
#endif
		RETURN

;;-------------------------------
;; input routines
;;-------------------------------
keymatrix_in:
	; one row is 8 bits wide, there are 128 rows, 8(8x8=64 inputs) per daughterboard,
	; with 16 daughterboards possible
	; 7-bit address to row in matrix is composed like this:
	; XXXXYYY
	; where XXXX are the lower 4 bits of adr154
	; and YYY are the lower 3 bits of adr4067
			; seven bit for offset in keymatrixstate[]
			; bits 0-2 from lower 3 bits of adr4067
			; bits 3-6 from adr154	

	; reads in 8 rows at a time (ie 1 daughterboard, 8x8=64 inputs)
keymatrix_nextrow:
		LFSR	FSR0, keymatrixstate	; load base address of state variable for keymatrix

			; now compute the offset from keymatrixstate[]
		RCALL	combine_adr4067_adr154
		ADDWF	FSR0L, F			; add to base address
		MOVWF	col_addr			; save copy of 7bit column address in col_addr

		MOVLW	8					; 8 bits in a byte
		MOVWF	counterkmx			; setup bit counter

		MOVF	INDF0, W			; read old state of current column from memory
		MOVWF	keyinput_old		; and save in keyinput_old variable

		COMF	KEYMATRIX_IN_PORT, W	; read in 8 buttons (1 row) from keymatrix, and invert 
		MOVWF	INDF0				; and store new value in keymatrixstate[offset]
		MOVWF	keyinput_new		; make copy into keyinput_new to compute changes (working copy)
		XORWF	keyinput_old, W		; W = keyinput_old XOR keyinput_new  => shows changed bits
		MOVWF	keyinput_delta		; save working copy of changed bits/keys mask		

		BTFSS	talk, 0				; is bit 0 of talk set ? (should we notify host of changed key?)
		RETURN						; we return if talk bit<0> is not set

keymatrix_checknextchanged:
		RLCF	keyinput_delta, F	; move MSB of delta information into carry flag
		BNC		keymatrix_nochange	; a '1' in carry denotes "button changed", otherwise jump over next sect.

keymatrix_ischange:
		; there was a change, so we tell the host
		MOVFF	col_addr, keytmp	; need a working copy of the 7bit column address
		RRNCF	keytmp, F			; build A9-A7 of first byte for packet from bits <6:4> of col_addr (1)
		RRNCF	keytmp, F			; which means that we must do four rotate-right's (2)
		RRNCF	keytmp, F			; (3)
		RRNCF	keytmp, W			; (4)
		ANDLW	b'00000111'			; mask out unused bits
		IORLW	b'00101000'			; and add packet-type header via OR
		RCALL	rs232_send			; and send to host
		; second byte
		; build lower 3 bits of addr, A2-A0		
		DECF	counterkmx, W		; need a zero-based version of bitcounter in W
		MOVWF	keytmp				; counterkmx points to the bit number (address) of the changed bit/keymatrix input
		RLNCF	keytmp, W			; shift into position for second byte, bits <3:1>
		ANDLW	b'00001110'			; mask it
		MOVWF	keytmp				; back into tmp storage
		; now find out if switch/button changed from "open" to "close"(0) or from "close" to "open"(1)
		RLCF	keyinput_new, F		; shift temp copy of new state to stay in sync with bit being worked on
		BTFSC	STATUS, C			; new state: button/switch closed or open ?
		BSF		keytmp, 0			; bit0 of second byte shows state of input
		; now add A6-3
		RLNCF	col_addr, W			; lower four bits of col_addr are needed (1)
		RLNCF	WREG, W				; and they need to be shifted left four bits (2)
		RLNCF	WREG, W				; (3)
		RLNCF	WREG, W				; (4)
		ANDLW	b'11110000'			; mask
		IORWF	keytmp, W			; and combine with the lower four bits part consisting of A2-A0 and the switch state
		RCALL	rs232_send			; send new state
		BRA		keymatrix_checkchanges_end	; jump around nochange code


keymatrix_nochange: 				; to keep track of shifts:
		RLNCF	keyinput_new, F		; shift temp copy of new state to stay in sync with bit being worked on
		
keymatrix_checkchanges_end:
		DECFSZ	counterkmx, F		; dec bitcounter
		BRA		keymatrix_checknextchanged	; check next bit for change
									; else we're done for this column
		RETURN


;;;;;;;;; ANALOG in ;;;;;;;;;;;;;;;;;;;;;

read_analog_pri1:
		LFSR	FSR2, analogin+(0*2)
		MOVLW	b'00100000'			; ANP1 connected to RA5/AN4
		BRA		read_analog_pri
read_analog_pri2:
		LFSR	FSR2, analogin+(1*2)
		MOVLW	b'00011000'			; ANP2 connected to RA3/AN3
		BRA		read_analog_pri
read_analog_pri3:
		LFSR	FSR2, analogin+(2*2)
		MOVLW	b'00010000'			; ANP3 connected to RA2/AN2
read_analog_pri
		MOVWF	antmp				; store for preparation to ADCON0
		MOVLW	b'11000111'			; mask out ADCON0<CHS0:CHS2>
		ANDWF	ADCON0, W			; AND it, leave result in W
		IORWF	antmp, W			; now OR the stored mask with values set for CHS0:CHS2 and leave in W
		MOVWF	ADCON0				; and store in ADCON0
		
		; wait some A/D aquisistion time after changing the channels
		RCALL 	delay250us			; about 110 microsecs for 100k

		BSF		ADCON0, GO_DONE		; start conversion process by setting bit GO/!DONE
anp_adc_wait:
		BTFSC	ADCON0, GO_DONE		; conversion is done when bit GO/!DONE is reset to zero by hardware
		BRA		anp_adc_wait



an_store:		
		MOVFF	FSR2L, an_addr		; save low byte of address for later
		CLRF	an_changed			; clear analog-value-changed flag
				;; store AD result, *high byte* ***first***
		MOVFF	ADRESH, anbufH		; working copies of ADRESH...
		MOVFF	ADRESL, anbufL		; ... and ADRESL are stored in anbufH/L

		RCALL	an_anti_jitter		; filters value and stores if necessary
		MOVFF	an_addr, FSR2L		; restore address

		; tell host of change immediately
		BTFSS	talk, 0				; is bit 0 of talk set ? (should we notify host of changed analog value?)
		RETURN						; we return if talk bit<0> is not set

an_send_host_update:
		BTFSS	an_changed, 0
		RETURN

an_was_change:
		; there was a change, so we tell the host
		MOVLW	b'01010000'
		CALL	rs232_send			; and send to host
		MOVFF	POSTINC2, anbufH	; get analog value high byte
		MOVFF	INDF2, anbufL		; get analog value low byte
		MOVLW	b'00000011'			; mask for the D<9:8> in anbufH
		ANDWF	anbufH, F			; bitwise AND it
		; get offset in analogin[]
		MOVFF	an_addr, FSR2L		; restore address
		MOVF	FSR2L, W
		; shift once right (divide by two) since each analog value takes up two bytes in memory
		RRNCF	WREG, W
		; shift it two bits left
		RLNCF	WREG, W
		RLNCF	WREG, W				
		ANDLW	b'11111100'			; mask out lower two bits, now we got A5-A0 in WREG<7:2>
		IORWF	anbufH, W			; combine A5-0 and D9-8 in WREG
		CALL	rs232_send			; and send to host
		MOVF	PREINC2, W			; retrieve low byte of analog value
		CALL	rs232_send			; and send to host
		RETURN


;;-------------------------------
;; read_analog_secA   read analog secondary channel A (connected to first 4067)
;;-------------------------------
read_analog_secA:
		LFSR	FSR2, analogin+(3*2)
		; add adr4067 to FSR0
		RLNCF	adr4067, W			; load the doubled(=shifted) address used for the 4067 into W
		ADDWF	FSR2L, F			; FSR2L=FSR2L+W
		CLRF	WREG				; W = 0, does not affect carry
		ADDWFC	FSR2H, F			; FSR2H=Carry+FSR2H+0
		
		MOVLW	b'00000000'			; 4067 #1 connected to RA0/AN0
		BRA		read_analog_sec

;;-------------------------------
;; read_analog_secB   read analog secondary channel B (connected to second 4067)
;;-------------------------------
read_analog_secB:
		LFSR	FSR2, analogin+(19*2)
		; add adr4067 to FSR0
		RLNCF	adr4067, W			; load the doubled(=shifted) address used for the 4067 into W
		ADDWF	FSR2L, F			; FSR2L=FSR0L+W
		CLRF	WREG				; W = 0, does not affect carry
		ADDWFC	FSR2H, F			; FSR2H=Carry+FSR2H+0
		
		MOVLW	b'00001000'			; 4067 #2 connected to RA1/AN1

read_analog_sec:
		MOVWF	antmp				; store for preparation to ADCON0
		MOVLW	b'11000111'			; mask out ADCON0<CHS0:CHS2>
		ANDWF	ADCON0, W			; AND it, leave result in W
		IORWF	antmp, W			; now OR the stored mask with values set for CHS0:CHS2 and leave in W
		MOVWF	ADCON0				; and store in ADCON0
		
		; wait some A/D aquisistion time after changing the channels
		RCALL 	delay250us			; about 110 microsecs for 100k

		BSF		ADCON0, GO_DONE		; start conversion process by setting bit GO/!DONE
ansec_adc_wait:
		BTFSC	ADCON0, GO_DONE		; conversion is done when bit GO/!DONE is reset to zero by hardware
		BRA		ansec_adc_wait

		BRA		an_store			; store analog value in memory

;;-------------------------------
;; an_anti_jitter
;;-------------------------------
;; conditions on call:
;;   FSR2 points to memory location analogin[] for *high byte* of current channel
;;   new analog value is in anbufH/anbufL
;;-------------------------------
an_anti_jitter:
		; basically a digital low pass filter
		; difference of saved and new value
		MOVF	anbufH, W			; high byte of new value in anbufH
		XORWF	POSTINC2, W			; gets XORed with old value at analogin[i]
		MOVWF	antmp				; store result of XOR

		MOVLW	0x0					; high byte changed at all ?
		CPFSGT	antmp				; compare XOR result with 1
		BRA		anH_zero			; not > 0 => its zero

		MOVLW	0x1					; difference in high byte can be at most 1 ('overflow')
		CPFSGT	antmp				; compare XOR result with 1
		BRA		anH_one				; result was > 1  => the change had a magnitude > 9 bits

		; when we get here, high byte XOR result is > 1
		BRA		bigchange

#define JITTER_FILTER_COEFF 0x01


anH_zero:
		; difference of low byte
		MOVF	anbufL, W			; low byte of new value in anbufL
		XORWF	INDF2, W			; gets XORed with old value at analogin[i] (post decrement address 
									;  so we are back pointing to the high byte)
		MOVWF	antmp				; store result of XOR
		MOVLW	JITTER_FILTER_COEFF	; jitter filter coefficient
		CPFSGT	antmp				; compare result with jitter filter coefficient (currently 1)
		RETURN						; no change in value after filtering
		BRA		bigchange

anH_one:
		; difference of low byte
		MOVF	anbufL, W			; low byte of new value in anbufL
		XORWF	INDF2, W			; gets XORed with old value at analogin[i] (post decrement address 
									;  so we are back pointing to the high byte)
		MOVWF	antmp				; store result of XOR
		MOVLW	256-JITTER_FILTER_COEFF	; 256 minus jitter filter coefficient 
		CPFSLT	antmp				; compare result with jitter filter coefficient subtracted from 256
		RETURN						; no change in value after filtering
		BRA		bigchange

bigchange:
		BSF		an_changed, 0		; set analog value changed flag
		; store it, FSR2 pionts to low byte when we get here
		MOVFF	anbufL, POSTDEC2	; store low byte, then point FSR2 to pos of hight byte
		MOVFF	anbufH, POSTINC2	; store high byte, set FSR2 the way its expected, ie. pointing to low byte
		RETURN



;;-------------------------------
;; combine adr4067 and adr154 addresses into on 7 bit address, leave in W
;;-------------------------------
combine_adr4067_adr154:
		MOVF	adr154, W			; load adr154 into W
		MOVWF	tmp					; store in tmp
		RLNCF	tmp, F				; left shift copy of adr154 in tmp three times
		RLNCF	tmp, F
		RLNCF	tmp, F
		MOVLW	b'01111000'			; bitmask for bits 3-6
		ANDWF	tmp, F				; mask out usused bits

		MOVLW	b'00000111'			; bitmask for bits 0-2
		ANDWF	adr4067, W			; those 3 bits come from adr4067
		IORWF	tmp, W				; combine into a 7 bit row-address
		RETURN


;;============================================
;; SETTING UP ALL SUBSYSTEMS
;;============================================
;;-------------------------------
;; setup routines
;;-------------------------------
setup_everything:
		CLRF	talk
		RCALL	setup_ints
		RCALL	setup_rs232
		RCALL	setup_DO_A
		RCALL	setup_DO_B
		RCALL	setup_analog
		RCALL	setup_keymatrix
		RCALL	setup_t0
;;;		RCALL	setup_t1    DON'T ENANABLE for now !!!!
;		RCALL	setup_i2c

		RETURN

setup_ints
		CLRF	INTCON				; clear/disable all interrupts
		BCF		RCON, IPEN			; no interrupt priorities
		RETURN


setup_t1:
		MOVLW	b'10110000'
		MOVWF	T1CON
		CLRF	TMR1H
		CLRF	TMR1L
		BCF		PIR1, TMR1IF		; clear int flag

;;ENABLING t1 can be dangerous since it tends to block the system such that there's no time for normal processing because all time is spent inside the interrupt service routine!!!!!!!!!!!!!!
;; there must be someting not clearly understood here!
		BSF		PIE1, TMR1IE			; set interrupt enable
;;;;;;;;	BCF	PIE1, TMR1IE ;;;;; DISABLED!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

		BSF		T1CON, TMR1ON		; start timer
		RETURN

setup_t0:
		MOVLW	b'00001000'			; Timer0: 16bit, int. src, edge don't care, no prescaler, don't start yet
		MOVWF	T0CON				; set timer
;; scan matrix every 50ms: 128 rows => scan a row every 400us -> every 4000 fosc/4
		; 2^16-4000 = 61536 => 0xF060 @ prescaler 1:1

		MOVLW	0xF0
		MOVWF	TMR0H
		MOVLW	0x60				; 0xF060 equals about 400us sec without prescaler
		MOVWF	TMR0L				; TMR0L should come after TMR0H (see datasheet)

;;;		MOVLW	b'00000111'			; Timer0 enabled, 16bit, int. source, edge whatever, prescaler on, 1:256
				;; timer0 at 16bit width, 1:256 prescaler should give a period of 1.6777sec @ 10MIPS
;;;		MOVWF	T0CON				; X seconds timer should be started now
;;;		MOVLW	0x01
;;;		MOVWF	TMR0H
;;;		MOVLW	0x04				; 0x0104 equals about 1 sec with prescaler 1:256
;;;		MOVWF	TMR0L				; TMR0L should come after TMR0H (see datasheet)
		BCF		INTCON, TMR0IF		; clear Timer0 interrupt flag
	BSF		INTCON, TMR0IE
;;	BCF	INTCON, TMR0IE  ;;;;; DISABLED!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

		BSF		T0CON, TMR0ON		; start timer0
		RETURN


setup_rs232:
		CLRF	inFIFO232first		; init variables, pointer to first unprocessed byte received
		CLRF	inFIFO232last		; last unprocessed byte received
		CLRF	outFIFO232first		; pointer to first unprocessed byte to send
		CLRF	outFIFO232last		; last unprocessed byte to send
		CLRF	FIFOrdy				; bitmapped register for fifo related stuff

;;;;;;; HIGHLY  EXPERIMENTAL: set serial port speed up to 115.2 kbps
		MOVLW	21					; BRGH=1, 21 => 115200 bps **** @40MHz ****
		MOVWF	SPBRG
;;;;;;;;;;;;;;;;;;;;;;;;;;;
		BSF		CTSTRIS				; CTS is an input
		BCF		RTSTRIS				; RTS is an output
		BSF		RTS 				; not ready to receive yet, this is controlled by rs232_recv
									; ATTENTION: RTS/CTS use inverted logic !!!
		;; enable interrupts
		BSF		INTCON, PEIE		; peripheral interrupts enable
		BCF		PIR1, RCIF			; clear interrupt flag for receive
		BCF		PIR1, TXIF			; clear interrupt flag for send
		BSF		PIE1, RCIE			; enable serial port receive interrupt
;;;;		BSF		PIE1, TXIE			; enable serial port send done interrupt
		BCF		RTS					; RTS/CTS hardware handshake: we're ready to receive

		RETURN

setup_DO_A:
		BCF		DO_A_DATA			; set data and 
		BCF		DO_A_CLK			; clock lines low for idle
		BCF		DO_A_CLK_TRIS		; clk is output
		BCF		DO_A_DATA_TRIS		; data is output
		RETURN

setup_DO_B:
		BCF		DO_B_CLK_TRIS		; clk is output
		BCF		DO_B_DATA_TRIS		; data is output
		BCF		DO_B_STO_TRIS		; store is output
		BCF		DO_B_CLK			; DO_B output pins low initially
		BCF		DO_B_DATA			; including DATA
		BCF		DO_B_STO			; and STOre
		RETURN

setup_4067:
		BCF		TRISA, 4			; output RA4 is bit 0 of adr4067
		BCF		TRISE, 0			; output RE0 is bit 1 of adr4067
		BCF		TRISE, 1			; output RE1 is bit 2 of adr4067
		BCF		TRISE, 2			; output RE2 is bit 3 of adr4067
		MOVLW	0x00
		MOVWF	adr4067				; set address for 4067 mux to 0
		RCALL	out_4067		
		RETURN

setup_analog:
		BSF		TRISA, 0
		BSF		TRISA, 1
		BSF		TRISA, 2
		BSF		TRISA, 3
		BSF		TRISA, 5			; setup RA<0-3,5> as analog input pins
		RCALL	setup_4067			; setup pins for 4067 mux
		MOVLW	b'10000010'			; right justified results, FOSC/2, AN0-4 are analog input
		MOVWF	ADCON1
		MOVLW	b'11000001'			; FRC (use AD RC for ADC timing), AN0 selected, ADON=1
		MOVWF	ADCON0

		MOVLW	ANALOG_CHANNEL_STORAGE_SIZE
		MOVWF	counteran
		LFSR	FSR2, analogin
clr_analog_next:
		CLRF	POSTINC2			; clear byte in data mem pointed to by FSR2, then increment FSR2
		DECFSZ	counteran, F		; decrement counter, if zero
		BRA		clr_analog_next		; no? repeat
									; yes? go on

		RETURN


setup_keymatrix:
		MOVLW	b'11111111'
		MOVWF	KEYMATRIX_IN_DIRREG	; all pins input for keymatrix columns
			; clear keymatrixstate[]
		MOVLW	128
		MOVWF	counterkmx
		LFSR	FSR0, keymatrixstate
clr_matrix_next:
		CLRF	POSTINC0			; clear byte in data mem pointed to by FSR0, then increment FSR0
		DECFSZ	counterkmx, F		; decrement counter, if zero
		BRA		clr_matrix_next		; no? repeat
									; yes? go on

		; ** duplicated code from setup_analog ** (in case analog is not used)
		RCALL	setup_4067			; setup pins for 4067 mux
		; ** end dupl. code **

		MOVLW	0x00
		MOVWF	adr154				; init 74x154 selector
		RCALL	out_154
		RETURN


setup_i2c:
		BSF		TRISC, 3			; SCL, input for I2C
		BSF		TRISC, 4			; SDA, input for I2C
		MOVLW	b'00101000'			; SSPEN: enable serial port, I2C master mode
		MOVWF	SSPCON1
;		MOVLW	b'0' ???					; 
		MOVWF	SSPCON2				

		RETURN



send_crlf:
		MOVLW	'\r'				; send a CR
		RCALL	rs232_send	
		MOVLW	'\n'				; send a LF
		RCALL	rs232_send	
		RETURN



;;-------------------------------
;; delay routines
;;-------------------------------
delay1ms:
		MOVLW	FREQTIMESFOUR
		MOVWF	delaycounter_freq
delay1_freq:
		MOVLW	1
		MOVWF	delaycounter_outer	; takes value in W for delay lenght in msec
delay1_outer:
		MOVLW	200					; 200*5usec = 1msec
		MOVWF	delaycounter_inner
delay1_inner:						; inner loop takes 5usec @4MHz
		NOP
		NOP
		DECFSZ	delaycounter_inner,F
		BRA		delay1_inner
		DECFSZ	delaycounter_outer,F
		BRA		delay1_outer
		DECFSZ	delaycounter_freq,F
		BRA		delay1_freq
		RETURN
;;-------------------------------
delay250ms:
		MOVLW	FREQTIMESFOUR
		MOVWF	delaycounter_freq
delay_250_freq:
		MOVLW	250					; 250 * 1msec = 250msec
		MOVWF	delaycounter_outer
delay250_outer:
		MOVLW	200					; 200*5usec = 1msec
		MOVWF	delaycounter_inner
delay250_inner:						; inner loop takes 5usec @4MHz
		NOP
		NOP
		DECFSZ	delaycounter_inner,F
		BRA		delay250_inner
		DECFSZ	delaycounter_outer,F
		BRA		delay250_outer
		DECFSZ	delaycounter_freq,F
		BRA		delay_250_freq
		RETURN
;;-------------------------------
delayXms:
		MOVLW	FREQTIMESFOUR
		MOVWF	delaycounter_freq
delay_freq:
		MOVWF	delaycounter_outer	; takes value in W for delay lenght in msec
delay_outer:
		MOVLW	200					; 200*5usec = 1msec
		MOVWF	delaycounter_inner
delay_inner:						; inner loop takes 5usec @4MHz
		NOP
		NOP
		DECFSZ	delaycounter_inner,F
		BRA		delay_inner
		DECFSZ	delaycounter_outer,F
		BRA		delay_outer
		DECFSZ	delaycounter_freq,F
		BRA		delay_freq
		RETURN

delayDOB:
		NOP
		NOP
		NOP
		NOP
		NOP
		RETURN

delayDOA:
		BRA		delay100us   ; testtestestestestestest
		;; .....
		RETURN

delay100us:  ;;@40MHZ => 10MIPS     1000 instructions in 100us
		MOVLW	200						; 200*5 = 1000 instructions
		MOVWF	delaycounter_inner
delay_us_100_inner:						; inner loop takes 5 cycles
		NOP
		NOP
		DECFSZ	delaycounter_inner,F 	; 1 cycle while looping 
		BRA		delay_us_100_inner		; 2 cycles 
		RETURN

delay50us:  ;;@40MHZ => 10MIPS     500 instructions in 50us
		MOVLW	100						; 100*5 = 500 instructions
		MOVWF	delaycounter_inner
delay_us_50_inner:						; inner loop takes 5 cycles
		NOP
		NOP
		DECFSZ	delaycounter_inner,F 	; 1 cycle while looping 
		BRA		delay_us_100_inner		; 2 cycles 
		RETURN


delay250us:
		MOVLW	FREQTIMESFOUR
		MOVWF	delaycounter_freq
delay_us_250_freq:
		MOVLW	1
		MOVWF	delaycounter_outer
delay_us_250_outer:
		MOVLW	50					; 50*5usec = 250usec @4MHz
		MOVWF	delaycounter_inner
delay_us_250_inner:					; inner loop takes 5usec @4MHz
		NOP
		NOP
		DECFSZ	delaycounter_inner,F
		BRA		delay_us_250_inner
		DECFSZ	delaycounter_outer,F
		BRA		delay_us_250_outer
		DECFSZ	delaycounter_freq,F
		BRA		delay_us_250_freq
		RETURN

delay1sec:
		RCALL	delay250ms
		RCALL	delay250ms
		RCALL	delay250ms
		RCALL	delay250ms
		RETURN
;;=================================================
;; End of program
;;=================================================
	END
