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
;; part of cockpit.varxec.net, a home cockpit building site dedicated
;; to the free software and the open source/free software, 
;; multiplatform flight simulator FlightGear.
;;
;;
;; Copyright (c) 2003, 2004, 2005 by Manuel Bessler <m.bessler AT gmx DOT net>
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
;;	FSR0 is used for DOA buffer, for RS232 send/receive buffer, ...
;; 	FSR1 is used for keymatrix buffer stuff
;;  FSR2 is used for analog in buffer stuff
;;
;;=================================================

        LIST P=18F452, R=DEC    ; what the PIC we use, decimal system as default
        #include "p18f452.inc"  ; include appropriate processor definitions

;;;; @40Mhz (10MHz x 4)

;;; #define REV3_BOARD			;; define the board/hardware revision you have

#define MAJOR_VERSION 0
#define MINOR_VERSION 1
#define MICRO_VERSION 8


#define FREQTIMESFOUR 10    ;; for delays, specifies the multiple of 4 MHz we're running at.
							;;  eg. for 16MHz this would be 4, for 40MHz it would be 10



#define TXD 			PORTC, 6		;; data outgoing
#define RXD 			PORTC, 7		;; data incoming
#define RTS 			PORTC, 5		;; outgoing hardware flow control
#define CTS 			PORTC, 2		;; incoming hardware flow control
#define RTSTRIS			TRISC, 5		;; direction register for RTS
#define CTSTRIS			TRISC, 2		;; direction register for CTS


#define	DOA_CLK 		PORTC, 1		;; Digital-Out Type A Clock line
#define DOA_DATA 		PORTC, 0		;; Digital-Out Type A Data line
#define	DOA_CLK_TRIS 	TRISC, 1		;; direction register setup
#define	DOA_DATA_TRIS 	TRISC, 0		;; direction register setup


#define DOB_CLK        PORTB, 5			;; Digital-Out Type B Clock line
#define DOB_DATA       PORTB, 4			;; Digital-Out Type B Data line
#define DOB_STO        PORTB, 6			;; Digital-Out Type B Store line
#define DOB_CLK_TRIS   TRISB, 5			;; direction register setup
#define DOB_DATA_TRIS  TRISB, 4			;; direction register setup
#define DOB_STO_TRIS   TRISB, 6			;; direction register setup


#define ANP1 			PORTA, 5		;; AN4
#define ANP2 			PORTA, 3		;; AN3
#define ANP3 			PORTA, 2		;; AN2
#define AN4067A 		PORTA, 0		;; AN0
#define AN4067B 		PORTA, 1		;; AN1

#define ADR0_154_TRIS	TRISB, 0		;; A0 of 74x154
#define ADR1_154_TRIS	TRISB, 1		;; A1 of 74x154
#define ADR2_154_TRIS	TRISB, 2		;; A2 of 74x154
#define ADR3_154_TRIS	TRISB, 3		;; A3 of 74x154

#define ADR0 			PORTA, 4		;; 4-bit address to 
#define ADR1 			PORTE, 0		;; the two 4067s  and 
#define ADR2 			PORTE, 1		;; 3-bit address for all 74x138
#define ADR3 			PORTE, 2		;;

#define KEYMATRIX_IN_PORT PORTD		;; port to read in a row of the keymatrix
#define KEYMATRIX_IN_DIRREG	TRISD	;; direction register for above port

#define LED				PORTB, 7
#define LED_TRIS		TRISB, 7

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
; ...
#define HOSTCMD_IDENT					0x20	; space char
#define HOSTCMD_TEST					0x21	; exclamation mark for tests via terminal program
#define	MAXCMD						0x021		; always the last/highest value of HOSTCMD_...

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
			;; ** ISR shadow variables **
			Wsave, BSRsave, STATUSsave, fsr0l_save, fsr0h_save

			;; ** global variables and flags **
			talk, gie_flag
			
		
			;; ** delay variables **
			delaycounter_freq, delaycounter_inner,delaycounter_outer
		
			;; ** RS232 variables **
			overrunflag
			inFIFO232first, inFIFO232last
			outFIFO232first, outFIFO232last
			recv_byte
			recv_state
			tmpFIFO, FIFOrdy
			fsr0l_232, fsr0h_232

			;; ** DOA variables **
			DOAfirst, DOAlast, DOArdy, DOAshiftout, packetbitcntr
			doa_devaddrIN, doa_subaddrIN, doa_dataIN
			doa_devaddr, doa_subaddr, doa_data

			;; ** DOB variables **
			dob_addr, dob_data
			DOBcounter

			;; ** analog-in  variables **
			an_changed
			anbufH, anbufL, an_addr
			anpctr
			counteran
		
			;; ** keymatrix-in variables **
			keyinput_old, keyinput_new, keyinput_delta, col_addr
			counterkmx, counterkmx2

			;; ** I/O addressing variables **
			adr4067
			adr154

			;; ** timer variables **
			timer0acq, timer0conv
			timer3L, timer3H
				
			;; ** I2C variables **
			i2cout_addr, i2cout_subaddr, i2cout_data

			;; **  variables **
			;; **  variables **
			;; **  variables **
				tmp, tmp2, antmp, keytmp
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

#define DOAbuffsize 85*3		;; should always be a multiple of 3 ??????
#define BANK_doa_buff 0x5
		CBLOCK 0x500				;; bank 5
			DOAbuff:DOAbuffsize
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

;;===============================
;;       --== ISR ==--
;;  Interrupt Service Routine
;;===============================
isr:
		;; save registers
		MOVWF	Wsave
		MOVFF	STATUS, STATUSsave
		MOVFF	BSR, BSRsave
		MOVFF	FSR0L, fsr0l_save
		MOVFF	FSR0H, fsr0h_save
		;; set up addressing to bank 0 for data memory

		;; start interrupt processing
		BTFSC	PIR1, RCIF			; was it a serial port receive int?
		CALL	rs232recv2fifo		; handle it

		BTFSC	PIR2, TMR3IF		; check timer3 interrupt flag
		CALL	timer3int

		BTFSC	PIE1, TXIE			; is the usart send-done interrupt enabled ?
		CALL	rs232TXint			; then call handler to see if something's to do
		
		BRA		end_interrupt		; non-handled interrupt? then go to end
;;;;;;;;;;;;;;;;;;;;

end_interrupt:
		;; restore addessing
		MOVFF	fsr0l_save, FSR0L
		MOVFF	fsr0h_save, FSR0H
		MOVFF	BSRsave, BSR
		MOVF	Wsave, W
		MOVFF	STATUSsave, STATUS
		RETFIE
;;===============================
;; --== ISR ==-- end
;;===============================


timer3int:
		BTFSS	PIE2, TMR3IE		; is timer3 int enabled?
		RETURN						; if no, return immediately

		;; do whatever this interrupt should do
		CALL	DOAnextbitout
		;; possible improvement:
		;; ---------------------
		;; we could implement a similar technique like with TXIF and rs232outFIFOprocess
		;; here using the DOAbuff status, doanextbitout state, and timer3 interrupt enable.

		MOVFF	timer3H, TMR3H		; reload value for TMR3H, has to be before TMR3L
		MOVFF	timer3L, TMR3L		; reload value for TMR3L
		BCF		PIR2, TMR3IF		; reset timer3 interrupt flag
		RETURN

		
rs232TXint:
		BTFSS	PIR1, TXIF			; is the usart send register empty, ready for more ?
		RETURN						; no, go back

		BRA		rs232fifo2send		; take a byte from the queue and send it (if queue non-empty)
		RETURN		

;;///////////////////////////////////////////
;;/////////////// PROGRAM INIT //////////////
;;///////////////////////////////////////////
init:
		CLRF	BSR					; set bank select register to default
		BCF		INTCON, GIE			; disable all interrupts that may be left over from bootloader

		CALL	led_blink_seq

		CALL	setup_everything	; call the setup routines for the different subsystems

		BSF		INTCON,	GIE			; GLOBALLY ENABLE INTERRUPTS

;;;--------------- INIT END -----------------
		
;;===========================================
;;===========================================
;;============= MAIN LOOP START =============
;;===========================================
;;===========================================
main:
		CALL	read_analog_secA
		CALL			start_acq_timer
		CALL	keymatrix_in
		
		CALL	inc_154
		NOP
		NOP
		NOP
		CALL	keymatrix_in
		
		CALL	inc_154
		NOP
		NOP
		NOP
		CALL	keymatrix_in
		
		CALL	inc_154
		NOP
		NOP
		NOP
		CALL	keymatrix_in
		
		CALL	inc_154
		CALL			yield
		CALL	analog_GO		; for secA
		CALL			start_conv_timer
		CALL	keymatrix_in
		
		CALL	inc_154
		NOP
		NOP
		NOP
		CALL	keymatrix_in
		
		CALL	inc_154
		NOP
		NOP
		NOP
		CALL	keymatrix_in
		
		CALL	inc_154
		NOP
		NOP
		NOP
		CALL	keymatrix_in
		
		CALL			yield
		CALL	analog_DONE		; for secA
		
		CALL	inc_154
;;;		CALL	inc_adr4067
		CALL	read_analog_secB
		CALL			start_acq_timer
		CALL	keymatrix_in
		
		CALL	inc_154
		NOP
		NOP
		NOP
		CALL	keymatrix_in
		
		CALL	inc_154
		NOP
		NOP
		NOP
		CALL	keymatrix_in
		
		CALL	inc_154
		NOP
		NOP
		NOP
		CALL	keymatrix_in
		
		CALL	inc_154
		CALL			yield
		CALL	analog_GO		; for secB
		CALL			start_conv_timer
		CALL	keymatrix_in
		
		CALL	inc_154
		NOP
		NOP
		NOP
		CALL	keymatrix_in
		
		CALL	inc_154
		NOP
		NOP
		NOP
		CALL	keymatrix_in
		
		CALL	inc_154
		NOP
		NOP
		NOP
		CALL	keymatrix_in
		
		CALL			yield
		CALL	analog_DONE		; for secB

		CALL	next_anp
		CALL			start_acq_timer
		CALL			yield
		CALL	analog_GO
		CALL			start_conv_timer
		CALL			yield
		CALL	analog_DONE
		CALL	inc_154
		CALL	inc_adr4067

		BRA		main

;;===========================================
;;===========================================
;;============= MAIN LOOP END ===============
;;===========================================
;;===========================================



		

		

;;===========================================
;;===========================================
;;===========================================
;;===========================================
;;===========================================
;;===========================================
;;===========================================
;;===========================================




;;;@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
;;;******************************************************
;;;** Analog Acquisition and Conversion Timers
;;;******************************************************
;;;@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
start_acq_timer:
		MOVFF	timer0acq, TMR0L	; load timer preload value for acquisition
		BRA		start_timer_common
;;- - - - - - - - - - - - - - - - 
start_conv_timer:
		MOVFF	timer0conv, TMR0L	; load timer preload value for conversion
;;-  -  -  -  -  -  -  -  -  -  - 
start_timer_common:
		BCF		INTCON, TMR0IF		; clear interrupt flag
		BSF		T0CON, TMR0ON		; turn timer0 on
		RETURN
;;--------------------------------
yield:
		CALL	rs232inFIFOprocess	; check fifo during wait for timer0 ("idle task", 
									; called whenever we've got nothin' better to do)
		BTFSS	INTCON, TMR0IF
		BRA		yield
		BCF		T0CON, TMR0ON		; turn timer0 off
		BCF		INTCON, TMR0IF		; clear interrupt flag
		RETURN
;;;//////////////////////////////////////////////////////
;;;// END of Analog Acquisition and Conversion Timers
;;;//////////////////////////////////////////////////////




		


;;;@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
;;;******************************************************
;;;** Initialization and Setup Routines
;;;******************************************************
;;;@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

;;============================================
;; SETTING UP ALL SUBSYSTEMS
;;============================================
setup_everything:
		CLRF	talk
		CALL	init_recv_statemachine		
		CALL	setup_ints
		CALL	setup_rs232
		CALL	setup_DOA
		CALL	setup_DOB
		CALL	setup_analog
		CALL	setup_keymatrix
		CALL	setup_t0
		;CALL	setup_t3			timer3 setup embedded withing setup_DOA
;		CALL	setup_i2c
		RETURN
;;--------------------------------
setup_ints
		CLRF	INTCON				; clear/disable all interrupts
		BCF		RCON, IPEN			; no interrupt priorities
		RETURN
;;--------------------------------
init_recv_statemachine:
		MOVLW	RECVSTATE_IDLE		; initialize state machine
		MOVWF	recv_state			; variable state holds the current state of the state machine
		RETURN
;;;//////////////////////////////////////////////////////
;;;// END of Initialization and Setup Routines
;;;//////////////////////////////////////////////////////






		

;;;@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
;;;******************************************************
;;;** Test Routines
;;;******************************************************
;;;@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
led_blink_seq:
		BCF		LED_TRIS		; LED pin is output
		BCF		LED				;; LED off
		CALL		delay1sec
		BSF		LED				;; LED on
		CALL		delay1sec
		BCF		LED				;; LED off
		CALL		delay1sec
		BSF		LED				;; LED on
		CALL		delay1sec
		RETURN
;;--------------------------------

;;;//////////////////////////////////////////////////////
;;;// END of 
;;;//////////////////////////////////////////////////////










;;;@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
;;;******************************************************
;;;** Timer Routines
;;;******************************************************
;;;@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
setup_t0:
		CLRF	T0CON				; reset all flags for timer0
		BCF		T0CON, TMR0ON		; turn timer0 off
		BSF		T0CON, T08BIT		; configure as 8bit timer
		BCF		T0CON, T0CS			; clock source select: internal
		BCF		T0CON, T0SE			; source edge select bit: low-to-high
		BSF		T0CON, PSA			; don't use the prescaler
		BCF		T0CON, T0PS2		; T0PS<2:0> are prescaler select 
		BCF		T0CON, T0PS1		; all three bits clear (000) 
		BCF		T0CON, T0PS0		;  => 1:2 prescale (just to be safe, prescaler is not used here, see above)
		
		;; we use timer0 only for approximate timing for adc acquisition and conversion
		;; no interrupt is used, just polling
		;; two different values are required:
		;; 13usec for acquisition
		;; and 20usec for conversion
		;; thus we keep two TMR0L initialization values
		MOVLW	0x70				; 0x70 equals roughly 13usec
		MOVWF	timer0acq
		MOVLW	0x30				; 0x30 equals roughly 20usec
		MOVWF	timer0conv
		
		BCF		INTCON, TMR0IE		; don't use interrupts for timer0
		BCF		INTCON, TMR0IF		; and clear the interupt flag
		
		
;dont		BSF		T0CON, TMR0ON		; turn timer0 on
		RETURN
;--------------------------------

;;;//////////////////////////////////////////////////////
;;;// END of Timer Routines
;;;//////////////////////////////////////////////////////







		


;;;@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
;;;******************************************************
;;;** Host->PHCC Receive State Machine, Outer Part
;;;******************************************************
;;;@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
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
;;- - - - - - - - - - - - - - - - 
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
;;--------------------------------
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
;;--------------------------------
state_reset2:
		MOVLW	RECVSTATE_IDLE		; next state will be idle, independent of received byte
		MOVWF	recv_state
		MOVF	recv_byte, W		; load copy of received byte
		XORLW	HOSTCMD_RESET		; did we receive another reset (#3)
		BTFSC	STATUS, Z			; zero flag set if command was reset
		RESET						; then reset the PIC, which should invoke the bootloader
		RETURN						; else return
;;--------------------------------
state_doasend1:
		MOVLW	RECVSTATE_DOASEND2	; transition to next state
		MOVWF	recv_state			; save to state variable
		MOVF	recv_byte, W		; get received byte
		MOVWF	doa_devaddrIN		; put byte into appropriate place
		RETURN						; done
;;--------------------------------
state_doasend2:
		MOVLW	RECVSTATE_DOASEND3	; transition to next state
		MOVWF	recv_state			; save to state variable
		MOVF	recv_byte, W		; get received byte
		MOVWF	doa_subaddrIN		; put byte into appropriate place
		RETURN						; done
;;--------------------------------
state_doasend3:
		MOVLW	RECVSTATE_IDLE		; transition to idle
		MOVWF	recv_state			; save to state variable
		MOVF	recv_byte, W		; get received byte
		MOVWF	doa_dataIN			; put byte into appropriate place
		BRA		DOAbuff_in			; enqueue DOA packet into DOAbuff (it also RETURNs for us
		RETURN						; done (fall-thru in this case only)
;;--------------------------------
state_dobsend1:
		MOVLW	RECVSTATE_DOBSEND2	; transition to next state
		MOVWF	recv_state			; save to state variable
		MOVF	recv_byte, W		; get received byte
		MOVWF	dob_addr			; put byte into appropriate place
		RETURN						; done
;;--------------------------------
state_dobsend2:
		MOVLW	RECVSTATE_IDLE		; transition to idle
		MOVWF	recv_state			; save to state variable
		MOVF	recv_byte, W		; get received byte
		MOVWF	dob_data			; put byte into appropriate place
		CALL	send_DOB			; data complete, call handler
		RETURN						; done
;;--------------------------------
state_i2csend1:
		MOVLW	RECVSTATE_I2CSEND2	; transition to next state
		MOVWF	recv_state			; save to state variable
		RETURN						; done
;;--------------------------------
state_i2csend2:
		MOVLW	RECVSTATE_I2CSEND3	; transition to next state
		MOVWF	recv_state			; save to state variable
		MOVF	recv_byte, W		; get received byte
		MOVWF	i2cout_addr			; put byte into appropriate place
		RETURN						; done
;;--------------------------------
state_i2csend3:
		MOVLW	RECVSTATE_I2CSEND4	; transition to next state
		MOVWF	recv_state			; save to state variable
		MOVF	recv_byte, W		; get received byte
		MOVWF	i2cout_subaddr		; put byte into appropriate place
		RETURN						; done
;;--------------------------------
state_i2csend4:
		MOVLW	RECVSTATE_IDLE		; transition to idle
		MOVWF	recv_state			; save to state variable
		MOVF	recv_byte, W		; get received byte
		MOVWF	i2cout_data			; put byte into appropriate place
;;;		CALL	send_i2c			; data complete, call handler
		RETURN						; done
;;;//////////////////////////////////////////////////////
;;;// END of Host->PHCC Receive State Machine, Outer Part
;;;//////////////////////////////////////////////////////







		

;;;@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
;;;******************************************************
;;;** Host->PHCC Receive State Machine, Inner Part
;;;******************************************************
;;;@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
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
;;- - - - - - - - - - - - - - - - 
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
		RETURN						; 0x09
		RETURN						; 0x0a
		RETURN						; 0x0b
		RETURN						; 0x0c
		RETURN						; 0x0d
		RETURN						; 0x0e
		RETURN						; 0x0f
		RETURN						; 0x10
		RETURN						; 0x11
		RETURN						; 0x12
		RETURN						; 0x13
		RETURN						; 0x14
		RETURN						; 0x15
		RETURN						; 0x16
		RETURN						; 0x17
		RETURN						; 0x18
		RETURN						; 0x19
		RETURN						; 0x1a
		RETURN						; 0x1b
		RETURN						; 0x1c
		RETURN						; 0x1d
		RETURN						; 0x1e
		RETURN						; 0x1f
		BRA		hostcmd_ident		; 0x20
		BRA		hostcmd_test		; 0x21
		RETURN
		RETURN
;;--------------------------------
hostcmd_outofrange:
		MOVLW	0x00				; just to be sure set
		MOVWF	recv_byte			; receive byte to zero
		RETURN	
;;--------------------------------
hostcmd_ident:
		MOVLW	'P'
		CALL	rs232_send
		MOVLW	'H'
		CALL	rs232_send
		MOVLW	'C'
		CALL	rs232_send
		MOVLW	'C'
		CALL	rs232_send
		MOVLW	'-'
		CALL	rs232_send
		MOVLW	MAJOR_VERSION + 0x30
		CALL	rs232_send
		MOVLW	'.'
		CALL	rs232_send
		MOVLW	MINOR_VERSION + 0x30
		CALL	rs232_send
		MOVLW	'.'
		CALL	rs232_send
		MOVLW	MICRO_VERSION + 0x30
		CALL	rs232_send
		RETURN
;;--------------------------------
hostcmd_test:
		MOVLW	'!'
		CALL	rs232_send
		RETURN
;;--------------------------------
hostcmd_starttalking:
		BSF		talk, 0
		RETURN
;;--------------------------------
hostcmd_stoptalking:
		BCF		talk, 0
		RETURN
;;--------------------------------
hostcmd_reset:
		MOVLW	RECVSTATE_RESET1	; go to state RESET1
		MOVWF	recv_state
		RETURN
;;--------------------------------
hostcmd_matrixmap:
		MOVLW	b'10000000'			; keymatrix full map packet
		RCALL	rs232_send			; send to host
		MOVLW	128					; number of bytes in keymatrixstate (128x8=1024 keys)
		MOVWF	counterkmx2
		LFSR	FSR0, keymatrixstate	; load base address of keymatrix[]
matrixmap_next:
		MOVF	POSTINC0, W			; load byte in keymatrixstate[]
		RCALL	rs232_send			; send to host
		DECFSZ	counterkmx2, F		; are we done with all bytes ?
		BRA		matrixmap_next		; no, send next
			; send term sequence to host
		MOVLW	b'11111111'			; 'all-bits-one-byte'
		RCALL	rs232_send			; send to host
		MOVLW	b'00000000'			; 'all-bits-zero-byte'
		RCALL	rs232_send			; send to host
		RETURN
;;--------------------------------
hostcmd_analogmap:
		MOVLW	b'10100000'			; analog all axes dump packet
		RCALL	rs232_send			; send to host
		MOVLW	NO_ANALOG_CHANNELS	; number of axes
		MOVWF	counteran
		LFSR	FSR0, analogin		; load base address of analogin[]
analogmap_next:
		MOVF	POSTINC0, W			; get the high byte of 10bit analog value, incrementing FSR2
		ANDLW	b'00000011'			; clip out the unwanted part (lower 2bits are used only)
		RCALL	rs232_send			; send to host  ---> NOT as defined in PHCC2HostProtcol Rev.0

		MOVF	POSTINC0, W			; lower 8 bits of analog value
		RCALL	rs232_send			; send to host

		DECFSZ	counteran, F		; are we done with all bytes ?
		BRA		analogmap_next		; no, send next
			; send term sequence to host
		MOVLW	b'11111111'			; 'all-bits-one-byte'
		RCALL	rs232_send			; send to host
		MOVLW	b'00000000'			; 'all-bits-zero-byte'
		RCALL	rs232_send			; send to host
		RETURN
;;--------------------------------
hostcmd_doasend:
		MOVLW	RECVSTATE_DOASEND1	; go to state DOASEND1
		MOVWF	recv_state		
		RETURN
;;--------------------------------
hostcmd_dobsend:
		MOVLW	RECVSTATE_DOBSEND1	; go to state DOBSEND1
		MOVWF	recv_state		
		RETURN
;;;//////////////////////////////////////////////////////
;;;// END of Host->PHCC Receive State Machine, Inner Part
;;;//////////////////////////////////////////////////////







		
	
;;;@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
;;;******************************************************
;;;** DOA routines
;;;******************************************************
;;;@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
setup_DOA:
		BCF		DOA_DATA			; set data and 
		BCF		DOA_CLK				; clock lines low for idle
		BCF		DOA_CLK_TRIS		; clk is output
		BCF		DOA_DATA_TRIS		; data is output
		CLRF	DOAlast				; reset some variables
		CLRF	DOAfirst
		CLRF	DOArdy
		CLRF	packetbitcntr

		;; setup timer for DOAnextbitout, called every 100us (equals 1000 cycles)
		;; using timer3

		;; setup for timer3
		CLRF	T3CON				; first reset timer3 control register
		BSF		T3CON, RD16			; select 16 bit access mode (write to TMR3H before TMR3L!)
		;;		T3CCP2:T3CCP1   stays 00 since we don't care about the CCP module
		;; 		T3CKPS1:T3CKPS0 stays 00 for 1:1 prescaler
		;; 		T3CON, T3SYNC   stays 0  since its ignored in TMR3CS=0 mode anyways
		BCF		T3CON, TMR3CS		; TMR3CS=0 to use internal clock (fosc/4) to run timer3
		;; TMR3ON is set further down to start timer3

		;; 0xFFFF minus 1000(dec) cycles = 0xFC17
		;; => round down to 0xFC00
		MOVLW	0xFC				; see above
		MOVWF	timer3H				; load value for TMR3H
		MOVWF	TMR3H				; load high byte of timer3 counter, has to be before TMR3L
		MOVLW	0x00				; see above
		MOVWF	timer3L				; load value for TMR3L
		MOVWF	TMR3L				; load low byte of timer3 counter

		BCF		PIR2, TMR3IF		; clear timer3 interrupt flag
		BSF		PIE2, TMR3IE		; enable timer3 interrupt
		BSF		T3CON, TMR3ON		; start timer3
		RETURN
;;--------------------------------

;;--------------------------------
;; DOAbuff_in
;;--------------------------------
;; when DOA requests are received from 
;; the host they are queued here first
;; 'DOAbuff' is a circular queue buffer
;;--------------------------------
DOAbuff_in:	
		LFSR	FSR0, DOAbuff		; pointer to buffer
		MOVF	DOAlast, W			; last position
		ADDWF	FSR0L, F			; is new position in buffer for received byte

		MOVFF	doa_devaddrIN, POSTINC0 ; store all three bytes of a doa packet at once
		MOVFF	doa_subaddrIN, POSTINC0 ; and postincrement pointer to buffer
		MOVFF	doa_dataIN, POSTINC0	; 

		MOVLW	0x03				; since we put three new bytes into the buffer...
		ADDWF	DOAlast, F			; move pointer three forward. Buffer MUST always be a MULTPILE of THREE (3)
									; such that it points to the next free position in the buffer
		; check for wrap-around
		MOVLW	DOAbuffsize			; buffersize -> for comparison
		CPFSEQ	DOAlast				; compare F with W, skip if eqal
		BRA		nowrap_DOAbufflast	; not equal, then move on
		CLRF	DOAlast				; otherwise we need to adjust inFIFO232last
nowrap_DOAbufflast:
		BSF		DOArdy, 0			; set first bit in DOAdy => data avail for processing
		RETURN
;;--------------------------------

;;--------------------------------
;; DOAbuff_process
;;--------------------------------
;;   processes data in DOAbuff queue
;;
;; gets BRA'ed to from DOAnextbitout
;; -> and ONLY from THERE
;; BRA's back to the middle of DOAnextbitout
;; when done
;;
;; 'DOAbuff' is a circular queue buffer
;;--------------------------------
DOAbuff_process:
		; check if data is available
		MOVF	DOAfirst, W
		CPFSEQ	DOAlast				; if( DOAfirst == DOAlast )
		BRA		DOAproc_do			; not equal, then data is available for processing
		RETURN						; otherwise the FIFO is empty, stop processing and RETURN in place of DOAnextbitout
DOAproc_do:
		; else process the byte
		LFSR	FSR0, DOAbuff
		ADDWF	FSR0L, F			; DOAfirst + base address of DOAbuff
		MOVFF	POSTINC0, doa_devaddr	; move bytes from buffer 
		MOVFF	POSTINC0, doa_subaddr	; to variables for 
		MOVFF	POSTINC0, doa_data		; DOAnextbitout state machine

		MOVLW	0x03				; since we took three bytes out of the buffer...
		ADDWF	DOAfirst, F			; move pointer three forward. Buffer MUST always be a MULTPILE of THREE (3)
		; check for wrap-around
		MOVLW	DOAbuffsize			; buffersize -> for comparison   (Buffer MUST always be a MULTPILE of THREE [3] )
		CPFSEQ	DOAfirst			; compare F with W, skip if eqal
		BRA		DOAproc_done		; not equal, then move on
		CLRF	DOAfirst			; otherwise we need to adjust start-of-buffer pointer
DOAproc_done:
		BRA		DOAbuff_process_return_here	; jump to expected return entry (since are we sort-of 'inlined' to DOAnextbitout in the C sense)
		;; NO RETURN
;;--------------------------------

;;--------------------------------
;; Routine:	DOA Send Next Bit Statemachine
;;--------------------------------
DOAnextbitout:
		MOVF	packetbitcntr, W
		XORLW	0
		BZ		DOAstate0
		MOVF	packetbitcntr, W
		ANDLW	0x01
		XORLW	1
		BZ		DOAstate_odd
		MOVLW	44
		CPFSLT	packetbitcntr
		BRA		DOAstate44
		MOVF	packetbitcntr, W
		XORLW	16
		BZ		DOAstate16
		MOVF	packetbitcntr, W
		XORLW	28
		BZ		DOAstate28
DOAstate_else:
		INCF	packetbitcntr, F
		RRCF	DOAshiftout, F
		BRA		DOA_bit_out
DOAstate0:
		; dequeue from DOAbuf
		BRA		DOAbuff_process	; DOAbuff_process will BRA back here, see label DOAbuff_process_return_here
								; unless there are no bytes available in the buffer, in which case
								; DOAbuff_process executes a RETURN for us
;;- - - - - - - - - - - - - - - - 
DOAbuff_process_return_here:
		MOVFF	doa_devaddr, DOAshiftout
		INCF	packetbitcntr, F
		RRCF	DOAshiftout, F
		BRA		DOA_bit_out
DOAstate_odd:
		INCF	packetbitcntr, F
		BRA		DOA_clk_down
DOAstate44:
		CLRF	packetbitcntr
		BRA		DOAnextbitout
DOAstate16:
		MOVFF	doa_subaddr, DOAshiftout
		INCF	packetbitcntr, F
		RRCF	DOAshiftout, F
		BRA		DOA_bit_out
DOAstate28:
		MOVFF	doa_data, DOAshiftout
		INCF	packetbitcntr, F
		RRCF	DOAshiftout, F
		BRA		DOA_bit_out
;;--------------------------------

;;-------------------------------
;; Routine:	DOA Bit Out
;;-------------------------------
;; responsible for actually wiggling 
;; the DOA lines
;;-------------------------------
DOA_bit_out:
		BTFSS	STATUS, C			; bit to send is in Carry
		BRA		doa_set0			; is it 0 or 1 ?
doa_set1:
		BTFSS	DOA_DATA			; if its 1, check if pin is already 1
		BSF		DOA_DATA			; and set if necessaryt
		BRA		doa_clk				; and go on with clk pin
doa_set0:
		BTFSC	DOA_DATA			; if its 0 check if the pin is already 0
		BCF		DOA_DATA			; and if not, reset pin to zero
doa_clk:
		NOP							; let the data out high/low settle for a little while
		NOP
		NOP
		NOP
		BSF		DOA_CLK			; then set the clock line high, signal receiving PICs to shift in the data bit
		RETURN

;;-------------------------------
;; Routine:	DOA Clock Down
;;-------------------------------
;; responsible for taking the clock
;; line to LOW after every DOA bit sent out
;;-------------------------------
DOA_clk_down:
		BCF		DOA_CLK			; clock line low after done sending, ready for next bit
		RETURN
;;;//////////////////////////////////////////////////////
;;;// END of DOA routines
;;;//////////////////////////////////////////////////////






		

		
		
		
;;;@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
;;;******************************************************
;;;** Keymatrix Routines
;;;******************************************************
;;;@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
setup_keymatrix:
		MOVLW	b'11111111'
		MOVWF	KEYMATRIX_IN_DIRREG	; all pins input for keymatrix columns
			; clear keymatrixstate[]

		; set direction register bits of A0-A3 of 74x154
		BCF		ADR0_154_TRIS
		BCF		ADR1_154_TRIS
		BCF		ADR2_154_TRIS
		BCF		ADR3_154_TRIS

		MOVLW	128
		MOVWF	counterkmx
		LFSR	FSR1, keymatrixstate
clr_matrix_next:
		CLRF	POSTINC1			; clear byte in data mem pointed to by FSR1, then increment FSR1
		DECFSZ	counterkmx, F		; decrement counter, if zero
		BRA		clr_matrix_next		; no? repeat
									; yes? go on

		; ** duplicated code from setup_analog ** (in case analog is not used)
		CALL	setup_4067			; setup pins for 4067 mux
		; ** end dupl. code **

		MOVLW	0x00
		MOVWF	adr154				; init 74x154 selector
		CALL	out_154
		RETURN
;;--------------------------------
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
keymatrix_in:
keymatrix_nextrow:
		LFSR	FSR1, keymatrixstate	; load base address of state variable for keymatrix

			; now compute the offset from keymatrixstate[]
		RCALL	combine_adr4067_adr154
		ADDWF	FSR1L, F			; add to base address
		MOVWF	col_addr			; save copy of 7bit column address in col_addr

		MOVLW	8					; 8 bits in a byte
		MOVWF	counterkmx			; setup bit counter

		MOVF	INDF1, W			; read old state of current column from memory
		MOVWF	keyinput_old		; and save in keyinput_old variable

		COMF	KEYMATRIX_IN_PORT, W	; read in 8 buttons (1 row) from keymatrix, and invert 
		MOVWF	INDF1				; and store new value in keymatrixstate[offset]
		MOVWF	keyinput_new		; make copy into keyinput_new to compute changes (working copy)
		XORWF	keyinput_old, W		; W = keyinput_old XOR keyinput_new  => shows changed bits
		MOVWF	keyinput_delta		; save working copy of changed bits/keys mask		

		BTFSS	talk, 0				; is bit 0 of talk set ? (should we notify host of changed key?)
		RETURN						; we return if talk bit<0> is not set
;;--------------------------------
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
;;--------------------------------

;;;//////////////////////////////////////////////////////
;;;// END of Keymatrix Routines
;;;//////////////////////////////////////////////////////
		
		



		
		
		
;;;@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
;;;******************************************************
;;;** analog input routines
;;;******************************************************
;;;@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
setup_analog:
		BSF		TRISA, 0
		BSF		TRISA, 1
		BSF		TRISA, 2
		BSF		TRISA, 3
		BSF		TRISA, 5			; setup RA<0-3,5> as analog input pins
		RCALL	setup_4067			; setup pins for 4067 mux
		MOVLW	b'11000010'			; right justified results, FOSC/16, AN0-4 are analog input
		MOVWF	ADCON1
;;;		MOVLW	b'11000001'			; FRC (use AD RC for ADC timing), AN0 selected, ADON=1
		MOVLW	b'01000001'			; ADCS0/1: 01 (=> FOSC/16 for ADCON1<ADCS2>=1), ADON=1 (=ADC running)
									;; why Fosc/16 ? see datasheet chapter 17.2
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
;;--------------------------------

;;-------------------------------
;; next_anp selects the next primary analog channel
;; in sequence and jumps to the appropriate setup routine
;;-------------------------------
next_anp:
		INCF	anpctr, W				; next pri. analog
		ANDLW	b'00000011'				; mask with 4
		MOVWF	anpctr					; save back to mem
		XORLW	b'00000011'				; check if its exactly 4
		BTFSC	STATUS, Z				; if it is
		CLRF	anpctr					; then reset to 0
										; now use it
		MOVF	anpctr, W				; get it
		XORLW	0						; compare with 0
		BZ		read_analog_pri1		; if( anpctr == 0 ) then run read_analog_pri1

		MOVF	anpctr, W				; get it
		XORLW	1						; compare with 1
		BZ		read_analog_pri2		; if( anpctr == 1 ) then run read_analog_pri2

		MOVF	anpctr, W				; get it
		XORLW	2						; compare with 2
		BZ		read_analog_pri3		; if( anpctr == 2 ) then run read_analog_pri3
		RETURN							; fall-thru safeguard
;;--------------------------------

;;-------------------------------
;; read_analog_pri1 
;; setup for analog primary channel 1
;;-------------------------------
read_analog_pri1:
		LFSR	FSR2, analogin+(0*2)
		MOVLW	b'00100000'			; ANP1 connected to RA5/AN4
		BRA		read_analog			; read_analog is shared among all analog inputs
;;--------------------------------

;;-------------------------------
;; read_analog_pri2 
;; setup for analog primary channel 2
;;-------------------------------
read_analog_pri2:
		LFSR	FSR2, analogin+(1*2)
		MOVLW	b'00011000'			; ANP2 connected to RA3/AN3
		BRA		read_analog			; read_analog is shared among all analog inputs
;;--------------------------------

;;-------------------------------
;; read_analog_pri3
;; setup for analog primary channel 3
;;-------------------------------
read_analog_pri3:
		LFSR	FSR2, analogin+(2*2)
		MOVLW	b'00010000'			; ANP3 connected to RA2/AN2
		BRA		read_analog			; read_analog is shared among all analog inputs
;;--------------------------------

;;-------------------------------
;; read_analog_secA
;; setup for analog secondary channel A (connected to first 4067)
;;-------------------------------
read_analog_secA:
		LFSR	FSR2, analogin+(3*2)
		; add adr4067 to FSR2
		RLNCF	adr4067, W			; load the doubled(=shifted) address used for the 4067 into W
		ADDWF	FSR2L, F			; FSR2L=FSR2L+W
		CLRF	WREG				; W = 0, does not affect carry
		ADDWFC	FSR2H, F			; FSR2H=Carry+FSR2H+0
		
		MOVLW	b'00000000'			; 4067 #1 connected to RA0/AN0
		BRA		read_analog			; read_analog is shared among all analog inputs
;;--------------------------------

;;-------------------------------
;; read_analog_secB   
;; setup for analog secondary channel B (connected to second 4067)
;;-------------------------------
read_analog_secB:
		LFSR	FSR2, analogin+(19*2)
		; add adr4067 to FSR2
		RLNCF	adr4067, W			; load the doubled(=shifted) address used for the 4067 into W
		ADDWF	FSR2L, F			; FSR2L=FSR2L+W
		CLRF	WREG				; W = 0, does not affect carry
		ADDWFC	FSR2H, F			; FSR2H=Carry+FSR2H+0
		
		MOVLW	b'00001000'			; 4067 #2 connected to RA1/AN1
;;- - - - - - - - - - - - - - - - 

;;-------------------------------
;; Common part for all read_analog_*
;; read_analog starts the aquisition process and returns
;;-------------------------------
read_analog:
		MOVWF	antmp				; store for preparation to ADCON0
		MOVLW	b'11000111'			; mask out ADCON0<CHS0:CHS2>
		ANDWF	ADCON0, W			; AND it, leave result in W
		IORWF	antmp, W			; now OR the stored mask with values set for CHS0:CHS2 and leave in W
		MOVWF	ADCON0				; and store in ADCON0

		;; now we need to wait until the ADC's holing capacitor charges.
		;; during this time we might as well be doing other stuff than running in a tight loop,
		;; so we
		RETURN
		;; for at least 13usec. after that, 
;;--------------------------------
		;; we continue here:
analog_GO:		
		BSF		ADCON0, GO_DONE		; start conversion process by setting bit GO/!DONE
		;; and since the acquisition process takes some time
		;; (approx. 20usec) we again
		RETURN						
		;; to do other useful stuff and let the ADC do its work alone.
;;--------------------------------
		;; finally, calling
analog_DONE:
		;; will do the last part of the work of reading an analog value, namely
		;; saving it to memory and running the filter algorithms
adc_wait:
		BTFSC	ADCON0, GO_DONE		; conversion is done when bit GO/!DONE is reset to zero by hardware
		BRA		adc_wait
an_retrieve:		
		MOVFF	FSR2L, an_addr		; save low byte of address for later
		CLRF	an_changed			; clear analog-value-changed flag
				;; store AD result, *high byte* ***first***
		MOVFF	ADRESH, anbufH		; working copies of ADRESH...
		MOVFF	ADRESL, anbufL		; ... and ADRESL are stored in anbufH/L

		RCALL	an_anti_jitter		; run anti_jitter filter algorithm and store values if necessary
		MOVFF	an_addr, FSR2L		; restore address

		; tell host of change immediately
		BTFSS	talk, 0				; if the host is not interested in notifications about changed analog values...
		RETURN						; we might as well just return
;;- - - - - - - - - - - - - - - - 
an_send_host_update:				
		BTFSS	an_changed, 0		; otherwise check if analog value differs from prev. value
		RETURN						; if no difference, return
;;- - - - - - - - - - - - - - - - 
an_was_change:						; ok, there was a change, so we tell the host
		MOVLW	b'01010000'			; this is a analog update packet
		CALL	rs232_send			; send it
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
		CALL	rs232_send			; and send it
		MOVF	PREINC2, W			; retrieve low byte of analog value
		CALL	rs232_send			; and send it
		RETURN						; DONE
;;--------------------------------

;;-------------------------------
;; an_anti_jitter - A filter algorithm to eliminate jitter
;;-------------------------------
;; conditions on call:
;;   FSR2 points to memory location analogin[] for *high byte* of current channel
;;   new analog value is in anbufH/anbufL
;;-------------------------------
#define JITTER_FILTER_COEFF 0x01
an_anti_jitter:
		; basically a digital low pass filter
		; difference of saved and new value
		MOVF	anbufH, W			; high byte of new value in anbufH
		SUBWF	POSTINC2, W			; get difference of old and new value
		BTFSC	STATUS, N			; result is negative if 'negative' flag  is set
		NEGF	WREG				; if result was negative, negate to make positive
		
		MOVWF	antmp				; store result of subtraction

		MOVLW	0x0					; high byte changed at all ?
		CPFSGT	antmp				; compare subtraction result with 1
		BRA		anH_zero			; not > 0 => its zero

		MOVLW	0x1					; difference in high byte can be at most 1 ('overflow')
		CPFSGT	antmp				; compare subtraction result with 1
		BRA		anH_one				; result was > 1  => the change had a magnitude > 9 bits

		; when we get here, high byte subtraction result is > 1
		BRA		bigchange

anH_zero:
		; difference of low byte
		MOVF	anbufL, W			; low byte of new value in anbufL
;;		XORWF	INDF2, W			; gets XORed with old value at analogin[i] (post decrement address
									;  so we are back pointing to the high byte)

		SUBWF	INDF2, W			; get difference of old and new value
		BTFSC	STATUS, N			; result is negative if 'negative' flag  is set
		NEGF	WREG				; if result was negative, negate to make positive
		
		MOVWF	antmp				; store result of subtraction
		MOVLW	JITTER_FILTER_COEFF	; jitter filter coefficient
		CPFSGT	antmp				; compare result with jitter filter coefficient (currently 1)
		RETURN						; no change in value after filtering
		BRA		bigchange

anH_one:
		; difference of low byte
		MOVF	anbufL, W			; low byte of new value in anbufL
;;		XORWF	INDF2, W			; gets XORed with old value at analogin[i] (post decrement address
									;  so we are back pointing to the high byte)
		SUBWF	INDF2, W			; get difference of old and new value
		BTFSC	STATUS, N			; result is negative if 'negative' flag is set
		NEGF	WREG				; if result was negative, negate to make positive

		MOVWF	antmp				; store result of subtraction
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
;;--------------------------------

;;--------------------------------

;;;//////////////////////////////////////////////////////
;;;// END of analog input routines
;;;//////////////////////////////////////////////////////







;;;@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
;;;******************************************************
;;;** I/O Adressing Routines for 4067 and 74x154
;;;******************************************************
;;;@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
setup_4067:
		BCF		TRISA, 4			; output RA4 is bit 0 of adr4067
		BCF		TRISE, 0			; output RE0 is bit 1 of adr4067
		BCF		TRISE, 1			; output RE1 is bit 2 of adr4067
		BCF		TRISE, 2			; output RE2 is bit 3 of adr4067
		MOVLW	0x00
		MOVWF	adr4067				; set address for 4067 mux to 0
		RCALL	out_4067		
		RETURN
;;--------------------------------

;;-------------------------------
;; combine adr4067 and adr154 addresses 
;; into on 7 bit address, leave in W
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
;;--------------------------------

;;-------------------------------
;; increment and output 4067 address
;;-------------------------------
inc_addr:
inc_adr4067:
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
;;--------------------------------

;;-------------------------------
;; increment and output 74x154 address
;;-------------------------------
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
;;--------------------------------

;;;//////////////////////////////////////////////////////
;;;// END of I/O Adressing Routines for 4067 and 74x154
;;;//////////////////////////////////////////////////////


		

		
		
;;;@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
;;;******************************************************
;;;** serial port routines
;;;******************************************************
;;;@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
setup_rs232:
		CLRF	inFIFO232first		; init variables, pointer to first unprocessed byte received
		CLRF	inFIFO232last		; last unprocessed byte received
		CLRF	outFIFO232first		; pointer to first unprocessed byte to send
		CLRF	outFIFO232last		; last unprocessed byte to send
		CLRF	FIFOrdy				; bitmapped register for fifo related stuff

		MOVLW	21					; BRGH=1, 21 => 115200 bps **** @40MHz ****
		MOVWF	SPBRG
		BSF		CTSTRIS				; CTS is an input
		BCF		RTSTRIS				; RTS is an output
		BSF		RTS 				; not ready to receive yet, this is controlled by rs232_recv
									; ATTENTION: RTS/CTS use inverted logic !!!
		;; enable interrupts
		BSF		INTCON, PEIE		; peripheral interrupts enable
		BCF		PIR1, RCIF			; clear interrupt flag for receive
		BCF		PIR1, TXIF			; clear interrupt flag for send
		BSF		PIE1, RCIE			; enable serial port receive interrupt
		BCF		PIE1, TXIE			; start up with serial port send-done interrupt disabled
		BCF		RTS					; RTS/CTS hardware handshake: we're ready to receive

		RETURN
;;--------------------------------

;;--------------------------------
;; rs232recv2fifo
;;--------------------------------
;;   gets called by ISR
;;   puts received byte in buffer
;;   adjusts buffer pointers
;; 'inFIFO232' is a circular queue buffer
;;--------------------------------
rs232recv2fifo:
		BSF		RTS					; RTS/CTS hardware handshake: NOT ready to receive while processing
		BTG		LED					; DEBUG: toggle LED
		LFSR	FSR0, inFIFO232		; pointer to buffer
		MOVF	inFIFO232last, W	; last position
		ADDWF	FSR0L, F			; is new position in buffer for received byte
		MOVF	RCREG, W			; serially received byte -> W
		MOVWF	INDF0				; store at last position

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

;;--------------------------------
;; rs232inFIFOprocess
;;--------------------------------
;;   processes data in inFIFO232
;; 'inFIFO232' is a circular queue buffer
;;--------------------------------
rs232inFIFOprocess:
		; check if data is available
		MOVF	inFIFO232first, W
		CPFSEQ	inFIFO232last		; if( inFIFOfirst == inFIFOlast )
		BRA		inFIFOproc_do		; not equal, then data is available for processing
		BRA		inFIFOproc_done		; equal, the FIFO is empty, we can stop processing
inFIFOproc_do:
		; else process the byte		
		LFSR	FSR0, inFIFO232
		ADDWF	FSR0L, F			; inFIFOfirst + base address of inFIFO
		MOVF	INDF0, W
		MOVWF	recv_byte			; save received byte where it is expected by some routines
		CALL	recvstatemachine	; let the statemachine decide what to do with the byte
		INCF	inFIFO232first, F	; byte in buffer processed, move pointer to next
		; check for wrap-around
		MOVLW	FIFOsize-1			; buffersize -> for comparison
		CPFSEQ	inFIFO232first		; compare F with W, skip if eqal
		BRA		inFIFOproc_done		; not equal, then move on
		CLRF	inFIFO232first		; otherwise we need to adjust inFIFO232first
inFIFOproc_done:		
		RETURN
;;--------------------------------

;;--------------------------------
;; rs232_send
;;--------------------------------
;; user callable function to send
;; a byte to host does NOT actually 
;; send, just puts byte in buffer
;; 'outFIFO232' is a circular queue buffer
;;--------------------------------
rs232_send:
		BSF		gie_flag, 0			; save GIE bit
		BTFSS	INTCON, GIE			; to differentiate if we were called from inside an interrupt handler
		BCF		gie_flag, 0
		BCF	INTCON, GIE				; disable interrupts

		MOVFF	FSR0L, fsr0l_232	; need to save fsr0 since caller might be using it
		MOVFF	FSR0H, fsr0h_232	;  dito
		MOVWF	tmpFIFO				; need to temporarily store byte
		LFSR	FSR0, outFIFO232	; load base address for indirect adressing
		MOVF	outFIFO232last, W	; last position
		ADDWF	FSR0L, F			; is new position in buffer for received byte
		MOVF	tmpFIFO, W			; get byte to send
		MOVWF	INDF0				; put in buffer
		INCF	outFIFO232last, F	; point to new (empty) position

		; enable usart send interrupt. this will cause rs232fifo2send 
		; to kick in and send the stuff in the buffer to the host
		BSF		PIE1, TXIE			; enable serial port send-done interrupt

		MOVFF	fsr0l_232, FSR0L	; restore fsr0 for caller (see above)
		MOVFF	fsr0h_232, FSR0H	;   dito

		BTFSC	gie_flag, 0			; were we called from outside an interrupt handler ?
		BSF	INTCON, GIE				; yes, then re-enable interrupts

		RETURN
;;--------------------------------

;;--------------------------------
;; rs232fifo2send
;;--------------------------------
;;   gets called by ISR???
;;   sends out byte via RS-232 (if outFIFO232 not empty)
;;   adjusts buffer pointers
;; 'outFIFO232' is a circular queue buffer
;;--------------------------------
rs232outFIFOprocess:
rs232fifo2send:
		MOVF	outFIFO232first, W	; load pointer to first unsent byte in buffer
		CPFSEQ	outFIFO232last		; check if( first unsent==last unsent) 
		BRA		fifo2send_hostrdy	; not equal, that means we do have something to send
		BRA		fifo2send_empty		; they are equal, fifo empty, nothing to send

fifo2send_hostrdy:
		BTFSS	TXSTA,TRMT			; send shift register empty? (prev. send done?)
		BRA		fifo2send_no		; no, not yet, maybe next cycle
		BTFSC	CTS					; RTS/CTS hardware handshake: is the host ready?
		BRA		fifo2send_no		; no, host not ready, maybe next time
		; else (host IS ready):
		LFSR	FSR0, outFIFO232	; load base address for indirect adressing
		ADDWF	FSR0L, F			; compute position in buffer of byte to send
		MOVF	INDF0, W			; get byte from buffer
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
fifo2send_empty:
		BCF		PIE1,TXIE			; the queue is empty, so disable serial port send-done interrupt
									; to avoid being called excessively for nothing (otherwise we 
									; would block everything else from executing
		RETURN
;;;//////////////////////////////////////////////////////
;;;// END of serial port routines
;;;//////////////////////////////////////////////////////




;;;@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
;;;******************************************************
;;;** DOB routines
;;;******************************************************
;;;@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
setup_DOB:
		BCF		DOB_CLK_TRIS		; clk is output
		BCF		DOB_DATA_TRIS		; data is output
		BCF		DOB_STO_TRIS		; store is output
		BCF		DOB_CLK				; DOB output pins low initially
		BCF		DOB_DATA			; including DATA
		BCF		DOB_STO				; and STOre
		RETURN
;;--------------------------------

;;-------------------------------
;; Digital Out Type B
;;-------------------------------
send_DOB:
		;; 74x595 CLK and STORE latch on positive (low-to-high) going transition
		MOVF	dob_data, W			; get byte to send out via DOB
		MOVWF	tmp					; make temp copy of byte to send
		MOVLW	8
		MOVWF	DOBcounter			; bit counter for 8 bits
nextbit_DOB:
		BSF		DOB_DATA			; preset condition for data output pin
		RLCF	tmp, F				; left rotate into carry, MSB gets shifted into 74x595 first
		BTFSS	STATUS, C			; bit to send now in carry
		BCF		DOB_DATA			; if carry not set: then send a 0 bit, else use preset condition
		CALL	delayDOB			; wait a specified time
		BSF		DOB_CLK				; raise clock line to clock bit into 74x595
		CALL	delayDOB			; wait a specified time
		BCF		DOB_CLK				; lower clock line again
		DECFSZ	DOBcounter, F		; decrement bit counter
		BRA		nextbit_DOB			; loop for all 8 bits

		; now that all bits are shifted into the 74x595 we can latch them into the parallel output register
		BSF		DOB_STO				; raise the store line
		CALL	delayDOB			; wait a specified time
		BCF		DOB_STO				; lower the store line

		RETURN
;;;//////////////////////////////////////////////////////
;;;// END of DOB routines
;;;//////////////////////////////////////////////////////


		

;;;@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
;;;******************************************************
;;;** I2C routines
;;;******************************************************
;;;@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
setup_i2c:
		BSF		TRISC, 3			; SCL, input for I2C
		BSF		TRISC, 4			; SDA, input for I2C
		MOVLW	b'00101000'			; SSPEN: enable serial port, I2C master mode
		MOVWF	SSPCON1
;		MOVLW	b'0' ???					; 
		MOVWF	SSPCON2				
		;; NOT FINISHED YET !!!
		RETURN
;;--------------------------------

;;;//////////////////////////////////////////////////////
;;;// END of I2C routines
;;;//////////////////////////////////////////////////////




;;;@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
;;;******************************************************
;;;** delay routines
;;;******************************************************
;;;@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
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
;;-------------------------------
delayDOB:
		NOP
		NOP
		NOP
		NOP
		NOP
		RETURN
;;-------------------------------
delay100us:  ;;@40MHZ => 10MIPS     1000 instructions in 100us
		MOVLW	200						; 200*5 = 1000 instructions
		MOVWF	delaycounter_inner
delay_us_100_inner:						; inner loop takes 5 cycles
		NOP
		NOP
		DECFSZ	delaycounter_inner,F 	; 1 cycle while looping 
		BRA		delay_us_100_inner		; 2 cycles 
		RETURN
;;-------------------------------
delay50us:  ;;@40MHZ => 10MIPS     500 instructions in 50us
		MOVLW	100						; 100*5 = 500 instructions
		MOVWF	delaycounter_inner
delay_us_50_inner:						; inner loop takes 5 cycles
		NOP
		NOP
		DECFSZ	delaycounter_inner,F 	; 1 cycle while looping 
		BRA		delay_us_100_inner		; 2 cycles 
		RETURN
;;-------------------------------
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
;;-------------------------------
delay1sec:
		RCALL	delay250ms
		RCALL	delay250ms
		RCALL	delay250ms
		RCALL	delay250ms
		RETURN

;;;//////////////////////////////////////////////////////
;;;// END of delay routines
;;;//////////////////////////////////////////////////////

;;=================================================
;; End of program
;;=================================================
	END
