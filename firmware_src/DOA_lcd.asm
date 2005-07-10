;;=================================================
;; DOA_char_lcd.asm
;; 
;; This is the firmware of the PHCC 8x Character LCD Controller Daughterboard
;;
;; Part of PHCC - PIC HomeCockpit Controller
;; 
;; PHCC is a Project of cockpit.varxec.net, a home cockpit building
;; site dedicated to free software and the 
;;   Open Source/free software, multiplatform flight simulator FlightGear.
;;
;;
;;
;; Copyright (c) 2005 by Manuel Bessler <m.bessler AT gmx DOT net>
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
;;
;; Revisions
;;
;;
;;=================================================
;; Circuit setup:
;;   test HD44780 based character LC Displays
;;   - uses 8 bit mode
;; RC<0:7> to LCD data 0-7
;; RA<0> to LCD ENable
;; RB<2> to LCD R/W
;; RB<3> to LCD R/S
;; 
;; Program layout:
;;
;;
;;=================================================

;;;;; uncomment the processor you use
;;		LIST P=18F2220, R=DEC    ; what the PIC we use, decimal system as default
		LIST P=18F252, R=DEC    ; what the PIC we use, decimal system as default

	IFDEF __18F2220
        #include "p18f2220.inc"  ; include appropriate processor definitions
		; set config bits:
		__CONFIG  _CONFIG1H, _IESO_OFF_1H & _FSCMEN_OFF_1H & _HSPLL_OSC_1H
		__CONFIG  _CONFIG2L, _PWRT_ON_2L & _BOR_ON_2L & _BORV_20_2L
		__CONFIG  _CONFIG2H, _WDT_OFF_2H & _WDTPS_32K_2H
		__CONFIG  _CONFIG3H, _MCLRE_ON_3H & _PBAD_DIG_3H & _CCP2MX_C1_3H
		__CONFIG  _CONFIG4L, _BKBUG_OFF_4L & _LVP_OFF_4L & _STVR_OFF_4L
		__CONFIG  _CONFIG5L, _CP0_OFF_5L & _CP1_OFF_5L & _CP2_OFF_5L & _CP3_OFF_5L
		__CONFIG  _CONFIG5H, _CPB_OFF_5H & _CPD_OFF_5H
		__CONFIG  _CONFIG6L, _WRT0_OFF_6L & _WRT1_OFF_6L & _WRT2_OFF_6L & _WRT3_OFF_6L
		__CONFIG  _CONFIG6H, _WRTC_OFF_6H & _WRTB_OFF_6H & _WRTD_OFF_6H
		__CONFIG  _CONFIG7L, _EBTR0_OFF_7L & _EBTR1_OFF_7L & _EBTR2_OFF_7L & _EBTR3_OFF_7L
		__CONFIG  _CONFIG7H, _EBTRB_OFF_7H
	ENDIF

	IFDEF __18F252
        #include "p18f252.inc"  ; include appropriate processor definitions
		; set config bits:
		__CONFIG  _CONFIG1H, _OSCS_OFF_1H & _HSPLL_OSC_1H
		__CONFIG  _CONFIG2L, _PWRT_ON_2L & _BOR_ON_2L & _BORV_20_2L
		__CONFIG  _CONFIG2H, _WDT_OFF_2H & _WDTPS_32_2H
		__CONFIG  _CONFIG3H, _CCP2MX_OFF_3H
		__CONFIG  _CONFIG4L, _STVR_ON_4L & _LVP_OFF_4L & _DEBUG_OFF_4L

		__CONFIG  _CONFIG5L, _CP0_OFF_5L & _CP1_OFF_5L & _CP2_OFF_5L & _CP3_OFF_5L
		__CONFIG  _CONFIG5H, _CPB_OFF_5H & _CPD_OFF_5H
		__CONFIG  _CONFIG6L, _WRT0_OFF_6L & _WRT1_OFF_6L & _WRT2_OFF_6L & _WRT3_OFF_6L
		__CONFIG  _CONFIG6H, _WRTC_OFF_6H & _WRTB_OFF_6H & _WRTD_OFF_6H
		__CONFIG  _CONFIG7L, _EBTR0_OFF_7L & _EBTR1_OFF_7L & _EBTR2_OFF_7L & _EBTR3_OFF_7L
		__CONFIG  _CONFIG7H, _EBTRB_OFF_7H
	ENDIF

;;;; @40Mhz


;; here you need to specify a unique 8bit device address for this DOA daughterboard
#define DEVADDR 0x50



#define	DO_A_CLK 		PORTB, 0		;; Digital-In Type A Clock line
#define DO_A_DATA 		PORTB, 1		;; Digital-In Type A Data line
#define	DO_A_CLK_TRIS 	TRISB, 0		;; direction register setup
#define	DO_A_DATA_TRIS 	TRISB, 1		;; direction register setup
#define STATE_ADDR		0x00
#define	STATE_SUBADDR	0x01
#define	STATE_DATA		0x02


#define LCD_DATA_PORT	PORTC
#define LCD_DATA_TRIS	TRISC

#define	EN_PIN			PORTA, 0
#define EN_TRIS			TRISA, 0

#define RS_PIN			PORTB, 3
#define RS_TRIS			TRISB, 3

#define RW_PIN  		PORTB, 2
#define RW_TRIS			TRISB, 2

#define ENABLELCD		RCALL enable_lcd
#define WRITELCD		BCF RW_PIN
#define READLCD			BSF RW_PIN
#define CMDLCD			BCF RS_PIN
#define DATALCD			BSF	RS_PIN

#define LED1TRIS	TRISB, 7
#define LED1ON	BSF	PORTB,7
#define LED1OFF	BCF	PORTB,7
#define LED1TOGGLE BTG PORTB,7

#define LED2TRIS	TRISB,6
#define LED2ON	BSF	PORTB,6
#define LED2OFF	BCF	PORTB,6
#define LED2TOGGLE BTG PORTB,6
	

#define LCD_DATA_SIZE	27

#define FREQTIMESFOUR 10    ;; for delays, speciefies the multiple of 4 MHz we're running at. eg. for 16MHz this would be 4, for 40MHz it would be 10


		; define SRAM data areas/variables/file registers
		CBLOCK 0x0
			Wsave, BSRsave, STATUSsave
			counter
			tmp,jmptmp
			ap2pp_state, ap2pp_bitcounter, ap2pp_addr, ap2pp_subaddr, ap2pp_data, ap2pp_tmp
			delaycounter_freq
			delaycounter_inner,delaycounter_outer
		ENDC

		CBLOCK 0x100
			lcddata:LCD_DATA_SIZE
			lcd_upd_flag
		ENDC


;;-------------------------------
;; program start point/reset vector
;;-------------------------------

		ORG		0x0000
		GOTO	init

;;-------------------------------
;; (high priority) interrupt vector
;;-------------------------------
		ORG		0x0008
interrupt:							; global interrupts automatically disabled on entry!
		;; save registers
		MOVWF	Wsave
		MOVFF	STATUS, STATUSsave
		MOVFF	BSR, BSRsave

		;; start interrupt processing

		;; portb pin change
		BTFSC	INTCON, INT0IF		; was it a pos edge trigger pin RB0 ?
		CALL	DOAint

		BRA		end_interrupt		; non-handled interrupt? then go to end
		
end_interrupt:
		;; restore addessing
		MOVFF	BSRsave, BSR
		MOVF	Wsave, W
		MOVFF	STATUSsave, STATUS
		RETFIE



;;-------------------------------
;; generic initialization
;;-------------------------------
init:
		CLRF	PORTA				; set portA pins low
		CLRF	PORTB				; set portB pins low
		CLRF	PORTC

		MOVLW	0x0F				; 0x0F => all ANx ports digital
		MOVWF	ADCON1				; 
	IFNDEF __18F252
		MOVLW	0x07
		MOVWF	CMCON				; all comparators off
	ENDIF
		;; SETtting a bit in the TRISx registers -> input
		;; CLEARing a bit in the TRISx registers -> output



;;-------------------------------
;; application specific initialization
;;-------------------------------

		BCF		LED1TRIS
		BCF		LED2TRIS

		MOVLW	LCD_DATA_SIZE		; clear lcddata RAM memory structure
		MOVWF	tmp
		LFSR	FSR0, lcddata		; load address of lcddata
clear_next:
		MOVLW	' '
		MOVWF	POSTINC0			; clear byte
		DECFSZ	tmp, F				; move to next, break if zero
		BRA		clear_next

		MOVLW	b'00000000'			; all pins as outputs for
		MOVWF	LCD_DATA_TRIS		; LCD D0-D7 
		CLRF	LCD_DATA_PORT		; set all pins low

		BCF		EN_TRIS				; ENable line is output
		BCF		RW_TRIS				; R/W line is output
		BCF		RS_TRIS				; RegisterSelect is output

		BCF		EN_PIN				; controls pins low
		BCF		RW_PIN
		BCF		RS_PIN

		CALL	delay250ms
		CALL	delay250ms
		CALL	delay250ms
		CALL	delay250ms

		CALL	initseq_lcd			; setup LCD init sequence

		LFSR	FSR2, lcd_upd_flag	; FSR2 is dedicated to lcd_upd_flag
		CLRF	INDF2				; clear lcd_upd_flag (indicates that new data arrived and has to be written to LCD)

		CALL	setup_AP2PP			; get ready to receive via DOA
;; set up interrupts
		BCF		RCON, IPEN			; don't use interrupt priorities
		BSF		INTCON, GIE			; globally enable ints
		BSF		INTCON, PEIE		; enable peripheral ints


;;-------------------------------
;; main loop
;;-------------------------------
main:
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
	LED1ON
		BTFSC	INDF2, 0			; check lcd_upd_flag
		RCALL	lcd_update			; if LSB set, then update LCD
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

	LED1OFF
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		BRA		main



;;-------------------------------
;; LCD specific routines
;;-------------------------------

enable_lcd:
		BSF		EN_PIN

;;		MOVLW	10
;;		RCALL	delayXms
		RCALL	delay250us

;;		SETF	tmp
;;en_delay:							; short delay for EN pin
;;		DECFSZ	tmp, F
;;		BRA		en_delay

		BCF		EN_PIN
;;		MOVLW	10
;;		RCALL	delayXms
		RCALL	delay250us
		RETURN

initseq_lcd:
;		MOVLW	0x30				; init seq 8bit thrice
;		RCALL	lcd_cmd
;		MOVLW	0x30				; init seq 8bit thrice
;		RCALL	lcd_cmd
;		MOVLW	0x30				; init seq 8bit thrice
;		RCALL	lcd_cmd

		MOVLW	0x38				; 8bit, 2line, 5x7
		RCALL	lcd_cmd

		MOVLW	0x0E				; dpy on, cursor on, blink off
		RCALL	lcd_cmd

		MOVLW	0x06				; entry: cursor increase, display no shift
		RCALL	lcd_cmd

		MOVLW	0x02				; goto HOME position
		RCALL	lcd_cmd

		NOP
		NOP
		NOP
		NOP

		; write some test string
		MOVLW	'V'
		RCALL	lcd_writechar
		MOVLW	'A'
		RCALL	lcd_writechar
		MOVLW	'R'
		RCALL	lcd_writechar
		MOVLW	'X'
		RCALL	lcd_writechar
		MOVLW	'E'
		RCALL	lcd_writechar
		MOVLW	'C'
		RCALL	lcd_writechar
		MOVLW	' '
		RCALL	lcd_writechar
		MOVLW	'A'
		RCALL	lcd_writechar
		MOVLW	'u'
		RCALL	lcd_writechar
		MOVLW	't'
		RCALL	lcd_writechar
		MOVLW	'o'
		RCALL	lcd_writechar
		MOVLW	'm'
		RCALL	lcd_writechar
		MOVLW	'o'
		RCALL	lcd_writechar
		MOVLW	't'
		RCALL	lcd_writechar
		MOVLW	'i'
		RCALL	lcd_writechar
		MOVLW	'v'
		RCALL	lcd_writechar
		MOVLW	'e'
		RCALL	lcd_writechar

		MOVLW	0x40				; goto second line
		IORLW	b'10000000'
		RCALL	lcd_cmd

		RCALL	delay1sec

		MOVLW	'M'
		RCALL	lcd_writechar
		MOVLW	'M'
		RCALL	lcd_writechar
		MOVLW	'C'
		RCALL	lcd_writechar
		MOVLW	'/'
		RCALL	lcd_writechar
		MOVLW	'S'
		RCALL	lcd_writechar
		MOVLW	'D'
		RCALL	lcd_writechar
		MOVLW	' '
		RCALL	lcd_writechar
		MOVLW	'M'
		RCALL	lcd_writechar
		MOVLW	'P'
		RCALL	lcd_writechar
		MOVLW	'3'
		RCALL	lcd_writechar
		MOVLW	' '
		RCALL	lcd_writechar
		MOVLW	'P'
		RCALL	lcd_writechar
		MOVLW	'l'
		RCALL	lcd_writechar
		MOVLW	'a'
		RCALL	lcd_writechar
		MOVLW	'y'
		RCALL	lcd_writechar
		MOVLW	'e'
		RCALL	lcd_writechar
		MOVLW	'r'
		RCALL	lcd_writechar
		MOVLW	' '
		RCALL	lcd_writechar
		MOVLW	' '
		RCALL	lcd_writechar
		MOVLW	' '
		RCALL	lcd_writechar
		MOVLW	' '
		RCALL	lcd_writechar
		MOVLW	' '
		RCALL	lcd_writechar
		MOVLW	'V'
		RCALL	lcd_writechar
		MOVLW	'o'
		RCALL	lcd_writechar
		MOVLW	'l'
		RCALL	lcd_writechar
		MOVLW	' '
		RCALL	lcd_writechar
		MOVLW	'*'
		RCALL	lcd_writechar

		RETURN		

lcd_cmd:
		BCF		RW_PIN				; write to display
		BCF		RS_PIN				; command mode
		MOVWF	LCD_DATA_PORT
		NOP							; let data settle
		NOP
		ENABLELCD
		NOP
		NOP
		RETURN

lcd_writechar:
		; now data to display
		BCF		RW_PIN				; write to display
		BSF		RS_PIN				; data mode
		MOVWF	LCD_DATA_PORT
		NOP							; let data settle
		NOP
		ENABLELCD
		NOP
		NOP
		BCF		RS_PIN				; back to command mode
		RETURN


lcd_home:
		MOVLW	0x02				; goto HOME position
		RCALL	lcd_cmd
		RETURN


lcd_update:
		LED2TOGGLE
		BCF		INDF2, 0			; clear update flag
		RCALL	lcd_home			; goto home pos (row 1, line 1)
		LFSR	FSR1, lcddata		; setup pointer for lcd data in FSR1
		MOVLW	LCD_DATA_SIZE		; loop size according to lcd chars*lcd lines
		MOVWF	counter				; into counter variable
lcd_update_next:
		MOVF	POSTINC1, W			; load lcd data to show
		RCALL	lcd_writechar
		DECFSZ	counter, F
		BRA		lcd_update_next
		RETURN


;;-------------------------------
;; AP2PP
;; Adressable PIC-TO-PIC protocol comm receiver (bit-banging type) routines
;;-------------------------------
;; Pins used:
;;  DO_A_CLK: Clock
;;  DO_A_DATA: Data
;;
;; protocol: 
;; data bit has to be stable before rising edge of clock
;; gets saved into 
;;-------------------------------
setup_AP2PP:

		BSF		DO_A_CLK_TRIS		; pin is input
		BSF		DO_A_DATA_TRIS		; pin is input

		BSF		INTCON2, INTEDG0	; throw interrupt on rising edge on RB0
		BCF		INTCON, INT0IF		; clear int0 interrupt flag
		BSF		INTCON, INT0IE		; enable int0/rb0 interrupt

		CLRF	ap2pp_state			; start in idle state
		MOVLW	0x08				; protocol wants 8 bits per DOA_addr
		MOVWF	ap2pp_bitcounter	; this register keeps track of the number of the number of missing bits
		RETURN
;;-------------------------------

DOAint:
		CALL	AP2PP_recv
		BCF		INTCON, INT0IF		; clear int0 interrupt flag
		BSF		INTCON, INT0IE		; enable int0/rb0 interrupt
		RETURN
;;-------------------------------

AP2PP_recv:
		BCF		STATUS, C			; use the Carry bit for temp storage of received data bit
		BTFSC	DO_A_DATA			; read received data bit from port
		BSF		STATUS, C			; 
		RRCF	ap2pp_tmp, F		; shift carry into tmp var
		MOVF	ap2pp_state, W

		MOVWF	jmptmp					; need tmp copy for shifting for address calc
		BCF		STATUS, C			; clean carry
		RLCF	jmptmp, F				; shift state left
		MOVLW	HIGH(statejmptbl)	; load high part of jump table address
		BTFSC	STATUS, C			; 
		INCF	WREG, W				; if carry then w=w+1
		MOVWF	PCLATH				; move into high byte of program counter
		MOVLW	LOW(statejmptbl)	; load low part of jump table address
		ADDWF	jmptmp, W
		BTFSC	STATUS, C
		INCF	PCLATH, F
		MOVWF	PCL					; state machine jump table
statejmptbl:
		BRA		state_addr			; 
		BRA		state_subaddr		;
		BRA		state_data			;
;;-------------------------------

state_addr:
		DECFSZ	ap2pp_bitcounter, F
		RETURN
		; bitcounter zero? then next state
		MOVLW	STATE_SUBADDR
		MOVWF	ap2pp_state
		MOVLW	0x06				; protocol wants 6 bits per DOA_subaddr
		MOVWF	ap2pp_bitcounter	; this register keeps track of the number of the number of missing bits
		MOVF	ap2pp_tmp, W		; load tmp var
		MOVWF	ap2pp_addr			; and store in its final place
		RETURN						; we're done, next bits are for subaddr
;;-------------------------------

state_subaddr:
		DECFSZ	ap2pp_bitcounter, F
		RETURN
		; bitcounter zero? then next state
		MOVLW	STATE_DATA
		MOVWF	ap2pp_state
		MOVLW	0x08				; protocol wants 8 bits per DOA_data
		MOVWF	ap2pp_bitcounter	; this register keeps track of the number of the number of missing bits
		MOVF	ap2pp_tmp, W		; load tmp var
		MOVWF	ap2pp_subaddr		; and store in its final place
		; since subaddr is only 6 bit and gets shifted in from the left, we need to pad the upper two bits
		BCF		STATUS, C			; subaddr is only 6 bit, 
		RRCF	ap2pp_subaddr, F	; so we need to 
		BCF		STATUS, C			; left pad it 
		RRCF	ap2pp_subaddr, F	; with two zero bits
		RETURN						; we're done, next bits are for data
;;-------------------------------

state_data:
		DECFSZ	ap2pp_bitcounter, F
		RETURN
		; bitcounter zero? then initial state
		MOVLW	STATE_ADDR
		MOVWF	ap2pp_state
		MOVLW	0x08				; protocol wants 8 bits per DOA_addr
		MOVWF	ap2pp_bitcounter	; this register keeps track of the number of the number of missing bits

		MOVLW	DEVADDR				; load my device address
		CPFSEQ	ap2pp_addr			; check if the packet devaddr == my devaddr
		RETURN						; not equal then skip
		MOVF	ap2pp_tmp, W		; else: load tmp var
		MOVWF	ap2pp_data			; and store in its final place
									; packet complete, handle it
		MOVLW	0x3f				; was the subaddress == 0x3f, (0x3f => all *6* bits of subaddr set)
		CPFSEQ	ap2pp_subaddr		;  the special address indicating that LCD shall be updated
		BRA		lcd_data			; no, then it was data, branch
		BSF		INDF2, 0			; yes, set the LSB of lcd_upd_flag (FSR2 points to it)
		RETURN

lcd_data
		LFSR	FSR0, lcddata		; set up addressing to lcddata
		MOVF	ap2pp_subaddr, W	; load subaddr into W
		MOVFF	ap2pp_data, PLUSW0	; move ap2pp_data to memory pointed to by FSR0+W(ap2pp_subaddr)
		RETURN						; we're done, and ready for next packet
;;-------------------------------
;;-------------------------------



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
		BRA	delay250_inner
		DECFSZ	delaycounter_outer,F
		BRA	delay250_outer
		DECFSZ	delaycounter_freq,F
		BRA	delay_250_freq
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
		BRA	delay_inner
		DECFSZ	delaycounter_outer,F
		BRA	delay_outer
		DECFSZ	delaycounter_freq,F
		BRA	delay_freq
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
		CALL	delay250ms
		CALL	delay250ms
		CALL	delay250ms
		CALL	delay250ms
		RETURN

;;=================================================
;; End of program
;;=================================================
	END
