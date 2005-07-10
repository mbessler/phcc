;;=================================================
;; DOA_7seg_2803.asm
;; 
;; part of the PHCC (PIC HomeCockpit Controller) Project
;;
;; This is the 16x 7-Segment Display controller that consists
;; of a PIC, 16 7-Segment displays, 8 Resistors, 
;; two 74HCT164 Shift Registers and two ULN2803 Darlington Arrays.
;;
;; Copyright (c) 2004 by Manuel Bessler <m.bessler AT gmx.net>
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
;;
;;=================================================
;; Pin Assingments:
;; RA0  output, CLK for 74x164 shift registers
;; RA1  output, goes to shift register serial input
;; RA2  input,  AP2PP CLK
;; RA3  input,  AP2PP Data
;; RA4  output, to segment #7
;; RA5  output, to segment #8
;; RC0  output, to segment #1
;; RC1  output, to segment #2
;; RC2  output, to segment #3
;; RC3  output, to segment #4
;; RC4  output, to segment #5
;; RC5  output, to segment #6
;;
;; - uses internal RC OSC
;; 
;;=================================================
;; Things added:
;;
;;
;;
;; 
;;=================================================



;;;;; uncomment the processor you use
		LIST P=18F2220, R=DEC    ; what the PIC we use, decimal system as default
;;		LIST P=18F252, R=DEC    ; what the PIC we use, decimal system as default
;;		LIST P=18F242, R=DEC    ; what the PIC we use, decimal system as default

	IFDEF __18F2220
        #include "p18f2220.inc"  ; include appropriate processor definitions
		; set config bits:
		__CONFIG  _CONFIG1H, _IESO_OFF_1H & _FSCM_OFF_1H & _HSPLL_OSC_1H
		__CONFIG  _CONFIG2L, _PWRT_ON_2L & _BOR_ON_2L & _BORV_20_2L
		__CONFIG  _CONFIG2H, _WDT_OFF_2H & _WDTPS_32K_2H
		__CONFIG  _CONFIG3H, _MCLRE_ON_3H & _PBAD_DIG_3H & _CCP2MX_C1_3H
		__CONFIG  _CONFIG4L, _DEBUG_OFF_4L & _LVP_OFF_4L & _STVR_OFF_4L
		__CONFIG  _CONFIG5L, _CP0_OFF_5L & _CP1_OFF_5L
		__CONFIG  _CONFIG5H, _CPB_OFF_5H & _CPD_OFF_5H
		__CONFIG  _CONFIG6L, _WRT0_OFF_6L & _WRT1_OFF_6L
		__CONFIG  _CONFIG6H, _WRTC_OFF_6H & _WRTB_OFF_6H & _WRTD_OFF_6H
		__CONFIG  _CONFIG7L, _EBTR0_OFF_7L & _EBTR1_OFF_7L
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

	IFDEF __18F242
        #include "p18f242.inc"  ; include appropriate processor definitions
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
;;#define DEVADDR 0x30


#define	DO_A_CLK 		PORTB, 0		;; Digital-In Type A Clock line
#define DO_A_DATA 		PORTB, 1		;; Digital-In Type A Data line
#define	DO_A_CLK_TRIS 	TRISB, 0		;; direction register setup
#define	DO_A_DATA_TRIS 	TRISB, 1		;; direction register setup
#define STATE_ADDR		0x00
#define	STATE_SUBADDR	0x01
#define	STATE_DATA		0x02


#define SHREG_CLR		PORTA, 5
#define SHREG_DATA		PORTA, 3
#define SHREG_CLK		PORTA, 2
#define SHREG_CLR_TRIS	TRISA, 5
#define SHREG_DATA_TRIS	TRISA, 3
#define SHREG_CLK_TRIS	TRISA, 2


#define FREQTIMESFOUR 10    ;; for delays, speciefies the multiple of 4 MHz we're running at. eg. for 16MHz this would be 4, for 40MHz it would be 10


		; define SRAM data areas/variables/file registers
		CBLOCK 0x0
			Wsave, BSRsave, STATUSsave
			tmp, jmptmp
			ap2pp_state, ap2pp_bitcounter, ap2pp_addr, ap2pp_subaddr, ap2pp_data, ap2pp_tmp
			delaycounter_freq
			delaycounter_inner,delaycounter_outer
			actdispl
			counter
			segtmp, dimtmp
			segdata:32
			dimming
			init_tmr1L, init_tmr1H
		ENDC


;; ============== 7 Segment specials for lookup/jump table
OFF		equ		10
DASH	equ		11
MINUS	equ		11
PLUS	equ		12
SEGa	equ		13
SEGb	equ		14
SEGc	equ		15
SEGd	equ		16
SEGe	equ		17
SEGf	equ		18
SEGg	equ		19
SEGdp	equ		20
;; ========================

;;-------------------------------
;; program start point
;;-------------------------------
		ORG		0x000				; program start point at 0x000
		BRA		init

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

		;; timer0
;;		BTFSC	INTCON, TMR0IF		; was it a timer0 overflow ?
;;		CALL	tmr0int

		;; timer1
		BTFSC	PIR1, TMR1IF		; was it a timer1 overflow ?
		CALL	tmr1int

		;; timer3
;		BTFSC	PIR2, TMR3IF		; was it a timer3 overflow ?
;		CALL	tmr3int

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
	IFNDEF __18F242
		MOVLW	0x07
		MOVWF	CMCON				; all comparators off
	ENDIF
	ENDIF
		;; SETtting a bit in the TRISx registers -> input
		;; CLEARing a bit in the TRISx registers -> output



;;-------------------------------
;; application specific initialization
;;-------------------------------
		CLRF	INTCON				; all interrupts off/reset
		CALL	setup_7seg
		CALL	setup_AP2PP

		;timer and interrupt stuff
		CALL	setup_timer1
		
		; aaaaaand ACTION ...
		BCF		RCON, IPEN			; don't use interrupt priorities
		BSF		INTCON, GIE			; globally enable ints
		BSF		INTCON, PEIE		; enable peripheral ints


		CALL	delay250ms




;;-------------------------------
;; main loop
;;-------------------------------
Main:
		NOP
		NOP
		NOP
		NOP
		NOP
		BRA		Main

;================================================





;;-------------------------------
;; interrupt handlers
;;-------------------------------
tmr1int:
		BCF		T1CON, TMR1ON		; stop timer1
		MOVF	init_tmr1H, W 		; re-load timer1
		MOVWF	TMR1H
		MOVF	init_tmr1L, W 
		MOVWF	TMR1L				; low byte MUST come after high byte

		CALL	next_display		; do something on interrupt

tmr1int_done:
		BCF		PIR1, TMR1IF		; clear timer1 interrupt flag
		BSF		PIE1, TMR1IE		; enable timer1 interrupt
		BSF		T1CON, TMR1ON		; re-start timer1
		RETURN
;;--------------------------------------
;;;; DOAint: see below in the AP2PP section
;;--------------------------------------
;;--------------------------------------



;;-------------------------------
;; RETURN to next display routine
;;-------------------------------
next_display:
		MOVLW	16					; number of common cathodes(anodes)
		SUBWF	actdispl, W			; W = actdispl - 16
		BTFSS	STATUS,C			; Carry is clear if result is negative
		BRA		no_display_scan_restart
	
display_scan_restart:
		LFSR	FSR1, segdata		; setup address to first set of 16 displays
		LFSR	FSR2, segdata+16	; setup address to second set of 16 displays
		CLRF	actdispl			; set the number of the active display to zero
		RCALL	clear_164			; reset shift registers
		RCALL	segments_off
		RCALL	shift_one_into164
		RCALL	segments_out
		BRA		next_display_end

no_display_scan_restart:
		MOVF	actdispl, W
		MOVWF	counter
		RCALL	clear_164			; reset shift registers
		RCALL	segments_off		; this avoids ghosting of display segments in neighboring displays
		RCALL	shift_one_into164	; the cycling bit

next_in_shift:
		RCALL	shift_zero_into164	; push the cycling bit to correct position in shift register
		DECFSZ	counter, F
		BRA		next_in_shift
		RCALL	segments_out		

next_display_end:
		INCF	actdispl, F			; next active display
		RETURN


;;-------------------------------
;; routine to ouput segment data to port pins
;;-------------------------------
segments_out:
		MOVLW	b'11111100'
		ANDWF	PORTA, F			; make sure segment pins are zero'd out for later ORing
		MOVLW	b'00000011'
		ANDWF	PORTB, F			; make sure segment pins are zero'd out for later ORing

;;;		MOVFF	INDF1, segtmp		; load segment data for displays 1-16
		MOVFF	POSTINC1, segtmp
		RLNCF	segtmp, F			; shift segment data for displays 1-16 left ...
		RLNCF	segtmp, F			; ... twice for RB<2:7> and save in segtmp since we need it again
		MOVF	segtmp, W			; 
		ANDLW	b'11111100'
		IORWF	PORTB, F			; OR segtmp in WREG with rest of PORTB 

		MOVF	segtmp, W			; here we need segtmp again for pins on RA<0:1>
		ANDLW	b'00000011'			; mask out unused part
		IORWF	PORTA, F			; OR segtmp in WREG with rest of PORTA

		MOVFF	POSTINC2, PORTC		; segments of displays 17-32 are connected to RC<0:7>

		RETURN

;;-------------------------------
;; turn off all segments routine
;;-------------------------------
segments_off:
		; turn off all segmnts
		MOVLW	b'00000011'
		ANDWF	PORTB, F
		MOVLW	b'11111100'
		ANDWF	PORTA, F
		CLRF	PORTC		
		RETURN


;;-------------------------------
;; routine to send out serial bit to 74x164
;;-------------------------------
shift_zero_into164:	
		BCF		SHREG_DATA			;serial data low
		NOP
		BSF		SHREG_CLK			; raise the CLK to 74x164 to clock in the serial data bit
		NOP
		BCF		SHREG_CLK			; and lower the CLK to 74x164 again
		RETURN

shift_one_into164:
		BSF		SHREG_DATA			;serial data high
		NOP
		BSF		SHREG_CLK			; raise the CLK to 74x164 to clock in the serial data bit
		NOP
		BCF		SHREG_CLK			; and lower the CLK to 74x164 again
		RETURN
;;-------------------------------
clear_164:
		BCF		SHREG_CLR
		NOP
		NOP
		BSF		SHREG_CLR
		RETURN
;;-------------------------------
;;-------------------------------
		


;;-------------------------------
;; setup_timer1
;; gets timer1 ready to run
;;-------------------------------
setup_timer1:
		MOVLW	0xE0				; initial timer setup @40MHz
		MOVWF	init_tmr1H			; first High byte,
		MOVLW	0x00
		MOVWF	init_tmr1L			; then the low byte !

		MOVLW	b'10000100'			; 16bit access, (load H first, 
									; then L which triggers load from buffer to real register)
									; other clk source,
									; 1:1 prescaler (00),
									; t1osc disabled,
									; don't care,
									; src is Fosc/4,
									; timer not yet started
		MOVWF	T1CON				; set up timer1

		MOVFF	init_tmr1H, TMR1H	; load timer1 with value for Xms
		MOVFF	init_tmr1L, TMR1L	; low byte MUST come after high byte

		BCF		PIR1, TMR1IF		; clear timer1 interrupt flag
		BSF		PIE1, TMR1IE		; enable timer1 interrupt

		BSF		T1CON, TMR1ON		; start timer1
		RETURN


;;-------------------------------
;; setup_7seg
;;-------------------------------
;; sets up variables and port pins for the seven segment displays
;; (BEWARE: this does not use std 7seg pin assignments)
;;-------------------------------
setup_7seg:
		BCF		SHREG_CLR_TRIS
		BCF		SHREG_DATA_TRIS
		BCF		SHREG_CLK_TRIS
		BCF		SHREG_CLR
		BCF		SHREG_DATA
		BCF		SHREG_CLK

		LFSR	FSR1, segdata	
		MOVLW	32
		MOVWF	counter
clear_next_digit:
;;		CLRF	POSTINC1
		SETF	POSTINC1
		DECFSZ	counter, F
		BRA		clear_next_digit

		; RB2-7 are segments 0-5 of displays 1-16
		; RA0 connected to segment 6
		; RA1 connected to segment 7
		BCF		TRISA, 0
		BCF		TRISA, 1
		BCF		TRISB, 2
		BCF		TRISB, 3
		BCF		TRISB, 4
		BCF		TRISB, 5
		BCF		TRISB, 6
		BCF		TRISB, 7

		MOVLW	b'00000000'
		MOVWF	TRISC				; PortC all outputs for segments of displays 17-32

		BSF		SHREG_CLR			; release clear of 74x164
		CLRF	actdispl
		MOVLW	1
		MOVWF	dimming
		MOVWF	dimtmp
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
		; now we can update the requested display
		LFSR	FSR0, segdata		; set up addressing to displays
		MOVF	ap2pp_subaddr, W	; load subaddr into W
		MOVFF	ap2pp_data, PLUSW0	; move ap2pp_data to memory pointed to by FSR0+W(ap2pp_subaddr)
		RETURN						; we're done, next bits are for data
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
