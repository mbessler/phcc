;;=================================================
;; DOA_40DO.asm
;; 
;; This is the firmware of the PHCC 40 channel digital out Controller Daughterboard
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
;; RB<3> to EN of all 74x595
;; RC<6> to SHCLR (shift register clear) of all 74x595
;; RC<7> to SHCLK (shift register clock) of all 74x595
;; RB<2> to LATCH0 (shift register to storage register) of first 74x595
;; RC<4> to LATCH1 (shift register to storage register) of second 74x595
;; RC<2> to LATCH2 (shift register to storage register) of third 74x595
;; RC<0> to LATCH3 (shift register to storage register) of fourth 74x595
;; RA<0> to LATCH4 (shift register to storage register) of fifth 74x595
;; RB<4> to SERDAT0 (serial data in of shift register) of first 74x595
;; RC<5> to SERDAT1 (serial data in of shift register) of second 74x595
;; RC<3> to SERDAT2 (serial data in of shift register) of third 74x595
;; RC<1> to SERDAT3 (serial data in of shift register) of fourth 74x595
;; RA<1> to SERDAT4 (serial data in of shift register) of fifth 74x595
;; 
;;
;;
;; RB<0> to DOA AP2PP_CLK
;; RB<1> to DOA AP2PP_DATA
;;
;; 
;; RA<2:5> unused (tied to +5V via pullup)
;; RB<5:7> unused (tied to +5V via pullup)
;; 
;; Program layout:
;;
;;
;;=================================================

;;;;; uncomment the processor you use
		LIST P=18F2220, R=DEC    ; what the PIC we use, decimal system as default
;;		LIST P=18F252, R=DEC    ; what the PIC we use, decimal system as default

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

;;;; @40Mhz


;; here you need to specify a unique 8bit device address for this DOA daughterboard
;;#define DEVADDR 0x10

#define OUTPUTS_BYTE	5				;; size for storage of outputs, in bytes. (5*8=40)

#define	DO_A_CLK 		PORTB, 0		;; Digital-In Type A Clock line
#define DO_A_DATA 		PORTB, 1		;; Digital-In Type A Data line
#define	DO_A_CLK_TRIS 	TRISB, 0		;; direction register setup
#define	DO_A_DATA_TRIS 	TRISB, 1		;; direction register setup

#define STATE_ADDR		0x00
#define	STATE_SUBADDR	0x01
#define	STATE_DATA		0x02


#define	EN_PIN			PORTB, 3
#define EN_TRIS			TRISB, 3

#define SHCLK_PIN		PORTC, 7
#define SHCLK_TRIS		TRISC, 7
#define SHCLR_PIN		PORTC, 6
#define SHCLR_TRIS		TRISC, 6

#define SERDAT0_PIN		PORTB, 4
#define SERDAT0_TRIS	TRISB, 4
#define SERDAT1_PIN		PORTC, 5
#define SERDAT1_TRIS	TRISC, 5
#define SERDAT2_PIN		PORTC, 3
#define SERDAT2_TRIS	TRISC, 3
#define SERDAT3_PIN		PORTC, 1
#define SERDAT3_TRIS	TRISC, 1
#define SERDAT4_PIN		PORTA, 1
#define SERDAT4_TRIS	TRISA, 1

#define LATCH0_PIN		PORTB, 2
#define LATCH0_TRIS		TRISB, 2
#define LATCH1_PIN		PORTC, 4
#define LATCH1_TRIS		TRISC, 4
#define LATCH2_PIN		PORTC, 2
#define LATCH2_TRIS		TRISC, 2
#define LATCH3_PIN		PORTC, 0
#define LATCH3_TRIS		TRISC, 0
#define LATCH4_PIN		PORTA, 0
#define LATCH4_TRIS		TRISA, 0

#define DISABLE_OUTPUTS	BSF	EN_PIN
#define ENABLE_OUTPUTS	BCF	EN_PIN

	
#define FREQTIMESFOUR 10    ;; for delays, speciefies the multiple of 4 MHz we're running at. eg. for 16MHz this would be 4, for 40MHz it would be 10


		; define SRAM data areas/variables/file registers
		CBLOCK 0x0
			Wsave, BSRsave, STATUSsave
			bitcounter
			tmp,jmptmp
			ap2pp_state, ap2pp_bitcounter, ap2pp_addr, ap2pp_subaddr, ap2pp_data, ap2pp_tmp
			delaycounter_freq
			delaycounter_inner,delaycounter_outer
			needsupdate
			outputscache:OUTPUTS_BYTE
		ENDC

		CBLOCK 0x100
			outputs:OUTPUTS_BYTE
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

		CLRF	needsupdate
		RCALL	setup_outputs
		RCALL	setup_AP2PP			; get ready to receive via DOA
;; set up interrupts
		BCF		RCON, IPEN			; don't use interrupt priorities
		BSF		INTCON, GIE			; globally enable ints
		BSF		INTCON, PEIE		; enable peripheral ints


;;-------------------------------
;; main loop
;;-------------------------------
main:
		NOP
		BTFSC	needsupdate, 0		; check flag if update is needed
		RCALL	output_update		; if LSB set, then update outputs
		NOP
		BRA		main


;;===============================



;;-------------------------------
;; output update routines
;;-------------------------------
output_update:
		BCF		needsupdate, 0		; reset flag
		MOVLW	8					; 8 bits in a byte
		MOVWF	bitcounter			; count bits

		BCF		LATCH0_PIN			; lower all latch inputs
		BCF		LATCH1_PIN
		BCF		LATCH2_PIN
		BCF		LATCH3_PIN
		BCF		LATCH4_PIN

		MOVLW	OUTPUTS_BYTE		; copy outputs RAM memory structure to temporary for shifting
		MOVWF	tmp
		LFSR	FSR1, outputs		; load address of outputs data structure
		LFSR	FSR2, outputscache	; load address of copy
copy_next:
		MOVFF	POSTINC1, POSTINC2
		DECFSZ	tmp, F				; move to next, break if zero
		BRA		copy_next

update_next:
		; prepare shift (to give the '595 more time)
		BCF		SHCLK_PIN			; set shift register clk to low

		; prepare pins, preset low
		BCF		SERDAT0_PIN
		BCF		SERDAT1_PIN
		BCF		SERDAT2_PIN
		BCF		SERDAT3_PIN
		BCF		SERDAT4_PIN

		; check bits
		BTFSC	outputscache+0, 7
		BSF		SERDAT0_PIN
		BTFSC	outputscache+1, 7
		BSF		SERDAT1_PIN
		BTFSC	outputscache+2, 7
		BSF		SERDAT2_PIN
		BTFSC	outputscache+3, 7
		BSF		SERDAT3_PIN
		BTFSC	outputscache+4, 7
		BSF		SERDAT4_PIN

		; shift bits in
		BSF		SHCLK_PIN			; clock in data on rising edge of shift register clk

		; shift bits in outputscache
		RLNCF	outputscache+0, F
		RLNCF	outputscache+1, F
		RLNCF	outputscache+2, F
		RLNCF	outputscache+3, F
		RLNCF	outputscache+4, F

		DECFSZ	bitcounter, F		; next bit, stop if all eight bits processed
		BRA		update_next

		BSF		LATCH0_PIN			; rising edge of all latch inputs (data in shift register is stored in output latch)
		BSF		LATCH1_PIN
		BSF		LATCH2_PIN
		BSF		LATCH3_PIN
		BSF		LATCH4_PIN

		RETURN

;;-------------------------------
;; output setup
;;-------------------------------
setup_outputs:
		BCF		EN_TRIS				; enable is output
		DISABLE_OUTPUTS

		BCF		SHCLK_TRIS			; SHCLK is output		
		BCF		SHCLK_PIN			; set shift register clock output pin to low 
									; (data is clocked in on rising edge)
		BCF		SHCLR_TRIS			; SHCLR is output		

		BCF		SERDAT0_TRIS		; serial data 0 is output		
		BCF		SERDAT1_TRIS		; serial data 1 is output		
		BCF		SERDAT2_TRIS		; serial data 2 is output		
		BCF		SERDAT3_TRIS		; serial data 3 is output		
		BCF		SERDAT4_TRIS		; serial data 4 is output		

		BCF		LATCH0_TRIS			; shift register latch 0 is output		
		BCF		LATCH1_TRIS			; shift register latch 1 is output		
		BCF		LATCH2_TRIS			; shift register latch 2 is output		
		BCF		LATCH3_TRIS			; shift register latch 3 is output		
		BCF		LATCH4_TRIS			; shift register latch 4 is output		

		BCF		SERDAT0_PIN			; set serial data 0 low
		BCF		SERDAT1_PIN			; set serial data 1 low
		BCF		SERDAT2_PIN			; set serial data 2 low
		BCF		SERDAT3_PIN			; set serial data 3 low
		BCF		SERDAT4_PIN			; set serial data 4 low

		BSF		LATCH0_PIN			; set shift register latch 0 pin to high (latching action on rising edge)
		BSF		LATCH1_PIN			; set shift register latch 1 pin to high (latching action on rising edge)
		BSF		LATCH2_PIN			; set shift register latch 2 pin to high (latching action on rising edge)
		BSF		LATCH3_PIN			; set shift register latch 3 pin to high (latching action on rising edge)
		BSF		LATCH4_PIN			; set shift register latch 4 pin to high (latching action on rising edge)


		MOVLW	OUTPUTS_BYTE		; clear outputs RAM memory structure
		MOVWF	tmp
		LFSR	FSR0, outputs		; load address of outputs data structure
clear_next:
		MOVLW	0
		MOVWF	POSTINC0			; clear byte
		DECFSZ	tmp, F				; move to next, break if zero
		BRA		clear_next


		BCF		SHCLR_PIN			; reset shift registers: low
		BSF		SHCLR_PIN			; .. high (ready for input)

		ENABLE_OUTPUTS				; enable the outputs, ready for action

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

		LFSR	FSR0, outputs		; set up addressing to output bits
		MOVF	ap2pp_subaddr, W	; load subaddr into W
		MOVFF	ap2pp_data, PLUSW0	; move ap2pp_data to memory pointed to by FSR0+W(ap2pp_subaddr)
		BSF		needsupdate, 0		; set flag to indicate that outputs need to be updated with new values
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
