;;=================================================
;; DOAanalogout.asm
;; 
;; This is the firmware of the PHCC 8x Analog Out Daughterboard
;;
;; Part of PHCC - PIC HomeCockpit Controller
;; 
;; PHCC is a Project of cockpit.varxec.net, a home cockpit building
;; site dedicated to free software and the 
;;   Open Source/free software, multiplatform flight simulator FlightGear.
;;
;; runs on 18F252
;;         18F2220
;; should run on:
;;         18F242
;;         18F248
;;         18F258
;;         18F2450
;;         18F2550
;;         18F2320
;;
;; Copyright (c) 2004 by Manuel Bessler <m.bessler AT gmx DOT net>
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
;;   20040728       MB     initial Release
;;
;;
;;=================================================
;; Circuit setup:
;;   the analog out is generated by software PWM 
;;   the two-wire DOA (Digital Out type A) communication
;;     port uses the AP2P Protocol with
;;       RB0 => AP2PP Clock
;;       RB1 => AP2PP Data
;;   a 10MHz crystal osc is necessary for correct function
;;      it uses the PLL to up the freq to 40MHz resulting in 
;;      an instruction rate of 10MIPS
;;   
;; 
;; Program layout:
;;   the main loop restarts every 20ms, during this time
;;     it refreshes each servo, one after each other
;;   each servo can be individually calibrated
;;      with a 16bit offset value
;;      with a 8bit Gain value 
;;   the comm protocol AP2PP uses RB0's INT facility
;;   for protocol specs, check http://cockpit.varxec.net/electronics/phcc/
;;   subaddress map:
;;     0-7: servo 1-8
;;     8-15: offset calibration value low byte for servo 1-8
;;     16-23: offset calibration value high byte for servo 1-8
;;     24-31: gain calibration value for servo 1-8
;;
;;
;;=================================================

;;;;; to select your processor, either (in this example the PIC18F2220
;;;  #define __18F2220
;;;;; -- or --
;;; use '-D__18F2220' as an assembler parameter on the commandline


	IFDEF __18F2220
		LIST P=18F2220, R=DEC    ; what the PIC we use, decimal system as default
        #include "p18f2220.inc"  ; include appropriate processor definitions
;; something's wrong with the 2220 implementation
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
		LIST P=18F252, R=DEC    ; what the PIC we use, decimal system as default
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
		LIST P=18F242, R=DEC    ; what the PIC we use, decimal system as default
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
;;#define DEVADDR 0x20



#define	DO_A_CLK 		PORTB, 0		;; Digital-In Type A Clock line
#define DO_A_DATA 		PORTB, 1		;; Digital-In Type A Data line
#define	DO_A_CLK_TRIS 	TRISB, 0		;; direction register setup
#define	DO_A_DATA_TRIS 	TRISB, 1		;; direction register setup
#define STATE_ADDR		0x00
#define	STATE_SUBADDR	0x01
#define	STATE_DATA		0x02

#define CHANNELTRIS0	TRISC, 0
#define CHANNELTRIS1	TRISC, 1
#define CHANNELTRIS2	TRISC, 2
#define CHANNELTRIS3	TRISC, 3
#define CHANNELTRIS4	TRISC, 4
#define CHANNELTRIS5	TRISC, 5
#define CHANNELTRIS6	TRISC, 6
#define CHANNELTRIS7	TRISC, 7

#define CHANNELTRIS8	TRISA, 0
#define CHANNELTRIS9	TRISA, 1
#define CHANNELTRIS10	TRISA, 2
#define CHANNELTRIS11	TRISA, 3
#define CHANNELTRIS12	TRISA, 5
#define CHANNELTRIS13	TRISB, 2
#define CHANNELTRIS14	TRISB, 3
#define CHANNELTRIS15	TRISB, 4

		
#define CHANNEL0	PORTC, 0
#define CHANNEL1	PORTC, 1
#define CHANNEL2	PORTC, 2
#define CHANNEL3	PORTC, 3
#define CHANNEL4	PORTC, 4
#define CHANNEL5	PORTC, 5
#define CHANNEL6	PORTC, 6
#define CHANNEL7	PORTC, 7

#define CHANNEL8	PORTA, 0
#define CHANNEL9	PORTA, 1
#define CHANNEL10	PORTA, 2
#define CHANNEL11	PORTA, 3
#define CHANNEL12	PORTA, 5
#define CHANNEL13	PORTB, 2
#define CHANNEL14	PORTB, 3
#define CHANNEL15	PORTB, 4


#define FREQTIMESFOUR 10    ;; for delays, speciefies the multiple of 4 MHz we're running at. eg. for 16MHz this would be 4, for 40MHz it would be 10


		; define SRAM data areas/variables/file registers
;		CBLOCK 0x100    0x100 gives trouble w/ LFSR/INDF stuff
		CBLOCK 0x0
			Wsave, BSRsave, STATUSsave
			tmrL, tmrH
			chan0,chan1,chan2,chan3,chan4,chan5,chan6,chan7, chan8,chan9,chan10,chan11,chan12,chan13,chan14,chan15
			gain
			gain_tmp
			an0,an1,an2,an3,an4,an5,an6,an7,an8,an9,an10,an11,an12,an13,an14,an15
			counter, duty_counter
			tmp,jmptmp
			ap2pp_state, ap2pp_bitcounter, ap2pp_addr, ap2pp_subaddr, ap2pp_data, ap2pp_tmp
			delaycounter_freq
			delaycounter_inner,delaycounter_outer
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

		;; set all ANx ports to digital
		;; 0x06 for 18f242 252 248 258
		;; 0x0f for 18f2220 2320 2480 2580 2420 2520
	IFDEF __18F242 
		MOVLW	0x06				; 0x06 => all ANx ports digital
	ENDIF
	IFDEF __18F252 
		MOVLW	0x06				; 0x06 => all ANx ports digital
	ENDIF
	IFDEF __18F248 
		MOVLW	0x06				; 0x06 => all ANx ports digital
	ENDIF
	IFDEF __18F258 
		MOVLW	0x06				; 0x06 => all ANx ports digital
	ENDIF

	IFDEF __18F2220
		MOVLW	0x0F				; 0x0F => all ANx ports digital (for pic18f2220 2320 4220 4320)
	ENDIF
	IFDEF __18F2320
		MOVLW	0x0F				; 0x0F => all ANx ports digital (for pic18f2220 2320 4220 4320)
	ENDIF
	IFDEF __18F2480
		MOVLW	0x0F				; 0x0F => all ANx ports digital (for pic18f2220 2320 4220 4320)
	ENDIF
	IFDEF __18F2580
		MOVLW	0x0F				; 0x0F => all ANx ports digital (for pic18f2220 2320 4220 4320)
	ENDIF
	IFDEF __18F2420
		MOVLW	0x0F				; 0x0F => all ANx ports digital (for pic18f2220 2320 4220 4320)
	ENDIF
	IFDEF __18F2520
		MOVLW	0x0F				; 0x0F => all ANx ports digital (for pic18f2220 2320 4220 4320)
	ENDIF

		MOVWF	ADCON1				; make it so



    IFNDEF __18F242
	IFNDEF __18F252
		MOVLW	0x07
		MOVWF	CMCON				; all comparators off
	ENDIF
	ENDIF
		
		;; SETtting a bit in the TRISx registers -> input
		;; CLEARing a bit in the TRISx registers -> output



;;-------------------------------
;; application specific initialization
;;-------------------------------
		BCF		CHANNELTRIS0		; set all analog output channels as output pins
		BCF		CHANNELTRIS1
		BCF		CHANNELTRIS2
		BCF		CHANNELTRIS3
		BCF		CHANNELTRIS4
		BCF		CHANNELTRIS5
		BCF		CHANNELTRIS6
		BCF		CHANNELTRIS7
		BCF		CHANNELTRIS8
		BCF		CHANNELTRIS9
		BCF		CHANNELTRIS10
		BCF		CHANNELTRIS11
		BCF		CHANNELTRIS12
		BCF		CHANNELTRIS13
		BCF		CHANNELTRIS14
		BCF		CHANNELTRIS15

		BCF		CHANNEL0			; make sure all pwm/analog out pins are low
		BCF		CHANNEL1
		BCF		CHANNEL2
		BCF		CHANNEL3
		BCF		CHANNEL4
		BCF		CHANNEL5
		BCF		CHANNEL6
		BCF		CHANNEL7
		BCF		CHANNEL8			; make sure all pwm/analog out pins are low
		BCF		CHANNEL9
		BCF		CHANNEL10
		BCF		CHANNEL11
		BCF		CHANNEL12
		BCF		CHANNEL13
		BCF		CHANNEL14
		BCF		CHANNEL15

		MOVLW	128
		MOVWF	chan0				; middle position is start pos for chan0
		MOVWF	chan1				; middle position is start pos for chan1
		MOVWF	chan2				; middle position is start pos for chan2
		MOVWF	chan3				; middle position is start pos for chan3
		MOVWF	chan4				; middle position is start pos for chan4
		MOVWF	chan5				; middle position is start pos for chan5
		MOVWF	chan6				; middle position is start pos for chan6
		MOVWF	chan7				; middle position is start pos for chan7
		MOVWF	chan8				; middle position is start pos for chan8
		MOVWF	chan9				; middle position is start pos for chan9
		MOVWF	chan10				; middle position is start pos for chan10
		MOVWF	chan11				; middle position is start pos for chan12
		MOVWF	chan12				; middle position is start pos for chan12
		MOVWF	chan13				; middle position is start pos for chan13
		MOVWF	chan14				; middle position is start pos for chan14
		MOVWF	chan15				; middle position is start pos for chan15

		CALL	setup_AP2PP			; get ready to receive via DOA

;; Calibration:
;;    gain controls the gain
;;

	MOVLW	0x01
;;		MOVLW	0x80
		MOVWF	gain				; initial value for gain: 128

		CALL	delay250ms


setup_tmr3:
		MOVLW	0xFF				; setup Xms timing
		MOVWF	tmrH			; 
;;;		MOVLW	0x00				;
	MOVLW	0x80
		MOVWF	tmrL			; 

		MOVLW	b'10000000'			; 16bit access, (load H first, then L which triggers load from buffer to real register)
									; CCP don't care (bits 6 and 3)
									; 1:1 prescaler (00),
									; sync don't care,
									; src is Fosc/4,
									; timer not yet started
		MOVWF	T3CON				; set up timer3

;; set up interrupts
		BCF		RCON, IPEN			; don't use interrupt priorities
		BSF		INTCON, GIE			; globally enable ints
		BSF		INTCON, PEIE		; enable peripheral ints


;;-------------------------------
;; main loop
;;-------------------------------
main:
		CLRF	tmp

nextcycle:
		; load all channels with their values
		LFSR	FSR1, chan0			; master value for channel
		LFSR	FSR2, an0 			; working value, gets decremented as we go
		MOVLW	0x10				; we have 16 channels to serve
		MOVWF	counter
preset_next:
		MOVFF	POSTINC1, POSTINC2	; load working value
		DECFSZ	counter, F
		BRA		preset_next

		; all channels on
		BSF		CHANNEL0
		BSF		CHANNEL1
		BSF		CHANNEL2
		BSF		CHANNEL3
		BSF		CHANNEL4
		BSF		CHANNEL5
		BSF		CHANNEL6
		BSF		CHANNEL7

		BSF		CHANNEL8
		BSF		CHANNEL9
		BSF		CHANNEL10
		BSF		CHANNEL11
		BSF		CHANNEL12
		BSF		CHANNEL13
		BSF		CHANNEL14
		BSF		CHANNEL15
		
				
		CLRF	duty_counter		; need to count 256 steps
next_step:
		DCFSNZ	an0, F
		BCF		CHANNEL0
		DCFSNZ	an1, F
		BCF		CHANNEL1
		DCFSNZ	an2, F
		BCF		CHANNEL2
		DCFSNZ	an3, F
		BCF		CHANNEL3
		DCFSNZ	an4, F
		BCF		CHANNEL4
		DCFSNZ	an5, F
		BCF		CHANNEL5
		DCFSNZ	an6, F
		BCF		CHANNEL6
		DCFSNZ	an7, F
		BCF		CHANNEL7
		DCFSNZ	an8, F
		BCF		CHANNEL8
		DCFSNZ	an9, F
		BCF		CHANNEL9
		DCFSNZ	an10, F
		BCF		CHANNEL10
		DCFSNZ	an11, F
		BCF		CHANNEL11
		DCFSNZ	an12, F
		BCF		CHANNEL12
		DCFSNZ	an13, F
		BCF		CHANNEL13
		DCFSNZ	an14, F
		BCF		CHANNEL14
		DCFSNZ	an15, F
		BCF		CHANNEL15

		
		; frequency shouldn't be too high, so we use a delay here which is controllable with the gain value
		MOVF	gain, W
		MOVWF	gain_tmp
next_gain:
		CALL	reset_t3
		BCF		PIR2, TMR3IF		; clear timer3 interrupt flag
		BSF		T3CON, TMR3ON		; START timer3
time_over:
		BTFSS	PIR2, TMR3IF		; poll for interrupt flag
		BRA		time_over
		DECFSZ	gain_tmp, F
		BRA		next_gain		


		DECFSZ	duty_counter, F		; has duty_counter decremented down to zero ?
		BRA		next_step			; no: we haven't done out 100% duty cycle
		BRA		nextcycle			; yes: the next cycle starts: do it all over again


		BRA		main				; fall-thru




;;-------------------------------
reset_t3:
		BCF		T3CON, TMR3ON		; STOP timer3 so we can safely mess with the values
		MOVF	tmrH, W
		MOVWF	TMR3H
		MOVF	tmrL, W
		MOVWF	TMR3L				; low byte MUST come after high byte
		RETURN
;;-------------------------------



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
		; now we can update the requested servo
		LFSR	FSR0, chan0			; set up addressing to analog channels
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