;;=================================================
;; DOAservo.asm
;; 
;; This is the firmware of the PHCC 8x Servo Controller Daughterboard
;;
;; Part of PHCC - PIC HomeCockpit Controller
;; 
;; PHCC is a Project of cockpit.varxec.net, a home cockpit building
;; site dedicated to free software and the 
;;   Open Source/free software, multiplatform flight simulator FlightGear.
;;
;;
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
;;   20040726       MB     initial Release
;;
;;
;;=================================================
;; Circuit setup:
;;   the PWM inputs of the 8 servos connect to PortC 
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


;;;;; uncomment the processor you use
;;		LIST P=18F2220, R=DEC    ; what the PIC we use, decimal system as default
;;		LIST P=18F252, R=DEC    ; what the PIC we use, decimal system as default
		LIST P=18F242, R=DEC    ; what the PIC we use, decimal system as default

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
;;;#define DEVADDR 0x20



#define	DO_A_CLK 		PORTB, 0		;; Digital-In Type A Clock line
#define DO_A_DATA 		PORTB, 1		;; Digital-In Type A Data line
#define	DO_A_CLK_TRIS 	TRISB, 0		;; direction register setup
#define	DO_A_DATA_TRIS 	TRISB, 1		;; direction register setup
#define STATE_ADDR		0x00
#define	STATE_SUBADDR	0x01
#define	STATE_DATA		0x02

#define SERVO_TRIS	TRISC
#define SERVO_PORT	PORTC


#define FREQTIMESFOUR 10    ;; for delays, speciefies the multiple of 4 MHz we're running at. eg. for 16MHz this would be 4, for 40MHz it would be 10


		; define SRAM data areas/variables/file registers
;		CBLOCK 0x100    0x100 gives trouble w/ LFSR/INDF stuff
		CBLOCK 0x0
			Wsave, BSRsave, STATUSsave
			servo_repeat_tmrL, servo_repeat_tmrH
			msec_tmrL, msec_tmrH, tmr_calcL, tmr_calcH
			servo0,servo1,servo2,servo3,servo4,servo5,servo6,servo7
			calibA0,calibA1,calibA2,calibA3,calibA4,calibA5,calibA6,calibA7
			calibB0,calibB1,calibB2,calibB3,calibB4,calibB5,calibB6,calibB7
			calibC0,calibC1,calibC2,calibC3,calibC4,calibC5,calibC6,calibC7
			servo
			actservo
			signal
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
		MOVLW	b'00000000'			; servos all output
		MOVWF	SERVO_TRIS			; on portC

		CLRF	actservo
		CLRF	signal

		MOVLW	128
		MOVWF	servo0				; middle position is start pos for servo0
		MOVWF	servo1				; middle position is start pos for servo1
		MOVWF	servo2				; middle position is start pos for servo2
		MOVWF	servo3				; middle position is start pos for servo3
		MOVWF	servo4				; middle position is start pos for servo4
		MOVWF	servo5				; middle position is start pos for servo5
		MOVWF	servo6				; middle position is start pos for servo6
		MOVWF	servo7				; middle position is start pos for servo7

		CALL	delay250ms



; clock: 40MHz
; Tinstr: 10MHz  -> 10,000,000 instr./sec
; servo pulses:
;   1ms -> 180deg -> 10,000 instr./msec
;   1 instr -> 0.1usec -> 100ns
;   256 steps resolution: 180deg/256 -> 0.7031deg
;     180/256deg -> 1ms/256 -> 0.0039msec -> 3.9usec -> 39 instr/step
;
; servo repeat:
;   20ms -> 200,000 instr.
;    timer1:
;       prescaler 1:8   -> 25,000 instructions
;       25,000 -> hex: 0x61A8
;       (0xFFFF+1)-0x61A8 = 0x9E58


setup_tmr0:
		MOVLW	b'11001000'			; timer on, 8bit counter, use Fosc/4, no prescaler
		MOVWF	T0CON

setup_tmr1:
		MOVLW	0x9E				; setup 20ms timing
		MOVWF	servo_repeat_tmrH	; first High byte,
		MOVLW	0x58
		MOVWF	servo_repeat_tmrL	; then the low byte !

		MOVLW	b'10110100'			; 16bit access, (load H first, then L which triggers load from buffer to real register)
									; other clk source,
									; 1:8 prescaler (11),
									; t1osc disabled,
									; don't care,
									; src is Fosc/4,
									; timer not yet started
		MOVWF	T1CON				; set up timer1

		MOVF	servo_repeat_tmrH, W ; load timer1 with value for 20ms 
		MOVWF	TMR1H
		MOVF	servo_repeat_tmrL, W 
		MOVWF	TMR1L				; low byte MUST come after high byte
		

		BCF		PIR1, TMR1IF		; clear timer1 interrupt flag
		BSF		PIE1, TMR1IE		; enable timer1 interrupt

		CALL	setup_AP2PP			; get ready to receive via DOA
		BSF		T1CON, TMR1ON		; start timer1

setup_tmr3:
		MOVLW	0xE8				; setup 1ms timing
		MOVWF	msec_tmrH			; 
		MOVLW	0xF0				;
		MOVWF	msec_tmrL			; 

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

;; Calibration:
;;    calibAx/calibBx control the 16bit offset 
;;           (Ax => low byte, Bx => high byte)
;;    calibCx controls the gain
;;

		MOVF	msec_tmrH, W
		MOVWF	calibB0
		MOVWF	calibB1
		MOVWF	calibB2
		MOVWF	calibB3
		MOVWF	calibB4
		MOVWF	calibB5
		MOVWF	calibB6
		MOVWF	calibB7
		MOVF	msec_tmrL, W
		MOVWF	calibA0
		MOVWF	calibA1
		MOVWF	calibA2
		MOVWF	calibA3
		MOVWF	calibA4
		MOVWF	calibA5
		MOVWF	calibA6
		MOVWF	calibA7
		MOVLW	200
		MOVWF	calibC0
		MOVWF	calibC1
		MOVWF	calibC2
		MOVWF	calibC3
		MOVWF	calibC4
		MOVWF	calibC5
		MOVWF	calibC6
		MOVWF	calibC7

nextcycle:
		BCF		signal,0			; reset signal flag for next cycle (every time all servos were refreshed)
		LFSR	FSR1, servo0		; setup indirect addressing to servos

waitforstart:
		BTFSS	signal, 0
		BRA		waitforstart

nextservo:
		CALL	reset_t3

		CALL	port_on
		
		BCF		PIR2, TMR3IF		; clear timer3 interrupt flag
		BSF		T3CON, TMR3ON		; START timer3
		; now that servo pin is high, we wait 1ms
msec_over:
		BTFSS	PIR2, TMR3IF		; poll for interrupt flag
		BRA		msec_over

		MOVF	INDF1, W			; load active servo position value
		MOVWF	servo				; put in working register for countdown

countdown:
		LFSR	FSR2, calibC0		; load gain calibration value
		MOVF	actservo, W			; use servo number as 
		MOVFF	PLUSW2, TMR0L		; offset address, using calibCx as calibration value for gain
		BCF		INTCON, TMR0IF		; clear timer0 int flag

wait_tmr0:
		BTFSS	INTCON, TMR0IF		; see if timer0 rolled over
		BRA		wait_tmr0

		DECFSZ	servo, F			; keep track of servo position 
		BRA		countdown			; no done yet, wait more

		CALL	port_off

		INCF	actservo, W			; next servo
		ANDLW	b'00000111'			; clip actservo to 0-7
		MOVWF	actservo			; put W back in actservo
		BTFSC	STATUS, Z			; is actservo == 0
		BRA		nextcycle			; next cycle (if actservo == 0)

		MOVF	POSTINC1, W			; increment INDF1
		BRA		nextservo			; next servo (if actservo > 0)

		BRA		main				; fall-thru


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
port_on:
		CLRF	WREG
		CPFSEQ	actservo
		BRA		pon1
		BSF		SERVO_PORT, 0		; turn on servo pwm pin
		RETURN
pon1:
		INCF	WREG, W
		CPFSEQ	actservo
		BRA		pon2
		BSF		SERVO_PORT, 1		; turn on servo pwm pin
		RETURN
pon2:
		INCF	WREG, W
		CPFSEQ	actservo
		BRA		pon3
		BSF		SERVO_PORT, 2		; turn on servo pwm pin
		RETURN
pon3:
		INCF	WREG, W
		CPFSEQ	actservo
		BRA		pon4
		BSF		SERVO_PORT, 3		; turn on servo pwm pin
		RETURN
pon4:
		INCF	WREG, W
		CPFSEQ	actservo
		BRA		pon5
		BSF		SERVO_PORT, 4		; turn on servo pwm pin
		RETURN
pon5:
		INCF	WREG, W
		CPFSEQ	actservo
		BRA		pon6
		BSF		SERVO_PORT, 5		; turn on servo pwm pin
		RETURN
pon6:
		INCF	WREG, W
		CPFSEQ	actservo
		BRA		pon7
		BSF		SERVO_PORT, 6		; turn on servo pwm pin
		RETURN
pon7:
		BSF		SERVO_PORT, 7		; turn on servo pwm pin
		RETURN



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
port_off:
		CLRF	WREG
		CPFSEQ	actservo
		BRA		poff1
		BCF		SERVO_PORT, 0		; turn off servo pwm pin
		RETURN
poff1:
		INCF	WREG, W
		CPFSEQ	actservo
		BRA		poff2
		BCF		SERVO_PORT, 1		; turn off servo pwm pin
		RETURN
poff2:
		INCF	WREG, W
		CPFSEQ	actservo
		BRA		poff3
		BCF		SERVO_PORT, 2		; turn off servo pwm pin
		RETURN
poff3:
		INCF	WREG, W
		CPFSEQ	actservo
		BRA		poff4
		BCF		SERVO_PORT, 3		; turn off servo pwm pin
		RETURN
poff4:
		INCF	WREG, W
		CPFSEQ	actservo
		BRA		poff5
		BCF		SERVO_PORT, 4		; turn off servo pwm pin
		RETURN
poff5:
		INCF	WREG, W
		CPFSEQ	actservo
		BRA		poff6
		BCF		SERVO_PORT, 5		; turn off servo pwm pin
		RETURN
poff6:
		INCF	WREG, W
		CPFSEQ	actservo
		BRA		poff7
		BCF		SERVO_PORT, 6		; turn off servo pwm pin
		RETURN
poff7:
		BCF		SERVO_PORT, 7		; turn off servo pwm pin
		RETURN

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

reset_t3:
		BCF		T3CON, TMR3ON		; STOP timer3 so we can safely mess with the values
		MOVF	msec_tmrH, W		; load timer3 with value for 1ms 
		MOVWF	tmr_calcH
		MOVF	msec_tmrL, W 		; then the low byte
		MOVWF	tmr_calcL
		; apply 16bit offset calibration
		LFSR	FSR2, calibA0		; load gain calibration value
		MOVF	actservo, W			; use servo number as 
		MOVFF	PLUSW2, tmr_calcL	; offset address, using calibAx as low byte calibration value

		LFSR	FSR2, calibB0		; load gain calibration value
		MOVF	actservo, W			; use servo number as 
		MOVFF	PLUSW2, tmr_calcH	; offset address, using calibBx as high byte calibration value

		MOVF	tmr_calcH, W
		MOVWF	TMR3H
		MOVF	tmr_calcL, W
		MOVWF	TMR3L				; low byte MUST come after high byte
		RETURN


;;-------------------------------
;; interrupt handlers
;;-------------------------------
tmr1int:
		BCF		T1CON, TMR1ON		; stop timer1
		MOVF	servo_repeat_tmrH, W ; re-load timer1 with value for 20ms 
		MOVWF	TMR1H
		MOVF	servo_repeat_tmrL, W 
		MOVWF	TMR1L				; low byte MUST come after high byte
		BCF		PIR1, TMR1IF		; clear timer1 interrupt flag
		BSF		PIE1, TMR1IE		; enable timer1 interrupt
		BSF		T1CON, TMR1ON		; re-start timer1
		BSF		signal, 0			; signal 'nextcycle' to start next cycle
		RETURN

tmr0int:
; currently not used
		RETURN

;tmr3int:
;		BCF		T3CON, TMR3ON		; stop timer3
;		MOVF	msec_tmrH, W ; re-load timer3 with value for 1ms 
;		MOVWF	TMR3H
;		MOVF	msec_tmrL, W 
;		MOVWF	TMR3L				; low byte MUST come after high byte
;		BCF		PIR2, TMR3IF		; clear timer3 interrupt flag
;		BSF		PIE2, TMR3IE		; enable timer3 interrupt
;		BSF		T3CON, TMR3ON		; re-start timer3
;		RETURN

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
		LFSR	FSR0, servo0		; set up addressing to servos
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
