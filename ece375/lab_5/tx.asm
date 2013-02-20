;***********************************************************
;*
;*	tx.asm
;*
;*	Transmit code for Lab 5 of ECE 375
;*
;***********************************************************
;*
;*	 Author: Soo-Hyun Yoo
;*	   Date: 20 February 2013
;*
;***********************************************************

.include "m128def.inc"			; Include definition file

;***********************************************************
;*	Internal Register Definitions and Constants
;***********************************************************
.def	mpr = r16				; Multi-Purpose Register

.def	waitcnt = r17			; Wait loop counter
.def	ilcnt = r18				; Inner loop counter
.def	olcnt = r19				; Outer loop counter


; Constants
.equ	WTime = 100

.equ	EngEnR = 4				; Right engine enable bit
.equ	EngEnL = 7				; Left engine enable bit
.equ	EngDirR = 5				; Right engine direction bit
.equ	EngDirL = 6				; Left engine direction bit

.equ	FwdBtn = 0
.equ	BckBtn = 1
.equ	TurnRBtn = 2
.equ	TurnLBtn = 3
.equ	HaltBtn = 4
.equ	FrzBtn = 5

; Use these commands between the remote and TekBot
; MSB = 1 thus:
; commands are shifted right by one and ORed with 0b10000000 = $80
.equ	MovFwd =  ($80|1<<(EngDirR-1)|1<<(EngDirL-1))	;0b10110000 Move Forwards Command
.equ	MovBck =  ($80|$00)								;0b10000000 Move Backwards Command
.equ	TurnR =   ($80|1<<(EngDirL-1))					;0b10100000 Turn Right Command
.equ	TurnL =   ($80|1<<(EngDirR-1))					;0b10010000 Turn Left Command
.equ	Halt =    ($80|1<<(EngEnR-1)|1<<(EngEnL-1))		;0b11001000 Halt Command
.equ	Freez =   ($80|$F8)
;***********************************************************
;*	Start of Code Segment
;***********************************************************
.cseg							; Beginning of code segment

;-----------------------------------------------------------
; Interrupt Vectors
;-----------------------------------------------------------
.org	$0000					; Beginning of IVs
		rjmp 	INIT			; Reset interrupt


.org	$0046					; End of Interrupt Vectors

;-----------------------------------------------------------
; Program Initialization
;-----------------------------------------------------------
INIT:
	;Stack Pointer (VERY IMPORTANT!!!!)
		ldi		mpr, low(RAMEND)
		out		SPL, mpr
		ldi		mpr, high(RAMEND)
		out		SPH, mpr

	;I/O Ports

		; Initialize Port D for input
		ldi		mpr, (0<<FwdBtn)|(0<<BckBtn)|(0<<TurnRBtn)|(0<<TurnLBtn)|(0<<HaltBtn)|(0<<FrzBtn)
		out		DDRD, mpr
		ldi		mpr, (1<<FwdBtn)|(1<<BckBtn)|(1<<TurnRBtn)|(1<<TurnLBtn)|(1<<HaltBtn)|(1<<FrzBtn)
		out		PORTD, mpr

		; Initialize external interrupts
		; Set the Interrupt Sense Control to low level
		ldi		mpr, (0<<ISC01)|(0<<ISC00)|(0<<ISC11)|(0<<ISC10)
		sts		EICRA, mpr		; Set INT0 and INT1 to trigger on low level
		ldi		mpr, $00
		out		EICRB, mpr		; Zero EICRB.

		; Set external interrupt mask
		ldi		mpr, (1<<INT0)|(1<<INT1)
		out		EIMSK, mpr

	;USART1
		;Set baudrate at 2400bps
		;Enable transmitter
		;Set frame format: 8data, 2 stop bit

	;Other

		; Turn on interrupts
		sei

;-----------------------------------------------------------
; Main Program
;-----------------------------------------------------------
MAIN:

		rjmp	MAIN

;***********************************************************
;*	Functions and Subroutines
;***********************************************************



;***********************************************************
;*	Stored Program Data
;***********************************************************



;***********************************************************
;*	Additional Program Includes
;***********************************************************
