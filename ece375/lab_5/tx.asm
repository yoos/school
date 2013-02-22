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
.equ	EngEnR = 4				; Right engine enable bit
.equ	EngEnL = 7				; Left engine enable bit
.equ	EngDirR = 5				; Right engine direction bit
.equ	EngDirL = 6				; Left engine direction bit

.equ	FwdBtn = 0
.equ	BckBtn = 1
.equ	TurnRBtn = 6
.equ	TurnLBtn = 7
.equ	HaltBtn = 2
.equ	FrzBtn = 5

.equ	DevID = 0b01010110

; Use these commands between the remote and TekBot
; MSB = 1 thus:
; commands are shifted right by one and ORed with 0b10000000 = $80
.equ	MovFwdCmd = ($80|1<<(EngDirR-1)|1<<(EngDirL-1))		;0b10110000 Move Forwards Command
.equ	MovBckCmd = ($80|$00)								;0b10000000 Move Backwards Command
.equ	TurnRCmd  = ($80|1<<(EngDirL-1))					;0b10100000 Turn Right Command
.equ	TurnLCmd  = ($80|1<<(EngDirR-1))					;0b10010000 Turn Left Command
.equ	HaltCmd   = ($80|1<<(EngEnR-1)|1<<(EngEnL-1))		;0b11001000 Halt Command
.equ	FrzCmd    = ($80|$F8)

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

		; Initialize Port B for output for LED debugging of signals sent.
		ldi		mpr, $00
		out		PORTB, mpr
		ldi		mpr, $FF
		out		DDRB, mpr

	;USART1
		;Set baudrate at 2400bps. See page 193 of datasheet.
		ldi		ZL,  low(UBRR1L)
		ldi		ZH, high(UBRR1L)
		ldi		mpr, low(416)
		st		Z, mpr
		ldi		ZL,  low(UBRR1H)
		ldi		ZH, high(UBRR1H)
		ldi		mpr, high(416)
		st		Z, mpr

		;Enable transmitter. See page 189 of datasheet.
		ldi		ZL,  low(UCSR1A)
		ldi		ZH, high(UCSR1A)
		ldi		mpr, 0
		st		Z, mpr
		ldi		ZL,  low(UCSR1B)
		ldi		ZH, high(UCSR1B)
		ldi		mpr, (1<<TXEN1)
		st		Z, mpr

		;Set frame format: 8data, 2 stop bit
		ldi		ZL,  low(UCSR1C)
		ldi		ZH, high(UCSR1C)
		ldi		mpr, (1<<USBS1)|(1<<UCSZ11)|(1<<UCSZ10)
		st		Z, mpr

	;Other

;-----------------------------------------------------------
; Main Program
;-----------------------------------------------------------
MAIN:
		ldi		mpr, 0			; Reset LEDs.
		out		PORTB, mpr

		in		mpr, PIND		; Read pins
		com		mpr				; Take complement
		andi	mpr, (1<<FwdBtn)|(1<<BckBtn)|(1<<TurnRBtn)|(1<<TurnLBtn)|(1<<HaltBtn)|(1<<FrzBtn)

		; Check for forward button press
		cpi		mpr, (1<<FwdBtn)
		brne	BCK
		ldi		mpr, MovFwdCmd
		rcall	TX
		rjmp	MAIN

BCK:	; Check for back button press
		cpi		mpr, (1<<BckBtn)
		brne	TURNR
		ldi		mpr, MovBckCmd
		rcall	TX
		rjmp	MAIN

TURNR:	; Check for turn right button press
		cpi		mpr, (1<<TurnRBtn)
		brne	TURNL
		ldi		mpr, TurnRCmd
		rcall	TX
		rjmp	MAIN

TURNL:	; Check for turn left button press
		cpi		mpr, (1<<TurnLBtn)
		brne	HALT
		ldi		mpr, TurnLCmd
		rcall	TX
		rjmp	MAIN

HALT:	; Check for halt button press
		cpi		mpr, (1<<HaltBtn)
		brne	FRZ
		ldi		mpr, HaltCmd
		rcall	TX
		rjmp	MAIN

FRZ:	; Check for freeze button press
		cpi		mpr, (1<<FrzBtn)
		brne	MAIN
		ldi		mpr, FrzCmd
		rcall	TX
		rjmp	MAIN


;***********************************************************
;*	Functions and Subroutines
;***********************************************************
TX:
		push	mpr				; Save mpr register
		push	waitcnt			; Save wait register
		in		mpr, SREG		; Save program state
		push	mpr

		; Send device ID
		ldi		ZL,  low(UDR1)
		ldi		ZH, high(UDR1)
		ldi		mpr, DevID
		sts		UDR1, mpr

		pop		mpr
		out		SREG, mpr
		pop		waitcnt
		pop		mpr

		; Send action code
		rcall	TXPUSH
		sts		UDR1, mpr
		out		PORTB, mpr
		rcall	WAIT

		ret

TXPUSH:
		push	mpr
		ldi		ZL,  low(UCSR1A)
		ldi		ZH, high(UCSR1A)
		ld		mpr, Z
		andi	mpr, (1<<UDRE1)
		cpi		mpr, (1<<UDRE1)
		brne	TXPUSH
		ldi		mpr, (1<<UDRE1)
		st		Z, mpr
		pop		mpr
		ret

;----------------------------------------------------------------
; Sub:	Wait
; Desc:	A wait loop that is 16 + 159975*waitcnt cycles or roughly 
;		waitcnt*10ms.  Just initialize wait for the specific amount 
;		of time in 10ms intervals. Here is the general eqaution
;		for the number of clock cycles in the wait loop:
;			((3 * ilcnt + 3) * olcnt + 3) * waitcnt + 13 + call
;----------------------------------------------------------------
WAIT:
		push	waitcnt			; Save wait register
		push	ilcnt			; Save ilcnt register
		push	olcnt			; Save olcnt register

Loop:	ldi		olcnt, 40		; load olcnt register
OLoop:	ldi		ilcnt, 40		; load ilcnt register
ILoop:	dec		ilcnt			; decrement ilcnt
		brne	ILoop			; Continue Inner Loop
		dec		olcnt			; decrement olcnt
		brne	OLoop			; Continue Outer Loop
		dec		waitcnt			; Decrement wait 
		brne	Loop			; Continue Wait loop	

		pop		olcnt		; Restore olcnt register
		pop		ilcnt		; Restore ilcnt register
		pop		waitcnt		; Restore wait register
		ret				; Return from subroutine


;***********************************************************
;*	Stored Program Data
;***********************************************************



;***********************************************************
;*	Additional Program Includes
;***********************************************************
