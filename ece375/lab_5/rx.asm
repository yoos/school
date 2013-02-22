;***********************************************************
;*
;*	rx.asm
;*
;*	Receive code for Lab 5 of ECE 375
;*
;***********************************************************
;*
;*	 Author: Soo-Hyun Yoo
;*	   Date: 20 February 2013
;*
;***********************************************************

.include "m128def.inc"				; Include definition file

;************************************************************
;* Variable and Constant Declarations
;************************************************************
.def	mpr = r16				; Multi-Purpose Register
.def	waitcnt = r17				; Wait Loop Counter
.def	ilcnt = r18				; Inner Loop Counter
.def	olcnt = r19				; Outer Loop Counter
.def	frzcnt = r20			; Freeze number counter

.equ	WTime = 100				; Time to wait in wait loop

.equ	WskrR = 0				; Right Whisker Input Bit
.equ	WskrL = 1				; Left Whisker Input Bit
.equ	EngEnR = 4				; Right Engine Enable Bit
.equ	EngEnL = 7				; Left Engine Enable Bit
.equ	EngDirR = 5				; Right Engine Direction Bit
.equ	EngDirL = 6				; Left Engine Direction Bit

.equ	DevID = 0b01010110

; Use these commands between the remote and TekBot
; MSB = 1 thus:
; commands are shifted right by one and ORed with 0b10000000 = $80
.equ	MovFwdCmd = ($80|1<<(EngDirR-1)|1<<(EngDirL-1))		;0b10110000 Move Forwards Command
.equ	MovBckCmd = ($80|$00)								;0b10000000 Move Backwards Command
.equ	TurnRCmd  = ($80|1<<(EngDirL-1))					;0b10100000 Turn Right Command
.equ	TurnLCmd  = ($80|1<<(EngDirR-1))					;0b10010000 Turn Left Command
.equ	HaltCmd   = ($80|1<<(EngEnR-1)|1<<(EngEnL-1))		;0b11001000 Halt Command
.equ	FrzCmd    = ($80|$F8)								; Freeze command
.equ	FrzSig    = ($80|$FD)								; Freeze signal

;/////////////////////////////////////////////////////////////
;These macros are the values to make the TekBot Move.
;/////////////////////////////////////////////////////////////

.equ	MovFwdVal = (1<<EngDirR|1<<EngDirL)	; Move Forwards Command
.equ	MovBckVal = $00				; Move Backwards Command
.equ	TurnRVal = (1<<EngDirL)			; Turn Right Command
.equ	TurnLVal = (1<<EngDirR)			; Turn Left Command
.equ	HaltVal = (1<<EngEnR|1<<EngEnL)		; Halt Command

;============================================================
; NOTE: Let me explain what the macros above are doing.  
; Every macro is executing in the pre-compiler stage before
; the rest of the code is compiled.  The macros used are
; left shift bits (<<) and logical or (|).  Here is how it 
; works:
;	Step 1.  .equ	MovFwd = (1<<EngDirR|1<<EngDirL)
;	Step 2.		substitute constants
;			 .equ	MovFwd = (1<<5|1<<6)
;	Step 3.		calculate shifts
;			 .equ	MovFwd = (b00100000|b01000000)
;	Step 4.		calculate logical or
;			 .equ	MovFwd = b01100000
; Thus MovFwd has a constant value of b01100000 or $60 and any
; instance of MovFwd within the code will be replaced with $60
; before the code is compiled.  So why did I do it this way 
; instead of explicitly specifying MovFwd = $60?  Because, if 
; I wanted to put the Left and Right Direction Bits on different 
; pin allocations, all I have to do is change thier individual 
; constants, instead of recalculating the new command and 
; everything else just falls in place.
;==============================================================

;**************************************************************
;* Beginning of code segment
;**************************************************************
.cseg

;--------------------------------------------------------------
; Interrupt Vectors
;--------------------------------------------------------------
.org	$0000				; Reset and Power On Interrupt
		rjmp	INIT		; Jump to program initialization

; Right whisker hit
.org	$0002
		rjmp	HitRight
		reti

; Left whisker hit
.org	$0004
		rjmp	HitLeft
		reti

; USART
.org	$003C
		rjmp	RX
		reti

.org	$0046				; End of Interrupt Vectors
;--------------------------------------------------------------
; Program Initialization
;--------------------------------------------------------------
INIT:
		; Initilize the Stack Pointer (VERY IMPORTANT!!!!)
		ldi		mpr, low(RAMEND)
		out		SPL, mpr		; Load SPL with low byte of RAMEND
		ldi		mpr, high(RAMEND)
		out		SPH, mpr		; Load SPH with high byte of RAMEND

		; Initialize Port B for output
		ldi		mpr, (1<<EngEnL)|(1<<EngEnR)|(1<<EngDirR)|(1<<EngDirL)		; Initialize Port B for outputs
		out		DDRB, mpr		; for output
		ldi		mpr, $00		; Set Port B Directional Register
		out		PORTB, mpr		; Port B outputs low

		; Initialize Port D for inputs
		ldi		mpr, (0<<WskrL)|(0<<WskrR)		; Set Port D Directional Register
		out		DDRD, mpr		; for inputs
		ldi		mpr, (1<<WskrL)|(1<<WskrR)		; Initialize Port D for inputs
		out		PORTD, mpr		; with Tri-State

		; USART1 setup
		;Set baudrate at 2400bps. See page 193 of datasheet.
		ldi		ZL,  low(UBRR1L)
		ldi		ZH, high(UBRR1L)
		ldi		mpr, low(416)
		st		Z, mpr
		ldi		ZL,  low(UBRR1H)
		ldi		ZH, high(UBRR1H)
		ldi		mpr, high(416)
		st		Z, mpr

		;Enable receiver and transmitter. See page 189 of datasheet.
		ldi		ZL,  low(UCSR1A)
		ldi		ZH, high(UCSR1A)
		ldi		mpr, 0
		st		Z, mpr
		ldi		ZL,  low(UCSR1B)
		ldi		ZH, high(UCSR1B)
		ldi		mpr, (1<<TXEN1)|(1<<RXEN1)|(1<<RXCIE1)	; Enable RX, TX, and interrupt
		st		Z, mpr

		;Set frame format: 8data, 2 stop bit
		ldi		ZL,  low(UCSR1C)
		ldi		ZH, high(UCSR1C)
		ldi		mpr, (1<<USBS1)|(1<<UCSZ11)|(1<<UCSZ10)
		st		Z, mpr

		; External interrupts
		ldi		mpr, (1<<ISC11)|(0<<ISC10)		; Rising edge detect
		sts		EICRA, mpr
		ldi		mpr, (1<<INT0)|(1<<INT1)
		out		EIMSK, mpr

		; Freeze count starts at four and decrements -- for some reason, the RX
		; interrupt seems to trigger at the beginning of every program cycle?
		ldi		frzcnt, 4

		; Set external interrupts.
		sei

;---------------------------------------------------------------
; Main Program
;---------------------------------------------------------------
MAIN:
		; Halt by default
		ldi		mpr, HaltVal
		out		PORTB, mpr

		rjmp	MAIN			; Continue through main

;****************************************************************
;* Subroutines and Functions
;****************************************************************

;----------------------------------------------------------------
; Sub:	HitRight
; Desc:	Handles functionality of the TekBot when the right whisker
;		is triggered.
;----------------------------------------------------------------
HitRight:
		cli					; Disable interrupts

		push	mpr			; Save mpr register
		push	waitcnt			; Save wait register
		in		mpr, SREG	; Save program state
		push	mpr			;

		; Move Backwards for a second
		ldi		mpr, MovBckVal	; Load Move Backwards command
		out		PORTB, mpr	; Send command to port
		ldi		waitcnt, WTime	; Wait for 1 second
		rcall	Wait			; Call wait function

		; Turn left for a second
		ldi		mpr, TurnLVal	; Load Turn Left Command
		out		PORTB, mpr	; Send command to port
		ldi		waitcnt, WTime	; Wait for 1 second
		rcall	Wait			; Call wait function

		pop		mpr		; Restore program state
		out		SREG, mpr	;
		pop		waitcnt		; Restore wait register
		pop		mpr		; Restore mpr

		sei					; Reenable interrupts

		ret				; Return from subroutine

;----------------------------------------------------------------
; Sub:	HitLeft
; Desc:	Handles functionality of the TekBot when the left whisker
;		is triggered.
;----------------------------------------------------------------
HitLeft:
		cli					; Disable interrupts

		push	mpr			; Save mpr register
		push	waitcnt			; Save wait register
		in		mpr, SREG	; Save program state
		push	mpr			;

		; Move Backwards for a second
		ldi		mpr, MovBckVal	; Load Move Backwards command
		out		PORTB, mpr	; Send command to port
		ldi		waitcnt, WTime	; Wait for 1 second
		rcall	Wait			; Call wait function

		; Turn right for a second
		ldi		mpr, TurnRVal	; Load Turn Left Command
		out		PORTB, mpr	; Send command to port
		ldi		waitcnt, WTime	; Wait for 1 second
		rcall	Wait			; Call wait function

		pop		mpr		; Restore program state
		out		SREG, mpr	;
		pop		waitcnt		; Restore wait register
		pop		mpr		; Restore mpr

		sei					; Reenable interrupts

		ret				; Return from subroutine

;----------------------------------------------------------------
; Sub:	RX
;----------------------------------------------------------------
RX:
		cli
		push	mpr

		lds		mpr, UDR1
		cpi		mpr, FrzSig		; Did I get the freeze signal?
		brne	MOVE			; If not, move.
		ldi		mpr, HaltVal	; Otherwise, stop.
		out		PORTB, mpr
		dec		frzcnt			; Count number of times frozen
		brne	FRZTMP			; If not, freeze temporarily.
FRZPRM:							; Otherwise, freeze permanently.
		rjmp	FRZPRM
FRZTMP:							; Freeze temporarily
		ldi		mpr, FrzSig
		out		PORTB, mpr
		ldi		ilcnt, 5		; ..for 5 seconds
FRZWAIT:
		rcall	Wait
		dec		ilcnt
		brne	FRZWAIT

		rjmp	ENDRX

MOVE:
		cpi		mpr, DevID		; Talking to me?
		brne	ENDRX
MOVEWAIT:						; Wait for command
		lds		mpr, UCSR1A
		andi	mpr, (1<<RXC1)	; Receive complete?
		cpi		mpr, (1<<RXC1)
		brne	MOVEWAIT
		lds		mpr, UDR1		; Read command

		; Check for forward command
		cpi		mpr, MovFwdCmd
		brne	BCK
		ldi		mpr, MovFwdVal
		out		PORTB, mpr
		rcall	WaitShort
		rjmp	ENDRX

BCK:
		; Check for back command
		cpi		mpr, MovBckCmd
		brne	TURNR
		ldi		mpr, MovBckVal
		out		PORTB, mpr
		rcall	WaitShort
		rjmp	ENDRX

TURNR:
		; Check for turn right command
		cpi		mpr, TurnRCmd
		brne	TURNL
		ldi		mpr, TurnRVal
		out		PORTB, mpr
		rcall	WaitShort
		rjmp	ENDRX

TURNL:
		; Check for turn left command
		cpi		mpr, TurnLCmd
		brne	HALT
		ldi		mpr, TurnLVal
		out		PORTB, mpr
		rcall	WaitShort
		rjmp	ENDRX

HALT:
		; Check for halt command
		cpi		mpr, HaltCmd
		brne	FRZ
		ldi		mpr, HaltVal
		out		PORTB, mpr
		rcall	WaitShort
		rjmp	ENDRX

FRZ:
		; Check for freeze command
		cpi		mpr, FrzCmd
		brne	ENDRX
		ldi		mpr, FrzSig
		rcall	TXFRZ					; Emit freeze signal
		rcall	WaitShort
		rjmp	ENDRX

ENDRX:
		pop		mpr
		out		PORTB, mpr

		sei
		ret

;----------------------------------------------------------------
; Sub:	TXFRZ
; Desc:	Emit freeze signal
;----------------------------------------------------------------
TXFRZ:
		push	mpr
		ldi		ZL,  low(UCSR1A)
		ldi		ZH, high(UCSR1A)
		ld		mpr, Z
TXPUSHLOOP:
		andi	mpr, (1<<UDRE1)
		cpi		mpr, (1<<UDRE1)
		brne	TXPUSHLOOP
		ldi		mpr, FrzSig
		sts		UDR1, mpr
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
Wait:
		push	waitcnt			; Save wait register
		push	ilcnt			; Save ilcnt register
		push	olcnt			; Save olcnt register

Loop:	ldi		olcnt, 224		; load olcnt register
OLoop:	ldi		ilcnt, 237		; load ilcnt register
ILoop:	dec		ilcnt			; decrement ilcnt
		brne	ILoop			; Continue Inner Loop
		dec		olcnt		; decrement olcnt
		brne	OLoop			; Continue Outer Loop
		dec		waitcnt		; Decrement wait 
		brne	Loop			; Continue Wait loop	

		pop		olcnt		; Restore olcnt register
		pop		ilcnt		; Restore ilcnt register
		pop		waitcnt		; Restore wait register
		ret				; Return from subroutine

;----------------------------------------------------------------
; Sub:	WaitShort
; Desc:	A short wait loop
;----------------------------------------------------------------
WaitShort:
		push	waitcnt
		push	olcnt
		push	ilcnt
SWLoop:	ldi		olcnt, 80
SWOLoop:ldi		ilcnt, 80
SWILoop:dec		ilcnt
		brne	SWILoop
		dec		olcnt
		brne	SWOLoop
		dec		waitcnt
		brne	SWLoop
		pop		ilcnt
		pop		olcnt
		pop		waitcnt

