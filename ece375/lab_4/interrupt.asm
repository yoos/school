;***********************************************************
;*
;*	Enter Name of file here
;*
;*	Enter the description of the program here
;*
;*	This is the skeleton file Lab 4 of ECE 375
;*
;***********************************************************
;*
;*	 Author: Enter your name
;*	   Date: Enter Date
;*
;***********************************************************

.include "m128def.inc"			; Include definition file

;***********************************************************
;*	Internal Register Definitions and Constants
;***********************************************************
.def	mpr = r16				; Multipurpose register 
.def	waitcnt = r17
.def	ilcnt = r18
.def	olcnt = r19
; Other register renames

; Constants
.equ	WTime = 100
.equ	WskrR = 4				; Right Whisker Input Bit
.equ	WskrL = 5				; Left Whisker Input Bit

.equ	EngEnR = 4				; Right engine enable bit
.equ	EngEnL = 7				; Left engine enable bit
.equ	EngDirR = 5				; Right engine direction bit
.equ	EngDirL = 6				; Left engine direction bit

.equ	MovFwd = (1<<EngDirR|1<<EngDirL)	; Move forward
.equ	MovBck = $00			; Move backward
.equ	TurnR = (1<<EngDirL)	; Turn right
.equ	TurnL = (1<<EngDirR)	; Turn left
.equ	Halt = (1<<EngEnR|1<<EngEnL)	; Halt


;***********************************************************
;*	Start of Code Segment
;***********************************************************
.cseg							; Beginning of code segment

;-----------------------------------------------------------
; Interrupt Vectors
;-----------------------------------------------------------
.org	$0000					; Beginning of IVs
		rjmp 	INIT			; Reset interrupt

; Set up the interrupt vectors for the interrupts, .i.e
;.org	$002E					; Analog Comparator IV
;		rcall	HandleAC		; Function to handle Interupt request
;		reti					; Return from interrupt
.org	$0002					; PIN0, PORTE
rcall	HitRight				; Call HitRight function
reti							; Return from interrupt

.org	$0004					; PIN1, PORTE
rcall	HitLeft
reti

.org	$0046					; End of Interrupt Vectors

;-----------------------------------------------------------
; Program Initialization
;-----------------------------------------------------------
INIT:	; The initialization routine
		; Initialize Stack Pointer
		ldi		mpr, high(RAMEND)
		out		SPH, mpr
		ldi		mpr, low(RAMEND)
		out		SPL, mpr

		; Initialize Port B for output
		ldi		mpr, (1<<EngEnL)|(1<<EngEnR)|(1<<EngDirR)|(1<<EngDirL)
		out		DDRB, mpr		; Set DDR register for PORTB
		ldi		mpr, $00
		out		PORTB, mpr		; Set default output for PORTB

		; Initialize Port D for input
		ldi		mpr, (0<<WskrL)|(0<<WskrR)
		out		DDRD, mpr		; Set DDR register for PORTD
		ldi		mpr, (1<<WskrL)|(1<<WskrR)
		out		PORTD, mpr		; Set PORTD to input with hi-z

		; Initialize external interrupts
		; Set the Interrupt Sense Control to low level 
		ldi		mpr, (0<<ISC01)|(0<<ISC00)|(0<<ISC11)|(0<<ISC10)
		out		EICRB, mpr		; Set INT4 and INT5 to trigger on low level
		ldi		mpr, $00
		sts		EICRA, mpr		; Use sts, EICRA in extended I/O space

		; Set the External Interrupt Mask
		ldi		mpr, (1<<INT0)|(1<<INT1)
		out		EIMSK, mpr

		; Move forward
		ldi		mpr, MovFwd		; Start moving forward
		out		PORTB, mpr		; Send command to motors

		; Turn on interrupts
		sei

;-----------------------------------------------------------
; Main Program
;-----------------------------------------------------------
MAIN:	; The Main program

		; Send command to Move Robot Forward 
		; That is all you should have in MAIN

		rjmp	MAIN			; Create an infinite while loop to signify the 
								; end of the program.

;***********************************************************
;*	Functions and Subroutines
;***********************************************************

;----------------------------------------------------------------
; Sub:	HitRight
; Desc:	Handles functionality of the TekBot when the right whisker
;		is triggered.
;----------------------------------------------------------------
HitRight:
		push	mpr			; Save mpr register
		push	waitcnt			; Save wait register
		in		mpr, SREG	; Save program state
		push	mpr			;

		; Move Backwards for a second
		ldi		mpr, MovBck	; Load Move Backwards command
		out		PORTB, mpr	; Send command to port
		ldi		waitcnt, WTime	; Wait for 1 second
		rcall	Wait			; Call wait function

		; Turn left for a second
		ldi		mpr, TurnL	; Load Turn Left Command
		out		PORTB, mpr	; Send command to port
		ldi		waitcnt, WTime	; Wait for 1 second
		rcall	Wait			; Call wait function

		; Move Forward again	
		ldi		mpr, MovFwd	; Load Move Forwards command
		out		PORTB, mpr	; Send command to port

		pop		mpr		; Restore program state
		out		SREG, mpr	;
		pop		waitcnt		; Restore wait register
		pop		mpr		; Restore mpr
		ret				; Return from subroutine

;----------------------------------------------------------------
; Sub:	HitLeft
; Desc:	Handles functionality of the TekBot when the left whisker
;		is triggered.
;----------------------------------------------------------------
HitLeft:
		push	mpr			; Save mpr register
		push	waitcnt			; Save wait register
		in		mpr, SREG	; Save program state
		push	mpr			;

		; Move Backwards for a second
		ldi		mpr, MovBck	; Load Move Backwards command
		out		PORTB, mpr	; Send command to port
		ldi		waitcnt, WTime	; Wait for 1 second
		rcall	Wait			; Call wait function

		; Turn right for a second
		ldi		mpr, TurnR	; Load Turn Left Command
		out		PORTB, mpr	; Send command to port
		ldi		waitcnt, WTime	; Wait for 1 second
		rcall	Wait			; Call wait function

		; Move Forward again	
		ldi		mpr, MovFwd	; Load Move Forwards command
		out		PORTB, mpr	; Send command to port

		pop		mpr		; Restore program state
		out		SREG, mpr	;
		pop		waitcnt		; Restore wait register
		pop		mpr		; Restore mpr
		ret				; Return from subroutine

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

;-----------------------------------------------------------
; Func: Template function header
; Desc: Cut and paste this and fill in the info at the 
;		beginning of your functions
;-----------------------------------------------------------
FUNC:	; Begin a function with a label

		; Save variable by pushing them to the stack

		; Execute the function here
		
		; Restore variable by popping them from the stack in reverse order

		ret		; End a function with RET

;***********************************************************
;*	Stored Program Data
;***********************************************************

; Enter any stored data you might need here

;***********************************************************
;*	Additional Program Includes
;***********************************************************
; There are no additional file includes for this program
