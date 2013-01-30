;***********************************************************
;*	myLCD.asm
;*	Drives LCD.
;***********************************************************
;*	Author: Soo-Hyun Yoo
;*	Date: 1/16/13
;***********************************************************

.include "m128def.inc"		; Include definition file

;***********************************************************
;*	Internal Register Definitions and Constants
;***********************************************************
.def	mpr = r16			; Multipurpose register required for LCD Driver
.def	ReadCnt = r19		; Counter used to read data from program memory

.def	rlo = r0			; Low byte of multiplication result
.def	rhi = r1			; High byte of multiplication result
.def	zero = r2			; Zero register, set to zero in INIT, useful for calculations.

.def	A   = r3			; 2-byte operand (first)
.def	B   = r4			; 2-byte operand (second)

.def	oloop = r17			; Outer loop counter
.def	iloop = r18			; Inner loop counter

.equ	addrA = $0100		; Beginning address of A (2 bytes)
.equ	addrB = $0102		; Beginning address of B (2 bytes)
.equ	LAddrP = $0104		; Beginning address of result (6 bytes)
.equ	HAddrP = $0109		; End address of result

.equ	valA = 10
.equ	valB = 20


;***********************************************************
;*	Start of Code Segment
;***********************************************************
.cseg						; Beginning of code segment

;-----------------------------------------------------------
; Interrupt Vectors
;-----------------------------------------------------------
.org	$0000				; Beginning of IVs
		rjmp INIT			; Reset interrupt

.org	$0046				; End of Interrupt Vectors

;-----------------------------------------------------------
; Program Initialization
;-----------------------------------------------------------
INIT:							; The initialization routine
	; Initialize Stack Pointer
	ldi		mpr, HIGH(RAMEND)
	out		SPH, mpr
	ldi		mpr, LOW(RAMEND)
	out		SPL, mpr

	; Initialize A
	ldi		ZL, low(addrA)	; Point Z to addrA.
	ldi		ZH, high(addrA)
	ldi		mpr, low(valA)	; Load low byte of valA.
	st		Z+, mpr			; Store byte at addrA and post increment Z.
	ldi		mpr, high(valA)	; Load high byte of valA.
	st		Z, mpr			; Store byte at high of addrA.

	; Initialize B
	ldi		ZL, low(addrB)	; Point Z to addrA.
	ldi		ZH, high(addrB)
	ldi		mpr, low(valB)	; Load low byte of valA.
	st		Z+, mpr			; Store byte at addrA and post increment Z.
	ldi		mpr, high(valB)	; Load high byte of valA.
	st		Z, mpr			; Store byte at high of addrA.

	; Point Z to product result
	ldi		ZL, low(LAddrP)
	ldi		ZH, high(LAddrP)

	; Set zero register to zero and don't load anything into it later.
	clr		zero

	; Initialize loop counter
	ldi		oloop, 6
INIT_ZERO:
	st		Z+, zero
	dec		oloop
	brne	INIT_ZERO

	; Initialize LCD Display
	rcall	LCDInit

	; Move strings from Program Memory to Data Memory
	ldi		ReadCnt, LCDMaxCnt
INIT_LCD:
	; Following two lines are an example of indirect addressing.
	lpm		mpr, Z+			; Load program memory and post-increment Z.
	st		Y+, mpr			; Store into data memory and post-increment Y.
	dec		ReadCnt			; Decrement counter
	brne	INIT_LCD		; Loop

;-----------------------------------------------------------
; Main Program
;-----------------------------------------------------------
MAIN:							; The Main program
	; Set up add function


	; Write first line
	ldi		ZL,  low(TXT0<<1)
	ldi		ZH, high(TXT0<<1)
	ldi		YL,  low(LCDLn1Addr)
	ldi		YH, high(LCDLn1Addr)
	ldi		ReadCnt, LCDMaxCnt
PRINT_LINE_1:
	lpm		mpr, Z+
	st		Y+, mpr
	dec		ReadCnt
	brne	PRINT_LINE_1
	rcall	LCDWrLn1

	; Write second line
	ldi		ZL,  low(TXT1<<1)
	ldi		ZH, high(TXT1<<1)
	ldi		YL,  low(LCDLn2Addr)
	ldi		YH, high(LCDLn2Addr)
	ldi		ReadCnt, LCDMaxCnt
PRINT_LINE_2:
	lpm		mpr, Z+
	st		Y+, mpr
	dec		ReadCnt
	brne	PRINT_LINE_2
	rcall	LCDWrLn2

DONE:	rjmp	DONE

;	rjmp	MAIN			; jump back to main and create an infinite
							; while loop.  Generally, every main program is an
							; infinite while loop, never let the main program
							; just run off


;-----------------------------------------------------------
; Function to add two 16-bit numbers
;-----------------------------------------------------------

;ADD16:
;	push	mpr				; Save state of machine
;	in		mpr, SREG		; Save SREG
;	push	mpr
;
;	ADC A, B
;
;	; Restore and return
;	pop mpr
;	out SREG, mpr
;	pop mpr
;	ret

;-----------------------------------------------------------
; Function to multiply two 24-bit numbers
;-----------------------------------------------------------



;***********************************************************
;*	Stored Program Data
;***********************************************************

;----------------------------------------------------------
; An example of storing a string, note the preceeding and
; appending labels, these help to access the data
;----------------------------------------------------------
TXT0:
.DB		"Soo-Hyun Yoo    "		; Storing the string in Program Memory
TXT1:
.DB		"Hello world!    "

;***********************************************************
;*	Additional Program Includes
;***********************************************************
.include "LCDDriver.asm"		; Include the LCD Driver
.include "ece375-L3_skeleton.asm"

