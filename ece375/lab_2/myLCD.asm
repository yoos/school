;***********************************************************
;*
;*	Enter Name of file here
;*
;*	Enter the description of the program here
;*
;*	This is the skeleton file Lab 2 of ECE 375
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
.def	mpr = r16				; Multipurpose register required for LCD Driver
.def	ReadCnt = r23			; Counter used to read data from program memory


;***********************************************************
;*	Start of Code Segment
;***********************************************************
.cseg							; Beginning of code segment

;-----------------------------------------------------------
; Interrupt Vectors
;-----------------------------------------------------------
.org	$0000					; Beginning of IVs
		rjmp INIT				; Reset interrupt

.org	$0046					; End of Interrupt Vectors

;-----------------------------------------------------------
; Program Initialization
;-----------------------------------------------------------
INIT:							; The initialization routine
		; Initialize Stack Pointer
		ldi		mpr, HIGH(RAMEND)
		out		SPH, mpr
		ldi		mpr, LOW(RAMEND)
		out		SPL, mpr

		; Initialize LCD Display
		rcall	LCDInit

		; Move strings from Program Memory to Data Memory
		ldi		ReadCnt, LCDMaxCnt
INIT_LINE1:
		lpm		mpr, Z+			; Read program memory
		st		Y+, mpr			; Store into data memory
		dec		ReadCnt			; Decrement counter
		brne	INIT_LINE1		; Loop

;-----------------------------------------------------------
; Main Program
;-----------------------------------------------------------
MAIN:							; The Main program
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

		rjmp	MAIN			; jump back to main and create an infinite
								; while loop.  Generally, every main program is an
								; infinite while loop, never let the main program
								; just run off


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
