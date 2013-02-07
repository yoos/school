;***********************************************************
;*
;*	Enter Name of file here
;*
;*	Enter the description of the program here
;*
;*	This is the RECEIVE skeleton file for Lab 5 of ECE 375
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
.def	mpr = r16				; Multi-Purpose Register

.equ	WskrR = 0				; Right Whisker Input Bit
.equ	WskrL = 1				; Left Whisker Input Bit
.equ	EngEnR = 4				; Right Engine Enable Bit
.equ	EngEnL = 7				; Left Engine Enable Bit
.equ	EngDirR = 5				; Right Engine Direction Bit
.equ	EngDirL = 6				; Left Engine Direction Bit

.equ	BotID = ;(Enter you group ID here (8bits)); Unique XD ID (MSB = 0)

;/////////////////////////////////////////////////////////////
;These macros are the values to make the TekBot Move.
;/////////////////////////////////////////////////////////////

.equ	MovFwd =  (1<<EngDirR|1<<EngDirL)	;0b01100000 Move Forwards Command
.equ	MovBck =  $00						;0b00000000 Move Backwards Command
.equ	TurnR =   (1<<EngDirL)				;0b01000000 Turn Right Command
.equ	TurnL =   (1<<EngDirR)				;0b00100000 Turn Left Command
.equ	Halt =    (1<<EngEnR|1<<EngEnL)		;0b10010000 Halt Command

;***********************************************************
;*	Start of Code Segment
;***********************************************************
.cseg							; Beginning of code segment

;-----------------------------------------------------------
; Interrupt Vectors
;-----------------------------------------------------------
.org	$0000					; Beginning of IVs
		rjmp 	INIT			; Reset interrupt

;Should have Interrupt vectors for:
;- Left wisker
;- Right wisker
;- USART receive

.org	$0046					; End of Interrupt Vectors

;-----------------------------------------------------------
; Program Initialization
;-----------------------------------------------------------
INIT:
	;Stack Pointer (VERY IMPORTANT!!!!)
	;I/O Ports
	;USART1
		;Set baudrate at 2400bps
		;Enable receiver and enable receive interrupts
		;Set frame format: 8data bits, 2 stop bit
	;External Interrupts
		;Set the External Interrupt Mask
		;Set the Interrupt Sense Control to Rising Edge detection

	;Other


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

