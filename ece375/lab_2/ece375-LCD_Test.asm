;***********************************************************
;*
;*	LCDTest.asm	-	V1.5
;*
;*	This program contains all the essentials to create and 
;*	use the LCD Display on the TekBots(tm) platform.  It 
;*	covers the basics of initializing and writing to the 
;*	display.  Additionally, it also uses the Bin2ASCII 
;*	function to illustrate the limited use of a printf-like
;*	statement.  ALL CALLS TO THE LCD DRIVER WILL HAVE 
;*	CAPITALIZED COMMENTS SO THAT YOU CAN EASILY FIND WHERE
;*	AND HOW TO USE THE LCD DRIVER.
;*
;*	This version has been updated to support the LCDDriver V2
;*
;***********************************************************
;*
;*	 Author: David Zier
;*	   Date: March 25, 2003
;*	Company: TekBots(TM), Oregon State University - EECS
;*	Version: 1.5
;*
;***********************************************************
;*	Rev	Date	Name		Description
;*----------------------------------------------------------
;*	-	7/15/02	Zier		Initial Creation of Version 1.0
;*	A	3/15/03	Zier		V1.5 - Updated for LCD Driver V2
;*
;*
;***********************************************************

;-----------------------------------------------------------
;	Basic Operation
;-----------------------------------------------------------
;	Line 1 of the LCD Display will contain the counter that
;	will continually count up from 0 to 255 and eventually
;	overlap back to 0.  This will illustrate how to use
;	Bin2ASCII function and how to use a timer interrupt as
;	a bonus.
;
;	Additionally, the second line will contain the test text 
;	strings.  Button 0 - 6 will select between the different
;	text messages and button 7 will clear the second line.
;		Button 0 - "OSU - EECS"
;		Button 1 - "TekBots"
;		Button 2 - "Hello World!"
;		Button 3 - "Engineering"
;		Button 4 - "Roger Traylor"
;		Button 5 - "Don Heer"
;		Button 6 - "David Zier :)"
;		Button 7 - Clear Line
;
;	Enjoy, ~DZ
;-----------------------------------------------------------

;***********************************************************
;*	Internal Register Definitions and Constants
;***********************************************************
.include "m128def.inc"			; Include the ATMega128 Definition Doc

.def	mpr = r16				; Multi-purpose register defined for LCDDV2
.def	ReadCnt = r23			; Counter used to read data from Program Memory
.def	counter = r4			; Counter used for Bin2ASCII demo
.def	val = r5				; Value to be compared with
.def	TimerCnt = r6			; Counter used for the timer

.equ	CountAddr = $0130		; Address of ASCII counter text

.cseg							; Beginning of code segment

;***********************************************************
;*	Interrupt Vectors
;***********************************************************
.org	$0000					; Beginning of IVs
		rjmp INIT				; Reset interrupt
.org	$001e	
		rjmp TIM0_OVF			; Timer0 Overflow Handler

;***********************************************************
;*	Program Initialization 
;***********************************************************
.org	$0046					; Origin of the Program, after IVs
INIT:							; Initialize Stack Pointer
		ldi		mpr, HIGH(RAMEND)
		out		SPH, mpr
		ldi		mpr, LOW(RAMEND)
		out		SPL, mpr

;		ldi		mpr, 170
;		ldi		XL, 10
;;		ldi		XH, 01
;		rcall   Bin2ASCII

		rcall	LCDInit			; INITIALIZE THE LCD DISPLAY

		; Timer/Counter 0 Initialization
		; Clock Source: System Clock
		; Clock Value: 15.625 kHz
		; Mode: CTC top=OCR0
		; OC0 output: Disconnected
		ldi		mpr, (1<<WGM01|1<<CS02|1<<CS01|1<<CS00)	
		out		TCCR0, mpr		; Set timer prescalar to (clk/64)
		ldi		mpr, $30		; Set timer value
		out		OCR0, mpr		; Set TCNT0
		ldi		mpr, (1<<OCIE0)	; Set Timer 0 OVF interrupt

		; Timer(s)/Counter(s) Interrupt(s) initialization
		out		TIMSK, mpr		; Set TIMSK

		ldi		mpr, 0
		mov		counter, mpr	; Clear the Counter
		clr		TimerCnt		; Clear Timer Counter

		; Write initial "Counter: " string to LCD line 1
		ldi		ZL, low(TXT0<<1); Init variable registers
		ldi		ZH, high(TXT0<<1)
		ldi		YL, low(LCDLn1Addr)
		ldi		YH, high(LCDLn1Addr)
		ldi		ReadCnt, LCDMaxCnt
INIT_LINE1:
		lpm		mpr, Z+			; Read Program memory
		st		Y+, mpr			; Store into memory
		dec		ReadCnt			; Decrement Read Counter
		brne	INIT_LINE1		; Continue untill all data is read
		rcall	LCDWrLn1		; WRITE LINE 1 DATA

		; Activate interrupts
		sei						; Turn on interrupts

;***********************************************************
;*	The main program 
;***********************************************************
MAIN:	in		val, PIND		; Get input from buttons
		com		val				; Complement input
SWITCH:							; Switch statement to
S_1:	mov		mpr, val		; Copy value into mpr
		andi	mpr, (1<<0)		; Check if B0 is pressed
		breq	S_2
		ldi		ZL, low(TXT1<<1); Load Z pointer with address
		ldi		ZH,high(TXT1<<1); of Text message 1
		rcall	WriteText		; Write the Text Message
		rjmp	MAIN			; Break !!
S_2:	mov		mpr, val		; Copy value into mpr
		andi	mpr, (1<<1)		; Check if B1 is pressed
		breq	S_3
		ldi		ZL, low(TXT2<<1); Load Z pointer with address
		ldi		ZH,high(TXT2<<1); of Text message 1
		rcall	WriteText		; Write the Text Message
		rjmp	MAIN			; Break !!
S_3:	mov		mpr, val		; Copy value into mpr
		andi	mpr, (1<<2)		; Check if B2 is pressed
		breq	S_4
		ldi		ZL, low(TXT3<<1); Load Z pointer with address
		ldi		ZH,high(TXT3<<1); of Text message 1
		rcall	WriteText		; Write the Text Message
		rjmp	MAIN			; Break !!
S_4:	mov		mpr, val		; Copy value into mpr
		andi	mpr, (1<<3)		; Check if B3 is pressed
		breq	S_5
		ldi		ZL, low(TXT4<<1); Load Z pointer with address
		ldi		ZH,high(TXT4<<1); of Text message 1
		rcall	WriteText		; Write the Text Message
		rjmp	MAIN			; Break !!
S_5:	mov		mpr, val		; Copy value into mpr
		andi	mpr, (1<<4)		; Check if B4 is pressed
		breq	S_6
		ldi		ZL, low(TXT5<<1); Load Z pointer with address
		ldi		ZH,high(TXT5<<1); of Text message 1
		rcall	WriteText		; Write the Text Message
		rjmp	MAIN			; Break !!
S_6:	mov		mpr, val		; Copy value into mpr
		andi	mpr, (1<<5)		; Check if B5 is pressed
		breq	S_7
		ldi		ZL, low(TXT6<<1); Load Z pointer with address
		ldi		ZH,high(TXT6<<1); of Text message 1
		rcall	WriteText		; Write the Text Message
		rjmp	MAIN			; Break !!
S_7:	mov		mpr, val		; Copy value into mpr
		andi	mpr, (1<<6)		; Check if B6 is pressed
		breq	S_8
		ldi		ZL, low(TXT7<<1); Load Z pointer with address
		ldi		ZH,high(TXT7<<1); of Text message 1
		rcall	WriteText		; Write the Text Message
		rjmp	MAIN			; Break !!
S_8:	mov		mpr, val		; Copy value into mpr
		andi	mpr, (1<<7)		; Check if B7 is pressed
		breq	NEXT
		rcall	LCDClrLn2		; CLEAR LINE 2 OF THE LCD DISPLAY
NEXT:	rjmp	MAIN			; Go back to beginning



;***********************************************************
;*	Functions and Subroutines
;***********************************************************

;***********************************************************
;* Func:	WriteText
;* Desc:	Writes the text that is pointed to by the Z pointer
;*			from Program Memory to the second line of the LCD 
;*			Display.
;***********************************************************
WriteText:
		push	mpr				; Save the mpr register
		push	ReadCnt			; Save the ReadCounter
		rcall	LCDClrLn2		; CLEAR LINE 2 OF LCD
								; LOAD THE LCD MAX LINE COUNT (16)
		ldi		ReadCnt, LCDMaxCnt
								; LOAD THE Y POINTER WITH THE DATA
								; ADDRESS FOR LINE 2 DATA
		ldi		YL, low(LCDLn2Addr)
		ldi		YH, high(LCDLn2Addr)
WriteText_lp:					; Loop that reads the data
		lpm		mpr, Z+			; Read program data
		st		Y+, mpr			; Store data to memory
		dec		ReadCnt			; Decrement counter
		brne	WriteText_lp	; Loop untill all data is read
		rcall	LCDWrLn2		; WRITE DATA TO LINE 2
		pop		ReadCnt			; Restore the ReadCounter
		pop		mpr				; Restore the mpr register
		ret						; Return from function


;***********************************************************
;*	Interrupt Service Routines
;***********************************************************

;***********************************************************
;* ISR:		Timer 0 Overflow Interrupt Handler
;* Desc:	This ISR will increment the counter and update
;*			line 1 of the LCD Display to reflect the value
;***********************************************************
TIM0_OVF:
		push	mpr				; Save the mpr
		in		mpr, SREG		; Save the SREG
		push	mpr				; 
		
		inc		TimerCnt		; Increment counter
		brne	TIM0_OVF_DONE	; If count is not 0, leave interrupt

		; Clear Data area
		ldi		XL, low(CountAddr)
		ldi		XH, high(CountAddr)
		ldi		count, 3		; Init X-ptr and count
		ldi		mpr, ' '		; Load mpr with space char
T0_L1:	st		X+, mpr			; Clear data area
		dec		count			; Decrement count
		brne	T0_L1			; Continue until all data is cleared

		; Convert binary counter to ASCII
		mov		mpr, counter	; MOVE DATA TO MPR FOR THE B2A CALL
								; SET THE INITIAL X-PTR ADDRESS
		ldi		XL, low(CountAddr)
		ldi		XH, high(CountAddr)
		rcall	Bin2ASCII		; CALL BIN2ASCII TO CONVERT DATA
								; NOTE, COUNT REG HOLDS HOW MANY CHARS WRITTEN
		
		; Write data to LCD display
		ldi		ReadCnt, 3		; always write three chars to overide existing data in LCD
		ldi		line, 1			; SET LINE TO 1 TO WRITE TO LINE 1
		ldi		count, 9		; SET COUNT TO 10 TO START WRITTING TO THE TENTH INDEX
T0_L2:	ld		mpr, X+			; LOAD MPR WITH DATA TO WRITE
		rcall	LCDWriteByte	; CALL LCDWRITEBYTE TO WRITE DATA TO LCD DISPLAY
		inc		count			; INCREMENT COUNT TO WRITE TO NEXT LCD INDEX
		dec		ReadCnt			; decrement read counter
		brne	T0_L2			; Countinue untill all data is written

		inc		counter			; Increment the counter for the next round

TIM0_OVF_DONE:
		pop		mpr				; Restore the SREG
		out		SREG, mpr		; 
		pop		mpr				; Restore the mpr
		reti					; Return from interrupt

;***********************************************************
;*	Data Definitions
;***********************************************************
TXT0:
.DB "Counter: %      "
TXT1:
.DB "   OSU - EECS   "
TXT2:
.DB "    TekBots     "
TXT3:
.DB "  Hello World!  "
TXT4:
.DB "  Engineering   "
TXT5:
.DB " Roger Traylor  "
TXT6:
.DB "    Don Heer    "
TXT7:
.DB " David Zier :-) "

;***********************************************************
;*	Additional Program Includes
;***********************************************************
.include "LCDDriver.asm"		; Include the LCD Driver
