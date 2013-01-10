
/*
This code will cause a TekBot v1.03 connected to the AVR board to 'dance' in a cool
pattern. Remember, the inputs for the v1.03 TekBot are 'active low'. This means you need
to have a '0' to activate them.

PORT MAP
Port B, Pin 4 -> Output -> Right Motor Enable
Port B, Pin 5 -> Output -> Right Motor Direction
Port B, Pin 7 -> Output -> Left Motor Enable
Port B, Pin 6 -> Output -> Left Motor Direction
*/
#define F_CPU 16000000
#include <avr/io.h> 
#include <util/delay.h> 
#include <stdio.h>

int main(void)
{              

DDRB =0b11110000;	//Setup Port B for Input/Output
PORTB=0b11110000;	//Turn off both motors

while (1)		//Loop Forever
      {
      PORTB=0b01100000;	//Make TekBot move forward
      _delay_ms(500);
      PORTB=0b00000000;	//Reverse
      _delay_ms(500);  
      PORTB=0b00100000; //Turn Left
      _delay_ms(1000);  
      PORTB=0b01000000;	//Turn Right
      _delay_ms(2000);  
      PORTB=0b00100000; //Turn Left
      _delay_ms(1000);
      };
}
