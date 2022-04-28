#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include "project.h"
volatile int PINDtot;
volatile unsigned char new_state, old_state;
volatile int count = 76;
volatile unsigned char a, b;
volatile unsigned char changed = 0;  // Flag for state change

void timer2_init(void)
{
    TCCR2A |= (0b11 << WGM00);  // Fast PWM mode, modulus = 256
    TCCR2A |= (0b10 << COM0A0); // Turn D11 on at 0x00 and off at OCR2A
    OCR2A = 128;                // Initial pulse duty cycle of 50%
    TCCR2B |= (0b111 << CS20);  // Prescaler = 1024 for 16ms period
}

ISR(PCINT2_vect){
    // lcd_moveto(0, 0);
    // lcd_stringout("test");
    PINDtot = PIND;
    b = PINDtot & (1<<PD2);
    a = PINDtot & (1<<PD3);

	// For each state, examine the two input bits to see if state
	// has changed, and if so set "new_state" to the new state,
	// and adjust the count value.
	if (old_state == 0) {
        if(a){
            new_state = 1;
            count++;
        }
        else if (b){
            new_state = 2;
            count--;
        }

	    // Handle A and B inputs for state 0
	}
	else if (old_state == 1) {

	    // Handle A and B inputs for state 1
        if(b){
            new_state = 3;
            count++;
        }
        else if (!a){
            count--;
            new_state = 0;
        }

	}
	else if (old_state == 2) {
        if(!b){
            count++;
            new_state = 0;
        }
        else if (a){
            count--;
            new_state = 3;
        }

	    // Handle A and B inputs for state 2

	}
	else {   // old_state = 3

	    // Handle A and B inputs for state 3
        if(!a){
            count++;
            new_state = 2;
        }
        else if (!b){
            count--;
            new_state = 1;
        }
	}
	// If state changed, update the value of old_state,
	// and set a flag that the state has changed.
	if (new_state != old_state) {
	    changed = 1;
	    old_state = new_state;
	}

    //set threshold limits
    if(count>90) count = 90;
    else if (count < 50) count = 50;
    // store in eeprom
    eeprom_update_byte((void *)100, count);

}
