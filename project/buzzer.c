#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include "project.h"
volatile int buzzer_on = 0;


void buzzer_init(void)
{
    // to set up timer0 and buzzer pin
    TCCR0B |= (1 << WGM12) | (1 << WGM13);
    TIMSK0 |= (1 << OCIE0A);
    OCR0A = 102;
    TCCR0B |= (1 << CS11);
    DDRC |= (1 << PC5);
    TCCR0B = (0b011 << CS10);
}

ISR(TIMER0_COMPA_vect) //buzzer interrupt method
{
    if (buzzer_on != 0) //turn on buzzer
    {
        PORTC ^= (1 << PC5); //toggle buzzer
        buzzer_on--;
    }
    else{ //keep buzzer at 0
        PORTC &= ~(1 << PC5);
    }
}
