#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include "project.h"

volatile char started = 0;
volatile char valid =0;
volatile char rembuf[5];
volatile int buf_index = 0;
//clock frequency
#define FOSC 16000000
//Baud rate used
#define BAUD 9600
// Value for UBRR0
#define MYUBRR (FOSC/16/BAUD-1)


void serial_init(void)
{
    UCSR0B |= (1 << RXCIE0); // enable the receiver interrupt
    DDRC |= (1 << PC4); // tri state buffer output
    UBRR0 = MYUBRR; //set BUAD rate
    UCSR0B |= (1 << RXEN0);
    UCSR0B |= (1 << TXEN0); //enable RX and TX
    UCSR0C = (3 << UCSZ00); //1 stop bit, 8 data bits
}

ISR(USART_RX_vect) //interrupt for received data
{
    char c = UDR0; //get and store received character

    if(c=='@') //start character
    {
        started = 1;
        valid = 0;
    }
    else if(c == '#') // handle end of message
    {
        if(buf_index>0)
            valid = 1;
        started = 0; //ended message
        buf_index = 0;
    }

    else if(c == '+' || c=='-'){
        if(started == 1 && buf_index != 0){
            started = 0;
            buf_index = 0;
            valid = 0;
        }
        else if(started && buf_index ==0 && c=='-'){ //add only if negative
            rembuf[buf_index] = c;
            buf_index++;
        }
    }

    else if(started == 1)
    {
        rembuf[buf_index] = c; //store character in buffer
        buf_index++;
    }
    else if(buf_index >=5 ){
        started = 0;
        valid = 0;
        buf_index = 0;
    }
}

void serial_tempout(char temp)
{
    //generate string out
    char rembuf[6];
    if (temp > 0)
        snprintf(rembuf, 6, "@+%d#", temp);
    else
        snprintf(rembuf, 6, "@-%d#", temp);

    //start transmitting data
    int i = 0;
    while (rembuf[i] != '\0') //end of string
    {
        while ((UCSR0A & (1 << UDRE0)) == 0) { } //wait for characters to be transmitted
        UDR0 = rembuf[i]; //next char
        i++;
    }
    while ((UCSR0A & (1 << UDRE0)) == 0) { } //wait for characters to be transmitted
    UDR0 = '\0'; //end char

}
