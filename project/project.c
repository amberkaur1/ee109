/********************************************
 *
 *  Name: Amber Garcha
 *  Email: agarcha@usc.edu
 *  Section: Fri 11am
 *  Assignment: PROJECT
 *
 ********************************************/

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>

#include "lcd.h"
#include "ds18b20.h"
#include "serial.h"
#include "project.h"
#include "buzzer.h"
#include "encoder.h"
#define green (1 << PB5)
#define red (1 << PB4)
#define buzzer (1 << PC5)
#define local_but (1 << PC1)
#define remote_but (1 << PC2)

void variable_delay_us(int);
void play_note(unsigned short);
enum states { COOL, WARM, HOT };


volatile int remcount = 12;
volatile char* remote_status = "  LOC";
volatile char tempstate;
volatile char oldtempstate;

volatile unsigned char tempchanged = 0;
volatile unsigned short tone, freq;
volatile char ctemp;
volatile int servocount = 0;
volatile int led_count = 0;
volatile char remote = 0; //check if remote or local
volatile char prev_remote = 0;
volatile char remote_changed = 0;
volatile int remote_temp;


void led_init(void)
{
    //set up the blinking LED timer for WARM
    DDRB |= (1 << PB4) | (1 << PB5);
    TCCR1B |= (1 << WGM12);
    TCCR1B &= ~(1 << WGM13);
    TIMSK1 |= (1 << OCIE1A);
    OCR1A = 25000;
    TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10));
}



int main(void) {
    unsigned char t[2];

    if (ds_init() == 0) {    // Initialize the DS18B20
         // Sensor not responding
    }
    sei();
    timer2_init();
    led_init();
    buzzer_init();
    serial_init();
    DDRB |= (1<<3); //PWM pulse output

    // Initialize DDR and PORT registers and LCD
    PCICR |= (1<<PCIE2);
    PCMSK2 |= (1<<PCINT19);
    PCMSK2 |= (1<<PCINT18);
    DDRC |= (1<<PC5); //buzzer
    PORTC |= (1<<PC1) | (1<<PC2); //pull-up resistors for local/remote buttons
    PORTC |= (1<<PC4); //pull-up resistor for tri-state buffer
    count = eeprom_read_byte((void *)100);
    if(count>90) count = 90;
    else if (count < 50) count = 50;
    // store in eeprom
    eeprom_update_byte((void *)100, count);
    DDRD &= ~((1 << PD2) | (1 << PD3));
    PORTD |= (1 << PD2) | (1 << PD3); //pull-up resistors for rotary encoders

    // Read the A and B inputs to determine the intial state.
    PINDtot = PIND;
    b = PINDtot & (1<<PD2);
    a = PINDtot & (1<<PD3);

    // In the state number, B is the MSB and A is the LSB.
    if (!b && !a)
    old_state = 0;
    else if (!b && a)
    old_state = 1;
    else if (b && !a)
    old_state = 2;
    else
    old_state = 3;

    new_state = old_state;


    ds_convert();    // Start first temperature conversion
    lcd_init();
    // Write a spash screen to the LCD
    lcd_writecommand(1);
    lcd_stringout("EE109 Project");

    char buf[16];

    lcd_moveto(1, 0);
    lcd_stringout("Amber Garcha");
    _delay_ms(1000);
    lcd_writecommand(1);
    unsigned int t1, t2;
    oldtempstate = COOL;
    tempstate = COOL;

    while (1) {                 // Loop forever
	// Read the input bits and determine A and B.
        if((PINC & local_but) == 0){
            remote = 0;
            remote_status = "  LOC";
            if(remote != prev_remote){
                remote_changed = 1;
            }
            prev_remote = remote;
        }
        else if((PINC & remote_but) == 0){
            remote = 1;
            remote_status = "  REM";
            if(remote != prev_remote){
                remote_changed = 1;
            }
            prev_remote = remote;
        }
        if (ds_temp(t)) {
            // lcd_writecommand(1);

            t1 = (t[1] << 8)+t[0];
            t1*=10;
            t1/=16;
            t2 = t1%10;
            t1/=10;
            // lcd_moveto(0, 0);
            // snprintf(buf, 12, "celc: %d.%d", t1, t2);
            // lcd_stringout(buf);

            t1*= 100;
            t2*= 10;
            t1 = ((t1+t2)*9)/5 + 3200;
            servocount = ((t1/10-400)*23/60)/10;
            servocount+= 12;
            t2 = (t1%100 )/10;
            t1 = t1/100;

            // lcd_writecommand(1);
            lcd_moveto(1, 11);
            char temp[5];
            snprintf(temp, 5, "R:%d", remote_temp);
            //write temp
            lcd_moveto(1, 0);
            snprintf(buf, 16, "temp %d.%d", t1, t2);
            lcd_stringout(buf);
            serial_tempout(t1); //send temp to serial

            //output tempstate
            if(tempstate==COOL)
            snprintf(buf, 16, "OK =<%d  %s", count, remote_status);
            else if(tempstate==WARM)
            snprintf(buf, 16, "WARM >%d  %s", count, remote_status);
            else
            snprintf(buf, 16, "HOT >>%d  %s", count, remote_status);
            lcd_moveto(0, 0);
            lcd_stringout(buf);
            ds_convert();
            if(remote == 0)
                OCR2A = servocount; //update servo to local temp
            _delay_ms(100);
        }
        if(valid){ //new remote temp
            valid = 0;
            sscanf(rembuf, "%d", &remote_temp);
            if(!(remote_temp>-10 && remote_temp<110)){ //weird temo
                break;
            }

            if(remote){
                remcount = (((remote_temp*100)/10-400)*23/60)/10;
                remcount+=12;
                lcd_moveto(1, 11);
                char temp[5];
                snprintf(temp, 5, "S:%d", remcount);
                OCR2A = remcount; //set servo to remote temp
                lcd_stringout(temp);
            }
            lcd_moveto(1, 11);
            char temp[5];
            snprintf(temp, 5, "R:%d", remote_temp);
            // snprintf(temp, 5, "%s", rembuf);
            lcd_stringout(temp);
        }

        //update LED state
        if(t1<=count){
            tempstate = COOL;
            PORTC ^= (1 << PC4);
        }
        else if(t1-3<count){
            tempstate = WARM;
        }
        else{
            tempstate = HOT;
        }

        if(oldtempstate != tempstate){ //note that temp state has changed
            tempchanged = 1;
        }
        oldtempstate = tempstate;
        if(tempchanged){
            tempchanged = 0;
        if (tempstate == COOL)
      {
        PORTB |= green; //turn on green
        PORTB &= ~red; //off red
        TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10)); //turn off timer1
      }
      else if (tempstate == WARM)
      {
        PORTB &= ~green;
        PORTB |= red;
        led_count = 0; //start blinking count
        TCCR1B |= (1 << CS11) | (1 << CS10); //turn on blinking timer
      }
      else //HOT
      {
        PORTB &= ~green;
        PORTB |= red;
        TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10)); //off blinker
        buzzer_on = 800; //start buzzer timer
      }
  }

        if (changed) { // Did Threshold change?
	    changed = 0;        // Reset changed flag

	    // Output threshold count to LCD
        // lcd_writecommand(1);
        //write temp
        lcd_moveto(1, 0);
        snprintf(buf, 12, "temp %d.%d", t1, t2);
        lcd_stringout(buf);

        //output tempstate
        if(tempstate==COOL)
        snprintf(buf, 16, "OK =<%d  %s", count, remote_status);
        else if(tempstate==WARM)
        snprintf(buf, 16, "WARM >%d  %s", count, remote_status);
        else
        snprintf(buf, 16, "HOT >>%d  %s", count, remote_status);
        lcd_moveto(0, 0);
        lcd_stringout(buf);

        }

    }
}

ISR(TIMER1_COMPA_vect)  //LeD interrupt
{
    if (tempstate==WARM)
    {
        if (led_count%10==0) //every 5 0.1 seconds
        {
            PORTB ^= red; //toggle LED
        }
        led_count++;
    }
}
