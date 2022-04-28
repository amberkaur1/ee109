#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>

#include "buzzer.h"
#include "lcd.h"
#include "ds18b20.h"
#include "servo.h"
#include "encoder.h"
#include "project.h"
#include "serial.h"
#include "led.h"

volatile unsigned char led_state = 0, prev_led_state = 0;
volatile int rmt_temp;
volatile unsigned char remote = 0;
volatile unsigned char prev_remote = 0;

int main()
{
  unsigned char t[2];

  // initialize all modules
  lcd_init();
  timer2_init();
  ds_init();
  // Start first temperature conversion
  encoder_init();
  led_init();
  buzzer_init();
  serial_init();
  buttons_init();
  ds_convert();

  // splash screen
  lcd_writecommand(1);
  lcd_stringout("Amber garcha");
  _delay_ms(1000);
  lcd_writecommand(1);

  sei();

  // get initial temp reading
  int f = 0;
  if (ds_temp(t))
  { // True if conversion complete
    /*
      Process the values returned in t[0]
      and t[1] to find the temperature.
    */
    f = temp_to_f(t);
    serial_tempout(f);

    ds_convert(); // Start next conversion
  }
  int prev_temp = f;
  char buf_init[20];

  // set initial screen
  snprintf(buf_init, 20, ">Tmp: 73.4    %d", threshold);
  lcd_moveto(0, 0);
  lcd_stringout(buf_init);
  lcd_moveto(1, 0);
  snprintf(buf_init, 20, " Rmt: 0");
  lcd_stringout(buf_init);
  while (1)
  {

    if (ds_temp(t))
    { // True if conversion complete
      /*
        Process the values returned in t[0]
        and t[1] to find the temperature.
      */
      // do this to get around the weird short that happens every so often in the breadboard
      if ((temp_to_f(t) < 1000 && temp_to_f(t) > 300))
        f = temp_to_f(t);

      ds_convert(); // Start next conversion
    }

    // update lcd if temp changed, and send new temp to remote, and update servo position
    if (f != prev_temp)
    {
      char tenths = f % 10;
      char farenheight = f / 10;

      char buf[10];
      // update lcd
      snprintf(buf, 10, "Tmp: %d.%d", farenheight, tenths);
      lcd_moveto(0, 1);
      lcd_stringout(buf);
      serial_tempout(farenheight);
      // if local mode, update servo
      if (!remote)
        OCR2A = f_to_time(f);

      prev_temp = f;
    }
    if (new_rmt_temp) // if new remote temp recieved
    {
      new_rmt_temp = 0; // acknowledge remote temp recieved
      char *str = next_rmt_temp;
      int test_rmt_temp;
      sscanf(str, "%d", &test_rmt_temp);             // convert remote temp string message to int
      if (test_rmt_temp < 100 && test_rmt_temp > 30) // if new remote temp value isn't garbage, update remote temp
      {
        // update lcd
        rmt_temp = test_rmt_temp;
        lcd_moveto(1, 6);
        char buf[10];

        snprintf(buf, 10, "%d", rmt_temp);
        lcd_stringout(buf);
        new_rmt_temp = 0;
        // if remote mode, update servo
        if (remote)
          OCR2A = f_to_time(f);
      }
    }
    if (changed) // if new threshold, update lcd
    {
      changed = 0;
      lcd_moveto(0, 14);
      char buf[3];
      snprintf(buf, 3, "%d", threshold);
      lcd_stringout(buf);
    }
    if (remote != prev_remote)
    {
      prev_remote = remote;
      if (remote)
      {
        // update lcd to show mode switch from local to remote
        lcd_moveto(0, 0);
        lcd_stringout(" ");
        lcd_moveto(1, 0);
        lcd_stringout(">");
      }
      else
      {
        // update lcd to show mode switch from remote to local
        lcd_moveto(0, 0);
        lcd_stringout(">");
        lcd_moveto(1, 0);
        lcd_stringout(" ");
        OCR2A = f_to_time(f); // set servo to remote temp
      }
    }
    // set test_temp used for led states, to remote temp or local temp
    int test_temp = f;
    if (remote)
    {
      test_temp = rmt_temp * 10;
      OCR2A = f_to_time(test_temp); // set servo to remote temp
    }
    // threshold states
    if (test_temp <= threshold * 10) // cool
    {
      led_state = 0;
    }
    else if (test_temp - 30 <= threshold * 10 && test_temp > threshold * 10) // warm
    {
      led_state = 1;
    }
    else // hot
    {
      led_state = 2;
    }

    if (prev_led_state != led_state)
    {
      // update lcd and leds for the various states
      lcd_moveto(1, 12);
      if (led_state == 0)
      {
        PORTB |= (1 << PB4);
        PORTB &= ~(1 << PB5);
        TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10));
        lcd_stringout("Cool");
      }
      else if (led_state == 1)
      {
        PORTB &= ~(1 << PB4);
        PORTB |= (1 << PB5);
        blink_count = 0;
        TCCR1B |= (1 << CS11) | (1 << CS10);
        lcd_stringout("Warm");
      }
      else
      {
        PORTB &= ~(1 << PB4);
        PORTB |= (1 << PB5);
        TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10));
        lcd_stringout(" Hot");
        timer_on = 1000;
      }
      prev_led_state = led_state;
    }
  }
  return 0;
}
