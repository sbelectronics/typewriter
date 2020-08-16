/* 
 * typewriter.ino
 *
 * Typewriter.
 *
 * (c) Scott Baker, https://www.smbaker.com/
 */

// Some of the PS2 code originally based on keyboard.c
//   for NerdKits with ATmega168
//   hevans@nerdkits.com


#include <Arduino.h>

#include "scancode_map.h"
#include "lpt.h"


//PIN configuration
//PD2 (PCINT18) -  PS2 Keyboard CLK
//PD3           -  PS2 Keyboard DATA

#define PS2_INT PCINT18
#define PS2_CLK (1<<PD2)
#define PS2_DATA (1<<PD3)

#define LED0 (1<<PD0)
#define LED1 (1<<PD1)

#define PS2_CAPSLOCK_SCANCODE 0x58
#define PS2_NUMLOCK_SCANCODE 0x77
#define PS2_PAUSE_SCANCODE 0x77
#define PS2_BACKSLASH_SCANCODE 0x5D
#define PS2_LSHIFT_SCANCODE 0x12
#define PS2_RSHIFT_SCANCODE 0x59
#define PS2_BACKQUOTE_SCANCODE 0x0E

#define CAPSLOCK_LEDBIT 0x04
#define NUMLOCK_LEDBIT 0x02

#define CAPSLOCK_SPECIAL 213
#define F1_SPECIAL 201
#define F2_SPECIAL 202
#define F3_SPECIAL 203
#define F4_SPECIAL 204
#define F5_SPECIAL 205
#define F6_SPECIAL 206
#define F7_SPECIAL 207
#define F8_SPECIAL 208
#define F9_SPECIAL 209
#define F10_SPECIAL 210
#define F11_SPECIAL 211
#define F12_SPECIAL 212

#define SCANCODE_NONE 0xFF

#define CR_SEND_LF

volatile uint8_t kbd_data;
volatile uint8_t kbd_event, kbd_down, kbd_up;
volatile uint8_t ack_event, nak_event;
volatile uint8_t next_extended, e1flag;
volatile uint8_t key_state[32];
volatile uint8_t disable_isr;
volatile uint8_t lshift_down, rshift_down;
uint8_t ps2_code;
uint8_t started;
uint8_t bit_count;
uint8_t leds;
uint8_t extended;
uint8_t release;
uint8_t capslock;
uint8_t numlock;
uint8_t ps2_argument;
uint8_t ps2_have_argument;
uint8_t last_reset;
uint8_t typewriter_map_code(uint8_t data, uint8_t extended, uint8_t shift_state, uint8_t capslock_state);

ISR(PCINT2_vect)
{
  //make sure clock line is low, if not ignore this transition
  if(PIND & PS2_CLK) {
    return;
  }

  if (disable_isr) {
    return;
  }

  //if we have not started, check for start bit on DATA line
  if(!started){
    if ( (PIND & PS2_DATA) == 0 ) {
      started = 1;
      bit_count = 0;
      kbd_data = 0;
      //printf_P(PSTR("%d"),started);
      return;
    }
  } else if(bit_count < 8) { //we started, read in the new bit
    //put a 1 in the right place of kdb_data if PC2 is high, leave
    //a 0 otherwise
    if (PIND & PS2_DATA) {
      kbd_data |= (1<<bit_count);
    }
    bit_count++;
    return;
  } else if(bit_count == 8){ //pairty bit
    //not implemented
    bit_count++;
    return;
  } else {  //stop bit
    //should check to make sure DATA line is high, what to do if not?
    started = 0;
    bit_count = 0;
  }

  if(kbd_data == 0xF0){ //release code
    release = 1;
    kbd_data = 0;
    return;
  } else if (kbd_data == 0xFA) {
    ack_event = 1;
    return;
  } else if (kbd_data == 0xFE) {
    nak_event = 1;
    return;
  } else if (kbd_data == 0xE0) { //extended code
    next_extended = 1;
    return;
  } else if (kbd_data == 0xE1) { //the other extended code (pause/break)
    e1flag = 1;
    return;
  } else { //not a special character
    if ((e1flag) && (kbd_data!=0x77) && (kbd_data!=0x14)) {
      // E1,14,77,E1,F0,14,F0,77 is the pause sequence.
      // If we see something else, reset the flag.
      e1flag = 0;
    }
    if (release) { //we were in release mode - exit release mode
      if (kbd_data == PS2_LSHIFT_SCANCODE) {
          lshift_down = 0;
      }
      if (kbd_data == PS2_RSHIFT_SCANCODE) {
          rshift_down = 0;
      }
      release = 0;
      ps2_code = typewriter_map_code(kbd_data, next_extended, lshift_down || rshift_down, capslock);
      next_extended = 0;
      if (ps2_code != SCANCODE_NONE) {
          uint8_t mask = 1<<(ps2_code & 0x07);
          key_state[ps2_code >> 3] &= ~mask;
          kbd_up = ps2_code;
          kbd_down = SCANCODE_NONE;
          kbd_event = 1;
      }
    } else {
      if (kbd_data == PS2_LSHIFT_SCANCODE) {
          lshift_down = 1;
      }
      if (kbd_data == PS2_RSHIFT_SCANCODE) {
          rshift_down = 1;
      }
      ps2_code = typewriter_map_code(kbd_data, next_extended, lshift_down || rshift_down, capslock);
      next_extended = 0;
      if (ps2_code != SCANCODE_NONE) {
          uint8_t mask = 1<<(ps2_code & 0x07);
          key_state[ps2_code >> 3] |= mask;
          kbd_down = ps2_code;
          kbd_up = SCANCODE_NONE;
          kbd_event = 1;
      }
    }
  }
}

void led0_off()
{
  PORTD |= LED0;
}

void led0_on()
{
  PORTD &= (~LED0);
}

void led1_off()
{
  PORTD |= LED1;
}

void led1_on()
{
  PORTD &= (~LED1);
}

void led_init()
{
  // make pc5 an output
  DDRD |= LED0 | LED1;

  led0_off();
  led1_off();
}

void init_keyboard()
{
  uint8_t i;

  for (i=0; i<32; i++) {
    key_state[i] = 0;
  }

  started = 0;
  kbd_data = 0;
  bit_count = 0;
  next_extended = 0;
  e1flag = 0;
  kbd_event = 0;
  kbd_down = SCANCODE_NONE;
  kbd_up = SCANCODE_NONE;
  disable_isr = 0;
  lshift_down = 0;
  rshift_down = 0;
  ps2_have_argument = 0;
  ack_event = 0;
  nak_event = 0;
  capslock = 0;
  leds = 0;

  //make PS2_CLK input pin
  //DDRC &= ~PS2_CLK;
  //turn on pullup resistor
  PORTD |= PS2_CLK;
  
  //Set the mask on Pin change interrupt 1 so that ISR for PS2_CLK fires
  //the interrupt.
  PCMSK2 |= (1<<PS2_INT);
}

void init_isr()
{
  // Clear pin change interrupt flag.
  //PCIFR |= (1<<PCIF2);

  //Enable PIN Change Interrupt 1 - This enables interrupts on pins
  PCICR |= (1<<PCIE2);
}

uint8_t wait_for_bit(uint8_t theBit, uint8_t highlow, uint16_t ms)
{
    if (highlow != LOW) {
      highlow = theBit;
    }

    uint16_t i;
    for (i=0; i<ms*1000; i++) {
      if ((PIND & theBit) == highlow) {
          return 1;
      }
      // note that delayMicroseconds() may not be accurate below 3
      delayMicroseconds(1);
    }
    // timeout
    return 0;
}

uint8_t compute_parity(uint16_t n) 
{ 
    uint8_t parity = 0; 
    while (n) 
    { 
        parity = !parity; 
        n      = n & (n - 1); 
    }         
    return parity; 
} 

void ps2_clk_low()
{
  PORTD &= ~PS2_CLK;
  DDRD |= PS2_CLK;
}

void ps2_clk_release()
{
  // open-collector output; to set high we just release
  DDRD &= ~PS2_CLK;
}

void ps2_data_low()
{
  PORTD &= ~PS2_DATA;
  DDRD |= PS2_DATA;
}

void ps2_data_release()
{
  // open-collector output; to set high we just release
  DDRD &= ~PS2_DATA;
}

// for making noticeable debug events
void blip()
{
  led1_on();
  delayMicroseconds(5);
  led1_off();
  delayMicroseconds(5);
  led1_on();
  delayMicroseconds(5);
}

uint8_t ps2_send_byte(uint16_t b)
{
    int i;

    led1_on();

    wait_for_bit(PS2_CLK, HIGH, 2);

    // bring clock low
    ps2_clk_low();
    // wait 100us
    delayMicroseconds(300);
    // bring data low
    ps2_data_low();
    // let data settle before releasing clock
    delayMicroseconds(1);
    // return clock to input state
    ps2_clk_release();

    // let clock settle before testing it
    // (without this the wait_for_bit exits immediately on
    // the atmega328... it worked fine on the tiny85!)
    delayMicroseconds(3);

    // wait for keyboard to bring clock low, up to 15ms
    if (!wait_for_bit(PS2_CLK, LOW, 15)) {
      ps2_data_release();
      led1_off();
      return 0;
    }

    if (!compute_parity(b)) {
      b |= 0x100;
    }

    // 8 data bits + one parity bit, assume the parity is in bit position 8
    for (i=0; i<9; i++) {
      // set the data bit
      if ((b&1)==1) {
        ps2_data_release();
      } else {
        ps2_data_low();
      }
      // wait for clock to go high
      if (!wait_for_bit(PS2_CLK, HIGH, 2)) {
        ps2_data_release();
        return 0;
      }
      if (!wait_for_bit(PS2_CLK, LOW, 2)) {
        ps2_data_release();
        led1_off();
        return 0;
      }
      b = b >> 1;
    }
    
    // return data to inptu state
    ps2_data_release();

    // wait for the ack
    if (!wait_for_bit(PS2_DATA, LOW, 2)) {
      led1_off();
      return 0;
    }
    if (!wait_for_bit(PS2_CLK, LOW, 2)) {
      led1_off();
      return 0;
    }

    led1_off();
    return 1;
}

void ps2_send_command_argument(uint8_t command, uint8_t argument)
{
  disable_isr = 1;
  if (ps2_send_byte(command)) {
      ps2_argument = argument;
      ps2_have_argument = 1;
  }
  disable_isr = 0;
}

void ps2_send_argument(uint8_t arg)
{
  disable_isr = 1;
  ps2_send_byte(ps2_argument);
  disable_isr = 0;
}


uint8_t typewriter_map_code(uint8_t data, uint8_t extended, uint8_t shift_state, uint8_t capslock_state)
{
    uint8_t scancode;

    if (extended!=0) {
        if (shift_state) {
            scancode = pgm_read_byte(&(shifted_extcode_map[data]));
        } else {
            scancode = pgm_read_byte(&(extcode_map[data]));
        }
    } else {
        if (capslock_state) {
            if (shift_state) {
                scancode = pgm_read_byte(&(caps_shifted_scancode_map[data]));
            } else {
                scancode = pgm_read_byte(&(caps_scancode_map[data]));  
            }
        } else {
            if (shift_state) {
                scancode = pgm_read_byte(&(shifted_scancode_map[data]));
            } else {
                scancode = pgm_read_byte(&(scancode_map[data]));  
            }
        }
    }

    return scancode;
}

void setup() {
  init_isr();

  init_keyboard();

  led_init();

  lpt_init();

  //enable interrupts
  sei();
}

void update_leds()
{
  ps2_send_command_argument(0xED, leds);
}

void loop() {
    if (kbd_event) {
        if (kbd_down != SCANCODE_NONE) {
          led0_on();
          switch (kbd_down) {
              case CAPSLOCK_SPECIAL:
                  capslock = !capslock;
                  if (capslock) {
                      leds = leds | CAPSLOCK_LEDBIT;
                      update_leds();
                  } else {
                      leds = leds & (~CAPSLOCK_LEDBIT);
                      update_leds();
                  }
                  break;
              case F1_SPECIAL:
                  lpt_set_power(0);
                  break;
              case F2_SPECIAL:
                  lpt_set_power(1);
                  break;
              case F3_SPECIAL:
                  lpt_set_power(2);
                  break;
              case F4_SPECIAL:
                  lpt_set_power(3);
                  break;
              case F5_SPECIAL:
                  lpt_set_bold();
                  break;
              case F6_SPECIAL:
                  lpt_set_shadow();
                  break;
              case F7_SPECIAL:
                  lpt_set_underscore();
                  break;
              case F10_SPECIAL:
                  lpt_reset_shadow_bold();
                  lpt_reset_underscore();
                  break;
              default:
                  if (kbd_down<128) {
                      lpt_write(kbd_down);
                  }
                  break;
          }
#ifdef CR_SEND_LF
          if (kbd_down == 0x0D) {
            lpt_write(0x0A);          
          }
#endif
          led0_off();
        }
        kbd_event = 0;
    }
    if (ack_event != 0) {
      if (ps2_have_argument) {
        ps2_send_argument(ps2_argument);
        ps2_have_argument = 0;
      }
      ack_event = 0;
    }
    if (nak_event != 0) {
        ps2_have_argument = 0;
        nak_event = 0;
    }
}
