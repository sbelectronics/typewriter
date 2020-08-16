#include <Arduino.h>
#include <util/delay.h>
#include "lpt.h"

#define LPT_ACK (1<<PC0)
#define LPT_BUSY (1<<PC1)
#define LPT_PE (1<<PC2)
#define LPT_SELECT (1<<PC3)
#define LPT_FAULT (1<<PC4)
#define LPT_STROBE (1<<PD4)
#define LPT_AUTOF (1<<PD5)
#define LPT_INIT (1<<PD6)
#define LPT_SELIN (1<<PD7)

#define LPT_ACK_INT PCINT8

#define ASM_NOP() asm volatile ("nop" :: )

volatile uint8_t ack_flag, ack_value;

ISR(PCINT1_vect)
{
    // Note: This will catch both High->Low and Low->High
    // transistions. Either will suffice to let us know the
    // ack fired.
    
    // read the ack gpio, to reset the PCINT
    ack_value = (PORTC & LPT_ACK);
    ack_flag=1;
}

void lpt_set_data(uint8_t x)
{
  PORTB = x;
}

void lpt_set_strobe(uint8_t x)
{
  if (x) {
      PORTD |= LPT_STROBE;
  } else {
      PORTD &= (~LPT_STROBE);
  }
}

void lpt_set_autof(uint8_t x)
{
  if (x) {
      PORTD |= LPT_AUTOF;
  } else {
      PORTD &= (~LPT_AUTOF);
  }
}

void lpt_set_init(uint8_t x)
{
  if (x) {
      PORTD |= LPT_INIT;
  } else {
      PORTD &= (~LPT_INIT);
  }
}

void lpt_set_selin(uint8_t x)
{
  if (x) {
      PORTD |= LPT_SELIN;
  } else {
      PORTD &= (~LPT_SELIN);
  }
}

// Office Master 2000, hammer power 0 = lightest, 1-2=medium, 3 = strong
void lpt_set_power(uint8_t x)
{
  lpt_write(27);
  lpt_write(18);
  // manual doesn't make this clear whether it's "0" or 0. Neither
  // one seems to have a noticeable effect.
  /*
  switch (x) {
      case 0: lpt_write(48); break;
      case 1: lpt_write(49); break;
      case 2: lpt_write(50); break;
      case 3: lpt_write(51); break;
  }
  */
  lpt_write(x);
}

void lpt_set_bold()
{
  lpt_write(27);
  lpt_write(79);
}

void lpt_set_shadow()
{
  lpt_write(27);
  lpt_write(87);
}

void lpt_reset_shadow_bold()
{
  lpt_write(27);
  lpt_write(38);
}

void lpt_set_underscore()
{
  lpt_write(27);
  lpt_write(69);
}

void lpt_reset_underscore()
{
  lpt_write(27);
  lpt_write(82);
}

void lpt_init()
{
   lpt_set_selin(0); // printer only receives when selin is low
   lpt_set_autof(1); 
   lpt_set_strobe(1); // data is strobed on high-to-low transition

   // reset the printer
   lpt_set_init(0);
   delayMicroseconds(50);
   lpt_set_init(1);

   lpt_set_data(0);

   DDRB = 0xFF;  // DB0..DB7
   DDRD |= LPT_STROBE | LPT_AUTOF | LPT_INIT | LPT_SELIN;

   // pullups
   PORTC |= LPT_BUSY | LPT_ACK;

   // setup ISR to catch the acks
   PCMSK1 |= (1<<LPT_ACK_INT);
   PCICR |= (1<<PCIF1);
}

void lpt_write(uint8_t x)
{
  // busy-wait while the printer is busy
  while ((PINC & LPT_BUSY) != 0) ;

  ack_flag = 0; // make sure we catch the ack
  lpt_set_data(x);
  ASM_NOP(); // should be 133ns
  lpt_set_strobe(0);
  ASM_NOP(); // should be 133ns
  lpt_set_strobe(1);

  // wait for the ack ISR to catch the ack
  while (!ack_flag) ;

  // busy wait for the printer to ack the data
  //while ((PINC & LPT_ACK) != 0) ;
}
