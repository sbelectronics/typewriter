#ifndef _LPT_H
#define _LPT_H

void lpt_init();
void lpt_write(uint8_t x);

void lpt_set_power(uint8_t x);
void lpt_set_bold();
void lpt_set_shadow();
void lpt_reset_shadow_bold();
void lpt_set_underscore();
void lpt_reset_underscore();

#endif
