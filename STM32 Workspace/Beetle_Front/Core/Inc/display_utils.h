#ifndef DISPLAY_UTILS_H
#define DISPLAY_UTILS_H

#include "main.h"
#include <stdint.h>
#include <string.h>

void two_ascii_bytes(uint8_t value, uint8_t *ascii_bytes);
void three_ascii_bytes(int16_t value, uint8_t *ascii_bytes);
void dwin_write(uint16_t vp, uint8_t *data, uint8_t data_len);
void set_state(uint16_t vp, uint16_t state);
void set_page(uint8_t page);
void set_battery(uint8_t percentage);
void write_two(uint16_t vp, uint8_t value);
void write_three(uint16_t vp, int16_t value);
void set_voltage(float voltage);
void set_current(float current);
void set_charge_kw(float charge_kw);
void set_current_bar(double current);



#endif
