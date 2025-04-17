#include "display_utils.h"
#include <stdbool.h>

extern UART_HandleTypeDef huart1;  // Use the UART1 handle from main.c
extern bool uart_dma_busy;

void two_ascii_bytes(uint8_t value, uint8_t *ascii_bytes) {
    if (value > 99) {
        value = 99;
    }
    ascii_bytes[0] = (value / 10) + '0';  // Get the tens digit and convert to ASCII
    ascii_bytes[1] = (value % 10) + '0';  // Get the ones digit and convert to ASCII
}

void three_ascii_bytes(int16_t value, uint8_t *ascii_bytes) {
    // Check if value is negative
    if (value < 0) {
        // Clamp negative values to -99
        if (value < -99) {
            value = -99;
        }

        // Format as "-XX"
        ascii_bytes[0] = '-';
        ascii_bytes[1] = ((-value / 10) % 10) + '0'; // Tens digit
        ascii_bytes[2] = (-value % 10) + '0';        // Ones digit
    } else {
        // For positive numbers, clamp to 999 and format as "XXX"
        if (value > 999) {
            value = 999;
        }

        ascii_bytes[0] = (value / 100) + '0';       // Hundreds digit
        ascii_bytes[1] = ((value / 10) % 10) + '0'; // Tens digit
        ascii_bytes[2] = (value % 10) + '0';        // Ones digit
    }
}

void write_three(uint16_t vp, int16_t value) {
    uint8_t ascii_bytes[3];
    three_ascii_bytes(value, ascii_bytes);
    dwin_write(vp, ascii_bytes, sizeof(ascii_bytes));
}

void dwin_write(uint16_t vp, uint8_t *data, uint8_t data_len) {
    uint8_t packet[6 + data_len];

    packet[0] = 0x5A;
    packet[1] = 0xA5;
    packet[2] = 3 + data_len;
    packet[3] = 0x82;
    packet[4] = (vp >> 8) & 0xFF;
    packet[5] = vp & 0xFF;

    memcpy(&packet[6], data, data_len);

    while (uart_dma_busy);
    uart_dma_busy = true;
    HAL_UART_Transmit_DMA(&huart1, packet, sizeof(packet));
    HAL_Delay(1);
}

void set_state(uint16_t vp, uint16_t state) {
    uint8_t hex_data[2];

    // Split the 16-bit state into two bytes (big-endian format)
    hex_data[0] = (state >> 8) & 0xFF;  // High Byte
    hex_data[1] = state & 0xFF;         // Low Byte

    dwin_write(vp, hex_data, sizeof(hex_data));
}


void set_page(uint8_t page) {
    uint8_t hex_data[] = {0x5A, 0x01, 0x00, page};
    dwin_write(0x0084, hex_data, sizeof(hex_data));
}

void write_two(uint16_t vp, uint8_t value) {
	uint8_t ascii_bytes[2];
	two_ascii_bytes(value, ascii_bytes);
	dwin_write(vp, ascii_bytes, sizeof(ascii_bytes));
}


void set_voltage(float voltage) {
    /* Clip voltage to valid range */
    if (voltage > 999.9f) voltage = 999.9f;
    if (voltage < 0.0f) voltage = 0.0f;

    /* Convert to integer with single decimal place */
    uint16_t volt_int = (uint16_t)(voltage * 10.0f + 0.5f); // +0.5f for proper rounding

    /* Extract digits */
    uint8_t hundreds = volt_int / 1000;
    uint8_t tens = (volt_int / 100) % 10;
    uint8_t ones = (volt_int / 10) % 10;
    uint8_t tenths = volt_int % 10;

    /* Convert to ASCII */
    uint8_t ascii_bytes[5];
    ascii_bytes[0] = '0' + hundreds;
    ascii_bytes[1] = '0' + tens;
    ascii_bytes[2] = '0' + ones;
    ascii_bytes[3] = '.';
    ascii_bytes[4] = '0' + tenths;

    dwin_write(0x2040, ascii_bytes, sizeof(ascii_bytes));
}

void set_current(float current) {
    /* Track if current is negative */
    bool is_negative = (current < 0.0f);

    /* Work with absolute value */
    current = fabsf(current);

    /* Clip current to valid range */
    if (current > 99.99f) current = 99.99f;

    /* Convert to integer with two decimal places */
    uint16_t curr_int = (uint16_t)(current * 100.0f + 0.5f); // +0.5f for proper rounding

    /* Extract digits */
    uint8_t tens = curr_int / 1000;
    uint8_t ones = (curr_int / 100) % 10;
    uint8_t tenths = (curr_int / 10) % 10;
    uint8_t hundredths = curr_int % 10;

    /* Convert to ASCII */
    uint8_t ascii_bytes[5];

    if (is_negative) {
        /* For negative values, use minus sign in first position */
        ascii_bytes[0] = '-';
        /* Only show one digit before decimal when negative */
        ascii_bytes[1] = '0' + ones;
    } else {
        /* For positive values, show two digits as before */
        ascii_bytes[0] = '0' + tens;
        ascii_bytes[1] = '0' + ones;
    }

    ascii_bytes[2] = '.';
    ascii_bytes[3] = '0' + tenths;
    ascii_bytes[4] = '0' + hundredths;

    /* Write to display at current value address */
    dwin_write(0x2030, ascii_bytes, sizeof(ascii_bytes));
}

/* Configurable current range parameters */
#define MIN_CURRENT -300.0f  // Current that maps to maximum bar value (200)
#define MAX_CURRENT 100.0f   // Current that maps to minimum bar value (0)
#define ZERO_POINT 0.0f      // Current that maps to middle bar value (100)

/**
 * Maps current to bar display value:
 * - MIN_CURRENT maps to 200
 * - ZERO_POINT maps to 100
 * - MAX_CURRENT maps to 0
 */
void set_current_bar(float current) {
    uint8_t bar_value;

    /* Implement piecewise linear mapping */
    if (current <= MIN_CURRENT) {
        /* Below minimum threshold, max bar value */
        bar_value = 200;
    }
    else if (current < ZERO_POINT) {
        /* Negative range: Map [MIN_CURRENT, ZERO_POINT] to [200, 100] */
        float range = ZERO_POINT - MIN_CURRENT;
        float position = ZERO_POINT - current;
        bar_value = 100 + (uint8_t)(100.0f * position / range);
    }
    else if (current <= MAX_CURRENT) {
        /* Positive range: Map [ZERO_POINT, MAX_CURRENT] to [100, 0] */
        float range = MAX_CURRENT - ZERO_POINT;
        float position = current - ZERO_POINT;
        bar_value = 100 - (uint8_t)(100.0f * position / range);
    }
    else {
        /* Above maximum threshold, min bar value */
        bar_value = 0;
    }

    /* Ensure we stay within valid range 0-200 */
    if (bar_value > 200) bar_value = 200;

    set_state(0x1600, bar_value);
    write_two(0x1590, abs(bar_value-100));
}


void set_charge_kw(float charge_kw) {
    /* Clip charge_kw to valid range */
    if (charge_kw > 9.999f) charge_kw = 9.999f;
    if (charge_kw < 0.0f) charge_kw = 0.0f;

    /* Convert to integer with three decimal places */
    uint16_t kw_int = (uint16_t)(charge_kw * 1000.0f + 0.5f); // +0.5f for proper rounding

    /* Extract digits */
    uint8_t ones = kw_int / 1000;
    uint8_t tenths = (kw_int / 100) % 10;
    uint8_t hundredths = (kw_int / 10) % 10;
    uint8_t thousandths = kw_int % 10;

    /* Convert to ASCII */
    uint8_t ascii_bytes[5];
    ascii_bytes[0] = '0' + ones;
    ascii_bytes[1] = '.';
    ascii_bytes[2] = '0' + tenths;
    ascii_bytes[3] = '0' + hundredths;
    ascii_bytes[4] = '0' + thousandths;

    /* Write to display at charge power address */
    dwin_write(0x2020, ascii_bytes, sizeof(ascii_bytes));
}

void set_battery(uint8_t percentage) {
	uint16_t sp_percentage = 0x7500;
	uint16_t vp_percentage = 0x1500;
	uint16_t vp_battery = 0x1770;
	uint16_t sp_charge_percentage = 0x8000;
	uint16_t sp_charge_percentage_stop = 0x8050;
	uint16_t sp_charge_percentage_decimal = 0x8010;
	uint16_t vp_charge_percentage = 0x2000;
	uint16_t vp_charge_battery = 0x2060;

	// write percentage to text
	write_two(vp_percentage, percentage);
	write_two(vp_charge_percentage, percentage);

	// change text and icon color
	// green
	if (percentage >= 75) {
		uint8_t colour[] = {0x11, 0x22};
		dwin_write(sp_percentage + 0x03, colour, sizeof(colour));
		dwin_write(sp_charge_percentage + 0x03, colour, sizeof(colour));
		dwin_write(sp_charge_percentage_stop + 0x03, colour, sizeof(colour));
		dwin_write(sp_charge_percentage_decimal + 0x03, colour, sizeof(colour));
		set_state(vp_battery, 0);
		set_state(vp_charge_battery, 0);
	}
	// blue
	else if (percentage >= 35) {
		uint8_t colour[] = {0x08, 0xC3};
		dwin_write(sp_percentage + 0x03, colour, sizeof(colour));
		dwin_write(sp_charge_percentage + 0x03, colour, sizeof(colour));
		dwin_write(sp_charge_percentage_stop + 0x03, colour, sizeof(colour));
		dwin_write(sp_charge_percentage_decimal + 0x03, colour, sizeof(colour));
		set_state(vp_battery, 1);
		set_state(vp_charge_battery, 1);
	}
	// orange
	else if (percentage >= 20) {
		uint8_t colour[] = {0x28, 0xA1};
		dwin_write(sp_percentage + 0x03, colour, sizeof(colour));
		dwin_write(sp_charge_percentage + 0x03, colour, sizeof(colour));
		dwin_write(sp_charge_percentage_stop + 0x03, colour, sizeof(colour));
		dwin_write(sp_charge_percentage_decimal + 0x03, colour, sizeof(colour));
		set_state(vp_battery, 2);
		set_state(vp_charge_battery, 2);
	}
	// red
	else {
		uint8_t colour[] = {0x28, 0xA1};
		dwin_write(sp_percentage + 0x03, colour, sizeof(colour));
		dwin_write(sp_charge_percentage + 0x03, colour, sizeof(colour));
		dwin_write(sp_charge_percentage_stop + 0x03, colour, sizeof(colour));
		dwin_write(sp_charge_percentage_decimal + 0x03, colour, sizeof(colour));
		set_state(vp_battery, 3);
		set_state(vp_charge_battery, 3);
	}
}
