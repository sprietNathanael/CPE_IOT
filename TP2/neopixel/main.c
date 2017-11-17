/****************************************************************************
 *   apps/rf_sub1G/rgb_led/main.c
 *
 * sub1G_module support code - USB version
 *
 * Copyright 2013-2014 Nathael Pajani <nathael.pajani@ed3l.fr>
 *
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *************************************************************************** */


#include "core/system.h"
#include "core/systick.h"
#include "core/pio.h"
#include "lib/stdio.h"
#include "drivers/serial.h"
#include "drivers/gpio.h"
#include "drivers/ssp.h"
#include "drivers/i2c.h"
#include "drivers/adc.h"
#include "drivers/timers.h"
#include "extdrv/cc1101.h"
#include "extdrv/status_led.h"
#include "extdrv/tmp101_temp_sensor.h"
#include "extdrv/ws2812.h"


#define MODULE_VERSION	0x02
#define MODULE_NAME "RF Sub1G - USB"


#define RF_868MHz  1
#define RF_915MHz  0
#if ((RF_868MHz) + (RF_915MHz) != 1)
#error Either RF_868MHz or RF_915MHz MUST be defined.
#endif


#define DEBUG 1
#define BUFF_LEN 60
#define TEXT_LENGTH_MAX_LCD_ONE 20
#define SELECTED_FREQ  FREQ_SEL_48MHz

void recv_text(uint8_t c);

/***************************************************************************** */
/* Pins configuration */
/* pins blocks are passed to set_pins() for pins configuration.
 * Unused pin blocks can be removed safely with the corresponding set_pins() call
 * All pins blocks may be safelly merged in a single block for single set_pins() call..
 */
const struct pio_config common_pins[] = {
	/* UART 0 */
	{ LPC_UART0_RX_PIO_0_1,  LPC_IO_DIGITAL },
	{ LPC_UART0_TX_PIO_0_2,  LPC_IO_DIGITAL },
	/* GPIO */
	{ LPC_GPIO_0_31, (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL) },
	ARRAY_LAST_PIO,
};

const struct pio status_led_green = LPC_GPIO_1_4;
const struct pio status_led_red = LPC_GPIO_1_5;
static volatile uint8_t text_buff[TEXT_LENGTH_MAX_LCD_ONE];
const struct pio ws2812_data_out_pin = LPC_GPIO_0_31; /* Led control data pin */
uint8_t rx_buff[TEXT_LENGTH_MAX_LCD_ONE];
int updated = 0;

/***************************************************************************** */
void system_init()
{
	/* Stop the watchdog */
	startup_watchdog_disable(); /* Do it right now, before it gets a chance to break in */
	system_set_default_power_state();
	clock_config(SELECTED_FREQ);
	set_pins(common_pins);
	gpio_on();
	status_led_config(&status_led_green, &status_led_red);
	/* System tick timer MUST be configured and running in order to use the sleeping
	 * functions */
	systick_timer_on(1); /* 1ms */
	systick_start();


}

/* Define our fault handler. This one is not mandatory, the dummy fault handler
 * will be used when it's not overridden here.
 * Note : The default one does a simple infinite loop. If the watchdog is deactivated
 * the system will hang.
 */
void fault_info(const char* name, uint32_t len)
{
	uprintf(UART0, name);
	while (1);
}


/***************************************************************************** */

void recv_text(uint8_t c)
{
	static uint8_t idx = 0;
	
	rx_buff[idx++] = c;
	
	if ((c == '\0') || (c == '\r') || (c == '\n')) {
		updated = 1;
		//rx_buff[(idx - 1)] = ' ';
		memcpy((void*)text_buff, rx_buff, idx);
		idx = 0;
		
	}
	if (idx >= TEXT_LENGTH_MAX_LCD_ONE) {
		idx = 0;
	}
}

void textToColor(uint8_t* text)
{
	int r = 0;
	int b = 0;
	int g = 0;
	r = text[0]*100+text[1]*10+text[2];
	b = text[4]*100+text[5]*10+text[6];
	r = text[8]*100+text[9]*10+text[10];
	ws2812_set_pixel(0, r, g, b);
	ws2812_send_frame(0);
	text[11]='\0';
	uprintf(UART0,"%s\n",text);
	uprintf(UART0,"#%d,#%d,#%d\n",r,g,b);
}


/***************************************************************************** */
int main(void)
{
	system_init();
	uart_on(UART0, 115200, recv_text);

	/* Led strip configuration */
	ws2812_config(&ws2812_data_out_pin);
	ws2812_set_pixel(0, 100,100,100);
	ws2812_send_frame(0);

	while (1) {
		if(updated)
		{
			textToColor(text_buff);
			updated = 0;
		}
	
		msleep(1);
	}
	return 0;
}




