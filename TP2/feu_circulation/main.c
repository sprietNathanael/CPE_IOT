/****************************************************************************
 *   apps/base/fau_ciculation/main.c
 *
 * feu circulation examples
 *
 * CopyrighPt 2013-2014 Nathael Pajani <nathael.pajani@ed3l.fr>
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
#include "drivers/i2c.h"
#include "drivers/serial.h"
#include "drivers/gpio.h"
#include "extdrv/status_led.h"


#define MODULE_VERSION    0x04
#define MODULE_NAME "GPIO Demo Module"

#define LED_RED    (1 << led_red.pin)
#define LED_ORANGE  (1 << led_orange.pin)
#define LED_GREEN  (1 << led_green.pin)


#define SELECTED_FREQ  FREQ_SEL_48MHz

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
	/* I2C 0 */
	{ LPC_I2C0_SCL_PIO_0_10, (LPC_IO_DIGITAL | LPC_IO_OPEN_DRAIN_ENABLE) },
	{ LPC_I2C0_SDA_PIO_0_11, (LPC_IO_DIGITAL | LPC_IO_OPEN_DRAIN_ENABLE) },
	ARRAY_LAST_PIO,
};


const struct pio led_red = LPC_GPIO_0_24;
const struct pio led_orange = LPC_GPIO_0_23;
const struct pio led_green = LPC_GPIO_0_31;

/***************************************************************************** */
void system_init()
{
	/* Stop the watchdog */
	startup_watchdog_disable(); /* Do it right now, before it gets a chance to break in */
	system_brown_out_detection_config(0); /* No ADC used */
	system_set_default_power_state();
	clock_config(SELECTED_FREQ);
	set_pins(common_pins);
	gpio_on();
	uint32_t mode = LPC_IO_MODE_PULL_UP | LPC_IO_DRIVE_HIGHCURENT;
	config_gpio(&led_red, mode, GPIO_DIR_OUT, 0);
	config_gpio(&led_orange, mode, GPIO_DIR_OUT, 0);
	config_gpio(&led_green, mode, GPIO_DIR_OUT, 0);

	/* System tick timer MUST be configured and running in order to use the sleeping
	 * functions */
	systick_timer_on(1); /* 1ms */
	systick_start();
}

/* Define our fault handler. This one is not mandatory, the dummy fault handler
 * will be used when it's not overridden here.
 * Note : The default one does a simple infinite loop. If the watchdog is deactivated
 * the system will hang.
 * An alternative would be to perform soft reset of the micro-controller.
 */
void fault_info(const char* name, uint32_t len)
{
	uprintf(UART0, name);
	while (1);
}



/***************************************************************************** */
int main(void)
{
	system_init();
	struct lpc_gpio* gpio_red = LPC_GPIO_REGS(led_red.port);
	struct lpc_gpio* gpio_orange = LPC_GPIO_REGS(led_orange.port);
	struct lpc_gpio* gpio_green = LPC_GPIO_REGS(led_green.port);

	while (1) {
		gpio_orange->clear = LED_ORANGE;
		gpio_red->set = LED_RED;
		msleep(1000);
		gpio_red->clear = LED_RED;
		gpio_green->set = LED_GREEN;
		msleep(1500);
		gpio_green->clear = LED_GREEN;
		gpio_orange->set = LED_ORANGE;
		msleep(500);

	}
	return 0;
}


