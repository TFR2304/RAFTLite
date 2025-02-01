#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/uart.h"
#include "pico/time.h"

#include "pico4drive.h"

#define DRIVER_TO_SLICE(driver)		(((driver) + 5) & 7)

#define bit(a)		(1U << (a))

// pico4drive fixed pins
#define PIN_ADC_IN	28

#define PIN_MUXA	18
#define PIN_MUXB	19
#define PIN_MUXC	20

#define PIN_TINY_CTRL	21



int battery_mV;		// battery voltage in mV
int on_button_state;	// current button state (1:pressed, 0:released)
uint32_t on_button_press_count;	// number of times the button has been pressed


static void drive_pin_high(int pin)
{
	gpio_init(pin);
	gpio_put(pin, true);
	gpio_set_dir(pin, true);
	gpio_put(pin, true);
}

static void init_motor_pwms(void)
{
	int slice, driver, pin_a, pin_b;

	// setup a configuration structure
	pwm_config config = pwm_get_default_config();
	pwm_config_set_phase_correct(&config, true);
	pwm_config_set_clkdiv_int(&config, 1);
	pwm_config_set_clkdiv_mode(&config, PWM_DIV_FREE_RUNNING);
	pwm_config_set_output_polarity(&config, true, true);
	pwm_config_set_wrap(&config, PWM_WRAP);

	// set the same configuration on all pwm's
	for (driver = 0; driver < 4; driver++) {
		slice = DRIVER_TO_SLICE(driver);
		pwm_init(slice, &config, false);
		pico4drive_set_motor_pwm(driver, 0.0);
		pwm_set_enabled(slice, true);
		// just try to spread the pwm's around by a quarter each, so
		// that we avoid having all transitions happening at the same
		// time most of the time
		while (pwm_get_counter(slice) < PWM_WRAP / 2)
			;

		pin_a = 10 + driver * 2;
		pin_b = pin_a + 1;

		// we need to set both pins high for more than 100us to wake up
		// the driver, if it is a DRV8212. The DRV8212P doesn't need
		// this, but it doesn't affect it either
		drive_pin_high(pin_a);
		drive_pin_high(pin_b);
		sleep_ms(1);

		// set the function of the pins to pwm
		gpio_set_function(pin_a, GPIO_FUNC_PWM);
		gpio_set_function(pin_b, GPIO_FUNC_PWM);
	}
}

void pico4drive_set_motor_pwm(int driver, float value)
{
	int value_a, value_b;

	// make sure we saturate the input to the valid range
	if (value > 1.0) value = 1.0;
	if (value < -1.0) value = -1.0;

	if (value >= 0.0) {
		value_b = 0;
		value_a = value * PWM_WRAP;
	} else {
		value_b = -value * PWM_WRAP;
		value_a = 0;
	}

	pwm_set_both_levels(DRIVER_TO_SLICE(driver), value_a, value_b);
}


static void pico4drive_adc_set_channel(int channel)
{
	gpio_put_masked(bit(PIN_MUXA) | bit(PIN_MUXB) | bit(PIN_MUXC), channel << PIN_MUXA);
}

static void pico4drive_adc_init(void)
{
	adc_init();
	adc_gpio_init(PIN_ADC_IN);
	adc_select_input(PIN_ADC_IN - 26);
	//adc_set_clkdiv(132);

	gpio_init_mask(bit(PIN_MUXA) | bit(PIN_MUXB) | bit(PIN_MUXC));
	gpio_set_dir_out_masked(bit(PIN_MUXA) | bit(PIN_MUXB) | bit(PIN_MUXC));
	pico4drive_adc_set_channel(0);
}

uint16_t pico4drive_read_adc(int channel)
{
	pico4drive_adc_set_channel(channel);
	adc_select_input(PIN_ADC_IN - 26);
	sleep_us(10);
	return adc_read();
}

void pico4drive_poweroff(void)
{
	gpio_set_dir(PIN_TINY_CTRL, GPIO_OUT);
	gpio_put(PIN_TINY_CTRL, 0);
	while (1)
		;
}

void pico4drive_update(void)
{
	int c, bstate;

	while (uart_is_readable(uart1)) {
		c = uart_getc(uart1);

		bstate = c & 1;
		if (bstate && !on_button_state)
			on_button_press_count++;
		on_button_state = bstate;

		battery_mV = (c >> 1) * 50 + 4800;
	}
}

void pico4drive_init(void)
{
	init_motor_pwms();
	pico4drive_adc_init();

	gpio_set_pulls(PIN_TINY_CTRL, true, false);
	uart_init(uart1, 115200);
	uart_set_hw_flow(uart1, false, false);
	gpio_set_function(PIN_TINY_CTRL, GPIO_FUNC_UART);
}