#ifndef PICO4DRIVE_H
#define PICO4DRIVE_H

#include <stdint.h>

#include "hardware/clocks.h"

#define F_CPU		(132 * MHZ)

// use a PWM frequency of 20kHz
// the driver accepts up to 100kHz, but the higher the frequency the less
// efficient will be and the less resolution we'll have on the pwm value
#define PWM_WRAP	((int)(F_CPU / 20000))


enum {	
	DRV_1 = 0,
	DRV_2 = 1,
	DRV_3 = 2,
	DRV_4 = 3,
};

// state of the battery and ON button, updated by calling "pico4drive_update"
extern int battery_mV;		// battery voltage in mV
extern int on_button_state;	// current button state (1:pressed, 0:released)
extern uint32_t on_button_press_count;	// number of times the button has been pressed

// initialize pico4drive peripherals (drivers, adc)
void pico4drive_init(void);

// set the pwm level on a motor:
// driver: DRV_1 ... DRV_4
// value: -1.0 to 1.0
// value > 0 will drive the 'A'(+) output higher than the 'B'(-) output
void pico4drive_set_motor_pwm(int driver, float value);

// read an analog input
uint16_t pico4drive_read_adc(int channel);

// turn off the power to the circuit
void pico4drive_poweroff(void);

// update battery voltage and button state. The board updates this at roughly
// 50Hz, but this function can be called at a different rate. Calling it once
// every 600ms is enough to not lose events (using the press count variable),
// but it can be called at higher rates, e.g. 100Hz.
void pico4drive_update(void);

#endif