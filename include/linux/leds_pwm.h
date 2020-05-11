/* SPDX-License-Identifier: GPL-2.0 */
/*
 * PWM LED driver data - see drivers/leds/leds-pwm.c
 */
#ifndef __LINUX_LEDS_PWM_H
#define __LINUX_LEDS_PWM_H

#define LED_PWM_FLAGS_ACTIVE_LOW	(1 << 0)
#define LED_PWM_FLAGS_GAMMA_2		(1 << 1)

struct led_pwm {
	const char	*name;
	const char	*default_trigger;
	unsigned	pwm_id __deprecated;
	unsigned	flags;
	unsigned 	max_brightness;
	unsigned	pwm_period_ns;
};

struct led_pwm_platform_data {
	int			num_leds;
	struct led_pwm	*leds;
};

#endif
