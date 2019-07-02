/*
 * PCA963X LED chip driver.
 *
 * Copyright 2012 bct electronic GmbH
 * Copyright 2013 Qtechnology A/S
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */

#ifndef __LINUX_PCA963X_H
#define __LINUX_PCA963X_H
#include <linux/leds.h>

enum pca963x_outdrv {
	PCA963X_OPEN_DRAIN,
	PCA963X_TOTEM_POLE, /* aka push-pull */
};

enum pca963x_blink_type {
	PCA963X_SW_BLINK,
	PCA963X_HW_BLINK,
};

/* Flags for each LED (leds->leds[i].flags) */
#define PCA963X_LED_FLAGS_ACTIVE_HIGH	BIT(0)
#define PCA963X_LED_FLAGS_KEEP_VALUE	BIT(1)
#define PCA963X_LED_FLAGS_DEFAULT_ON	BIT(2)
#define PCA963X_LED_FLAGS_TYPE_PWM	BIT(3)
#define PCA963X_LED_FLAGS_TYPE_GPIO	BIT(4)
#define PCA963X_LED_FLAGS_TYPE_MASK					\
	(PCA963X_LED_FLAGS_TYPE_PWM | PCA963X_LED_FLAGS_TYPE_GPIO)

struct pca963x_platform_data {
	struct led_platform_data leds;
	enum pca963x_outdrv outdrv;
	enum pca963x_blink_type blink_type;
};

#endif /* __LINUX_PCA963X_H*/
