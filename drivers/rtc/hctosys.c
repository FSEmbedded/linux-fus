/*
 * RTC subsystem, initialize system time on startup
 *
 * Copyright (C) 2005 Tower Technologies
 * Author: Alessandro Zummo <a.zummo@towertech.it>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/rtc.h>

/* IMPORTANT: the RTC only stores whole seconds. It is arbitrary
 * whether it stores the most close value or the value with partial
 * seconds truncated. However, it is important that we use it to store
 * the truncated value. This is because otherwise it is necessary,
 * in an rtc sync function, to read both xtime.tv_sec and
 * xtime.tv_nsec. On some processors (i.e. ARM), an atomic read
 * of >32bits is not possible. So storing the most close value would
 * slow down the sync API. So here we have the truncated value and
 * the best guess is to add 0.5s.
 */

static int __init do_rtc_hctosys(char *name)
{
	int err = -ENODEV;
	struct rtc_time tm;
	struct timespec64 tv64 = {
		.tv_nsec = NSEC_PER_SEC >> 1,
	};
	struct rtc_device *rtc = rtc_class_open(name);

	if (rtc == NULL) {
		pr_info("unable to open rtc device (%s)\n", name);
		goto err_open;
	}

	err = rtc_read_time(rtc, &tm);
	if (err) {
		dev_err(rtc->dev.parent,
			"hctosys: unable to read the hardware clock\n");
		goto err_read;

	}

	err = rtc_valid_tm(&tm);
	if (err) {
		dev_err(rtc->dev.parent,
			"hctosys: invalid date/time\n");
		goto err_invalid;
	}

	if ((tm.tm_year == 70) && (tm.tm_mon == 0) && (tm.tm_mday == 1)) {
		dev_err(rtc->dev.parent,
			"hctosys: 1970-01-01 not accepted as valid date\n");
		err = -EINVAL;
		goto err_invalid;
	}

	tv64.tv_sec = rtc_tm_to_time64(&tm);

	err = do_settimeofday64(&tv64);

	dev_info(rtc->dev.parent,
		"setting system clock to "
		"%d-%02d-%02d %02d:%02d:%02d UTC (%lld)\n",
		tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
		tm.tm_hour, tm.tm_min, tm.tm_sec,
		(long long) tv64.tv_sec);

err_invalid:
err_read:
	rtc_class_close(rtc);

err_open:
	rtc_hctosys_ret = err;

	return err;
}

static int __init rtc_hctosys(void)
{
#ifdef CONFIG_RTC_HCTOSYS_OPT
	/* Check the optional RTC first (e.g. more precise external RTC) */
	if (!do_rtc_hctosys(CONFIG_RTC_HCTOSYS_OPT_DEVICE))
		return 0;
#endif

	return do_rtc_hctosys(CONFIG_RTC_HCTOSYS_DEVICE);
}

late_initcall(rtc_hctosys);
