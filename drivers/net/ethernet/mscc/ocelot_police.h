/* SPDX-License-Identifier: (GPL-2.0 OR MIT) */
/* Microsemi Ocelot Switch driver
 *
 * Copyright (c) 2019 Microsemi Corporation
 */

#ifndef _MSCC_OCELOT_POLICE_H_
#define _MSCC_OCELOT_POLICE_H_

#include "ocelot.h"

enum mscc_qos_rate_mode {
	MSCC_QOS_RATE_MODE_DISABLED, /* Policer/shaper disabled */
	MSCC_QOS_RATE_MODE_LINE, /* Measure line rate in kbps incl. IPG */
	MSCC_QOS_RATE_MODE_DATA, /* Measures data rate in kbps excl. IPG */
	MSCC_QOS_RATE_MODE_FRAME, /* Measures frame rate in fps */
	__MSCC_QOS_RATE_MODE_END,
	NUM_MSCC_QOS_RATE_MODE = __MSCC_QOS_RATE_MODE_END,
	MSCC_QOS_RATE_MODE_MAX = __MSCC_QOS_RATE_MODE_END - 1,
};

int ocelot_port_policer_add(struct ocelot *ocelot, int port,
			    struct ocelot_policer *pol);

int ocelot_port_policer_del(struct ocelot *ocelot, int port);

#endif /* _MSCC_OCELOT_POLICE_H_ */
