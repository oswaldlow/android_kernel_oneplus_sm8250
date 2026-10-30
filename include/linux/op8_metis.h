/* SPDX-License-Identifier: GPL-2.0 */
/* OnePlus 8 HyperOS port: drivers/misc/metis.c hooks */
#ifndef _LINUX_OP8_METIS_H
#define _LINUX_OP8_METIS_H

/* panel powered on or left AOD: short WALT full-throttle placement boost (lock screen clock animation) */
void op8_metis_display_on(void);
/* kernel/sched/core.c: per-task boost for the UI threads SchedBoostService makes SCHED_FIFO (S63) */
extern int op8_ui_fifo_boost;

#endif
