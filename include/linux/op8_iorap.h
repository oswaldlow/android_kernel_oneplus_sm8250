/* SPDX-License-Identifier: GPL-2.0 */
/* OnePlus 8 HyperOS port: launch prefetch recording hook (drivers/misc/op8_iorap.c) */
#ifndef _LINUX_OP8_IORAP_H
#define _LINUX_OP8_IORAP_H

#include <linux/types.h>

struct file;

void op8_iorap_record(struct file *file, pgoff_t index, unsigned long nr);

#endif
