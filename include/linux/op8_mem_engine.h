/* SPDX-License-Identifier: GPL-2.0 */
/* OnePlus 8 HyperOS port: Xiaomi mi_mem_engine launch scene (drivers/misc/op8_mem_engine.c) */
#ifndef _LINUX_OP8_MEM_ENGINE_H
#define _LINUX_OP8_MEM_ENGINE_H

#include <linux/types.h>

struct task_struct;

bool op8_mem_scene_task(struct task_struct *p);

#endif
