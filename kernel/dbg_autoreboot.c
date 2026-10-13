// SPDX-License-Identifier: GPL-2.0
/*
 * Debug helper: with "dbg_autoreboot=<seconds>" on the kernel command line, dump the
 * blocked tasks and init's stack after that many seconds, then warm-reboot to the
 * bootloader so the console log survives in ramoops/pstore.
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/reboot.h>
#include <linux/rcupdate.h>
#include <linux/sched.h>
#include <linux/sched/debug.h>
#include <linux/pid.h>
#include <linux/workqueue.h>

static unsigned int dbg_autoreboot;

static int __init dbg_autoreboot_setup(char *s)
{
	return kstrtouint(s, 10, &dbg_autoreboot) == 0;
}
__setup("dbg_autoreboot=", dbg_autoreboot_setup);

static void dbg_autoreboot_fn(struct work_struct *work)
{
	struct task_struct *init_task_p;

	pr_emerg("dbg_autoreboot: %u s elapsed, dumping state and rebooting to bootloader\n",
		 dbg_autoreboot);
	rcu_read_lock();
	init_task_p = pid_task(find_vpid(1), PIDTYPE_PID);
	if (init_task_p)
		sched_show_task(init_task_p);
	rcu_read_unlock();
	show_state_filter(TASK_UNINTERRUPTIBLE);
	kernel_restart("bootloader");
}
static DECLARE_DELAYED_WORK(dbg_autoreboot_work, dbg_autoreboot_fn);

static int __init dbg_autoreboot_init(void)
{
	if (dbg_autoreboot) {
		pr_info("dbg_autoreboot: armed for %u s\n", dbg_autoreboot);
		schedule_delayed_work(&dbg_autoreboot_work, dbg_autoreboot * HZ);
	}
	return 0;
}
late_initcall(dbg_autoreboot_init);
