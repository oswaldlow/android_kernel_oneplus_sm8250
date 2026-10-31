// SPDX-License-Identifier: GPL-2.0
/*
 * OnePlus 8 HyperOS port: /proc/mi_mem_engine/scene_tid for Xiaomi's "unfair memory allocation" launch boost.
 *
 * Xiaomi's mi_mem_engine is not public. Its user here is miui-services UnfairMemoryAllocationStubImpl
 * (system_server): from ActivityRecord.showStartingWindow(), i.e. inside every app start under the WM lock, it
 * writes "<mode> <tids...>" (the launcher's threads for mode 1, then android.anim, the surface animation thread,
 * the top app's pid and RenderThread), and 600 ms later "0 0 0". Without the node every start threw a
 * FileNotFoundException and printed its whole stack to System.err while holding the WM lock.
 *
 * While a scene is active the listed threads, and every thread of SurfaceFlinger (whose pid init writes into
 * /proc/mi_mem_engine/sf_pid: "writepid" in surfaceflinger.rc), allocate like realtime tasks do (ALLOC_HARDER in
 * gfp_to_alloc_flags(): may dip below the min watermark a little before direct reclaim), for at most SCENE_MS.
 * Reading scene_tid shows the current scene and counters. Owners / modes come from Xiaomi's init.rc.
 */
#include <linux/atomic.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/op8_mem_engine.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/uidgid.h>

#define MAX_TIDS	16
#define SCENE_MS	1000	/* the framework ends a scene after 600 ms */

static DEFINE_SPINLOCK(scene_lock);
static pid_t scene_tids[MAX_TIDS];
static int scene_mode;
static unsigned long scene_until;	/* jiffies */
static bool scene_active;		/* fast path, read without the lock */
static pid_t sf_pid;			/* SurfaceFlinger's tgid */
static atomic_t nr_begin, nr_end, nr_boosted;

bool op8_mem_scene_task(struct task_struct *p)
{
	int i;

	if (likely(!READ_ONCE(scene_active)))
		return false;
	if (time_after(jiffies, READ_ONCE(scene_until))) {
		WRITE_ONCE(scene_active, false);
		return false;
	}
	/* racy against a concurrent write: at worst one allocation more or less gets the boost */
	for (i = 0; i < MAX_TIDS; i++)
		if (READ_ONCE(scene_tids[i]) == p->pid)
			goto boost;
	if (p->tgid == READ_ONCE(sf_pid))
		goto boost;
	return false;
boost:
	atomic_inc(&nr_boosted);
	return true;
}

/* init writes SurfaceFlinger's pid here when it starts the service ("writepid" appends a newline) */
static ssize_t sf_pid_write(struct file *f, const char __user *ubuf, size_t len, loff_t *pos)
{
	char buf[16];
	int v;

	if (len >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, len))
		return -EFAULT;
	buf[len] = '\0';
	if (kstrtoint(strim(buf), 10, &v) || v < 0)
		return -EINVAL;
	WRITE_ONCE(sf_pid, v);
	return len;
}

static int sf_pid_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", READ_ONCE(sf_pid));
	return 0;
}

static int sf_pid_open(struct inode *inode, struct file *file)
{
	return single_open(file, sf_pid_show, NULL);
}

static const struct file_operations sf_pid_fops = {
	.owner		= THIS_MODULE,
	.open		= sf_pid_open,
	.read		= seq_read,
	.write		= sf_pid_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static ssize_t scene_write(struct file *f, const char __user *ubuf, size_t len, loff_t *pos)
{
	char buf[256], *s = buf, *tok;
	pid_t tids[MAX_TIDS] = { 0 };
	unsigned long flags;
	int mode = 0, n = 0, v, i;

	if (len >= sizeof(buf))
		len = sizeof(buf) - 1;
	if (copy_from_user(buf, ubuf, len))
		return -EFAULT;
	buf[len] = '\0';
	tok = strsep(&s, " \t\n");
	if (!tok || kstrtoint(tok, 10, &mode))
		return -EINVAL;
	while ((tok = strsep(&s, " \t\n")) && n < MAX_TIDS) {
		if (!*tok || kstrtoint(tok, 10, &v))
			continue;
		if (v > 0)
			tids[n++] = v;
	}

	spin_lock_irqsave(&scene_lock, flags);
	for (i = 0; i < MAX_TIDS; i++)
		WRITE_ONCE(scene_tids[i], tids[i]);
	scene_mode = mode;
	WRITE_ONCE(scene_until, jiffies + msecs_to_jiffies(SCENE_MS));
	WRITE_ONCE(scene_active, mode != 0 && n > 0);
	spin_unlock_irqrestore(&scene_lock, flags);
	atomic_inc(mode ? &nr_begin : &nr_end);
	return len;
}

static int scene_show(struct seq_file *m, void *v)
{
	unsigned long flags;
	pid_t tids[MAX_TIDS];
	int mode, i;
	bool active;
	long left;

	spin_lock_irqsave(&scene_lock, flags);
	memcpy(tids, scene_tids, sizeof(tids));
	mode = scene_mode;
	active = scene_active && time_before(jiffies, scene_until);
	left = active ? jiffies_to_msecs(scene_until - jiffies) : 0;
	spin_unlock_irqrestore(&scene_lock, flags);

	seq_printf(m, "mode %d active %d left_ms %ld tids", mode, active, left);
	for (i = 0; i < MAX_TIDS && tids[i]; i++)
		seq_printf(m, " %d", tids[i]);
	seq_printf(m, "\nbegin %d end %d boosted_allocs %d\n", atomic_read(&nr_begin), atomic_read(&nr_end),
		   atomic_read(&nr_boosted));
	return 0;
}

static int scene_open(struct inode *inode, struct file *file)
{
	return single_open(file, scene_show, NULL);
}

static const struct file_operations scene_fops = {
	.owner		= THIS_MODULE,
	.open		= scene_open,
	.read		= seq_read,
	.write		= scene_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init op8_mem_engine_init(void)
{
	struct proc_dir_entry *dir, *e;

	/* Xiaomi init.rc: chmod 0771 the dir, chown system root + chmod 0664 scene_tid, chown system root sf_pid */
	dir = proc_mkdir_mode("mi_mem_engine", 0771, NULL);
	if (!dir)
		return -ENOMEM;
	e = proc_create("scene_tid", 0664, dir, &scene_fops);
	if (!e)
		return -ENOMEM;
	proc_set_user(e, KUIDT_INIT(1000), GLOBAL_ROOT_GID);	/* system_server (uid system) writes it */
	e = proc_create("sf_pid", 0644, dir, &sf_pid_fops);
	if (!e)
		return -ENOMEM;
	proc_set_user(e, KUIDT_INIT(1000), GLOBAL_ROOT_GID);
	return 0;
}
device_initcall(op8_mem_engine_init);
