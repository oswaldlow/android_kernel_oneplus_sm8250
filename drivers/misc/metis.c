// SPDX-License-Identifier: GPL-2.0
/*
 * OnePlus 8 HyperOS port: /dev/metis + /sys/module/metis/parameters for Xiaomi's TurboSched.
 *
 * Xiaomi's "metis" scheduler driver is not public. TurboSched (system_server), the HWUI plugin in every app
 * (libhwuiinsptrdl.so, RenderInspector::getRenderToken / releaseRenderToken) and SurfaceFlinger
 * (libturboschedmanager_ssi.so) use it to make UI / render threads "VIP" around frames and animations.
 * Without it every unlock logged dozens of "open /dev/metis failed!" and SystemUI / launcher frames ran on the
 * A55 cores. This reimplements the part of the interface those users exercise (reverse engineered from
 * libmiui_runtime.so, libhwuiinsptrdl.so, libturboschedmanager_ssi.so, miui-services TurboSchedManagerService)
 * on top of the WALT per-task boost already in this kernel (p->boost = TASK_BOOST_ON_MID: prefer the big
 * cluster; boost_period / boost_expires: expires by itself).
 *
 * /dev/metis ioctls (arg = NULL unless noted; names from miui-framework TurboSchedMonitorImpl):
 *   0 GET_UI_TOKEN, 1 / 0x13 (HWUI render token), _IO('U', 15) (SF token): calling thread becomes VIP,
 *                           safety expiry TOKEN_MS
 *   3 RELEASE_UI_TOKEN, 4 (HWUI), _IO('U', 17) (SF): calling thread stops being VIP
 *   6 FRAME_BOOST_TOKEN, _IO('U', 6): calling thread VIP for mi_boost_duration ms (FRAME_MS if unset)
 *   8 SET_DYNAMIC_VIP {tid, ms} / 9 RELEASE_DYNAMIC_VIP {tid, 0}
 *   10 REQ_ASYNC_BINDER_LINK {tid, ms} / 11 RELEASE_ASYNC_BINDER_LINK (undoes the caller's last 10)
 *   _IOW('U', 7, 8) {tid, s}    task boost for s seconds
 *   _IOW('U', 12, 8) {pid, ms}  cluster boost start; _IOW('U', 13, 8) {pid, ms} end
 *   anything else            accepted, no effect, counted per request number in stats
 * /dev/metis_link: 3 = calling thread VIP (TOKEN_MS), 4 = stop.
 * Parameters: mi_viptask "t1:t2:t3-ms", add_/del_mi_viptask_sched_priority "t1:t2", common_persist_vip_task tid
 * (never expires or gets cleared), metis_wakeup_task "pid,..." (cold start; "0" clears), metis_wakeup_enable,
 * mi_boost_duration, sserver_tgid (stored only), version, stats (read-only counters),
 * display_on_boost_ms (screen-on / AOD-exit placement boost, see op8_metis_display_on(); 0 disables),
 * display_on_trigger (write-only: start that boost; perfservice.rc writes it when the screen turns on).
 * Callers with an app uid may only target threads of their own process.
 */
#include <linux/cred.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/moduleparam.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/op8_metis.h>
#include <linux/workqueue.h>
#include <linux/rcupdate.h>
#include <linux/sched.h>
#include <linux/sched/clock.h>
#include <linux/sched/sysctl.h>
#include <linux/sched/task.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/uidgid.h>

#define TOKEN_MS	2000	/* render / SF token without a release */
#define PRIO_MS		10000	/* add_mi_viptask_sched_priority without a del */
#define WAKEUP_MS	3000	/* cold start VIP */
#define FRAME_MS	200	/* FRAME_BOOST_TOKEN when mi_boost_duration is unset */
#define MAX_MS		10000
#define MAX_PERSIST	8
#define FIRST_APP_UID	10000

#define METIS_FRAME_BOOST	_IO('U', 6)
#define METIS_BOOST_U7		_IOW('U', 7, __u64)
#define METIS_CLUSTER_START	_IOW('U', 12, __u64)
#define METIS_CLUSTER_END	_IOW('U', 13, __u64)
#define METIS_SF_TOKEN		_IO('U', 15)
#define METIS_SF_RELEASE	_IO('U', 17)

enum { ST_TOKEN, ST_RELEASE, ST_FRAME, ST_DYNAMIC, ST_DYNAMIC_DEL, ST_LINK, ST_LINK_DEL, ST_TASKBOOST, ST_CLUSTER,
       ST_VIPTASK, ST_PRIO, ST_PRIO_MULTI, ST_PRIO_DEL, ST_PERSIST, ST_WAKEUP, ST_DISPLAY_ON, ST_REASSERT, ST_DENIED,
       ST_OTHER, ST_NR };
static const char * const st_name[ST_NR] = { "token", "release", "frame", "dynamic", "dynamic_del", "link",
	"link_del", "taskboost", "cluster", "viptask", "prio", "prio_multi", "prio_del", "persist", "wakeup",
	"display_on", "boost_reassert", "denied", "other" };
static atomic_t stats[ST_NR];
static int mi_boost_duration;	/* ms, written by SF / TurboSched before FRAME_BOOST */

/* requests we accept without acting on them, by number (shown in stats) */
#define MAX_OTHER	16
static DEFINE_SPINLOCK(other_lock);
static unsigned int other_cmd[MAX_OTHER];
static unsigned int other_cnt[MAX_OTHER];

static void count_other(unsigned int cmd)
{
	unsigned long flags;
	int i;

	atomic_inc(&stats[ST_OTHER]);
	spin_lock_irqsave(&other_lock, flags);
	for (i = 0; i < MAX_OTHER; i++) {
		if (other_cnt[i] && other_cmd[i] == cmd)
			break;
		if (!other_cnt[i]) {
			other_cmd[i] = cmd;
			break;
		}
	}
	if (i < MAX_OTHER)
		other_cnt[i]++;
	spin_unlock_irqrestore(&other_lock, flags);
}

static DEFINE_SPINLOCK(persist_lock);
static pid_t persist[MAX_PERSIST];
static pid_t wakeup_pid;

static bool is_persist(pid_t tid)
{
	unsigned long flags;
	bool ret = false;
	int i;

	spin_lock_irqsave(&persist_lock, flags);
	for (i = 0; i < MAX_PERSIST; i++)
		if (persist[i] == tid)
			ret = true;
	spin_unlock_irqrestore(&persist_lock, flags);
	return ret;
}

/* ms > 0: VIP for ms; ms == 0: clear; ms < 0: VIP without expiry */
static void boost_task(struct task_struct *p, int ms)
{
	if (is_persist(p->pid) && ms >= 0)
		return;
	/* S63 mark (op8_ui_fifo_mark() in kernel/sched/core.c): it ends when the thread leaves SCHED_FIFO, not here */
	if (p->policy == SCHED_FIFO && p->boost == TASK_BOOST_ON_MID && !p->boost_period)
		return;
	if (ms == 0) {
		p->boost = TASK_BOOST_NONE;
		p->boost_period = 0;
		p->boost_expires = 0;
		return;
	}
	if (ms > MAX_MS)
		ms = MAX_MS;
	if (ms > 0) {
		/* expiry first: per_task_boost() drops the boost once sched_clock() passes boost_expires */
		p->boost_expires = sched_clock() + (u64)ms * NSEC_PER_MSEC;
		p->boost_period = (u64)ms * NSEC_PER_MSEC;
	} else {
		p->boost_period = 0;
		p->boost_expires = 0;
	}
	p->boost = TASK_BOOST_ON_MID;
}

/* Target another thread; app-uid callers only inside their own process. */
static int boost_tid(pid_t tid, int ms)
{
	struct task_struct *p;
	bool app = from_kuid(&init_user_ns, current_uid()) >= FIRST_APP_UID;

	if (tid <= 0)
		return -EINVAL;
	rcu_read_lock();
	p = find_task_by_vpid(tid);
	if (p && (!app || p->tgid == current->tgid)) {
		get_task_struct(p);
		rcu_read_unlock();
		boost_task(p, ms);
		put_task_struct(p);
		return 0;
	}
	rcu_read_unlock();
	if (p)
		atomic_inc(&stats[ST_DENIED]);
	return p ? -EPERM : -ESRCH;
}

static int clamp_ms(long long v)
{
	return v <= 0 ? 0 : v > MAX_MS ? MAX_MS : (int)v;
}

/* REQ_ASYNC_BINDER_LINK (10) remembers which thread a caller boosted, RELEASE_ASYNC_BINDER_LINK (11) undoes it */
#define MAX_LINK	32
static DEFINE_SPINLOCK(link_lock);
static pid_t link_from[MAX_LINK], link_to[MAX_LINK];

static void link_set(pid_t from, pid_t to)
{
	unsigned long flags;
	int i, slot = 0;

	spin_lock_irqsave(&link_lock, flags);
	for (i = 0; i < MAX_LINK; i++) {
		if (link_from[i] == from) {
			slot = i;
			break;
		}
		if (!link_from[i])
			slot = i;
	}
	link_from[slot] = from;	/* table full: overwrite slot 0 (the old link just expires) */
	link_to[slot] = to;
	spin_unlock_irqrestore(&link_lock, flags);
}

static pid_t link_take(pid_t from)
{
	unsigned long flags;
	pid_t to = 0;
	int i;

	spin_lock_irqsave(&link_lock, flags);
	for (i = 0; i < MAX_LINK; i++)
		if (link_from[i] == from) {
			to = link_to[i];
			link_from[i] = link_to[i] = 0;
		}
	spin_unlock_irqrestore(&link_lock, flags);
	return to;
}

/*
 * Request numbers are Xiaomi's (miui-framework TurboSchedMonitorImpl): 0 GET_UI_TOKEN, 3 RELEASE_UI_TOKEN,
 * 6 FRAME_BOOST_TOKEN, 8 SET_DYNAMIC_VIP, 9 RELEASE_DYNAMIC_VIP, 10 REQ_ASYNC_BINDER_LINK,
 * 11 RELEASE_ASYNC_BINDER_LINK; 1 / 4 / 0x13 are the HWUI plugin's render token get / release / get.
 */
static long metis_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
	struct { __s32 a, b; } d;
	pid_t to;

	switch (cmd) {
	case 0:
	case 1:
	case 0x13:
	case METIS_SF_TOKEN:
		atomic_inc(&stats[ST_TOKEN]);
		boost_task(current, TOKEN_MS);
		return 0;
	case 3:
	case 4:
	case METIS_SF_RELEASE:
		atomic_inc(&stats[ST_RELEASE]);
		boost_task(current, 0);
		return 0;
	case 6:
	case METIS_FRAME_BOOST:
		atomic_inc(&stats[ST_FRAME]);
		boost_task(current, mi_boost_duration > 0 ? clamp_ms(mi_boost_duration) : FRAME_MS);
		return 0;
	case 11:
		atomic_inc(&stats[ST_LINK_DEL]);
		to = link_take(current->pid);
		return to ? boost_tid(to, 0) : 0;
	case 8:
	case 9:
	case 10:
	case METIS_BOOST_U7:
	case METIS_CLUSTER_START:
	case METIS_CLUSTER_END:
		if (copy_from_user(&d, (void __user *)arg, sizeof(d)))
			return -EFAULT;
		break;
	default:
		count_other(cmd);
		return 0;
	}
	switch (cmd) {
	case 8:
		atomic_inc(&stats[ST_DYNAMIC]);
		return boost_tid(d.a, clamp_ms(d.b));
	case 9:
		atomic_inc(&stats[ST_DYNAMIC_DEL]);
		return boost_tid(d.a, 0);
	case 10:
		atomic_inc(&stats[ST_LINK]);
		if (d.a > 0)
			link_set(current->pid, d.a);
		return boost_tid(d.a, d.b > 0 ? clamp_ms(d.b) : TOKEN_MS);
	case METIS_BOOST_U7:
		atomic_inc(&stats[ST_TASKBOOST]);
		return boost_tid(d.a, clamp_ms((long long)d.b * 1000));
	case METIS_CLUSTER_START:
		atomic_inc(&stats[ST_CLUSTER]);
		return boost_tid(d.a, d.b > 0 ? clamp_ms(d.b) : TOKEN_MS);
	default:	/* METIS_CLUSTER_END */
		return boost_tid(d.a, 0);
	}
}

/*
 * Screen on / AOD exit: WALT full-throttle placement boost (sched_set_boost(1), refcounted, so the perf HAL's own
 * boosts are untouched) for display_on_boost_ms. Triggered by dsi_panel_enable() / dsi_panel_set_nolp() and by
 * user space (display_on_trigger): waking from AOD on this port keeps the panel in DPMS ON, so no panel event. Per-thread VIP was not enough here: SystemUI's lock screen
 * RenderThread (clock animation) kept running ~90 % on the A55 cores even with TASK_BOOST_ON_MID, because the
 * system is overutilized right after wake and EAS placement is bypassed. Measured (4 wakes, gaps > 15 ms in the
 * first 2 s): 11 / 12 / 9 / 6 without, 4 / 3 / 4 / 4 with a 1.5 s boost.
 */
static int display_on_boost_ms = 1500;
static DEFINE_MUTEX(display_boost_lock);
static bool display_boosted;
static unsigned long display_boost_until;	/* jiffies */

/*
 * The QTI perf HAL writes /proc/sys/kernel/sched_boost for its own hints and writes 0 when they end, and 0 means
 * "disable all boosts" (sched_boost_disable_all()), ours included. Re-check every 100 ms during the window and
 * re-enable when it was wiped (kernel #30 measured the boost surviving only the first of several wakes).
 */
static void display_boost_tick_fn(struct work_struct *w);
static DECLARE_DELAYED_WORK(display_boost_tick, display_boost_tick_fn);

static void display_boost_tick_fn(struct work_struct *w)
{
	mutex_lock(&display_boost_lock);
	if (time_after_eq(jiffies, display_boost_until)) {
		if (display_boosted)
			sched_set_boost(-1);	/* FULL_THROTTLE_BOOST_DISABLE (no-op if already wiped) */
		display_boosted = false;
		mutex_unlock(&display_boost_lock);
		return;
	}
	if (!display_boosted || !sysctl_sched_boost) {
		if (!sched_set_boost(1)) {	/* FULL_THROTTLE_BOOST */
			if (display_boosted)
				atomic_inc(&stats[ST_REASSERT]);
			display_boosted = true;
		}
	}
	mutex_unlock(&display_boost_lock);
	queue_delayed_work(system_highpri_wq, &display_boost_tick, msecs_to_jiffies(100));
}

static void display_boost_on_fn(struct work_struct *w)
{
	int ms = display_on_boost_ms;

	if (ms <= 0)
		return;
	if (ms > MAX_MS)
		ms = MAX_MS;
	mutex_lock(&display_boost_lock);
	display_boost_until = jiffies + msecs_to_jiffies(ms);
	mutex_unlock(&display_boost_lock);
	mod_delayed_work(system_highpri_wq, &display_boost_tick, 0);
}
static DECLARE_WORK(display_boost_on, display_boost_on_fn);

void op8_metis_display_on(void)
{
	atomic_inc(&stats[ST_DISPLAY_ON]);
	queue_work(system_highpri_wq, &display_boost_on);
}

static const struct file_operations metis_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl	= metis_ioctl,
	.compat_ioctl	= metis_ioctl,
	.llseek		= noop_llseek,
};

static struct miscdevice metis_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "metis",
	.fops	= &metis_fops,
};

/* /dev/metis_link (TurboSchedMonitorImpl setLinkBoost / cleanLinkBoost): 3 = boost the calling thread, 4 = stop */
static long metis_link_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
	if (cmd == 3) {
		atomic_inc(&stats[ST_LINK]);
		boost_task(current, TOKEN_MS);
	} else if (cmd == 4) {
		atomic_inc(&stats[ST_LINK_DEL]);
		boost_task(current, 0);
	} else {
		count_other(0x10000 | cmd);
	}
	return 0;
}

static const struct file_operations metis_link_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl	= metis_link_ioctl,
	.compat_ioctl	= metis_link_ioctl,
	.llseek		= noop_llseek,
};

static struct miscdevice metis_link_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "metis_link",
	.fops	= &metis_link_fops,
};

/* "t1:t2:t3" (optionally "-ms" when with_time) -> boost each tid; returns how many valid tids there were */
static int parse_tids(const char *val, int ms_default, bool with_time, int stat)
{
	char buf[128], *s, *tok, *dash;
	int ms = ms_default, tid, n = 0;

	strlcpy(buf, val, sizeof(buf));
	s = strim(buf);
	dash = strchr(s, '-');
	if (dash) {
		*dash = '\0';
		if (with_time) {
			long long t;

			if (!kstrtoll(dash + 1, 10, &t))
				ms = clamp_ms(t);
		}
	}
	atomic_inc(&stats[stat]);
	while ((tok = strsep(&s, ":,")) != NULL) {
		if (*tok && !kstrtoint(tok, 10, &tid) && tid > 0) {
			boost_tid(tid, ms);
			n++;
		}
	}
	return n;
}

static int set_viptask(const char *val, const struct kernel_param *kp)
{
	parse_tids(val, TOKEN_MS, true, ST_VIPTASK);
	return 0;
}

/*
 * The launcher (TurboSched caller allow list) submits its UI / render threads together, 3-5 tids at once, when it
 * starts one of its animations (unlock into the home screen, app open / close); the other PRIORITY users send one
 * tid. Per-thread VIP alone left the unlock animation at ~3.4 hitches per unlock; adding the 1.5 s placement boost
 * (op8_metis_display_on) brought it to ~2.3.
 */
static int set_prio_add(const char *val, const struct kernel_param *kp)
{
	if (parse_tids(val, PRIO_MS, false, ST_PRIO) >= 2) {
		atomic_inc(&stats[ST_PRIO_MULTI]);
		op8_metis_display_on();
	}
	return 0;
}

static int set_prio_del(const char *val, const struct kernel_param *kp)
{
	parse_tids(val, 0, false, ST_PRIO_DEL);
	return 0;
}

static int set_persist(const char *val, const struct kernel_param *kp)
{
	unsigned long flags;
	int tid, i, slot = -1;

	if (kstrtoint(val, 10, &tid) || tid <= 0)
		return -EINVAL;
	rcu_read_lock();
	spin_lock_irqsave(&persist_lock, flags);
	for (i = 0; i < MAX_PERSIST; i++) {
		if (persist[i] == tid)
			slot = i;
		else if (slot < 0 && (!persist[i] || !find_vpid(persist[i])))
			slot = i;	/* free or dead */
	}
	if (slot >= 0)
		persist[slot] = 0;	/* boost_task() below must not skip it */
	spin_unlock_irqrestore(&persist_lock, flags);
	rcu_read_unlock();
	if (slot < 0)
		return -ENOSPC;
	atomic_inc(&stats[ST_PERSIST]);
	boost_tid(tid, -1);
	spin_lock_irqsave(&persist_lock, flags);
	persist[slot] = tid;
	spin_unlock_irqrestore(&persist_lock, flags);
	return 0;
}

static int get_persist(char *buffer, const struct kernel_param *kp)
{
	int i, n = 0;

	for (i = 0; i < MAX_PERSIST; i++)
		if (persist[i])
			n += scnprintf(buffer + n, PAGE_SIZE - n, "%d ", persist[i]);
	return n + scnprintf(buffer + n, PAGE_SIZE - n, "\n");
}

static int set_wakeup(const char *val, const struct kernel_param *kp)
{
	int pid = 0;

	sscanf(val, "%d", &pid);
	atomic_inc(&stats[ST_WAKEUP]);
	if (pid > 0) {
		wakeup_pid = pid;
		return boost_tid(pid, WAKEUP_MS) == -EINVAL ? -EINVAL : 0;
	}
	if (wakeup_pid > 0)
		boost_tid(wakeup_pid, 0);
	wakeup_pid = 0;
	return 0;
}

static int get_stats(char *buffer, const struct kernel_param *kp)
{
	int i, n = 0;

	for (i = 0; i < ST_NR; i++)
		n += scnprintf(buffer + n, PAGE_SIZE - n, "%s %d\n", st_name[i], atomic_read(&stats[i]));
	for (i = 0; i < MAX_OTHER && other_cnt[i]; i++)
		n += scnprintf(buffer + n, PAGE_SIZE - n, "other 0x%x %u\n", other_cmd[i], other_cnt[i]);
	return n;
}

static int get_empty(char *buffer, const struct kernel_param *kp)
{
	return scnprintf(buffer, PAGE_SIZE, "\n");
}

static const struct kernel_param_ops viptask_ops = { .set = set_viptask, .get = get_empty };
static const struct kernel_param_ops prio_add_ops = { .set = set_prio_add, .get = get_empty };
static const struct kernel_param_ops prio_del_ops = { .set = set_prio_del, .get = get_empty };
static const struct kernel_param_ops persist_ops = { .set = set_persist, .get = get_persist };
static const struct kernel_param_ops wakeup_ops = { .set = set_wakeup, .get = get_empty };
static const struct kernel_param_ops stats_ops = { .get = get_stats };

/* perfservice.rc: on property:debug.tracing.screen_state=2 (system_server: screen on), any value */
static int set_display_on(const char *val, const struct kernel_param *kp)
{
	op8_metis_display_on();
	return 0;
}
static const struct kernel_param_ops display_on_ops = { .set = set_display_on, .get = get_empty };
module_param_cb(display_on_trigger, &display_on_ops, NULL, 0220);

module_param_cb(mi_viptask, &viptask_ops, NULL, 0664);
module_param_cb(add_mi_viptask_sched_priority, &prio_add_ops, NULL, 0664);
module_param_cb(del_mi_viptask_sched_priority, &prio_del_ops, NULL, 0664);
module_param_cb(common_persist_vip_task, &persist_ops, NULL, 0664);
module_param_cb(metis_wakeup_task, &wakeup_ops, NULL, 0664);
module_param_cb(stats, &stats_ops, NULL, 0444);

static int metis_wakeup_enable = 1;
module_param(metis_wakeup_enable, int, 0664);
module_param(mi_boost_duration, int, 0664);
module_param(display_on_boost_ms, int, 0664);
module_param_named(ui_fifo_boost, op8_ui_fifo_boost, int, 0664);
static int sserver_tgid;
module_param(sserver_tgid, int, 0664);
static char *version = "op8-1";
module_param(version, charp, 0444);

static int __init metis_init(void)
{
	int ret = misc_register(&metis_misc);

	return ret ? ret : misc_register(&metis_link_misc);
}
device_initcall(metis_init);
