// SPDX-License-Identifier: GPL-2.0
/*
 * OnePlus 8 HyperOS port: /dev/iorap_dev for Xiaomi's app launch prefetch ("launch boost").
 *
 * Xiaomi's driver is not public; this is a clean reimplementation of the interface its two
 * users expect (reverse engineered from /system_ext/bin/iorapd and libmiui_service.so):
 *
 *   all ioctls: _IOWR('L', nr, 0x108), argument starts with struct op8_iorap_req
 *   COLLECT (1)  system_server, app process / render thread started: record what this uid reads
 *   CLEAR (3)    system_server, app installed / dexopted / removed: drop its profile
 *   STOP (5)     system_server, launch finished or cancelled: merge the recording into the profile
 *   SIZE (7)     iorapd, launch intent: u64 at +0x108 = buffer size needed for FETCH (0 = no profile)
 *   FETCH (8)    iorapd: u64 at +0x110 = buffer size; we fill entries from +0x118 and put their
 *                count at +0x108. iorapd then posix_fadvise(WILLNEED)s every set bit (= page) of
 *                every file whose st_mtime still matches.
 *   0x10-0x12    PTE snapshot / preread / stop: not implemented, succeed silently.
 *
 * Recording hooks: __do_page_cache_readahead() (read() and fault readahead) and filemap_fault()
 * (mmap access, including pages already cached). Only uids >= 10000 that are being recorded are
 * looked at; everything else returns after one atomic read. Profiles live in RAM only, like
 * Xiaomi's (the framework re-clears them on package changes anyway).
 */
#include <linux/atomic.h>
#include <linux/bitmap.h>
#include <linux/cred.h>
#include <linux/dcache.h>
#include <linux/fs.h>
#include <linux/hashtable.h>
#include <linux/jiffies.h>
#include <linux/kdev_t.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/op8_iorap.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/uidgid.h>

#define IORAP_COLLECT		_IOWR('L', 0x01, struct op8_iorap_req)
#define IORAP_CLEAR		_IOWR('L', 0x03, struct op8_iorap_req)
#define IORAP_STOP		_IOWR('L', 0x05, struct op8_iorap_req)
#define IORAP_SIZE		_IOWR('L', 0x07, struct op8_iorap_req)
#define IORAP_FETCH		_IOWR('L', 0x08, struct op8_iorap_req)
#define IORAP_PTE_SNAPSHOT	_IOWR('L', 0x10, struct op8_iorap_req)
#define IORAP_PTE_PREREAD	_IOWR('L', 0x11, struct op8_iorap_req)
#define IORAP_PTE_STOP		_IOWR('L', 0x12, struct op8_iorap_req)

#define PKG_LEN		256
#define PATH_LEN	256
#define HDR_COUNT	0x108	/* SIZE: u64 size out; FETCH: u64 entry count out */
#define HDR_BUFSIZE	0x110	/* FETCH: u64 buffer size in */
#define HDR_LEN		0x118	/* FETCH: entries start here */
#define ENT_HDR_LEN	0x128	/* entry header, then u64 bitmap words */

#define MAX_SESSIONS	4
#define SESSION_SECS	20
#define MAX_FILES	512	/* per session / profile */
#define MAX_RANGES	1024	/* per file */
#define MAX_PAGES	(1U << 20)	/* 4 GB per file */
#define MAX_PROFILES	64
#define FIRST_APP_UID	10000

struct op8_iorap_req {
	__u32 uid;
	__u32 pid;
	char pkg[PKG_LEN];
};

struct iorap_range {
	u32 start, end;		/* pages, [start, end) */
};

struct iorap_file {
	struct hlist_node hnode;	/* session hash */
	struct list_head node;
	dev_t dev;
	u64 ino;
	u64 mtime;
	u32 nr, cap;
	struct iorap_range *r;
	char path[PATH_LEN];
};

struct iorap_session {
	bool active;		/* slot in use */
	bool expired;		/* past SESSION_SECS: keep, but stop recording */
	u32 uid;
	unsigned long start;
	char pkg[PKG_LEN];
	u32 nr_files;
	struct list_head files;
	DECLARE_HASHTABLE(hash, 6);
};

struct iorap_profile {
	struct list_head node;	/* most recently used first */
	u32 uid;
	char pkg[PKG_LEN];
	u32 nr_files;
	struct list_head files;
};

static atomic_t nr_recording = ATOMIC_INIT(0);
static DEFINE_SPINLOCK(rec_lock);		/* sessions[] and their files */
static struct iorap_session sessions[MAX_SESSIONS];
static DEFINE_MUTEX(prof_lock);			/* profiles */
static LIST_HEAD(profiles);
static unsigned int nr_profiles;

static void free_file(struct iorap_file *f)
{
	kfree(f->r);
	kfree(f);
}

static void free_files(struct list_head *head)
{
	struct iorap_file *f, *t;

	list_for_each_entry_safe(f, t, head, node) {
		list_del(&f->node);
		free_file(f);
	}
}

/* Merge [start, end) into the sorted, non-overlapping range list. Caller may hold a spinlock. */
static void ra_add(struct iorap_file *f, u32 start, u32 end, gfp_t gfp)
{
	u32 i, j;

	if (start >= end)
		return;
	for (i = 0; i < f->nr && f->r[i].end < start; i++)
		;
	/* ranges i..j-1 overlap or touch [start, end) */
	for (j = i; j < f->nr && f->r[j].start <= end; j++) {
		start = min(start, f->r[j].start);
		end = max(end, f->r[j].end);
	}
	if (j > i) {
		f->r[i].start = start;
		f->r[i].end = end;
		memmove(&f->r[i + 1], &f->r[j], (f->nr - j) * sizeof(*f->r));
		f->nr -= j - i - 1;
		return;
	}
	if (f->nr == f->cap) {
		u32 cap = f->cap ? f->cap * 2 : 8;
		struct iorap_range *r;

		if (cap > MAX_RANGES)
			return;
		r = krealloc(f->r, cap * sizeof(*r), gfp);
		if (!r)
			return;
		f->r = r;
		f->cap = cap;
	}
	memmove(&f->r[i + 1], &f->r[i], (f->nr - i) * sizeof(*f->r));
	f->r[i].start = start;
	f->r[i].end = end;
	f->nr++;
}

static struct iorap_session *session_for_uid(u32 uid)
{
	int i;

	for (i = 0; i < MAX_SESSIONS; i++)
		if (sessions[i].active && sessions[i].uid == uid)
			return &sessions[i];
	return NULL;
}

static struct iorap_file *session_lookup(struct iorap_session *s, dev_t dev, u64 ino)
{
	struct iorap_file *f;

	hash_for_each_possible(s->hash, f, hnode, ino ^ dev)
		if (f->ino == ino && f->dev == dev)
			return f;
	return NULL;
}

static bool recordable_fs(struct inode *inode)
{
	const char *name = inode->i_sb->s_type->name;

	return S_ISREG(inode->i_mode) &&
	       (!strcmp(name, "ext4") || !strcmp(name, "f2fs") || !strcmp(name, "erofs"));
}

void op8_iorap_record(struct file *file, pgoff_t index, unsigned long nr)
{
	struct iorap_session *s;
	struct iorap_file *f, *nf;
	struct inode *inode;
	unsigned long flags;
	u32 uid, start, end;
	char *p;

	if (likely(!atomic_read(&nr_recording)) || !file || !nr)
		return;
	uid = from_kuid(&init_user_ns, current_uid());
	if (uid < FIRST_APP_UID)
		return;
	inode = file_inode(file);
	if (!recordable_fs(inode) || index >= MAX_PAGES)
		return;
	start = index;
	end = min_t(unsigned long, index + nr, MAX_PAGES);

	spin_lock_irqsave(&rec_lock, flags);
	s = session_for_uid(uid);
	if (!s || s->expired) {
		spin_unlock_irqrestore(&rec_lock, flags);
		return;
	}
	if (time_after(jiffies, s->start + SESSION_SECS * HZ)) {
		s->expired = true;
		atomic_dec(&nr_recording);
		spin_unlock_irqrestore(&rec_lock, flags);
		return;
	}
	f = session_lookup(s, inode->i_sb->s_dev, inode->i_ino);
	if (f) {
		ra_add(f, start, end, GFP_ATOMIC | __GFP_NOWARN);
		spin_unlock_irqrestore(&rec_lock, flags);
		return;
	}
	if (s->nr_files >= MAX_FILES) {
		spin_unlock_irqrestore(&rec_lock, flags);
		return;
	}
	spin_unlock_irqrestore(&rec_lock, flags);

	/* first access to this file in this session: build the entry outside the lock */
	nf = kzalloc(sizeof(*nf), GFP_NOWAIT | __GFP_NOWARN);
	if (!nf)
		return;
	p = d_path(&file->f_path, nf->path, PATH_LEN);
	if (IS_ERR(p) || *p != '/' || d_unlinked(file->f_path.dentry)) {
		kfree(nf);
		return;
	}
	memmove(nf->path, p, strlen(p) + 1);
	nf->dev = inode->i_sb->s_dev;
	nf->ino = inode->i_ino;
	nf->mtime = inode->i_mtime.tv_sec;

	spin_lock_irqsave(&rec_lock, flags);
	s = session_for_uid(uid);
	if (!s || s->expired || s->nr_files >= MAX_FILES) {
		spin_unlock_irqrestore(&rec_lock, flags);
		kfree(nf);
		return;
	}
	f = session_lookup(s, nf->dev, nf->ino);
	if (!f) {
		f = nf;
		nf = NULL;
		hash_add(s->hash, &f->hnode, f->ino ^ f->dev);
		list_add_tail(&f->node, &s->files);
		s->nr_files++;
	}
	ra_add(f, start, end, GFP_ATOMIC | __GFP_NOWARN);
	spin_unlock_irqrestore(&rec_lock, flags);
	kfree(nf);
}

/* Detach a session's files into @out (empty list if none). Caller holds rec_lock. */
static void session_end_locked(struct iorap_session *s, struct list_head *out)
{
	list_splice_init(&s->files, out);
	if (!s->expired)
		atomic_dec(&nr_recording);
	s->active = false;
	s->nr_files = 0;
}

static void session_end(u32 uid, struct list_head *out, char *pkg)
{
	struct iorap_session *s;
	unsigned long flags;

	spin_lock_irqsave(&rec_lock, flags);
	s = session_for_uid(uid);
	if (s) {
		if (pkg)
			memcpy(pkg, s->pkg, PKG_LEN);
		session_end_locked(s, out);
	}
	spin_unlock_irqrestore(&rec_lock, flags);
}

static int do_collect(struct op8_iorap_req *req)
{
	struct iorap_session *s = NULL;
	LIST_HEAD(stale);
	unsigned long flags;
	int i;

	if (req->uid < FIRST_APP_UID)
		return 0;
	spin_lock_irqsave(&rec_lock, flags);
	if (session_for_uid(req->uid)) {	/* render / dex threads of a launch already recorded */
		spin_unlock_irqrestore(&rec_lock, flags);
		return 0;
	}
	for (i = 0; i < MAX_SESSIONS && !s; i++)
		if (!sessions[i].active)
			s = &sessions[i];
	/* all busy: reuse one that ran out of time and never got its STOP */
	for (i = 0; i < MAX_SESSIONS && !s; i++)
		if (sessions[i].expired ||
		    time_after(jiffies, sessions[i].start + SESSION_SECS * HZ)) {
			s = &sessions[i];
			session_end_locked(s, &stale);
		}
	if (s) {
		s->active = true;
		s->expired = false;
		s->uid = req->uid;
		s->start = jiffies;
		memcpy(s->pkg, req->pkg, PKG_LEN);
		s->nr_files = 0;
		INIT_LIST_HEAD(&s->files);
		hash_init(s->hash);
		atomic_inc(&nr_recording);
	}
	spin_unlock_irqrestore(&rec_lock, flags);
	free_files(&stale);
	return 0;
}

static struct iorap_profile *profile_find(u32 uid, const char *pkg)
{
	struct iorap_profile *p;

	list_for_each_entry(p, &profiles, node)
		if (p->uid == uid && !strcmp(p->pkg, pkg))
			return p;
	return NULL;
}

static void profile_free(struct iorap_profile *p)
{
	list_del(&p->node);
	free_files(&p->files);
	kfree(p);
	nr_profiles--;
}

/* Union a finished recording into the (uid, pkg) profile. Takes ownership of @files. */
static void profile_merge(u32 uid, const char *pkg, struct list_head *files)
{
	struct iorap_profile *p;
	struct iorap_file *f, *t, *pf;
	u32 pages = 0, nfiles = 0, i;

	mutex_lock(&prof_lock);
	p = profile_find(uid, pkg);
	if (!p) {
		p = kzalloc(sizeof(*p), GFP_KERNEL);
		if (!p)
			goto out;
		p->uid = uid;
		strscpy(p->pkg, pkg, PKG_LEN);
		INIT_LIST_HEAD(&p->files);
		if (nr_profiles >= MAX_PROFILES)
			profile_free(list_last_entry(&profiles, struct iorap_profile, node));
		nr_profiles++;
	} else {
		list_del(&p->node);
	}
	list_add(&p->node, &profiles);

	list_for_each_entry_safe(f, t, files, node) {
		list_del(&f->node);
		pf = NULL;
		list_for_each_entry(pf, &p->files, node)
			if (pf->dev == f->dev && pf->ino == f->ino)
				break;
		if (&pf->node == &p->files)
			pf = NULL;
		if (pf && pf->mtime == f->mtime && !strcmp(pf->path, f->path)) {
			for (i = 0; i < f->nr; i++)
				ra_add(pf, f->r[i].start, f->r[i].end, GFP_KERNEL);
			free_file(f);
			continue;
		}
		if (pf) {	/* file changed (app updated): the new recording replaces it */
			list_del(&pf->node);
			free_file(pf);
			p->nr_files--;
		}
		if (p->nr_files >= MAX_FILES) {
			free_file(f);
			continue;
		}
		list_add_tail(&f->node, &p->files);
		p->nr_files++;
	}
	list_for_each_entry(pf, &p->files, node) {
		nfiles++;
		for (i = 0; i < pf->nr; i++)
			pages += pf->r[i].end - pf->r[i].start;
	}
	pr_info("op8_iorap: %s (uid %u): profile %u files, %u pages\n", pkg, uid, nfiles, pages);
out:
	mutex_unlock(&prof_lock);
	free_files(files);
}

static int do_stop(struct op8_iorap_req *req)
{
	LIST_HEAD(files);
	char *pkg;

	pkg = kmalloc(PKG_LEN, GFP_KERNEL);
	if (!pkg)
		return -ENOMEM;
	pkg[0] = '\0';
	session_end(req->uid, &files, pkg);
	/* the profile is keyed by the package iorapd asks for, i.e. the one recorded */
	if (!list_empty(&files))
		profile_merge(req->uid, pkg[0] ? pkg : req->pkg, &files);
	kfree(pkg);
	return 0;
}

static int do_clear(struct op8_iorap_req *req)
{
	struct iorap_profile *p;
	LIST_HEAD(files);

	session_end(req->uid, &files, NULL);
	free_files(&files);
	mutex_lock(&prof_lock);
	p = profile_find(req->uid, req->pkg);
	if (p)
		profile_free(p);
	mutex_unlock(&prof_lock);
	return 0;
}

static u32 file_nbits(struct iorap_file *f)
{
	return f->nr ? f->r[f->nr - 1].end : 0;
}

static size_t entry_len(struct iorap_file *f)
{
	return ENT_HDR_LEN + DIV_ROUND_UP(file_nbits(f), 64) * 8;
}

static int do_size(struct op8_iorap_req *req, void __user *arg)
{
	struct iorap_profile *p;
	struct iorap_file *f;
	u64 size = 0;

	mutex_lock(&prof_lock);
	p = profile_find(req->uid, req->pkg);
	if (p && p->nr_files) {
		size = HDR_LEN;
		list_for_each_entry(f, &p->files, node)
			size += entry_len(f);
	}
	mutex_unlock(&prof_lock);
	return put_user(size, (u64 __user *)(arg + HDR_COUNT));
}

static int do_fetch(struct op8_iorap_req *req, void __user *arg)
{
	struct iorap_profile *p;
	struct iorap_file *f;
	u64 bufsize, count = 0;
	size_t off = HDR_LEN, len;
	u8 *ent = NULL;
	u32 nbits, nwords, i;
	unsigned long *bm;
	int ret = 0;

	if (get_user(bufsize, (u64 __user *)(arg + HDR_BUFSIZE)))
		return -EFAULT;
	if (bufsize < HDR_LEN)
		return -EINVAL;

	mutex_lock(&prof_lock);
	p = profile_find(req->uid, req->pkg);
	if (p) {
		list_del(&p->node);	/* most recently used */
		list_add(&p->node, &profiles);
		list_for_each_entry(f, &p->files, node) {
			len = entry_len(f);
			if (off + len > bufsize)
				break;	/* grew since SIZE: send what fits */
			ent = kvzalloc(len, GFP_KERNEL);
			if (!ent) {
				ret = -ENOMEM;
				break;
			}
			nbits = file_nbits(f);
			nwords = DIV_ROUND_UP(nbits, 64);
			*(u64 *)(ent + 0x00) = f->mtime;
			*(u64 *)(ent + 0x08) = f->ino;
			*(u64 *)(ent + 0x10) = new_encode_dev(f->dev);
			*(u32 *)(ent + 0x18) = nbits;
			strscpy(ent + 0x20, f->path, PATH_LEN);
			*(u32 *)(ent + 0x120) = nwords;
			bm = (unsigned long *)(ent + ENT_HDR_LEN);
			for (i = 0; i < f->nr; i++)
				bitmap_set(bm, f->r[i].start, f->r[i].end - f->r[i].start);
			if (copy_to_user(arg + off, ent, len)) {
				ret = -EFAULT;
				break;
			}
			kvfree(ent);
			ent = NULL;
			off += len;
			count++;
		}
	}
	mutex_unlock(&prof_lock);
	kvfree(ent);
	if (ret)
		return ret;
	return put_user(count, (u64 __user *)(arg + HDR_COUNT));
}

static long iorap_ioctl(struct file *filp, unsigned int cmd, unsigned long uarg)
{
	void __user *arg = (void __user *)uarg;
	struct op8_iorap_req *req;
	long ret;

	switch (cmd) {
	case IORAP_PTE_SNAPSHOT:
	case IORAP_PTE_PREREAD:
	case IORAP_PTE_STOP:
		return 0;
	case IORAP_COLLECT:
	case IORAP_CLEAR:
	case IORAP_STOP:
	case IORAP_SIZE:
	case IORAP_FETCH:
		break;
	default:
		return -ENOTTY;
	}
	req = kmalloc(sizeof(*req), GFP_KERNEL);
	if (!req)
		return -ENOMEM;
	if (copy_from_user(req, arg, sizeof(*req))) {
		kfree(req);
		return -EFAULT;
	}
	req->pkg[PKG_LEN - 1] = '\0';
	switch (cmd) {
	case IORAP_COLLECT:
		ret = do_collect(req);
		break;
	case IORAP_CLEAR:
		ret = do_clear(req);
		break;
	case IORAP_STOP:
		ret = do_stop(req);
		break;
	case IORAP_SIZE:
		ret = do_size(req, arg);
		break;
	default:
		ret = do_fetch(req, arg);
		break;
	}
	kfree(req);
	return ret;
}

static const struct file_operations iorap_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl	= iorap_ioctl,
	.compat_ioctl	= iorap_ioctl,
	.llseek		= noop_llseek,
};

static struct miscdevice iorap_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "iorap_dev",
	.fops	= &iorap_fops,
};

static int __init op8_iorap_init(void)
{
	BUILD_BUG_ON(sizeof(struct op8_iorap_req) != 0x108);
	return misc_register(&iorap_misc);
}
device_initcall(op8_iorap_init);
