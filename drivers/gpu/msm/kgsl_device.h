/* Copyright (c) 2002,2007-2012,2014 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef __KGSL_DEVICE_H
#define __KGSL_DEVICE_H

#include <linux/idr.h>
#include <linux/pm_qos.h>
#include <linux/earlysuspend.h>

#include "kgsl.h"
#include "kgsl_mmu.h"
#include "kgsl_pwrctrl.h"
#include "kgsl_log.h"
#include "kgsl_pwrscale.h"
#include <linux/sync.h>

#define KGSL_TIMEOUT_NONE           0
#define KGSL_TIMEOUT_DEFAULT        0xFFFFFFFF
#define KGSL_TIMEOUT_PART           50 
#define KGSL_TIMEOUT_LONG_IB_DETECTION  2000 

#define FIRST_TIMEOUT (HZ / 2)



#define KGSL_STATE_NONE		0x00000000
#define KGSL_STATE_INIT		0x00000001
#define KGSL_STATE_ACTIVE	0x00000002
#define KGSL_STATE_NAP		0x00000004
#define KGSL_STATE_SLEEP	0x00000008
#define KGSL_STATE_SUSPEND	0x00000010
#define KGSL_STATE_HUNG		0x00000020
#define KGSL_STATE_DUMP_AND_FT	0x00000040
#define KGSL_STATE_SLUMBER	0x00000080

#define KGSL_GRAPHICS_MEMORY_LOW_WATERMARK  0x1000000

#define KGSL_IS_PAGE_ALIGNED(addr) (!((addr) & (~PAGE_MASK)))


#define KGSL_EVENT_TIMESTAMP_RETIRED 0
#define KGSL_EVENT_CANCELLED 1

struct kgsl_device;
struct platform_device;
struct kgsl_device_private;
struct kgsl_context;
struct kgsl_power_stats;
struct kgsl_event;

struct kgsl_functable {
	void (*regread) (struct kgsl_device *device,
		unsigned int offsetwords, unsigned int *value);
	void (*regwrite) (struct kgsl_device *device,
		unsigned int offsetwords, unsigned int value);
	int (*idle) (struct kgsl_device *device);
	unsigned int (*isidle) (struct kgsl_device *device);
	int (*suspend_context) (struct kgsl_device *device);
	int (*init) (struct kgsl_device *device);
	int (*start) (struct kgsl_device *device);
	int (*stop) (struct kgsl_device *device);
	int (*getproperty) (struct kgsl_device *device,
		enum kgsl_property_type type, void *value,
		unsigned int sizebytes);
	int (*waittimestamp) (struct kgsl_device *device,
		struct kgsl_context *context, unsigned int timestamp,
		unsigned int msecs);
	unsigned int (*readtimestamp) (struct kgsl_device *device,
		struct kgsl_context *context, enum kgsl_timestamp_type type);
	int (*issueibcmds) (struct kgsl_device_private *dev_priv,
		struct kgsl_context *context, struct kgsl_ibdesc *ibdesc,
		unsigned int sizedwords, uint32_t *timestamp,
		unsigned int flags);
	int (*setup_pt)(struct kgsl_device *device,
		struct kgsl_pagetable *pagetable);
	void (*cleanup_pt)(struct kgsl_device *device,
		struct kgsl_pagetable *pagetable);
	void (*power_stats)(struct kgsl_device *device,
		struct kgsl_power_stats *stats);
	void (*irqctrl)(struct kgsl_device *device, int state);
	unsigned int (*gpuid)(struct kgsl_device *device, unsigned int *chipid);
	void * (*snapshot)(struct kgsl_device *device, void *snapshot,
		int *remain, int hang);
	irqreturn_t (*irq_handler)(struct kgsl_device *device);
	void (*setstate) (struct kgsl_device *device, unsigned int context_id,
			uint32_t flags);
	int (*drawctxt_create) (struct kgsl_device *device,
		struct kgsl_pagetable *pagetable, struct kgsl_context *context,
		uint32_t *flags);
	void (*drawctxt_destroy) (struct kgsl_device *device,
		struct kgsl_context *context);
	long (*ioctl) (struct kgsl_device_private *dev_priv,
		unsigned int cmd, void *data);
	int (*setproperty) (struct kgsl_device *device,
		enum kgsl_property_type type, void *value,
		unsigned int sizebytes);
	int (*postmortem_dump) (struct kgsl_device *device, int manual);
	int (*next_event)(struct kgsl_device *device,
		struct kgsl_event *event);
};

struct kgsl_mh {
	unsigned int     mharb;
	unsigned int     mh_intf_cfg1;
	unsigned int     mh_intf_cfg2;
	uint32_t         mpu_base;
	int              mpu_range;
};

typedef void (*kgsl_event_func)(struct kgsl_device *, void *, u32, u32, u32);

struct kgsl_event {
	struct kgsl_context *context;
	uint32_t timestamp;
	kgsl_event_func func;
	void *priv;
	struct list_head list;
	void *owner;
	unsigned int created;
};

struct kgsl_gpubusy {
	s64 busy;
	s64 total;
};

struct kgsl_device {
	struct device *dev;
	const char *name;
	unsigned int ver_major;
	unsigned int ver_minor;
	uint32_t flags;
	enum kgsl_deviceid id;
	unsigned long reg_phys;
	void *reg_virt;
	unsigned int reg_len;
	struct kgsl_memdesc memstore;
	const char *iomemname;

	struct kgsl_mh mh;
	struct kgsl_mmu mmu;
	struct completion hwaccess_gate;
	const struct kgsl_functable *ftbl;
	struct work_struct idle_check_ws;
	struct work_struct hang_check_ws;
	struct timer_list idle_timer;
	struct timer_list hang_timer;
	struct kgsl_pwrctrl pwrctrl;
	int open_count;

	struct mutex mutex;
	uint32_t state;
	uint32_t requested_state;

	unsigned int active_cnt;
	struct completion suspend_gate;

	wait_queue_head_t wait_queue;
	struct workqueue_struct *work_queue;
	struct device *parentdev;
	struct completion ft_gate;
	struct dentry *d_debugfs;
	struct idr context_idr;
	struct early_suspend display_off;
	rwlock_t context_lock;

	void *snapshot;		
	int snapshot_maxsize;   
	int snapshot_size;      
	u32 snapshot_timestamp;	
	int snapshot_frozen;	
	struct kobject snapshot_kobj;

	struct list_head snapshot_obj_list;

	
	int cmd_log;
	int ctxt_log;
	int drv_log;
	int mem_log;
	int pwr_log;
	int ft_log;
	int pm_dump_enable;
	struct kgsl_pwrscale pwrscale;
	struct kobject pwrscale_kobj;
	struct pm_qos_request pm_qos_req_dma;
	struct work_struct ts_expired_ws;
	struct list_head events;
	struct list_head events_pending_list;
	s64 on_time;

	
	int pm_regs_enabled;
	int pm_ib_enabled;

	int reset_counter; 
	int snapshot_no_panic;

	
	struct kgsl_gpubusy gputime;
	struct kgsl_gpubusy gputime_in_state[KGSL_MAX_PWRLEVELS];
#ifdef CONFIG_MSM_KGSL_GPU_USAGE
	struct kgsl_process_private *current_process_priv;
#endif
};

void kgsl_process_events(struct work_struct *work);
void kgsl_check_fences(struct work_struct *work);

#define KGSL_DEVICE_COMMON_INIT(_dev) \
	.hwaccess_gate = COMPLETION_INITIALIZER((_dev).hwaccess_gate),\
	.suspend_gate = COMPLETION_INITIALIZER((_dev).suspend_gate),\
	.ft_gate = COMPLETION_INITIALIZER((_dev).ft_gate),\
	.idle_check_ws = __WORK_INITIALIZER((_dev).idle_check_ws,\
			kgsl_idle_check),\
	.hang_check_ws = __WORK_INITIALIZER((_dev).hang_check_ws,\
			kgsl_hang_check),\
	.ts_expired_ws  = __WORK_INITIALIZER((_dev).ts_expired_ws,\
			kgsl_process_events),\
	.context_idr = IDR_INIT((_dev).context_idr),\
	.events = LIST_HEAD_INIT((_dev).events),\
	.events_pending_list = LIST_HEAD_INIT((_dev).events_pending_list), \
	.wait_queue = __WAIT_QUEUE_HEAD_INITIALIZER((_dev).wait_queue),\
	.mutex = __MUTEX_INITIALIZER((_dev).mutex),\
	.state = KGSL_STATE_INIT,\
	.ver_major = DRIVER_VERSION_MAJOR,\
	.ver_minor = DRIVER_VERSION_MINOR


struct kgsl_context {
	struct kref refcount;
	uint32_t id;
	struct kgsl_device_private *dev_priv;
	void *devctxt;
	unsigned int reset_status;
	bool wait_on_invalid_ts;
	struct sync_timeline *timeline;
	struct list_head events;
	struct list_head events_list;
};

struct kgsl_process_private {
	unsigned long priv;
	pid_t pid;
	spinlock_t mem_lock;

	
	struct kref refcount;
	
	struct mutex process_private_mutex;

	struct rb_root mem_rb;
	struct idr mem_idr;
	struct kgsl_pagetable *pagetable;
	struct list_head list;
	struct kobject kobj;
	struct dentry *debug_root;

	struct {
		unsigned int cur;
		unsigned int max;
	} stats[KGSL_MEM_ENTRY_MAX];

#ifdef CONFIG_MSM_KGSL_GPU_USAGE
	struct kgsl_gpubusy gputime;
	struct kgsl_gpubusy gputime_in_state[KGSL_MAX_PWRLEVELS];
#endif
};

enum kgsl_process_priv_flags {
	KGSL_PROCESS_INIT = 0,
};

struct kgsl_device_private {
	struct kgsl_device *device;
	struct kgsl_process_private *process_priv;
};

struct kgsl_power_stats {
	s64 total_time;
	s64 busy_time;
};

struct kgsl_device *kgsl_get_device(int dev_idx);

int kgsl_add_event(struct kgsl_device *device, u32 id, u32 ts,
	kgsl_event_func func, void *priv, void *owner);

static inline void kgsl_process_add_stats(struct kgsl_process_private *priv,
	unsigned int type, size_t size)
{
	priv->stats[type].cur += size;
	if (priv->stats[type].max < priv->stats[type].cur)
		priv->stats[type].max = priv->stats[type].cur;
}

static inline void kgsl_process_sub_stats(struct kgsl_process_private *priv,
	unsigned int type, size_t size)
{
	priv->stats[type].cur -= size;
}

static inline void kgsl_regread(struct kgsl_device *device,
				unsigned int offsetwords,
				unsigned int *value)
{
	device->ftbl->regread(device, offsetwords, value);
}

static inline void kgsl_regwrite(struct kgsl_device *device,
				 unsigned int offsetwords,
				 unsigned int value)
{
	device->ftbl->regwrite(device, offsetwords, value);
}

static inline int kgsl_idle(struct kgsl_device *device)
{
	return device->ftbl->idle(device);
}

static inline unsigned int kgsl_gpuid(struct kgsl_device *device,
	unsigned int *chipid)
{
	return device->ftbl->gpuid(device, chipid);
}

static inline unsigned int kgsl_readtimestamp(struct kgsl_device *device,
					      struct kgsl_context *context,
					      enum kgsl_timestamp_type type)
{
	return device->ftbl->readtimestamp(device, context, type);
}

static inline int kgsl_create_device_sysfs_files(struct device *root,
	const struct device_attribute **list)
{
	int ret = 0, i;
	for (i = 0; list[i] != NULL; i++)
		ret |= device_create_file(root, list[i]);
	return ret;
}

static inline void kgsl_remove_device_sysfs_files(struct device *root,
	const struct device_attribute **list)
{
	int i;
	for (i = 0; list[i] != NULL; i++)
		device_remove_file(root, list[i]);
}

static inline struct kgsl_mmu *
kgsl_get_mmu(struct kgsl_device *device)
{
	return (struct kgsl_mmu *) (device ? &device->mmu : NULL);
}

static inline struct kgsl_device *kgsl_device_from_dev(struct device *dev)
{
	int i;

	for (i = 0; i < KGSL_DEVICE_MAX; i++) {
		if (kgsl_driver.devp[i] && kgsl_driver.devp[i]->dev == dev)
			return kgsl_driver.devp[i];
	}

	return NULL;
}

static inline int kgsl_create_device_workqueue(struct kgsl_device *device)
{
	device->work_queue = create_singlethread_workqueue(device->name);
	if (!device->work_queue) {
		KGSL_DRV_ERR(device,
			     "create_singlethread_workqueue(%s) failed\n",
			     device->name);
		return -EINVAL;
	}
	return 0;
}



int kgsl_check_timestamp(struct kgsl_device *device,
		struct kgsl_context *context, unsigned int timestamp);

int kgsl_device_platform_probe(struct kgsl_device *device);

void kgsl_device_platform_remove(struct kgsl_device *device);

const char *kgsl_pwrstate_to_str(unsigned int state);

int kgsl_device_snapshot_init(struct kgsl_device *device);
int kgsl_device_snapshot(struct kgsl_device *device, int hang);
void kgsl_device_snapshot_close(struct kgsl_device *device);

static inline struct kgsl_device_platform_data *
kgsl_device_get_drvdata(struct kgsl_device *dev)
{
	struct platform_device *pdev =
		container_of(dev->parentdev, struct platform_device, dev);

	return pdev->dev.platform_data;
}

void kgsl_context_destroy(struct kref *kref);

static inline void
kgsl_context_put(struct kgsl_context *context)
{
	if (context)
		kref_put(&context->refcount, kgsl_context_destroy);
}

static inline int _kgsl_context_get(struct kgsl_context *context)
{
	int ret = 0;
	if (context)
		ret = kref_get_unless_zero(&context->refcount);
	return ret;
}

static inline struct kgsl_context *kgsl_context_get(struct kgsl_device *device,
		uint32_t id)
{
	int result = 0;
	struct kgsl_context *context = NULL;

	read_lock(&device->context_lock);

	context = idr_find(&device->context_idr, id);

	result = _kgsl_context_get(context);

	read_unlock(&device->context_lock);

	if (!result)
		return NULL;
	return context;
}

static inline struct kgsl_context *kgsl_context_get_owner(
		struct kgsl_device_private *dev_priv, uint32_t id)
{
	struct kgsl_context *context;

	context = kgsl_context_get(dev_priv->device, id);

	
	if (context && context->dev_priv != dev_priv) {
		kgsl_context_put(context);
		return NULL;
	}

	return context;
}

static inline void kgsl_context_cancel_events(struct kgsl_device *device,
	struct kgsl_context *context)
{
	kgsl_signal_events(device, context, KGSL_EVENT_CANCELLED);
}

static inline void kgsl_cancel_events_timestamp(struct kgsl_device *device,
	struct kgsl_context *context, unsigned int timestamp)
{
	kgsl_signal_event(device, context, timestamp, KGSL_EVENT_CANCELLED);
}

static inline int kgsl_sysfs_store(const char *buf, unsigned int *ptr)
{
	unsigned int val;
	int rc;

	rc = kstrtou32(buf, 0, &val);
	if (rc)
		return rc;

	if (ptr)
		*ptr = val;

	return 0;
}
#endif  
