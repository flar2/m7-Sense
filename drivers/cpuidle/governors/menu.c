/*
 * menu.c - the menu idle governor
 *
 * Copyright (C) 2006-2007 Adam Belay <abelay@novell.com>
 * Copyright (C) 2009 Intel Corporation
 * Author:
 *        Arjan van de Ven <arjan@linux.intel.com>
 *
 * This code is licenced under the GPL version 2 as described
 * in the COPYING file that acompanies the Linux Kernel.
 */

#include <linux/kernel.h>
#include <linux/cpuidle.h>
#include <linux/pm_qos.h>
#include <linux/time.h>
#include <linux/ktime.h>
#include <linux/hrtimer.h>
#include <linux/tick.h>
#include <linux/sched.h>
#include <linux/math64.h>
#include <linux/module.h>

#define BUCKETS 12
#define INTERVALS 8
#define RESOLUTION 1024
#define DECAY 8
#define MAX_INTERESTING 50000
#define STDDEV_THRESH 400

/* 60 * 60 > STDDEV_THRESH * INTERVALS = 400 * 8 */
#define MAX_DEVIATION 60

static DEFINE_PER_CPU(struct hrtimer, menu_hrtimer);
static DEFINE_PER_CPU(int, hrtimer_status);
/* menu hrtimer mode */
enum {MENU_HRTIMER_STOP, MENU_HRTIMER_REPEAT, MENU_HRTIMER_GENERAL};


/*
 * The C-state residency is so long that is is worthwhile to exit
 * from the shallow C-state and re-enter into a deeper C-state.
 */
static unsigned int perfect_cstate_ms __read_mostly = 30;
module_param(perfect_cstate_ms, uint, 0000);

struct menu_device {
	int		last_state_idx;
	int             needs_update;

	unsigned int	expected_us;
	u64		predicted_us;
	unsigned int	exit_us;
	unsigned int	bucket;
	u64		correction_factor[BUCKETS];
	u32		intervals[INTERVALS];
	int		interval_ptr;
};


#define LOAD_INT(x) ((x) >> FSHIFT)
#define LOAD_FRAC(x) LOAD_INT(((x) & (FIXED_1-1)) * 100)

static inline int which_bucket(unsigned int duration)
{
	int bucket = 0;

	if (nr_iowait_cpu(smp_processor_id()))
		bucket = BUCKETS/2;

	if (duration < 10)
		return bucket;
	if (duration < 100)
		return bucket + 1;
	if (duration < 1000)
		return bucket + 2;
	if (duration < 10000)
		return bucket + 3;
	if (duration < 100000)
		return bucket + 4;
	return bucket + 5;
}

static inline int performance_multiplier(void)
{
	int mult = 1;

	

	

	
	mult += 10 * nr_iowait_cpu(smp_processor_id());

	return mult;
}

static DEFINE_PER_CPU(struct menu_device, menu_devices);

static void menu_update(struct cpuidle_driver *drv, struct cpuidle_device *dev);

static u64 div_round64(u64 dividend, u32 divisor)
{
	return div_u64(dividend + (divisor / 2), divisor);
}

void menu_hrtimer_cancel(void)
{
	int cpu = smp_processor_id();
	struct hrtimer *hrtmr = &per_cpu(menu_hrtimer, cpu);

	if (per_cpu(hrtimer_status, cpu)) {
		hrtimer_cancel(hrtmr);
		per_cpu(hrtimer_status, cpu) = MENU_HRTIMER_STOP;
	}
}
EXPORT_SYMBOL_GPL(menu_hrtimer_cancel);

static enum hrtimer_restart menu_hrtimer_notify(struct hrtimer *hrtimer)
{
	int cpu = smp_processor_id();
	struct menu_device *data = &per_cpu(menu_devices, cpu);

	/* In general case, the expected residency is much larger than
	 *  deepest C-state target residency, but prediction logic still
	 *  predicts a small predicted residency, so the prediction
	 *  history is totally broken if the timer is triggered.
	 *  So reset the correction factor.
	 */
	if (per_cpu(hrtimer_status, cpu) == MENU_HRTIMER_GENERAL)
		data->correction_factor[data->bucket] = RESOLUTION * DECAY;

	per_cpu(hrtimer_status, cpu) = MENU_HRTIMER_STOP;

	return HRTIMER_NORESTART;
}

static u32 get_typical_interval(struct menu_device *data)
{
	int i = 0, divisor = 0;
	uint64_t max = 0, avg = 0, stddev = 0;
	int64_t thresh = LLONG_MAX; /* Discard outliers above this value. */
	unsigned int ret = 0; 
	
again: 

	/* first calculate average and standard deviation of the past */
	max = avg = divisor = stddev = 0;
	for (i = 0; i < INTERVALS; i++) {
		int64_t value = data->intervals[i];
		if (value <= thresh) {
			avg += value;
			divisor++;
			if (value > max)
				max = value;
		}
	}
	do_div(avg, divisor);

	for (i = 0; i < INTERVALS; i++) {
		int64_t value = data->intervals[i];
		if (value <= thresh) {
			int64_t diff = value - avg;
			stddev += diff * diff;
 		}
	}
	do_div(stddev, divisor);
	stddev = int_sqrt(stddev); 


	if (((avg > stddev * 6) && (divisor * 4 >= INTERVALS * 3))
	              || stddev <= 20) { 
		data->predicted_us = avg;
		ret = 1;
	return ret;

	} else if ((divisor * 4) > INTERVALS * 3) {
		/* Exclude the max interval */
		thresh = max - 1;
		goto again; 
	}

	return ret;
}

static int menu_select(struct cpuidle_driver *drv, struct cpuidle_device *dev)
{
	struct menu_device *data = &__get_cpu_var(menu_devices);
	int latency_req = pm_qos_request(PM_QOS_CPU_DMA_LATENCY);
	int power_usage = -1;
	int i;
	int multiplier;
	struct timespec t;
	int repeat = 0, low_predicted = 0;
	int cpu = smp_processor_id();
	struct hrtimer *hrtmr = &per_cpu(menu_hrtimer, cpu);

	if (data->needs_update) {
		menu_update(drv, dev);
		data->needs_update = 0;
	}

	data->last_state_idx = 0;
	data->exit_us = 0;

	
	if (unlikely(latency_req == 0))
		return 0;

	
	t = ktime_to_timespec(tick_nohz_get_sleep_length());
	data->expected_us =
		t.tv_sec * USEC_PER_SEC + t.tv_nsec / NSEC_PER_USEC;


	data->bucket = which_bucket(data->expected_us);

	multiplier = performance_multiplier();

	if (data->correction_factor[data->bucket] == 0)
		data->correction_factor[data->bucket] = RESOLUTION * DECAY;

	
	data->predicted_us = div_round64(data->expected_us * data->correction_factor[data->bucket],
					 RESOLUTION * DECAY);

	repeat = get_typical_interval(data);

	if (data->expected_us > 5 &&
		dev->states_usage[CPUIDLE_DRIVER_STATE_START].disable == 0)
		data->last_state_idx = CPUIDLE_DRIVER_STATE_START;

	for (i = CPUIDLE_DRIVER_STATE_START; i < drv->state_count; i++) {
		struct cpuidle_state *s = &drv->states[i];
		struct cpuidle_state_usage *su = &dev->states_usage[i];

		if (su->disable)
			continue;
		if (s->target_residency > data->predicted_us) {
			low_predicted = 1;
			continue;
		}
		if (s->exit_latency > latency_req)
			continue;
		if (s->exit_latency * multiplier > data->predicted_us)
			continue;

		if (s->power_usage < power_usage) {
			power_usage = s->power_usage;
			data->last_state_idx = i;
			data->exit_us = s->exit_latency;
		}
	}

	/* not deepest C-state chosen for low predicted residency */
	if (low_predicted) {
		unsigned int timer_us = 0;
		unsigned int perfect_us = 0;

		/*
		 * Set a timer to detect whether this sleep is much
		 * longer than repeat mode predicted.  If the timer
		 * triggers, the code will evaluate whether to put
		 * the CPU into a deeper C-state.
		 * The timer is cancelled on CPU wakeup.
		 */
		timer_us = 2 * (data->predicted_us + MAX_DEVIATION);

		perfect_us = perfect_cstate_ms * 1000;

		if (repeat && (4 * timer_us < data->expected_us)) {
			hrtimer_start(hrtmr, ns_to_ktime(1000 * timer_us),
				HRTIMER_MODE_REL_PINNED);
			/* In repeat case, menu hrtimer is started */
			per_cpu(hrtimer_status, cpu) = MENU_HRTIMER_REPEAT;
		} else if (perfect_us < data->expected_us) {
			/*
			 * The next timer is long. This could be because
			 * we did not make a useful prediction.
			 * In that case, it makes sense to re-enter
			 * into a deeper C-state after some time.
			 */
			hrtimer_start(hrtmr, ns_to_ktime(1000 * timer_us),
				HRTIMER_MODE_REL_PINNED);
			/* In general case, menu hrtimer is started */
			per_cpu(hrtimer_status, cpu) = MENU_HRTIMER_GENERAL;
		}

	}

	return data->last_state_idx;
}

static void menu_reflect(struct cpuidle_device *dev, int index)
{
	struct menu_device *data = &__get_cpu_var(menu_devices);
	data->last_state_idx = index;
	if (index >= 0)
		data->needs_update = 1;
}

static void menu_update(struct cpuidle_driver *drv, struct cpuidle_device *dev)
{
	struct menu_device *data = &__get_cpu_var(menu_devices);
	int last_idx = data->last_state_idx;
	unsigned int last_idle_us = cpuidle_get_last_residency(dev);
	struct cpuidle_state *target = &drv->states[last_idx];
	unsigned int measured_us;
	u64 new_factor;

	if (unlikely(!(target->flags & CPUIDLE_FLAG_TIME_VALID)))
		last_idle_us = data->expected_us;


	measured_us = last_idle_us;

	if (measured_us > data->exit_us)
		measured_us -= data->exit_us;


	

	new_factor = data->correction_factor[data->bucket]
			* (DECAY - 1) / DECAY;

	if (data->expected_us > 0 && measured_us < MAX_INTERESTING)
		new_factor += RESOLUTION * measured_us / data->expected_us;
	else
		new_factor += RESOLUTION;

	if (new_factor == 0)
		new_factor = 1;

	data->correction_factor[data->bucket] = new_factor;

	
	data->intervals[data->interval_ptr++] = last_idle_us;
	if (data->interval_ptr >= INTERVALS)
		data->interval_ptr = 0;
}

static int menu_enable_device(struct cpuidle_driver *drv,
				struct cpuidle_device *dev)
{
	struct menu_device *data = &per_cpu(menu_devices, dev->cpu);
	struct hrtimer *t = &per_cpu(menu_hrtimer, dev->cpu);
	hrtimer_init(t, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	t->function = menu_hrtimer_notify;

	memset(data, 0, sizeof(struct menu_device));

	return 0;
}

static struct cpuidle_governor menu_governor = {
	.name =		"menu",
	.rating =	20,
	.enable =	menu_enable_device,
	.select =	menu_select,
	.reflect =	menu_reflect,
	.owner =	THIS_MODULE,
};

static int __init init_menu(void)
{
	return cpuidle_register_governor(&menu_governor);
}

static void __exit exit_menu(void)
{
	cpuidle_unregister_governor(&menu_governor);
}

MODULE_LICENSE("GPL");
module_init(init_menu);
module_exit(exit_menu);
