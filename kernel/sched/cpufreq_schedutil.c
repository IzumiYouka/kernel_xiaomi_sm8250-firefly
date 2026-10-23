/*
 * CPUFreq governor based on scheduler-provided CPU utilization data.
 *
 * Copyright (C) 2016, Intel Corporation
 * Author: Rafael J. Wysocki <rafael.j.wysocki@intel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include "sched.h"
#include "sched-pelt.h"

#include <linux/sched/cpufreq.h>
#include <trace/events/power.h>
#include <linux/sched/sysctl.h>

DEFINE_PER_CPU_READ_MOSTLY(unsigned long, response_time_mult);

struct sugov_tunables {
	struct gov_attr_set	attr_set;
	unsigned int		rate_limit_us;
	unsigned int		response_time_ms;
};

struct sugov_policy {
	struct cpufreq_policy	*policy;

	struct sugov_tunables	*tunables;
	struct list_head	tunables_hook;

	raw_spinlock_t		update_lock;	/* For shared policies */
	u64			last_freq_update_time;
	s64			freq_update_delay_ns;
	unsigned int		freq_response_time_ms;
	unsigned int		next_freq;
	unsigned int		cached_raw_freq;

	/* The next fields are only needed if fast switch cannot be used: */
	struct			irq_work irq_work;
	struct			kthread_work work;
	struct			mutex work_lock;
	struct			kthread_worker worker;
	struct task_struct	*thread;
	bool			work_in_progress;

	bool			limits_changed;
	bool			need_freq_update;

	/*
	 * DVFS headroom lookup table, indexed by absolute utilization
	 * (0..SCHED_CAPACITY_SCALE) and holding the headroom to add for this
	 * policy's nominal CPU capacity.
	 *
	 * Built once in sugov_start() before any update-util hook is published,
	 * and never rebuilt: the delay it is derived from is a compile-time
	 * constant, so no writer can race the lock-free readers.
	 */
	u16			dvfs_headroom_lut[SCHED_CAPACITY_SCALE + 1];
};

struct sugov_cpu {
	struct update_util_data	update_util;
	struct sugov_policy	*sg_policy;
	unsigned int		cpu;

	u64			last_update;
	u64			last_iowait_update;

	unsigned int		iowait_boost;
	bool			iowait_boost_pending;

	unsigned long util;

	unsigned long		bw_min;
	unsigned long		util_max;
	unsigned long		min;
	unsigned long		max;
};

static DEFINE_PER_CPU(struct sugov_cpu, sugov_cpu);
static DEFINE_PER_CPU(struct sugov_tunables *, cached_tunables);

/************************ Governor internals ***********************/

#define IOWAIT_BOOST_MIN	(SCHED_CAPACITY_SCALE / 8)

/*
 * The IO wait boost is reset after a tick without any IO wakeup on this CPU.
 *
 * The stamp must be the last *iowait-flagged* update, not sg_cpu->last_update:
 * the latter is refreshed by every update callback, including the frequent
 * cpufreq_update_util(rq, 0) calls that __update_blocked_averages() issues
 * while decaying blocked load. Keying the reset off last_update therefore
 * never observes a quiet tick, and the boost ratchets up to the capacity
 * ceiling and stays there.
 */
static bool sugov_iowait_reset(struct sugov_cpu *sg_cpu, u64 time,
			       bool set_iowait_boost)
{
	s64 delta_ns = time - sg_cpu->last_iowait_update;

	/* Reset boost only if a tick has elapsed since last request */
	if (delta_ns <= TICK_NSEC)
		return false;

	sg_cpu->iowait_boost = set_iowait_boost ? IOWAIT_BOOST_MIN : 0;
	sg_cpu->iowait_boost_pending = set_iowait_boost;

	return true;
}

/**
 * sugov_iowait_boost - Updates the IO boost status of a CPU
 * @sg_cpu: the sugov data for the CPU to boost
 * @time: the update time from the caller
 * @flags: SCHED_CPUFREQ_IOWAIT if the task is waking up after an IO wait
 *
 * Each time a task wakes up after an IO operation, the CPU utilization can be
 * boosted to a certain utilization which doubles at each "frequent and
 * successive" wakeup from IO, ranging from IOWAIT_BOOST_MIN to the utilization
 * of the maximum OPP.
 *
 * To keep doubling, an IO boost has to be requested at least once per tick,
 * otherwise we restart from the utilization of the minimum OPP.
 */
static void sugov_iowait_boost(struct sugov_cpu *sg_cpu, u64 time,
			       unsigned int flags)
{
	bool set_iowait_boost = flags & SCHED_CPUFREQ_IOWAIT;

	/* Reset boost if the CPU appears to have been idle enough */
	if (sg_cpu->iowait_boost &&
	    sugov_iowait_reset(sg_cpu, time, set_iowait_boost))
		return;

	/* Boost only tasks waking up after IO */
	if (!set_iowait_boost)
		return;

	/* Record the last IO wakeup for the next reset decision */
	sg_cpu->last_iowait_update = time;

	/* Ensure boost doubles only one time at each request */
	if (sg_cpu->iowait_boost_pending)
		return;
	sg_cpu->iowait_boost_pending = true;

	/* Double the boost at each request */
	if (sg_cpu->iowait_boost) {
		sg_cpu->iowait_boost =
			min_t(unsigned int, sg_cpu->iowait_boost << 1,
			      SCHED_CAPACITY_SCALE);
		return;
	}

	/* First wakeup after IO: start with minimum boost */
	sg_cpu->iowait_boost = IOWAIT_BOOST_MIN;
}

/**
 * sugov_iowait_apply - Apply the IO boost to a CPU
 * @sg_cpu: the sugov data for the CPU to boost
 * @time: the update time from the caller
 * @max_cap: the max CPU capacity
 *
 * A CPU running a task which woken up after an IO operation can have its
 * utilization boosted to speed up the completion of those IO operations.
 * The IO boost value is increased in sugov_iowait_boost(), and it's instead
 * decreased by this function, each time an increase has not been requested
 * (!iowait_boost_pending).
 *
 * A CPU which also appears to have been idle for at least one tick has also
 * its IO boost utilization reset.
 */
static unsigned long sugov_iowait_apply(struct sugov_cpu *sg_cpu, u64 time,
					unsigned long max_cap)
{
	/* No boost currently required */
	if (!sg_cpu->iowait_boost)
		return 0;

	/* Reset boost if the CPU appears to have been idle enough */
	if (sugov_iowait_reset(sg_cpu, time, false))
		return 0;

	if (!sg_cpu->iowait_boost_pending) {
		/*
		 * No boost pending; reduce the boost value.
		 */
		sg_cpu->iowait_boost >>= 1;
		if (sg_cpu->iowait_boost < IOWAIT_BOOST_MIN) {
			sg_cpu->iowait_boost = 0;
			return 0;
		}
	}

	sg_cpu->iowait_boost_pending = false;

	/*
	 * sg_cpu->util is already in capacity scale; convert iowait_boost
	 * into the same scale so we can compare.
	 */
	return (sg_cpu->iowait_boost * max_cap) >> SCHED_CAPACITY_SHIFT;
}

/*
 * Vendor scripts like to disable rate limiting outright by writing 0,
 * which lets the frequency flap on every utilization dip.  Keep a small
 * floor; with frequency-invariant utilization the up direction already
 * ignores the rate limit, so this effectively only bounds reductions.
 */
#define SUGOV_RATE_LIMIT_MIN_US	2000U

static bool sugov_should_update_freq(struct sugov_policy *sg_policy, u64 time)
{
	s64 delta_ns;

	/*
	 * Since cpufreq_update_util() is called with rq->lock held for
	 * the @target_cpu, our per-CPU data is fully serialized.
	 *
	 * However, drivers cannot in general deal with cross-CPU
	 * requests, so while get_next_freq() will work, our
	 * sugov_update_commit() call may not for the fast switching platforms.
	 *
	 * Hence stop here for remote requests if they aren't supported
	 * by the hardware, as calculating the frequency is pointless if
	 * we cannot in fact act on it.
	 *
	 * This is needed on the slow switching platforms too to prevent CPUs
	 * going offline from leaving stale IRQ work items behind.
	 */
	if (!cpufreq_this_cpu_can_update(sg_policy->policy))
		return false;

	if (unlikely(READ_ONCE(sg_policy->limits_changed))) {
		WRITE_ONCE(sg_policy->limits_changed, false);
		sg_policy->need_freq_update = true;

		/*
		 * The above limits_changed update must occur before the reads
		 * of policy limits in cpufreq_driver_resolve_freq() or a policy
		 * limits update might be missed, so use a memory barrier to
		 * ensure it.
		 *
		 * This pairs with the write memory barrier in sugov_limits().
		 */
		smp_mb();

		return true;
	} else if (sg_policy->need_freq_update) {
		/* ignore_dl_rate_limit() wants a new frequency to be found. */
		return true;
	}

	/*
	 * When frequency-invariant utilization tracking is present, there's no
	 * rate limit when increasing frequency. Therefore, the next frequency
	 * must be determined before a decision can be made to rate limit the
	 * frequency change, hence the rate limit check is bypassed here.
	 */
	if (arch_scale_freq_invariant())
		return true;

	delta_ns = time - sg_policy->last_freq_update_time;
	return delta_ns >= sg_policy->freq_update_delay_ns;
}

static bool sugov_update_next_freq(struct sugov_policy *sg_policy, u64 time,
				   unsigned int next_freq)
{
	if (sg_policy->next_freq == next_freq)
		return false;

	sg_policy->next_freq = next_freq;
	sg_policy->last_freq_update_time = time;

	return true;
}

static void sugov_fast_switch(struct sugov_policy *sg_policy, u64 time,
			      unsigned int next_freq)
{
	struct cpufreq_policy *policy = sg_policy->policy;
	unsigned int cpu;

	if (!sugov_update_next_freq(sg_policy, time, next_freq))
		return;

	next_freq = cpufreq_driver_fast_switch(policy, next_freq);
	if (!next_freq)
		return;

	policy->cur = next_freq;

	if (trace_cpu_frequency_enabled()) {
		for_each_cpu(cpu, policy->cpus)
			trace_cpu_frequency(next_freq, cpu);
	}
}

static void sugov_deferred_update(struct sugov_policy *sg_policy, u64 time,
				  unsigned int next_freq)
{
	if (!sugov_update_next_freq(sg_policy, time, next_freq))
		return;

	sg_policy->work_in_progress = true;
	irq_work_queue(&sg_policy->irq_work);
}

/**
 * get_next_freq - Compute a new frequency for a given cpufreq policy.
 * @sg_policy: schedutil policy object to compute the new frequency for.
 * @util: Current CPU utilization.
 * @max: CPU capacity.
 *
 * If the utilization is frequency-invariant, choose the new frequency to be
 * proportional to it, that is
 *
 * next_freq = C * max_freq * util / max
 *
 * Otherwise, approximate the would-be frequency-invariant utilization by
 * util_raw * (curr_freq / max_freq) which leads to
 *
 * next_freq = C * curr_freq * util_raw / max
 *
 * Take C = 1.25 for the frequency tipping point at (util / max) = 0.8.
 *
 * The lowest driver-supported frequency which is equal or greater than the raw
 * next_freq (as calculated above) is returned, subject to policy min/max and
 * cpufreq driver limitations.
 */
static unsigned int get_next_freq(struct sugov_policy *sg_policy,
				  unsigned long util, unsigned long max)
{
	struct cpufreq_policy *policy = sg_policy->policy;
	unsigned int freq = arch_scale_freq_invariant() ?
				policy->cpuinfo.max_freq : policy->cur;

	freq = freq * util / max;
	trace_sugov_next_freq(policy->cpu, util, max, freq);

	if (freq == sg_policy->cached_raw_freq && !sg_policy->need_freq_update)
		return sg_policy->next_freq;

	sg_policy->need_freq_update = false;
	sg_policy->cached_raw_freq = freq;
	return cpufreq_driver_resolve_freq(policy, freq);
}

static unsigned int sugov_second_max_freq(struct cpufreq_policy *policy)
{
	unsigned int max_freq = policy->cpuinfo.max_freq;
	unsigned int sec_max_freq = 0;
	int i;

	for (i = 0; policy->freq_table[i].frequency != CPUFREQ_TABLE_END; i++) {
		unsigned int freq = policy->freq_table[i].frequency;

		if (freq == CPUFREQ_ENTRY_INVALID || freq >= max_freq)
			continue;
		if (freq > sec_max_freq)
			sec_max_freq = freq;
	}

	return sec_max_freq ? sec_max_freq : max_freq;
}

static inline u64 sugov_calc_freq_response_ms(struct sugov_policy *sg_policy)
{
	int cpu = cpumask_first(sg_policy->policy->cpus);
	unsigned long cap = arch_scale_cpu_capacity(cpu);
	unsigned int max_freq = sg_policy->policy->cpuinfo.max_freq;
	unsigned int sec_max_freq;

	sec_max_freq = sugov_second_max_freq(sg_policy->policy);

	/*
	 * We will request max_freq as soon as util crosses the capacity at
	 * the second highest frequency, so the response time is the time
	 * needed to ramp util up to that point.
	 */
	cap = sec_max_freq * cap / max_freq;

	return approximate_runtime(cap + 1);
}

static inline void sugov_update_response_time_mult(struct sugov_policy *sg_policy)
{
	unsigned long mult;
	unsigned int response_time_ms;
	int cpu;

	if (unlikely(!sg_policy->freq_response_time_ms))
		sg_policy->freq_response_time_ms = sugov_calc_freq_response_ms(sg_policy);

	/*
	 * Zero means "no user scaling": fall back to each policy's own ramp
	 * time. Tunables are shared between policies on this driver, so
	 * storing a concrete default here would scale every other cluster's
	 * utilization by the first policy's ramp time.
	 */
	response_time_ms = sg_policy->tunables->response_time_ms;
	if (!response_time_ms)
		response_time_ms = sg_policy->freq_response_time_ms;

	mult = sg_policy->freq_response_time_ms * SCHED_CAPACITY_SCALE;
	mult /= response_time_ms;

	if (SCHED_WARN_ON(!mult))
		mult = SCHED_CAPACITY_SCALE;

	for_each_cpu(cpu, sg_policy->policy->cpus)
		per_cpu(response_time_mult, cpu) = mult;
}

/*
 * Shrink or expand how long it takes to reach the maximum performance point
 * of the policy. freq_response_time_ms is fixed by PELT HALFLIFE and policy
 * capacity; response_time_ms is user tunable. Lowering it speeds frequency
 * ramp up, raising it slows the ramp down and saves power.
 */
static inline unsigned long
sugov_apply_response_time(unsigned long util, int cpu)
{
	unsigned long mult = per_cpu(response_time_mult, cpu);

	if (unlikely(!mult))
		return util;

	return mult * util >> SCHED_CAPACITY_SHIFT;
}

static unsigned long calc_dvfs_headroom(unsigned long util, unsigned long cap,
					unsigned long h_max, unsigned long approx);
static void sugov_build_dvfs_headroom_lut(struct sugov_policy *sg_policy);

/**
 * sugov_apply_dvfs_headroom - Add DVFS headroom to a utilization value
 * @util: Utilization to add headroom to
 * @cpu: CPU the utilization belongs to
 *
 * Applies the per-policy DVFS headroom lookup table and the optional
 * runtime limits controlled through sched_hr_limit_level.
 */
static inline unsigned long sugov_apply_dvfs_headroom(unsigned long util, int cpu)
{
	struct sugov_cpu *sg_cpu = &per_cpu(sugov_cpu, cpu);
	struct sugov_policy *sg_policy = sg_cpu->sg_policy;
	unsigned long headroom, cap;
	unsigned int limit_level, scale;
	unsigned long h_max, approx;

	if (!util)
		return 0;

	if (util > SCHED_CAPACITY_SCALE)
		util = SCHED_CAPACITY_SCALE;

	/*
	 * Use the static CPU capacity: this helper is also called from the
	 * EAS energy estimation, which must not depend on a previous
	 * governor update having set sg_cpu->max.
	 */
	cap = arch_scale_cpu_capacity(cpu);

	h_max = sg_policy->dvfs_headroom_lut[0];
	approx = sg_policy->dvfs_headroom_lut[util];

	headroom = calc_dvfs_headroom(util, cap, h_max, approx);

	/* User scaling, in percent of the PELT-growth ideal */
	scale = READ_ONCE(sysctl_sched_hr_scale);
	if (scale != 100)
		headroom = headroom * scale / 100;

	/* Safety cap: never add more than 25% of the original capacity */
	if (headroom > cap / 4)
		headroom = cap / 4;

	limit_level = READ_ONCE(sysctl_sched_hr_limit_level);
	if (limit_level) {
		headroom = min(headroom, cap * 20 / 100);
		if (limit_level == 2)
			headroom = min(headroom,
				       util * 768 >> SCHED_CAPACITY_SHIFT);
	}

	util += headroom;
	if (util > cap)
		util = cap;

	return util;
}

unsigned long sugov_effective_cpu_perf(int cpu, unsigned long actual,
					unsigned long min,
					unsigned long max)
{
	/* Scale response time, then add DVFS headroom */
	actual = sugov_apply_response_time(actual, cpu);
	actual = sugov_apply_dvfs_headroom(actual, cpu);

	/*
	 * Ensure at least minimum performance while providing more compute
	 * capacity when possible, clamped to the maximum allowed.
	 */
	if (unlikely(min >= max))
		return min;

	return clamp(actual, min, max);
}

static unsigned long sugov_get_util(struct sugov_cpu *sg_cpu,
				    unsigned long boost)
{
	struct rq *rq = cpu_rq(sg_cpu->cpu);
	unsigned long util_cfs = cpu_util_cfs(rq);
	unsigned long min, max;

	sg_cpu->max = arch_scale_cpu_capacity(sg_cpu->cpu);

	util_cfs = effective_cpu_util(sg_cpu->cpu, util_cfs, &min, &max);
	sg_cpu->bw_min = min;
	sg_cpu->util_max = max;

	return max(util_cfs, boost);
}

/*
 * Make sugov_should_update_freq() ignore the rate limit when DL
 * has increased the utilization.
 */
static inline void ignore_dl_rate_limit(struct sugov_cpu *sg_cpu, struct sugov_policy *sg_policy)
{
	if (cpu_bw_dl(cpu_rq(sg_cpu->cpu)) > sg_cpu->bw_min)
		sg_policy->need_freq_update = true;
}

static void sugov_update_single(struct update_util_data *hook, u64 time,
				unsigned int flags)
{
	struct sugov_cpu *sg_cpu = container_of(hook, struct sugov_cpu, update_util);
	struct sugov_policy *sg_policy = sg_cpu->sg_policy;
	unsigned long util, max, boost;
	unsigned int next_f;

	sugov_iowait_boost(sg_cpu, time, flags);
	sg_cpu->last_update = time;

	ignore_dl_rate_limit(sg_cpu, sg_policy);

	if (!sugov_should_update_freq(sg_policy, time))
		return;

	max = sg_cpu->max;
	boost = sugov_iowait_apply(sg_cpu, time, max);
	sg_cpu->util = util = sugov_get_util(sg_cpu, boost);

	util = sugov_effective_cpu_perf(sg_cpu->cpu, util,
					sg_cpu->bw_min, sg_cpu->util_max);
	next_f = get_next_freq(sg_policy, util, max);

	/*
	 * This code runs under rq->lock for the target CPU, so it won't run
	 * concurrently on two different CPUs for the same target and it is not
	 * necessary to acquire the lock in the fast switch case.
	 */
	if (sg_policy->policy->fast_switch_enabled) {
		sugov_fast_switch(sg_policy, time, next_f);
	} else {
		raw_spin_lock(&sg_policy->update_lock);
		sugov_deferred_update(sg_policy, time, next_f);
		raw_spin_unlock(&sg_policy->update_lock);
	}
}

static unsigned int sugov_next_freq_shared(struct sugov_cpu *sg_cpu, u64 time)
{
	struct sugov_policy *sg_policy = sg_cpu->sg_policy;
	struct cpufreq_policy *policy = sg_policy->policy;
	unsigned long util = 0, max = 1;
	unsigned int j;

	/*
	 * The iowait boost is expressed as a fraction of the policy's max
	 * capacity, so scale it with the policy's own capacity rather than
	 * with each sibling's, matching how it is applied on the single path.
	 */
	for_each_cpu(j, policy->cpus) {
		struct sugov_cpu *j_sg_cpu = &per_cpu(sugov_cpu, j);
		unsigned long j_util, j_max, boost;

		boost = sugov_iowait_apply(j_sg_cpu, time,
					   arch_scale_cpu_capacity(j));
		j_util = sugov_get_util(j_sg_cpu, boost);
		j_max = j_sg_cpu->max;
		j_util = sugov_effective_cpu_perf(j_sg_cpu->cpu, j_util,
						  j_sg_cpu->bw_min,
						  j_sg_cpu->util_max);

		if (j_util * max >= j_max * util) {
			util = j_util;
			max = j_max;
		}
	}

	return get_next_freq(sg_policy, util, max);
}

static void
sugov_update_shared(struct update_util_data *hook, u64 time, unsigned int flags)
{
	struct sugov_cpu *sg_cpu = container_of(hook, struct sugov_cpu, update_util);
	struct sugov_policy *sg_policy = sg_cpu->sg_policy;
	unsigned int next_f;

	raw_spin_lock(&sg_policy->update_lock);

	sugov_iowait_boost(sg_cpu, time, flags);
	sg_cpu->last_update = time;

	ignore_dl_rate_limit(sg_cpu, sg_policy);

	if (sugov_should_update_freq(sg_policy, time) &&
	    !(flags & SCHED_CPUFREQ_CONTINUE)) {
		next_f = sugov_next_freq_shared(sg_cpu, time);

		if (sg_policy->policy->fast_switch_enabled)
			sugov_fast_switch(sg_policy, time, next_f);
		else
			sugov_deferred_update(sg_policy, time, next_f);
	}

	raw_spin_unlock(&sg_policy->update_lock);
}

static void sugov_work(struct kthread_work *work)
{
	struct sugov_policy *sg_policy = container_of(work, struct sugov_policy, work);
	unsigned int freq;
	unsigned long flags;

	/*
	 * Hold sg_policy->update_lock shortly to handle the case where:
	 * incase sg_policy->next_freq is read here, and then updated by
	 * sugov_deferred_update() just before work_in_progress is set to false
	 * here, we may miss queueing the new update.
	 *
	 * Note: If a work was queued after the update_lock is released,
	 * sugov_work() will just be called again by kthread_work code; and the
	 * request will be proceed before the sugov thread sleeps.
	 */
	raw_spin_lock_irqsave(&sg_policy->update_lock, flags);
	freq = sg_policy->next_freq;
	sg_policy->work_in_progress = false;
	raw_spin_unlock_irqrestore(&sg_policy->update_lock, flags);

	mutex_lock(&sg_policy->work_lock);
	__cpufreq_driver_target(sg_policy->policy, freq, CPUFREQ_RELATION_L);
	mutex_unlock(&sg_policy->work_lock);
}

static void sugov_irq_work(struct irq_work *irq_work)
{
	struct sugov_policy *sg_policy;

	sg_policy = container_of(irq_work, struct sugov_policy, irq_work);

	kthread_queue_work(&sg_policy->worker, &sg_policy->work);
}

/************************** sysfs interface ************************/

static struct sugov_tunables *global_tunables;
static DEFINE_MUTEX(global_tunables_lock);

static inline struct sugov_tunables *to_sugov_tunables(struct gov_attr_set *attr_set)
{
	return container_of(attr_set, struct sugov_tunables, attr_set);
}

static ssize_t rate_limit_us_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->rate_limit_us);
}

static ssize_t rate_limit_us_store(struct gov_attr_set *attr_set,
				   const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	struct sugov_policy *sg_policy;
	unsigned int rate_limit_us;

	if (kstrtouint(buf, 10, &rate_limit_us))
		return -EINVAL;

	rate_limit_us = max(rate_limit_us, SUGOV_RATE_LIMIT_MIN_US);
	tunables->rate_limit_us = rate_limit_us;

	list_for_each_entry(sg_policy, &attr_set->policy_list, tunables_hook)
		sg_policy->freq_update_delay_ns = rate_limit_us * NSEC_PER_USEC;

	return count;
}

static struct governor_attr rate_limit_us = __ATTR_RW(rate_limit_us);

static ssize_t response_time_ms_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->response_time_ms);
}

static ssize_t
response_time_ms_store(struct gov_attr_set *attr_set, const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	struct sugov_policy *sg_policy;
	unsigned int response_time_ms;

	if (kstrtouint(buf, 10, &response_time_ms) || !response_time_ms)
		return -EINVAL;

	tunables->response_time_ms = response_time_ms;

	list_for_each_entry(sg_policy, &attr_set->policy_list, tunables_hook) {
		if (sg_policy->tunables == tunables) {
			sugov_update_response_time_mult(sg_policy);
			break;
		}
	}

	return count;
}

static struct governor_attr response_time_ms = __ATTR_RW(response_time_ms);

static struct attribute *sugov_attributes[] = {
	&rate_limit_us.attr,
	&response_time_ms.attr,
	NULL
};

static void sugov_tunables_free(struct kobject *kobj)
{
	struct gov_attr_set *attr_set = container_of(kobj, struct gov_attr_set, kobj);

	kfree(to_sugov_tunables(attr_set));
}

static struct kobj_type sugov_tunables_ktype = {
	.default_attrs = sugov_attributes,
	.sysfs_ops = &governor_sysfs_ops,
	.release = &sugov_tunables_free,
};

/********************** cpufreq governor interface *********************/

static struct cpufreq_governor schedutil_gov;

/*
 * DVFS decisions are made at discrete points. If the CPU stays busy, its
 * utilization keeps growing, so it may need to run at a higher frequency
 * before the next decision point is reached. The headroom below caters for
 * that delay and is precomputed per policy so the hot path performs a
 * single table lookup.
 */

/*
 * DVFS headroom from PELT growth.
 *
 * h_max is the utilization a fully busy CPU can gain during one governor
 * delay window, and approx is where a CPU sitting at @util will be after
 * that same window. Both come straight out of
 * accumulate_sum()/approximate_util_avg() in pelt.c, so the curve follows
 * the configured PELT half-life and the EAS ramp-up path instead of being a
 * hand-tuned polynomial.
 *
 * Let g = approx - util be the growth this CPU can still undergo during the
 * window. The headroom is that growth, capped at the fraction of capacity a
 * full window could consume:
 *
 *   ceiling = C * h_max / 1024        (= C * alpha)
 *   H       = min(g, ceiling)
 *
 * Scaling the ceiling with C rather than subtracting a constant penalty
 * keeps small cores usable: a fixed h_max * (1024 - C) / 1024 term is
 * proportionally far harsher on a 253-capacity silver than on the 1024
 * prime, which used to starve them of headroom entirely.
 */
static unsigned long calc_dvfs_headroom(unsigned long util, unsigned long cap,
					unsigned long h_max, unsigned long approx)
{
	unsigned long growth, ceiling;
	s64 g;

	g = (s64)approx - (s64)util;
	if (g <= 0)
		return 0;

	growth = (unsigned long)g;
	ceiling = (u64)h_max * cap >> SCHED_CAPACITY_SHIFT;

	return min(growth, ceiling);
}

static void sugov_build_dvfs_headroom_lut(struct sugov_policy *sg_policy)
{
	unsigned long util;
	u64 delay;

	/*
	 * The utilization can only be re-sampled at the governor rate limit
	 * or at the next tick, whichever comes later. The rate limit the table
	 * is derived from is taken at sugov_start(), when the tunable still
	 * holds the driver's default; a rate_limit_us written later only
	 * affects the governor's decision points, not the table.
	 */
	delay = max_t(u64, sg_policy->tunables->rate_limit_us, TICK_USEC);

	for (util = 0; util <= SCHED_CAPACITY_SCALE; util++)
		sg_policy->dvfs_headroom_lut[util] =
			(u16)approximate_util_avg(util, delay);
}

static struct sugov_policy *sugov_policy_alloc(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy;

	sg_policy = kzalloc(sizeof(*sg_policy), GFP_KERNEL);
	if (!sg_policy)
		return NULL;

	sg_policy->policy = policy;
	raw_spin_lock_init(&sg_policy->update_lock);
	return sg_policy;
}

static void sugov_policy_free(struct sugov_policy *sg_policy)
{
	kfree(sg_policy);
}

static int sugov_kthread_create(struct sugov_policy *sg_policy)
{
	struct task_struct *thread;
	struct sched_param param = { .sched_priority = MAX_USER_RT_PRIO / 2 };
	struct cpufreq_policy *policy = sg_policy->policy;
	int ret;

	/* kthread only required for slow path */
	if (policy->fast_switch_enabled)
		return 0;

	kthread_init_work(&sg_policy->work, sugov_work);
	kthread_init_worker(&sg_policy->worker);
	thread = kthread_create(kthread_worker_fn, &sg_policy->worker,
				"sugov:%d",
				cpumask_first(policy->related_cpus));
	if (IS_ERR(thread)) {
		pr_err("failed to create sugov thread: %ld\n", PTR_ERR(thread));
		return PTR_ERR(thread);
	}

	ret = sched_setscheduler_nocheck(thread, SCHED_FIFO, &param);
	if (ret) {
		kthread_stop(thread);
		pr_warn("%s: failed to set SCHED_FIFO\n", __func__);
		return ret;
	}

	sg_policy->thread = thread;
	kthread_bind_mask(thread, policy->related_cpus);
	init_irq_work(&sg_policy->irq_work, sugov_irq_work);
	mutex_init(&sg_policy->work_lock);

	wake_up_process(thread);

	return 0;
}

static void sugov_kthread_stop(struct sugov_policy *sg_policy)
{
	/* kthread only required for slow path */
	if (sg_policy->policy->fast_switch_enabled)
		return;

	kthread_flush_worker(&sg_policy->worker);
	kthread_stop(sg_policy->thread);
	mutex_destroy(&sg_policy->work_lock);
}

static struct sugov_tunables *sugov_tunables_alloc(struct sugov_policy *sg_policy)
{
	struct sugov_tunables *tunables;

	tunables = kzalloc(sizeof(*tunables), GFP_KERNEL);
	if (tunables) {
		gov_attr_set_init(&tunables->attr_set, &sg_policy->tunables_hook);
		if (!have_governor_per_policy())
			global_tunables = tunables;
	}
	return tunables;
}

static void sugov_tunables_save(struct cpufreq_policy *policy,
		struct sugov_tunables *tunables)
{
	int cpu;
	struct sugov_tunables *cached = per_cpu(cached_tunables, policy->cpu);

	if (!have_governor_per_policy())
		return;

	if (!cached) {
		cached = kzalloc(sizeof(*tunables), GFP_KERNEL);
		if (!cached)
			return;

		for_each_cpu(cpu, policy->related_cpus)
			per_cpu(cached_tunables, cpu) = cached;
	}

	cached->rate_limit_us = tunables->rate_limit_us;
	cached->response_time_ms = tunables->response_time_ms;
}

static void sugov_clear_global_tunables(void)
{
	if (!have_governor_per_policy())
		global_tunables = NULL;
}

static void sugov_tunables_restore(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy = policy->governor_data;
	struct sugov_tunables *tunables = sg_policy->tunables;
	struct sugov_tunables *cached = per_cpu(cached_tunables, policy->cpu);

	if (!cached)
		return;

	tunables->rate_limit_us = cached->rate_limit_us;
	tunables->response_time_ms = cached->response_time_ms;
	sugov_update_response_time_mult(sg_policy);
}

static int sugov_init(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy;
	struct sugov_tunables *tunables;
	int ret = 0;

	/* State should be equivalent to EXIT */
	if (policy->governor_data)
		return -EBUSY;

	cpufreq_enable_fast_switch(policy);

	sg_policy = sugov_policy_alloc(policy);
	if (!sg_policy) {
		ret = -ENOMEM;
		goto disable_fast_switch;
	}

	ret = sugov_kthread_create(sg_policy);
	if (ret)
		goto free_sg_policy;

	mutex_lock(&global_tunables_lock);

	if (global_tunables) {
		if (WARN_ON(have_governor_per_policy())) {
			ret = -EINVAL;
			goto stop_kthread;
		}
		policy->governor_data = sg_policy;
		sg_policy->tunables = global_tunables;

		gov_attr_set_get(&global_tunables->attr_set, &sg_policy->tunables_hook);
		sugov_update_response_time_mult(sg_policy);
		goto out;
	}

	tunables = sugov_tunables_alloc(sg_policy);
	if (!tunables) {
		ret = -ENOMEM;
		goto stop_kthread;
	}

	/*
	 * Keep the default short so bursty workloads ramp up quickly; with
	 * frequency-invariant utilization the up direction ignores the limit
	 * entirely anyway, so this mostly bounds the down direction.
	 */
	tunables->rate_limit_us = 2000;

	policy->governor_data = sg_policy;
	sg_policy->tunables = tunables;

	sugov_tunables_restore(policy);
	sugov_update_response_time_mult(sg_policy);

	ret = kobject_init_and_add(&tunables->attr_set.kobj, &sugov_tunables_ktype,
				   get_governor_parent_kobj(policy), "%s",
				   schedutil_gov.name);
	if (ret)
		goto fail;

out:
	mutex_unlock(&global_tunables_lock);
	return 0;

fail:
	kobject_put(&tunables->attr_set.kobj);
	policy->governor_data = NULL;
	sugov_clear_global_tunables();

stop_kthread:
	sugov_kthread_stop(sg_policy);
	mutex_unlock(&global_tunables_lock);

free_sg_policy:
	sugov_policy_free(sg_policy);

disable_fast_switch:
	cpufreq_disable_fast_switch(policy);

	pr_err("initialization failed (error %d)\n", ret);
	return ret;
}

static void sugov_exit(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy = policy->governor_data;
	struct sugov_tunables *tunables = sg_policy->tunables;
	unsigned int count;

	mutex_lock(&global_tunables_lock);

	count = gov_attr_set_put(&tunables->attr_set, &sg_policy->tunables_hook);
	policy->governor_data = NULL;
	if (!count) {
		sugov_tunables_save(policy, tunables);
		sugov_clear_global_tunables();
	}

	mutex_unlock(&global_tunables_lock);

	sugov_kthread_stop(sg_policy);
	sugov_policy_free(sg_policy);
	cpufreq_disable_fast_switch(policy);
}

static int sugov_start(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy = policy->governor_data;
	unsigned int cpu;

	sg_policy->freq_update_delay_ns =
		sg_policy->tunables->rate_limit_us * NSEC_PER_USEC;
	sg_policy->last_freq_update_time	= 0;
	sg_policy->next_freq			= 0;
	sg_policy->work_in_progress		= false;
	sg_policy->limits_changed		= false;
	sg_policy->need_freq_update		= false;
	sg_policy->cached_raw_freq		= 0;

	/* Build before publishing any update-util hook that may read it. */
	sugov_build_dvfs_headroom_lut(sg_policy);

	for_each_cpu(cpu, policy->cpus) {
		struct sugov_cpu *sg_cpu = &per_cpu(sugov_cpu, cpu);

		memset(sg_cpu, 0, sizeof(*sg_cpu));
		sg_cpu->cpu			= cpu;
		sg_cpu->sg_policy		= sg_policy;
		sg_cpu->min			=
			(SCHED_CAPACITY_SCALE * policy->cpuinfo.min_freq) /
			policy->cpuinfo.max_freq;
	}

	for_each_cpu(cpu, policy->cpus) {
		struct sugov_cpu *sg_cpu = &per_cpu(sugov_cpu, cpu);

		cpufreq_add_update_util_hook(cpu, &sg_cpu->update_util,
					     policy_is_shared(policy) ?
							sugov_update_shared :
							sugov_update_single);
	}
	return 0;
}

static void sugov_stop(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy = policy->governor_data;
	unsigned int cpu;

	for_each_cpu(cpu, policy->cpus)
		cpufreq_remove_update_util_hook(cpu);

	synchronize_sched();

	if (!policy->fast_switch_enabled) {
		irq_work_sync(&sg_policy->irq_work);
		kthread_cancel_work_sync(&sg_policy->work);
	}
}

static void sugov_limits(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy = policy->governor_data;
	unsigned long flags, now;
	unsigned int freq;

	if (!policy->fast_switch_enabled) {
		mutex_lock(&sg_policy->work_lock);
		raw_spin_lock_irqsave(&sg_policy->update_lock, flags);
		raw_spin_unlock_irqrestore(&sg_policy->update_lock, flags);
		cpufreq_policy_apply_limits(policy);
		mutex_unlock(&sg_policy->work_lock);
	} else {
		raw_spin_lock_irqsave(&sg_policy->update_lock, flags);
		freq = policy->cur;
		now = ktime_get_ns();

		/*
		 * cpufreq_driver_resolve_freq() has a clamp, so we do not need
		 * to do any sort of additional validation here.
		 */
		freq = cpufreq_driver_resolve_freq(policy, freq);
		sg_policy->cached_raw_freq = freq;
		sugov_fast_switch(sg_policy, now, freq);
		raw_spin_unlock_irqrestore(&sg_policy->update_lock, flags);
	}

	/*
	 * The limits_changed update below must take place before the updates
	 * of policy limits in cpufreq_set_policy() or a policy limits update
	 * might be missed, so use a memory barrier to ensure it.
	 *
	 * This pairs with the memory barrier in sugov_should_update_freq().
	 */
	smp_wmb();

	WRITE_ONCE(sg_policy->limits_changed, true);
}

static struct cpufreq_governor schedutil_gov = {
	.name			= "schedutil",
	.owner			= THIS_MODULE,
	.dynamic_switching	= true,
	.init			= sugov_init,
	.exit			= sugov_exit,
	.start			= sugov_start,
	.stop			= sugov_stop,
	.limits			= sugov_limits,
};

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_SCHEDUTIL
struct cpufreq_governor *cpufreq_default_governor(void)
{
	return &schedutil_gov;
}
#endif

static int __init sugov_register(void)
{
	return cpufreq_register_governor(&schedutil_gov);
}
fs_initcall(sugov_register);
