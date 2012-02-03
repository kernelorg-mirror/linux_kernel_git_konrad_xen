/*
 * Copyright 2012 by Oracle Inc
 * Author: Konrad Rzeszutek Wilk <konrad.wilk@oracle.com>
 *
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */

/*
 *  Known limitations
 *
 * The driver currently initializes for_each_online_cpu() upon modprobe.
 * Meaning if you boot with dom0_max_cpus=X it will _only_ parse up to X
 * processors.
 */

#define DEBUG 1

#include <linux/cpumask.h>
#include <linux/cpufreq.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <acpi/acpi_bus.h>
#include <acpi/acpi_drivers.h>
#include <acpi/processor.h>

#include <xen/interface/platform.h>
#include <asm/xen/hypercall.h>

#define DRV_NAME "processor-passthrough-xen"
MODULE_AUTHOR("Konrad Rzeszutek Wilk <konrad.wilk@oracle.com>");
MODULE_DESCRIPTION("ACPI Power Management driver to pass Cx and Pxx data to Xen hypervisor");
MODULE_LICENSE("GPL");


MODULE_PARM_DESC(nocall, "Inhibit the hypercall.");
static int no_hypercall;
module_param_named(nocall, no_hypercall, int, 0400);

static DEFINE_MUTEX(processors_done_mutex);
static DECLARE_BITMAP(processors_done, NR_CPUS);

#define POLL_TIMER msecs_to_jiffies(5000 /* 5 sec */)
static struct task_struct *xen_processor_thread;

static int xen_push_cxx_to_hypervisor(struct acpi_processor *_pr)
{
	struct xen_platform_op op = {
		.cmd			= XENPF_set_processor_pminfo,
		.interface_version	= XENPF_INTERFACE_VERSION,
		.u.set_pminfo.id	= _pr->acpi_id,
		.u.set_pminfo.type	= XEN_PM_CX,
	};
	struct xen_processor_cx *xen_cx, *xen_cx_states = NULL;
	struct acpi_processor_cx *cx;
	int i, ok, ret = 0;

	xen_cx_states = kcalloc(_pr->power.count,
				sizeof(struct xen_processor_cx), GFP_KERNEL);
	if (!xen_cx_states)
		return -ENOMEM;

	for (ok = 0, i = 1; i <= _pr->power.count; i++) {
		cx = &_pr->power.states[i];
		if (!cx->valid)
			continue;

		xen_cx = &(xen_cx_states[ok++]);

		xen_cx->reg.space_id = ACPI_ADR_SPACE_SYSTEM_IO;
		if (cx->entry_method == ACPI_CSTATE_SYSTEMIO) {
			xen_cx->reg.bit_width = 8;
			xen_cx->reg.bit_offset = 0;
			xen_cx->reg.access_size = 1;
		} else {
			xen_cx->reg.space_id = ACPI_ADR_SPACE_FIXED_HARDWARE;
			if (cx->entry_method == ACPI_CSTATE_FFH) {
				/* NATIVE_CSTATE_BEYOND_HALT */
				xen_cx->reg.bit_offset = 2;
				xen_cx->reg.bit_width = 1; /* VENDOR_INTEL */
			}
			xen_cx->reg.access_size = 0;
		}
		xen_cx->reg.address = cx->address;

		xen_cx->type = cx->type;
		xen_cx->latency = cx->latency;
		xen_cx->power = cx->power;

		xen_cx->dpcnt = 0;
		set_xen_guest_handle(xen_cx->dp, NULL);
#ifdef DEBUG
		pr_debug(DRV_NAME ": CX: ID:%d [C%d:%s] entry:%d\n", _pr->acpi_id,
			 cx->type, cx->desc, cx->entry_method);
#endif
	}
	if (!ok) {
		pr_err(DRV_NAME ": No available Cx info for cpu %d\n", _pr->acpi_id);
		kfree(xen_cx_states);
		return -EINVAL;
	}
	op.u.set_pminfo.power.count = ok;
	op.u.set_pminfo.power.flags.bm_control = _pr->flags.bm_control;
	op.u.set_pminfo.power.flags.bm_check = _pr->flags.bm_check;
	op.u.set_pminfo.power.flags.has_cst = _pr->flags.has_cst;
	op.u.set_pminfo.power.flags.power_setup_done =
		_pr->flags.power_setup_done;

	set_xen_guest_handle(op.u.set_pminfo.power.states, xen_cx_states);

	if (!no_hypercall && xen_initial_domain())
		ret = HYPERVISOR_dom0_op(&op);

	kfree(xen_cx_states);

	return ret;
}



static struct xen_processor_px *xen_copy_pss_data(struct acpi_processor *_pr,
						  struct xen_processor_performance *xen_perf)
{
	struct xen_processor_px *xen_states = NULL;
	int i;

	xen_states = kcalloc(_pr->performance->state_count,
			     sizeof(struct xen_processor_px), GFP_KERNEL);
	if (!xen_states)
		return ERR_PTR(-ENOMEM);

	xen_perf->state_count = _pr->performance->state_count;

	BUILD_BUG_ON(sizeof(struct xen_processor_px) !=
		     sizeof(struct acpi_processor_px));
	for (i = 0; i < _pr->performance->state_count; i++) {

		/* Fortunatly for us, they both have the same size */
		memcpy(&(xen_states[i]), &(_pr->performance->states[i]),
		       sizeof(struct acpi_processor_px));
#ifdef DEBUG
		{
			struct xen_processor_px *_px;
			_px = &(xen_states[i]);

			pr_debug(DRV_NAME ": _PSS: [%2d]: %d, %d, %d, %d, %d, %d\n",
				 i, (u32)_px->core_frequency, (u32)_px->power,
				 (u32)_px->transition_latency,
				 (u32)_px->bus_master_latency, (u32)_px->control,
				 (u32)_px->status);
		}
#endif
	}
	return xen_states;
}
static int xen_copy_psd_data(struct acpi_processor *_pr,
			     struct xen_processor_performance *xen_perf)
{
	BUILD_BUG_ON(sizeof(struct xen_psd_package) !=
		     sizeof(struct acpi_psd_package));

	if (_pr->performance->shared_type != CPUFREQ_SHARED_TYPE_NONE) {
		xen_perf->shared_type = _pr->performance->shared_type;

		memcpy(&(xen_perf->domain_info), &(_pr->performance->domain_info),
		       sizeof(struct acpi_psd_package));
	} else {
		if ((&cpu_data(0))->x86_vendor != X86_VENDOR_AMD)
			return -EINVAL;

		/* On AMD, the powernow-k8 is loaded before acpi_cpufreq
		 * meaning that acpi_processor_preregister_performance never
		 * gets called which would parse the _CST.
		 */
		xen_perf->shared_type = CPUFREQ_SHARED_TYPE_ALL;
		xen_perf->domain_info.num_processors = num_online_cpus();
	}
#ifdef DEBUG
	pr_debug(DRV_NAME ": PSD %d for %lld CPUs.\n", xen_perf->shared_type,
		 xen_perf->domain_info.num_processors);
#endif
	return 0;
}
static int xen_copy_pct_data(struct acpi_pct_register *pct,
			     struct xen_pct_register *_pct)
{
	/* It would be nice if you could just do 'memcpy(pct, _pct') but
	 * sadly the Xen structure did not have the proper padding
	 * so the descriptor field takes two (_pct) bytes instead of one (pct).
	 */
	_pct->descriptor = pct->descriptor;
	_pct->length = pct->length;
	_pct->space_id = pct->space_id;
	_pct->bit_width = pct->bit_width;
	_pct->bit_offset = pct->bit_offset;
	_pct->reserved = pct->reserved;
	_pct->address = pct->address;
#ifdef DEBUG
	pr_debug(DRV_NAME ": _PCT: %d, %d, %d, %d, %d, %d, 0x%llx\n",
		pct->descriptor, pct->length, pct->space_id, pct->bit_width,
		pct->bit_offset, pct->reserved, pct->address);
#endif
	return 0;
}
static int xen_push_pxx_to_hypervisor(struct acpi_processor *_pr)
{
	int ret = 0;
	struct xen_platform_op op = {
		.cmd			= XENPF_set_processor_pminfo,
		.interface_version	= XENPF_INTERFACE_VERSION,
		.u.set_pminfo.id	= _pr->acpi_id,
		.u.set_pminfo.type	= XEN_PM_PX,
	};
	struct xen_processor_performance *xen_perf;
	struct xen_processor_px *xen_states = NULL;

	xen_perf = &op.u.set_pminfo.perf;

	xen_perf->platform_limit = _pr->performance_platform_limit;
	xen_perf->flags |= XEN_PX_PPC;
	xen_copy_pct_data(&(_pr->performance->control_register),
			  &xen_perf->control_register);
	xen_copy_pct_data(&(_pr->performance->status_register),
			  &xen_perf->status_register);
	xen_perf->flags |= XEN_PX_PCT;
	xen_states = xen_copy_pss_data(_pr, xen_perf);
	if (!IS_ERR_OR_NULL(xen_states)) {
		set_xen_guest_handle(xen_perf->states, xen_states);
		xen_perf->flags |= XEN_PX_PSS;
	}
	if (!xen_copy_psd_data(_pr, xen_perf))
		xen_perf->flags |= XEN_PX_PSD;

	if (!no_hypercall && xen_initial_domain())
		ret = HYPERVISOR_dom0_op(&op);

	if (!IS_ERR_OR_NULL(xen_states))
		kfree(xen_states);

	return ret;
}
/*
 * We read out the struct acpi_processor, and serialize access
 * so that there is only one caller. This is so that we won't
 * race with the CPU hotplug code.
 */
static int xen_process_data(struct acpi_processor *_pr, int cpu)
{
	int err = 0;

	mutex_lock(&processors_done_mutex);
	if (cpumask_test_cpu(cpu, to_cpumask(processors_done))) {
		mutex_unlock(&processors_done_mutex);
		return -EBUSY;
	}
	if (_pr->flags.power)
		err = xen_push_cxx_to_hypervisor(_pr);

	if (_pr->performance && _pr->performance->states)
		err |= xen_push_pxx_to_hypervisor(_pr);

	cpumask_set_cpu(cpu, to_cpumask(processors_done));
	mutex_unlock(&processors_done_mutex);
	if (err)
		pr_err(DRV_NAME ": Failed to push CPU%d data (%d)\n", cpu, err);
	return err;
}

static int xen_processor_check(void)
{
	struct cpufreq_policy *policy;
	int cpu;

	policy = cpufreq_cpu_get(smp_processor_id());
	if (!policy)
		return -EBUSY;

	get_online_cpus();
	for_each_online_cpu(cpu) {
		struct acpi_processor *_pr;

		_pr = per_cpu(processors, cpu);
		if (!_pr)
			continue;

		(void)xen_process_data(_pr, cpu);
	}
	put_online_cpus();

	cpufreq_cpu_put(policy);
	return 0;
}
/*
 * The purpose of this timer is to wait for the ACPI processor
 * and CPUfreq drivers to load up and parse the Pxx and Cxx information
 * before we attempt to read it.
 */

static void xen_processor_timeout(unsigned long arg)
{
	wake_up_process((struct task_struct *)arg);
}
static int xen_processor_thread_func(void *dummy)
{
	struct timer_list timer;

	setup_deferrable_timer_on_stack(&timer, xen_processor_timeout,
					(unsigned long)current);

	do {
		__set_current_state(TASK_INTERRUPTIBLE);
		mod_timer(&timer, jiffies + POLL_TIMER);
		schedule();
		if (xen_processor_check() != -EBUSY)
			break;
	} while (!kthread_should_stop());

	del_timer_sync(&timer);
	destroy_timer_on_stack(&timer);
	return 0;
}

static int xen_cpu_soft_notify(struct notifier_block *nfb,
			       unsigned long action, void *hcpu)
{
	unsigned int cpu = (unsigned long)hcpu;
	struct acpi_processor *_pr = per_cpu(processors, cpu);

	if (action == CPU_ONLINE && _pr)
		(void)xen_process_data(_pr, cpu);

	return NOTIFY_OK;
}

static struct notifier_block xen_cpu_notifier = {
	.notifier_call = xen_cpu_soft_notify,
	.priority = -1, /* Be the last one */
};

static int __init check_prereq(void)
{
	struct cpuinfo_x86 *c = &cpu_data(0);

	if (!xen_initial_domain())
		return -ENODEV;

	if (!acpi_gbl_FADT.smi_command)
		return -ENODEV;

	if (c->x86_vendor == X86_VENDOR_INTEL) {
		if (!cpu_has(c, X86_FEATURE_EST))
			return -ENODEV;

		return 0;
	}
	if (c->x86_vendor == X86_VENDOR_AMD) {
		u32 hi = 0, lo = 0;
		/* Copied from powernow-k8.h, can't include ../cpufreq/powernow
		 * as we get compile warnings for the static functions.
		 */
#define MSR_PSTATE_CUR_LIMIT    0xc0010061 /* pstate current limit MSR */
		rdmsr(MSR_PSTATE_CUR_LIMIT, lo, hi);

		/* If the MSR cannot provide the data, the powernow-k8
		 * won't process the data properly either.
		 */
		if (hi || lo)
			return 0;
	}
	return -ENODEV;
}

static int __init xen_processor_passthrough_init(void)
{
	int rc = check_prereq();

	if (rc)
		return rc;

	xen_processor_thread = kthread_run(xen_processor_thread_func, NULL, DRV_NAME);
	if (IS_ERR(xen_processor_thread)) {
		pr_err(DRV_NAME ": Failed to create thread. Aborting.\n");
		return -ENOMEM;
	}
	register_hotcpu_notifier(&xen_cpu_notifier);
	return 0;
}
static void __exit xen_processor_passthrough_exit(void)
{
	unregister_hotcpu_notifier(&xen_cpu_notifier);
	if (xen_processor_thread)
		kthread_stop(xen_processor_thread);
}
late_initcall(xen_processor_passthrough_init);
module_exit(xen_processor_passthrough_exit);
