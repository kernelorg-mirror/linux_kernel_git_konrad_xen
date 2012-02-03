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
 * It it unaware of subsequent processors hot-added to the system.
 * This means that if you boot with maxcpus=n and later online
 * processors above n, those processors will use C1 only.
 *
 * There is currently no kernel-based automatic probing/loading mechanism
 * if the driver is built as a module.
 */

#define DEBUG 1

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <acpi/acpi_bus.h>
#include <acpi/acpi_drivers.h>
#include <acpi/processor.h>
#include <linux/cpumask.h>

#include <xen/interface/platform.h>
#include <asm/xen/hypercall.h>

#define DRV_NAME	"xen-acpi-passthrough"
MODULE_AUTHOR("Konrad Rzeszutek Wilk <konrad.wilk@oracle.com>");
MODULE_DESCRIPTION("ACPI Power Management driver to pass Cx and Pxx data to Xen hypervisor");
MODULE_LICENSE("GPL");


MODULE_PARM_DESC(nohypercall, "Inhibit the hypercall.");
static int no_hypercall;
module_param_named(nohypercall, no_hypercall, int, 0400);

static int __init push_cxx_to_hypervisor(struct acpi_processor *_pr)
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

	if (!_pr->flags.power_setup_done)
		return -ENODEV;

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
			/* TODO: double check whether anybody cares about it */
			xen_cx->reg.bit_width = 8;
			xen_cx->reg.bit_offset = 0;
		} else {
			xen_cx->reg.space_id = ACPI_ADR_SPACE_FIXED_HARDWARE;
			if (cx->entry_method == ACPI_CSTATE_FFH) {
				/* NATIVE_CSTATE_BEYOND_HALT */
				xen_cx->reg.bit_offset = 2;
				xen_cx->reg.bit_width = 1; /* VENDOR_INTEL */
			}
		}
		xen_cx->reg.access_size = 0;
		xen_cx->reg.address = cx->address;

		xen_cx->type = cx->type;
		xen_cx->latency = cx->latency;
		xen_cx->power = cx->power;

		xen_cx->dpcnt = 0;
		set_xen_guest_handle(xen_cx->dp, NULL);
#ifdef DEBUG
		pr_debug(DRV_NAME ": CX: ID:%d [C%d:%s]\n", _pr->acpi_id, i, cx->desc);
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



static struct xen_processor_px *
__init xen_copy_pss_data(struct acpi_processor *_pr,
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
static int __init xen_copy_psd_data(struct acpi_processor *_pr,
				    struct xen_processor_performance *xen_perf)
{
	xen_perf->shared_type = _pr->performance->shared_type;

	BUILD_BUG_ON(sizeof(struct xen_psd_package) !=
		     sizeof(struct acpi_psd_package));
	memcpy(&(xen_perf->domain_info), &(_pr->performance->domain_info),
	       sizeof(struct acpi_psd_package));

	return 0;
}
static int __init xen_copy_pct_data(struct acpi_pct_register *pct,
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

	return 0;
}
static int __init push_pxx_to_hypervisor(struct acpi_processor *_pr)
{
	int ret = -EINVAL;
	struct xen_platform_op op = {
		.cmd			= XENPF_set_processor_pminfo,
		.interface_version	= XENPF_INTERFACE_VERSION,
		.u.set_pminfo.id	= _pr->acpi_id,
		.u.set_pminfo.type	= XEN_PM_PX,
	};
	struct xen_processor_performance *xen_perf;
	struct xen_processor_px *xen_states = NULL;

	if (!_pr->performance)
		return -ENODEV;

	xen_perf = &op.u.set_pminfo.perf;

	/* PPC */
	xen_perf->platform_limit = _pr->performance_platform_limit;
	xen_perf->flags |= XEN_PX_PPC;
	/* PCT */
	xen_copy_pct_data(&(_pr->performance->control_register),
			  &xen_perf->control_register);
	xen_copy_pct_data(&(_pr->performance->status_register),
			  &xen_perf->status_register);
	xen_perf->flags |= XEN_PX_PCT;
	/* PSS */
	xen_states = xen_copy_pss_data(_pr, xen_perf);
	if (!IS_ERR_OR_NULL(xen_states)) {
		set_xen_guest_handle(xen_perf->states, xen_states);
		xen_perf->flags |= XEN_PX_PSS;
	}
	/* PSD */
	if (!xen_copy_psd_data(_pr, xen_perf))
		xen_perf->flags |= XEN_PX_PSD;

	if (!no_hypercall && xen_initial_domain())
		ret = HYPERVISOR_dom0_op(&op);

	if (!IS_ERR_OR_NULL(xen_states))
		kfree(xen_states);
	return ret;
}

static int __init acpi_xen_passthrough_init(void)
{
	int cpu;
	int err = -ENODEV;
	struct acpi_processor *_pr;
	struct cpuinfo_x86 *c = &cpu_data(0);

	if (!xen_initial_domain())
		return -ENODEV;

	/* TODO: Under AMD, the information is populated
	 * using the powernow-k8 driver which does an MSR_PSTATE_CUR_LIMIT
	 * MSR which returns the wrong value (under Xen) so the population
	 * of 'processors' has bogus data. So only run this under
	 * Intel for right now. */
	if (!cpu_has(c, X86_FEATURE_EST)) {
		pr_err(DRV_NAME ": AMD platform is not supported (yet)\n");
		return -ENODEV;
	}
	/*
	 * It is imperative that we get called _after_ acpi_processor has
	 * loaded. Otherwise the _pr might be bogus.
	*/
	if (request_module("processor")) {
		pr_err(DRV_NAME ": Unable to load ACPI processor module!\n");
		return -ENODEV;
	}

	get_online_cpus();
	for_each_online_cpu(cpu) {
		_pr = per_cpu(processors, cpu);
		if (!_pr)
			continue;

		if (_pr->flags.power)
			err = push_cxx_to_hypervisor(_pr);

		if (_pr->performance->states)
			err |= push_pxx_to_hypervisor(_pr);
		if (err)
			break;
	}
	put_online_cpus();

	return -ENODEV; /* force it to unload */
}
static void __exit acpi_xen_passthrough_exit(void)
{
}
module_init(acpi_xen_passthrough_init);
module_exit(acpi_xen_passthrough_exit);
