#ifndef INCLUDE_XEN_OPS_H
#define INCLUDE_XEN_OPS_H

#include <linux/percpu.h>

DECLARE_PER_CPU(struct vcpu_info *, xen_vcpu);

void xen_arch_pre_suspend(void);
void xen_arch_post_suspend(int suspend_cancelled);
void xen_arch_hvm_post_suspend(int suspend_cancelled);

void xen_mm_pin_all(void);
void xen_mm_unpin_all(void);

void xen_timer_resume(void);
void xen_arch_resume(void);

int xen_setup_shutdown_event(void);

extern unsigned long *xen_contiguous_bitmap;
int xen_create_contiguous_region(unsigned long vstart, unsigned int order,
				unsigned int address_bits);

void xen_destroy_contiguous_region(unsigned long vstart, unsigned int order);

struct vm_area_struct;
struct xen_pvh_pfn_info;
int xen_remap_domain_mfn_range(struct vm_area_struct *vma,
			       unsigned long addr,
			       unsigned long mfn, int nr,
			       pgprot_t prot, unsigned domid,
			       struct xen_pvh_pfn_info *pvhp);
int xen_unmap_domain_mfn_range(struct vm_area_struct *vma,
			       struct xen_pvh_pfn_info *pvhp);

struct xen_pvh_pfn_info {
	struct page **pi_paga;		/* pfn info page array */
	int 	      pi_num_pgs;
	int 	      pi_next_todo;
};

#endif /* INCLUDE_XEN_OPS_H */
