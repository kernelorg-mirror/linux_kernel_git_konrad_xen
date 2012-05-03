
/*
 * Back-end of the driver for virtual network devices. This portion of the
 * driver exports a 'unified' network-device interface that can be accessed
 * by any operating system that implements a compatible front end. A
 * reference front-end implementation can be found in:
 *  drivers/net/xen-netfront.c
 *
 * Copyright (c) 2002-2005, K A Fraser
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation; or, when distributed
 * separately from the Linux kernel or incorporated into other
 * software packages, subject to the following license:
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this source file (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy, modify,
 * merge, publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */

#include "common.h"
#include "page_pool.h"

#include <linux/kthread.h>
#include <linux/if_vlan.h>
#include <linux/udp.h>
#include <linux/cpu.h>

#include <net/tcp.h>

#include <xen/events.h>
#include <xen/interface/memory.h>

#include <asm/xen/hypercall.h>
#include <asm/xen/page.h>

unsigned int MODPARM_netback_max_rx_ring_page_order = NETBK_MAX_RING_PAGE_ORDER;
module_param_named(netback_max_rx_ring_page_order,
		   MODPARM_netback_max_rx_ring_page_order, uint, 0);
MODULE_PARM_DESC(netback_max_rx_ring_page_order,
		 "Maximum supported receiver ring page order");

unsigned int MODPARM_netback_max_tx_ring_page_order = NETBK_MAX_RING_PAGE_ORDER;
module_param_named(netback_max_tx_ring_page_order,
		   MODPARM_netback_max_tx_ring_page_order, uint, 0);
MODULE_PARM_DESC(netback_max_tx_ring_page_order,
		 "Maximum supported transmitter ring page order");

DEFINE_PER_CPU(struct gnttab_copy *, tx_copy_ops);

/*
 * Given MAX_BUFFER_OFFSET of 4096 the worst case is that each
 * head/fragment page uses 2 copy operations because it
 * straddles two buffers in the frontend.
 */
DEFINE_PER_CPU(struct gnttab_copy *, grant_copy_op);
DEFINE_PER_CPU(struct xenvif_rx_meta *, meta);

static void xenvif_idx_release(struct xenvif *vif, u16 pending_idx);
static void make_tx_response(struct xenvif *vif,
			     struct xen_netif_tx_request *txp,
			     s8       st);

static inline int tx_work_todo(struct xenvif *vif);
static inline int rx_work_todo(struct xenvif *vif);

static struct xen_netif_rx_response *make_rx_response(struct xenvif *vif,
					     u16      id,
					     s8       st,
					     u16      offset,
					     u16      size,
					     u16      flags);

static inline unsigned long idx_to_pfn(struct xenvif *vif,
				       u16 idx)
{
	return page_to_pfn(to_page(vif->mmap_pages[idx]));
}

static inline unsigned long idx_to_kaddr(struct xenvif *vif,
					 u16 idx)
{
	return (unsigned long)pfn_to_kaddr(idx_to_pfn(vif, idx));
}

/*
 * This is the amount of packet we copy rather than map, so that the
 * guest can't fiddle with the contents of the headers while we do
 * packet processing on them (netfilter, routing, etc).
 */
#define PKT_PROT_LEN    (ETH_HLEN + \
			 VLAN_HLEN + \
			 sizeof(struct iphdr) + MAX_IPOPTLEN + \
			 sizeof(struct tcphdr) + MAX_TCP_OPTION_SPACE)

static u16 frag_get_pending_idx(skb_frag_t *frag)
{
	return (u16)frag->page_offset;
}

static void frag_set_pending_idx(skb_frag_t *frag, u16 pending_idx)
{
	frag->page_offset = pending_idx;
}

static inline pending_ring_idx_t pending_index(unsigned i)
{
	return i & (MAX_PENDING_REQS-1);
}

static inline pending_ring_idx_t nr_pending_reqs(struct xenvif *vif)
{
	return MAX_PENDING_REQS -
		vif->pending_prod + vif->pending_cons;
}

static int max_required_rx_slots(struct xenvif *vif)
{
	int max = DIV_ROUND_UP(vif->dev->mtu, PAGE_SIZE);

	if (vif->can_sg || vif->gso || vif->gso_prefix)
		max += MAX_SKB_FRAGS + 1; /* extra_info + frags */

	return max;
}

int xenvif_rx_ring_full(struct xenvif *vif)
{
	RING_IDX peek   = vif->rx_req_cons_peek;
	RING_IDX needed = max_required_rx_slots(vif);

	return ((vif->rx.sring->req_prod - peek) < needed) ||
	       ((vif->rx.rsp_prod_pvt +
		 NETBK_RX_RING_SIZE(vif->nr_rx_handles) - peek) < needed);
}

int xenvif_must_stop_queue(struct xenvif *vif)
{
	if (!xenvif_rx_ring_full(vif))
		return 0;

	vif->rx.sring->req_event = vif->rx_req_cons_peek +
		max_required_rx_slots(vif);
	mb(); /* request notification /then/ check the queue */

	return xenvif_rx_ring_full(vif);
}

/*
 * Returns true if we should start a new receive buffer instead of
 * adding 'size' bytes to a buffer which currently contains 'offset'
 * bytes.
 */
static bool start_new_rx_buffer(int offset, unsigned long size, int head)
{
	/* simple case: we have completely filled the current buffer. */
	if (offset == MAX_BUFFER_OFFSET)
		return true;

	/*
	 * complex case: start a fresh buffer if the current frag
	 * would overflow the current buffer but only if:
	 *     (i)   this frag would fit completely in the next buffer
	 * and (ii)  there is already some data in the current buffer
	 * and (iii) this is not the head buffer.
	 *
	 * Where:
	 * - (i) stops us splitting a frag into two copies
	 *   unless the frag is too large for a single buffer.
	 * - (ii) stops us from leaving a buffer pointlessly empty.
	 * - (iii) stops us leaving the first buffer
	 *   empty. Strictly speaking this is already covered
	 *   by (ii) but is explicitly checked because
	 *   netfront relies on the first buffer being
	 *   non-empty and can crash otherwise.
	 *
	 * This means we will effectively linearise small
	 * frags but do not needlessly split large buffers
	 * into multiple copies tend to give large frags their
	 * own buffers as before.
	 */
	if ((offset + size > MAX_BUFFER_OFFSET) &&
	    (size <= MAX_BUFFER_OFFSET) && offset && !head)
		return true;

	return false;
}

/*
 * Figure out how many ring slots we're going to need to send @skb to
 * the guest. This function is essentially a dry run of
 * xenvif_gop_frag_copy.
 */
unsigned int xenvif_count_skb_slots(struct xenvif *vif, struct sk_buff *skb)
{
	unsigned int count;
	int i, copy_off;

	count = DIV_ROUND_UP(
			offset_in_page(skb->data)+skb_headlen(skb), PAGE_SIZE);

	copy_off = skb_headlen(skb) % PAGE_SIZE;

	if (skb_shinfo(skb)->gso_size)
		count++;

	for (i = 0; i < skb_shinfo(skb)->nr_frags; i++) {
		unsigned long size = skb_frag_size(&skb_shinfo(skb)->frags[i]);
		unsigned long bytes;
		while (size > 0) {
			BUG_ON(copy_off > MAX_BUFFER_OFFSET);

			if (start_new_rx_buffer(copy_off, size, 0)) {
				count++;
				copy_off = 0;
			}

			bytes = size;
			if (copy_off + bytes > MAX_BUFFER_OFFSET)
				bytes = MAX_BUFFER_OFFSET - copy_off;

			copy_off += bytes;
			size -= bytes;
		}
	}
	return count;
}

struct netrx_pending_operations {
	unsigned copy_prod, copy_cons;
	unsigned meta_prod, meta_cons;
	struct gnttab_copy *copy;
	struct xenvif_rx_meta *meta;
	int copy_off;
	grant_ref_t copy_gref;
};

static struct xenvif_rx_meta *get_next_rx_buffer(struct xenvif *vif,
					struct netrx_pending_operations *npo)
{
	struct xenvif_rx_meta *meta;
	struct xen_netif_rx_request *req;

	req = RING_GET_REQUEST(&vif->rx, vif->rx.req_cons++);

	meta = npo->meta + npo->meta_prod++;
	meta->gso_size = 0;
	meta->size = 0;
	meta->id = req->id;

	npo->copy_off = 0;
	npo->copy_gref = req->gref;

	return meta;
}

/*
 * Set up the grant operations for this fragment. If it's a flipping
 * interface, we also set up the unmap request from here.
 */
static void xenvif_gop_frag_copy(struct xenvif *vif, struct sk_buff *skb,
				 struct netrx_pending_operations *npo,
				 struct page *page, unsigned long size,
				 unsigned long offset, int *head)
{
	struct gnttab_copy *copy_gop;
	struct xenvif_rx_meta *meta;
	/*
	 * These variables are used iff get_page_ext returns true,
	 * in which case they are guaranteed to be initialized.
	 */
	unsigned int uninitialized_var(idx);
	int foreign = is_in_pool(page, &idx);
	unsigned long bytes;

	/* Data must not cross a page boundary. */
	BUG_ON(size + offset > PAGE_SIZE);

	meta = npo->meta + npo->meta_prod - 1;

	while (size > 0) {
		BUG_ON(npo->copy_off > MAX_BUFFER_OFFSET);

		if (start_new_rx_buffer(npo->copy_off, size, *head)) {
			/*
			 * Netfront requires there to be some data in the head
			 * buffer.
			 */
			BUG_ON(*head);

			meta = get_next_rx_buffer(vif, npo);
		}

		bytes = size;
		if (npo->copy_off + bytes > MAX_BUFFER_OFFSET)
			bytes = MAX_BUFFER_OFFSET - npo->copy_off;

		copy_gop = npo->copy + npo->copy_prod++;
		copy_gop->flags = GNTCOPY_dest_gref;
		if (foreign) {
			struct pending_tx_info *src_pend = to_txinfo(idx);
			struct xenvif *rvif = to_vif(idx);

			copy_gop->source.domid = rvif->domid;
			copy_gop->source.u.ref = src_pend->req.gref;
			copy_gop->flags |= GNTCOPY_source_gref;
		} else {
			void *vaddr = page_address(page);
			copy_gop->source.domid = DOMID_SELF;
			copy_gop->source.u.gmfn = virt_to_mfn(vaddr);
		}
		copy_gop->source.offset = offset;
		copy_gop->dest.domid = vif->domid;

		copy_gop->dest.offset = npo->copy_off;
		copy_gop->dest.u.ref = npo->copy_gref;
		copy_gop->len = bytes;

		npo->copy_off += bytes;
		meta->size += bytes;

		offset += bytes;
		size -= bytes;

		/* Leave a gap for the GSO descriptor. */
		if (*head && skb_shinfo(skb)->gso_size && !vif->gso_prefix)
			vif->rx.req_cons++;

		*head = 0; /* There must be something in this buffer now. */

	}
}

/*
 * Prepare an SKB to be transmitted to the frontend.
 *
 * This function is responsible for allocating grant operations, meta
 * structures, etc.
 *
 * It returns the number of meta structures consumed. The number of
 * ring slots used is always equal to the number of meta slots used
 * plus the number of GSO descriptors used. Currently, we use either
 * zero GSO descriptors (for non-GSO packets) or one descriptor (for
 * frontend-side LRO).
 */
static int xenvif_gop_skb(struct sk_buff *skb,
			  struct netrx_pending_operations *npo)
{
	struct xenvif *vif = netdev_priv(skb->dev);
	int nr_frags = skb_shinfo(skb)->nr_frags;
	int i;
	struct xen_netif_rx_request *req;
	struct xenvif_rx_meta *meta;
	unsigned char *data;
	int head = 1;
	int old_meta_prod;

	old_meta_prod = npo->meta_prod;

	/* Set up a GSO prefix descriptor, if necessary */
	if (skb_shinfo(skb)->gso_size && vif->gso_prefix) {
		req = RING_GET_REQUEST(&vif->rx, vif->rx.req_cons++);
		meta = npo->meta + npo->meta_prod++;
		meta->gso_size = skb_shinfo(skb)->gso_size;
		meta->size = 0;
		meta->id = req->id;
	}

	req = RING_GET_REQUEST(&vif->rx, vif->rx.req_cons++);
	meta = npo->meta + npo->meta_prod++;

	if (!vif->gso_prefix)
		meta->gso_size = skb_shinfo(skb)->gso_size;
	else
		meta->gso_size = 0;

	meta->size = 0;
	meta->id = req->id;
	npo->copy_off = 0;
	npo->copy_gref = req->gref;

	data = skb->data;
	while (data < skb_tail_pointer(skb)) {
		unsigned int offset = offset_in_page(data);
		unsigned int len = PAGE_SIZE - offset;

		if (data + len > skb_tail_pointer(skb))
			len = skb_tail_pointer(skb) - data;

		xenvif_gop_frag_copy(vif, skb, npo,
				     virt_to_page(data), len, offset, &head);
		data += len;
	}

	for (i = 0; i < nr_frags; i++) {
		xenvif_gop_frag_copy(vif, skb, npo,
				     skb_frag_page(&skb_shinfo(skb)->frags[i]),
				     skb_frag_size(&skb_shinfo(skb)->frags[i]),
				     skb_shinfo(skb)->frags[i].page_offset,
				     &head);
	}

	return npo->meta_prod - old_meta_prod;
}

/*
 * This is a twin to xenvif_gop_skb.  Assume that xenvif_gop_skb was
 * used to set up the operations on the top of
 * netrx_pending_operations, which have since been done.  Check that
 * they didn't give any errors and advance over them.
 */
static int xenvif_check_gop(struct xenvif *vif, int nr_meta_slots,
			    struct netrx_pending_operations *npo)
{
	struct gnttab_copy     *copy_op;
	int status = XEN_NETIF_RSP_OKAY;
	int i;

	for (i = 0; i < nr_meta_slots; i++) {
		copy_op = npo->copy + npo->copy_cons++;
		if (copy_op->status != GNTST_okay) {
			netdev_dbg(vif->dev,
				   "Bad status %d from copy to DOM%d.\n",
				   copy_op->status, vif->domid);
			status = XEN_NETIF_RSP_ERROR;
		}
	}

	return status;
}

static void xenvif_add_frag_responses(struct xenvif *vif, int status,
				      struct xenvif_rx_meta *meta,
				      int nr_meta_slots)
{
	int i;
	unsigned long offset;

	/* No fragments used */
	if (nr_meta_slots <= 1)
		return;

	nr_meta_slots--;

	for (i = 0; i < nr_meta_slots; i++) {
		int flags;
		if (i == nr_meta_slots - 1)
			flags = 0;
		else
			flags = XEN_NETRXF_more_data;

		offset = 0;
		make_rx_response(vif, meta[i].id, status, offset,
				 meta[i].size, flags);
	}
}

struct skb_cb_overlay {
	int meta_slots_used;
};

static void xenvif_kick_thread(struct xenvif *vif)
{
	wake_up(&vif->wq);
}

void xenvif_rx_action(struct xenvif *vif)
{
	s8 status;
	u16 flags;
	struct xen_netif_rx_response *resp;
	struct sk_buff_head rxq;
	struct sk_buff *skb;
	LIST_HEAD(notify);
	int ret;
	int nr_frags;
	int count;
	unsigned long offset;
	struct skb_cb_overlay *sco;
	int need_to_notify = 0;
	static int unusable_count;

	struct gnttab_copy *gco = get_cpu_var(grant_copy_op);
	struct xenvif_rx_meta *m = get_cpu_var(meta);

	struct netrx_pending_operations npo = {
		.copy  = gco,
		.meta  = m,
	};

	if (gco == NULL || m == NULL) {
		put_cpu_var(grant_copy_op);
		put_cpu_var(meta);
		if (unusable_count == 1000) {
			pr_alert("CPU %x scratch space is not usable,"
				 " not doing any TX work for vif%u.%u\n",
				 smp_processor_id(),
				 vif->domid, vif->handle);
			unusable_count = 0;
		}
		return;
	}

	skb_queue_head_init(&rxq);

	count = 0;

	while ((skb = skb_dequeue(&vif->rx_queue)) != NULL) {
		vif = netdev_priv(skb->dev);
		nr_frags = skb_shinfo(skb)->nr_frags;

		sco = (struct skb_cb_overlay *)skb->cb;
		sco->meta_slots_used = xenvif_gop_skb(skb, &npo);

		count += nr_frags + 1;

		__skb_queue_tail(&rxq, skb);

		/* Filled the batch queue? */
		if (count + MAX_SKB_FRAGS >=
		    NETBK_RX_RING_SIZE(vif->nr_rx_handles))
			break;
	}

	BUG_ON(npo.meta_prod > MAX_PENDING_REQS);

	if (!npo.copy_prod) {
		put_cpu_var(grant_copy_op);
		put_cpu_var(meta);
		return;
	}

	BUG_ON(npo.copy_prod > (2 * NETBK_MAX_RX_RING_SIZE));
	ret = HYPERVISOR_grant_table_op(GNTTABOP_copy, gco,
					npo.copy_prod);
	BUG_ON(ret != 0);

	while ((skb = __skb_dequeue(&rxq)) != NULL) {
		sco = (struct skb_cb_overlay *)skb->cb;

		if (m[npo.meta_cons].gso_size && vif->gso_prefix) {
			resp = RING_GET_RESPONSE(&vif->rx,
						vif->rx.rsp_prod_pvt++);

			resp->flags = XEN_NETRXF_gso_prefix | XEN_NETRXF_more_data;

			resp->offset = m[npo.meta_cons].gso_size;
			resp->id = m[npo.meta_cons].id;
			resp->status = sco->meta_slots_used;

			npo.meta_cons++;
			sco->meta_slots_used--;
		}


		vif->dev->stats.tx_bytes += skb->len;
		vif->dev->stats.tx_packets++;

		status = xenvif_check_gop(vif, sco->meta_slots_used, &npo);

		if (sco->meta_slots_used == 1)
			flags = 0;
		else
			flags = XEN_NETRXF_more_data;

		if (skb->ip_summed == CHECKSUM_PARTIAL) /* local packet? */
			flags |= XEN_NETRXF_csum_blank | XEN_NETRXF_data_validated;
		else if (skb->ip_summed == CHECKSUM_UNNECESSARY)
			/* remote but checksummed. */
			flags |= XEN_NETRXF_data_validated;

		offset = 0;
		resp = make_rx_response(vif, m[npo.meta_cons].id,
					status, offset,
					m[npo.meta_cons].size,
					flags);

		if (m[npo.meta_cons].gso_size && !vif->gso_prefix) {
			struct xen_netif_extra_info *gso =
				(struct xen_netif_extra_info *)
				RING_GET_RESPONSE(&vif->rx,
						  vif->rx.rsp_prod_pvt++);

			resp->flags |= XEN_NETRXF_extra_info;

			gso->u.gso.size = m[npo.meta_cons].gso_size;
			gso->u.gso.type = XEN_NETIF_GSO_TYPE_TCPV4;
			gso->u.gso.pad = 0;
			gso->u.gso.features = 0;

			gso->type = XEN_NETIF_EXTRA_TYPE_GSO;
			gso->flags = 0;
		}

		xenvif_add_frag_responses(vif, status,
					  m + npo.meta_cons + 1,
					  sco->meta_slots_used);

		RING_PUSH_RESPONSES_AND_CHECK_NOTIFY(&vif->rx, ret);
		if (ret)
			need_to_notify = 1;

		xenvif_notify_tx_completion(vif);

		npo.meta_cons += sco->meta_slots_used;
		dev_kfree_skb(skb);
	}

	if (need_to_notify)
		notify_remote_via_irq(vif->rx_irq);

	if (!skb_queue_empty(&vif->rx_queue))
		xenvif_kick_thread(vif);

	put_cpu_var(grant_copy_op);
	put_cpu_var(meta);
}

void xenvif_queue_tx_skb(struct xenvif *vif, struct sk_buff *skb)
{
	skb_queue_tail(&vif->rx_queue, skb);

	xenvif_kick_thread(vif);
}

void xenvif_check_rx_xenvif(struct xenvif *vif)
{
	int more_to_do;

	RING_FINAL_CHECK_FOR_REQUESTS(&vif->tx, more_to_do);

	/* In this check function, we are supposed to do be's rx,
	 * which means fe's tx */

	if (more_to_do)
		napi_schedule(&vif->napi);
}

static void tx_add_credit(struct xenvif *vif)
{
	unsigned long max_burst, max_credit;

	/*
	 * Allow a burst big enough to transmit a jumbo packet of up to 128kB.
	 * Otherwise the interface can seize up due to insufficient credit.
	 */
	max_burst = RING_GET_REQUEST(&vif->tx, vif->tx.req_cons)->size;
	max_burst = min(max_burst, 131072UL);
	max_burst = max(max_burst, vif->credit_bytes);

	/* Take care that adding a new chunk of credit doesn't wrap to zero. */
	max_credit = vif->remaining_credit + vif->credit_bytes;
	if (max_credit < vif->remaining_credit)
		max_credit = ULONG_MAX; /* wrapped: clamp to ULONG_MAX */

	vif->remaining_credit = min(max_credit, max_burst);
}

static void tx_credit_callback(unsigned long data)
{
	struct xenvif *vif = (struct xenvif *)data;
	tx_add_credit(vif);
	xenvif_check_rx_xenvif(vif);
}

static void xenvif_tx_err(struct xenvif *vif,
			  struct xen_netif_tx_request *txp, RING_IDX end)
{
	RING_IDX cons = vif->tx.req_cons;

	do {
		make_tx_response(vif, txp, XEN_NETIF_RSP_ERROR);
		if (cons >= end)
			break;
		txp = RING_GET_REQUEST(&vif->tx, cons++);
	} while (1);
	vif->tx.req_cons = cons;
}

static int xenvif_count_requests(struct xenvif *vif,
				struct xen_netif_tx_request *first,
				struct xen_netif_tx_request *txp,
				int work_to_do)
{
	RING_IDX cons = vif->tx.req_cons;
	int frags = 0;

	if (!(first->flags & XEN_NETTXF_more_data))
		return 0;

	do {
		if (frags >= work_to_do) {
			netdev_dbg(vif->dev, "Need more frags\n");
			return -frags;
		}

		if (unlikely(frags >= MAX_SKB_FRAGS)) {
			netdev_dbg(vif->dev, "Too many frags\n");
			return -frags;
		}

		memcpy(txp, RING_GET_REQUEST(&vif->tx, cons + frags),
		       sizeof(*txp));
		if (txp->size > first->size) {
			netdev_dbg(vif->dev, "Frags galore\n");
			return -frags;
		}

		first->size -= txp->size;
		frags++;

		if (unlikely((txp->offset + txp->size) > PAGE_SIZE)) {
			netdev_dbg(vif->dev, "txp->offset: %x, size: %u\n",
				 txp->offset, txp->size);
			return -frags;
		}
	} while ((txp++)->flags & XEN_NETTXF_more_data);
	return frags;
}

static struct page *xenvif_alloc_page(struct xenvif *vif,
				      struct sk_buff *skb,
				      u16 pending_idx)
{
	struct page *page;
	int idx;
	page = page_pool_get(vif, &idx);
	if (!page)
		return NULL;
	vif->mmap_pages[pending_idx] = idx;
	return page;
}

static struct gnttab_copy *xenvif_get_requests(struct xenvif *vif,
					       struct sk_buff *skb,
					       struct xen_netif_tx_request *txp,
					       struct gnttab_copy *gop)
{
	struct skb_shared_info *shinfo = skb_shinfo(skb);
	skb_frag_t *frags = shinfo->frags;
	u16 pending_idx = *((u16 *)skb->data);
	int i, start;

	/* Skip first skb fragment if it is on same page as header fragment. */
	start = (frag_get_pending_idx(&shinfo->frags[0]) == pending_idx);

	for (i = start; i < shinfo->nr_frags; i++, txp++) {
		struct page *page;
		pending_ring_idx_t index;
		int idx;
		struct pending_tx_info *pending_tx_info;

		index = pending_index(vif->pending_cons++);
		pending_idx = vif->pending_ring[index];
		page = xenvif_alloc_page(vif, skb, pending_idx);
		if (!page)
			return NULL;

		idx = vif->mmap_pages[pending_idx];
		pending_tx_info = to_txinfo(idx);

		gop->source.u.ref = txp->gref;
		gop->source.domid = vif->domid;
		gop->source.offset = txp->offset;

		gop->dest.u.gmfn = virt_to_mfn(page_address(page));
		gop->dest.domid = DOMID_SELF;
		gop->dest.offset = txp->offset;

		gop->len = txp->size;
		gop->flags = GNTCOPY_source_gref;

		gop++;

		memcpy(&pending_tx_info->req, txp, sizeof(*txp));

		frag_set_pending_idx(&frags[i], pending_idx);
	}

	return gop;
}

static int xenvif_tx_check_gop(struct xenvif *vif,
			       struct sk_buff *skb,
			       struct gnttab_copy **gopp)
{
	struct gnttab_copy *gop = *gopp;
	u16 pending_idx = *((u16 *)skb->data);
	struct pending_tx_info *pending_tx_info;
	int idx;
	struct xen_netif_tx_request *txp;
	struct skb_shared_info *shinfo = skb_shinfo(skb);
	int nr_frags = shinfo->nr_frags;
	int i, err, start;

	/* Check status of header. */
	err = gop->status;
	if (unlikely(err)) {
		pending_ring_idx_t index;
		index = pending_index(vif->pending_prod++);
		idx = vif->mmap_pages[index];
		pending_tx_info = to_txinfo(idx);
		txp = &pending_tx_info->req;
		make_tx_response(vif, txp, XEN_NETIF_RSP_ERROR);
		vif->pending_ring[index] = pending_idx;
	}

	/* Skip first skb fragment if it is on same page as header fragment. */
	start = (frag_get_pending_idx(&shinfo->frags[0]) == pending_idx);

	for (i = start; i < nr_frags; i++) {
		int j, newerr;
		pending_ring_idx_t index;

		pending_idx = frag_get_pending_idx(&shinfo->frags[i]);

		/* Check error status: if okay then remember grant handle. */
		newerr = (++gop)->status;
		if (likely(!newerr)) {
			/* Had a previous error? Invalidate this fragment. */
			if (unlikely(err))
				xenvif_idx_release(vif, pending_idx);
			continue;
		}

		/* Error on this fragment: respond to client with an error. */
		idx = vif->mmap_pages[pending_idx];
		txp = &to_txinfo(idx)->req;
		make_tx_response(vif, txp, XEN_NETIF_RSP_ERROR);
		index = pending_index(vif->pending_prod++);
		vif->pending_ring[index] = pending_idx;

		/* Not the first error? Preceding frags already invalidated. */
		if (err)
			continue;

		/* First error: invalidate header and preceding fragments. */
		pending_idx = *((u16 *)skb->data);
		xenvif_idx_release(vif, pending_idx);
		for (j = start; j < i; j++) {
			pending_idx = frag_get_pending_idx(&shinfo->frags[j]);
			xenvif_idx_release(vif, pending_idx);
		}

		/* Remember the error: invalidate all subsequent fragments. */
		err = newerr;
	}

	*gopp = gop + 1;
	return err;
}

static void xenvif_fill_frags(struct xenvif *vif, struct sk_buff *skb)
{
	struct skb_shared_info *shinfo = skb_shinfo(skb);
	int nr_frags = shinfo->nr_frags;
	int i;

	for (i = 0; i < nr_frags; i++) {
		skb_frag_t *frag = shinfo->frags + i;
		struct xen_netif_tx_request *txp;
		struct page *page;
		u16 pending_idx;
		int idx;
		struct pending_tx_info *pending_tx_info;

		pending_idx = frag_get_pending_idx(frag);

		idx = vif->mmap_pages[pending_idx];
		pending_tx_info = to_txinfo(idx);

		txp = &pending_tx_info->req;
		page = virt_to_page(idx_to_kaddr(vif, pending_idx));
		__skb_fill_page_desc(skb, i, page, txp->offset, txp->size);
		skb->len += txp->size;
		skb->data_len += txp->size;
		skb->truesize += txp->size;

		/* Take an extra reference to offset xenvif_idx_release */
		get_page(page);
		xenvif_idx_release(vif, pending_idx);
	}
}

static int xenvif_get_extras(struct xenvif *vif,
			     struct xen_netif_extra_info *extras,
			     int work_to_do)
{
	struct xen_netif_extra_info extra;
	RING_IDX cons = vif->tx.req_cons;

	do {
		if (unlikely(work_to_do-- <= 0)) {
			netdev_dbg(vif->dev, "Missing extra info\n");
			return -EBADR;
		}

		memcpy(&extra, RING_GET_REQUEST(&vif->tx, cons),
		       sizeof(extra));
		if (unlikely(!extra.type ||
			     extra.type >= XEN_NETIF_EXTRA_TYPE_MAX)) {
			vif->tx.req_cons = ++cons;
			netdev_dbg(vif->dev,
				   "Invalid extra type: %d\n", extra.type);
			return -EINVAL;
		}

		memcpy(&extras[extra.type - 1], &extra, sizeof(extra));
		vif->tx.req_cons = ++cons;
	} while (extra.flags & XEN_NETIF_EXTRA_FLAG_MORE);

	return work_to_do;
}

static int xenvif_set_skb_gso(struct xenvif *vif,
			      struct sk_buff *skb,
			      struct xen_netif_extra_info *gso)
{
	if (!gso->u.gso.size) {
		netdev_dbg(vif->dev, "GSO size must not be zero.\n");
		return -EINVAL;
	}

	/* Currently only TCPv4 S.O. is supported. */
	if (gso->u.gso.type != XEN_NETIF_GSO_TYPE_TCPV4) {
		netdev_dbg(vif->dev, "Bad GSO type %d.\n", gso->u.gso.type);
		return -EINVAL;
	}

	skb_shinfo(skb)->gso_size = gso->u.gso.size;
	skb_shinfo(skb)->gso_type = SKB_GSO_TCPV4;

	/* Header must be checked, and gso_segs computed. */
	skb_shinfo(skb)->gso_type |= SKB_GSO_DODGY;
	skb_shinfo(skb)->gso_segs = 0;

	return 0;
}

static int checksum_setup(struct xenvif *vif, struct sk_buff *skb)
{
	struct iphdr *iph;
	unsigned char *th;
	int err = -EPROTO;
	int recalculate_partial_csum = 0;

	/*
	 * A GSO SKB must be CHECKSUM_PARTIAL. However some buggy
	 * peers can fail to set NETRXF_csum_blank when sending a GSO
	 * frame. In this case force the SKB to CHECKSUM_PARTIAL and
	 * recalculate the partial checksum.
	 */
	if (skb->ip_summed != CHECKSUM_PARTIAL && skb_is_gso(skb)) {
		vif->rx_gso_checksum_fixup++;
		skb->ip_summed = CHECKSUM_PARTIAL;
		recalculate_partial_csum = 1;
	}

	/* A non-CHECKSUM_PARTIAL SKB does not require setup. */
	if (skb->ip_summed != CHECKSUM_PARTIAL)
		return 0;

	if (skb->protocol != htons(ETH_P_IP))
		goto out;

	iph = (void *)skb->data;
	th = skb->data + 4 * iph->ihl;
	if (th >= skb_tail_pointer(skb))
		goto out;

	skb->csum_start = th - skb->head;
	switch (iph->protocol) {
	case IPPROTO_TCP:
		skb->csum_offset = offsetof(struct tcphdr, check);

		if (recalculate_partial_csum) {
			struct tcphdr *tcph = (struct tcphdr *)th;
			tcph->check = ~csum_tcpudp_magic(iph->saddr, iph->daddr,
							 skb->len - iph->ihl*4,
							 IPPROTO_TCP, 0);
		}
		break;
	case IPPROTO_UDP:
		skb->csum_offset = offsetof(struct udphdr, check);

		if (recalculate_partial_csum) {
			struct udphdr *udph = (struct udphdr *)th;
			udph->check = ~csum_tcpudp_magic(iph->saddr, iph->daddr,
							 skb->len - iph->ihl*4,
							 IPPROTO_UDP, 0);
		}
		break;
	default:
		if (net_ratelimit())
			netdev_err(vif->dev,
				   "Attempting to checksum a non-TCP/UDP packet, dropping a protocol %d packet\n",
				   iph->protocol);
		goto out;
	}

	if ((th + skb->csum_offset + 2) > skb_tail_pointer(skb))
		goto out;

	err = 0;

out:
	return err;
}

static bool tx_credit_exceeded(struct xenvif *vif, unsigned size)
{
	unsigned long now = jiffies;
	unsigned long next_credit =
		vif->credit_timeout.expires +
		msecs_to_jiffies(vif->credit_usec / 1000);

	/* Timer could already be pending in rare cases. */
	if (timer_pending(&vif->credit_timeout))
		return true;

	/* Passed the point where we can replenish credit? */
	if (time_after_eq(now, next_credit)) {
		vif->credit_timeout.expires = now;
		tx_add_credit(vif);
	}

	/* Still too big to send right now? Set a callback. */
	if (size > vif->remaining_credit) {
		vif->credit_timeout.data     =
			(unsigned long)vif;
		vif->credit_timeout.function =
			tx_credit_callback;
		mod_timer(&vif->credit_timeout,
			  next_credit);

		return true;
	}

	return false;
}

static unsigned xenvif_tx_build_gops(struct xenvif *vif,
				     struct gnttab_copy *tco)
{
	struct gnttab_copy *gop = tco, *request_gop;
	struct sk_buff *skb;
	int ret;

	while ((nr_pending_reqs(vif) + MAX_SKB_FRAGS) < MAX_PENDING_REQS) {
		struct xen_netif_tx_request txreq;
		struct xen_netif_tx_request txfrags[MAX_SKB_FRAGS];
		struct page *page;
		struct xen_netif_extra_info extras[XEN_NETIF_EXTRA_TYPE_MAX-1];
		u16 pending_idx;
		RING_IDX idx;
		int work_to_do;
		unsigned int data_len;
		pending_ring_idx_t index;
		int pool_idx;
		struct pending_tx_info *pending_tx_info;

		work_to_do = RING_HAS_UNCONSUMED_REQUESTS(&vif->tx);
		if (!work_to_do) {
			break;
		}

		idx = vif->tx.req_cons;
		rmb(); /* Ensure that we see the request before we copy it. */
		memcpy(&txreq, RING_GET_REQUEST(&vif->tx, idx), sizeof(txreq));

		/* Credit-based traffic shaping. */
		if (txreq.size > vif->remaining_credit &&
		    tx_credit_exceeded(vif, txreq.size)) {
			break;
		}

		vif->remaining_credit -= txreq.size;

		work_to_do--;
		vif->tx.req_cons = ++idx;

		memset(extras, 0, sizeof(extras));
		if (txreq.flags & XEN_NETTXF_extra_info) {
			work_to_do = xenvif_get_extras(vif, extras,
							  work_to_do);
			idx = vif->tx.req_cons;
			if (unlikely(work_to_do < 0)) {
				xenvif_tx_err(vif, &txreq, idx);
				break;
			}
		}

		ret = xenvif_count_requests(vif, &txreq, txfrags, work_to_do);
		if (unlikely(ret < 0)) {
			xenvif_tx_err(vif, &txreq, idx - ret);
			break;
		}
		idx += ret;

		if (unlikely(txreq.size < ETH_HLEN)) {
			netdev_dbg(vif->dev,
				   "Bad packet size: %d\n", txreq.size);
			xenvif_tx_err(vif, &txreq, idx);
			break;
		}

		/* No crossing a page as the payload mustn't fragment. */
		if (unlikely((txreq.offset + txreq.size) > PAGE_SIZE)) {
			netdev_dbg(vif->dev,
				   "txreq.offset: %x, size: %u, end: %lu\n",
				   txreq.offset, txreq.size,
				   (txreq.offset&~PAGE_MASK) + txreq.size);
			xenvif_tx_err(vif, &txreq, idx);
			break;
		}

		index = pending_index(vif->pending_cons);
		pending_idx = vif->pending_ring[index];

		data_len = (txreq.size > PKT_PROT_LEN &&
			    ret < MAX_SKB_FRAGS) ?
			PKT_PROT_LEN : txreq.size;

		skb = alloc_skb(data_len + NET_SKB_PAD + NET_IP_ALIGN,
				GFP_ATOMIC | __GFP_NOWARN);
		if (unlikely(skb == NULL)) {
			netdev_dbg(vif->dev,
				   "Can't allocate a skb in start_xmit.\n");
			xenvif_tx_err(vif, &txreq, idx);
			break;
		}

		/* Packets passed to netif_rx() must have some headroom. */
		skb_reserve(skb, NET_SKB_PAD + NET_IP_ALIGN);

		if (extras[XEN_NETIF_EXTRA_TYPE_GSO - 1].type) {
			struct xen_netif_extra_info *gso;
			gso = &extras[XEN_NETIF_EXTRA_TYPE_GSO - 1];

			if (xenvif_set_skb_gso(vif, skb, gso)) {
				kfree_skb(skb);
				xenvif_tx_err(vif, &txreq, idx);
				break;
			}
		}

		/* XXX could copy straight to head */
		page = xenvif_alloc_page(vif, skb, pending_idx);
		if (!page) {
			kfree_skb(skb);
			xenvif_tx_err(vif, &txreq, idx);
			break;
		}

		gop->source.u.ref = txreq.gref;
		gop->source.domid = vif->domid;
		gop->source.offset = txreq.offset;

		gop->dest.u.gmfn = virt_to_mfn(page_address(page));
		gop->dest.domid = DOMID_SELF;
		gop->dest.offset = txreq.offset;

		gop->len = txreq.size;
		gop->flags = GNTCOPY_source_gref;

		gop++;

		pool_idx = vif->mmap_pages[pending_idx];
		pending_tx_info = to_txinfo(pool_idx);

		memcpy(&pending_tx_info->req,
		       &txreq, sizeof(txreq));

		*((u16 *)skb->data) = pending_idx;

		__skb_put(skb, data_len);

		skb_shinfo(skb)->nr_frags = ret;
		if (data_len < txreq.size) {
			skb_shinfo(skb)->nr_frags++;
			frag_set_pending_idx(&skb_shinfo(skb)->frags[0],
					     pending_idx);
		} else {
			frag_set_pending_idx(&skb_shinfo(skb)->frags[0],
					     INVALID_PENDING_IDX);
		}

		__skb_queue_tail(&vif->tx_queue, skb);

		vif->pending_cons++;

		request_gop = xenvif_get_requests(vif,
						     skb, txfrags, gop);
		if (request_gop == NULL) {
			kfree_skb(skb);
			xenvif_tx_err(vif, &txreq, idx);
			break;
		}
		gop = request_gop;

		vif->tx.req_cons = idx;

		if ((gop - tco) >= MAX_PENDING_REQS)
			break;
	}

	return gop - tco;
}

static int xenvif_tx_submit(struct xenvif *vif,
			    struct gnttab_copy *tco,
			    int budget)
{
	struct gnttab_copy *gop = tco;
	struct sk_buff *skb;
	int work_done = 0;

	while ((work_done < budget) &&
	       (skb = __skb_dequeue(&vif->tx_queue)) != NULL) {
		struct xen_netif_tx_request *txp;
		u16 pending_idx;
		unsigned data_len;
		int idx;
		struct pending_tx_info *pending_tx_info;

		pending_idx = *((u16 *)skb->data);

		idx = vif->mmap_pages[pending_idx];
		pending_tx_info = to_txinfo(idx);

		txp = &pending_tx_info->req;

		/* Check the remap error code. */
		if (unlikely(xenvif_tx_check_gop(vif, skb, &gop))) {
			netdev_dbg(vif->dev, "netback grant failed.\n");
			skb_shinfo(skb)->nr_frags = 0;
			kfree_skb(skb);
			continue;
		}

		data_len = skb->len;
		memcpy(skb->data,
		       (void *)(idx_to_kaddr(vif, pending_idx)|txp->offset),
		       data_len);
		if (data_len < txp->size) {
			/* Append the packet payload as a fragment. */
			txp->offset += data_len;
			txp->size -= data_len;
		} else {
			/* Schedule a response immediately. */
			xenvif_idx_release(vif, pending_idx);
		}

		if (txp->flags & XEN_NETTXF_csum_blank)
			skb->ip_summed = CHECKSUM_PARTIAL;
		else if (txp->flags & XEN_NETTXF_data_validated)
			skb->ip_summed = CHECKSUM_UNNECESSARY;

		xenvif_fill_frags(vif, skb);

		/*
		 * If the initial fragment was < PKT_PROT_LEN then
		 * pull through some bytes from the other fragments to
		 * increase the linear region to PKT_PROT_LEN bytes.
		 */
		if (skb_headlen(skb) < PKT_PROT_LEN && skb_is_nonlinear(skb)) {
			int target = min_t(int, skb->len, PKT_PROT_LEN);
			__pskb_pull_tail(skb, target - skb_headlen(skb));
		}

		skb->dev      = vif->dev;
		skb->protocol = eth_type_trans(skb, skb->dev);

		if (checksum_setup(vif, skb)) {
			netdev_dbg(vif->dev,
				   "Can't setup checksum in net_tx_action\n");
			kfree_skb(skb);
			continue;
		}

		vif->dev->stats.rx_bytes += skb->len;
		vif->dev->stats.rx_packets++;

		work_done++;

		netif_receive_skb(skb);
	}

	return work_done;
}

/* Called after netfront has transmitted */

int xenvif_tx_action(struct xenvif *vif, int budget)
{
	unsigned nr_gops;
	int ret;
	int work_done;
	struct gnttab_copy *tco;
	static int unusable_count;

	if (unlikely(!tx_work_todo(vif)))
		return 0;

	tco = get_cpu_var(tx_copy_ops);

	if (tco == NULL) {
		put_cpu_var(tx_copy_ops);
		unusable_count++;
		if (unusable_count == 1000) {
			pr_alert("CPU %x scratch space"
				 " is not usable,"
				 " not doing any RX work for vif%u.%u\n",
				 smp_processor_id(),
				 vif->domid, vif->handle);
			unusable_count = 0;
		}
		return -ENOMEM;
	}

	nr_gops = xenvif_tx_build_gops(vif, tco);

	if (nr_gops == 0) {
		put_cpu_var(tx_copy_ops);
		return 0;
	}

	ret = HYPERVISOR_grant_table_op(GNTTABOP_copy,
					tco, nr_gops);
	BUG_ON(ret);

	work_done = xenvif_tx_submit(vif, tco, budget);

	put_cpu_var(tx_copy_ops);

	return work_done;
}

static void xenvif_idx_release(struct xenvif *vif, u16 pending_idx)
{
	struct pending_tx_info *pending_tx_info;
	pending_ring_idx_t index;
	int idx;

	/* Already complete? */
	if (vif->mmap_pages[pending_idx] == INVALID_ENTRY)
		return;

	idx = vif->mmap_pages[pending_idx];
	pending_tx_info = to_txinfo(idx);

	make_tx_response(vif, &pending_tx_info->req, XEN_NETIF_RSP_OKAY);

	index = pending_index(vif->pending_prod++);
	vif->pending_ring[index] = pending_idx;

	page_pool_put(vif->mmap_pages[pending_idx]);

	vif->mmap_pages[pending_idx] = INVALID_ENTRY;
}

static void make_tx_response(struct xenvif *vif,
			     struct xen_netif_tx_request *txp,
			     s8       st)
{
	RING_IDX i = vif->tx.rsp_prod_pvt;
	struct xen_netif_tx_response *resp;
	int notify;

	resp = RING_GET_RESPONSE(&vif->tx, i);
	resp->id     = txp->id;
	resp->status = st;

	if (txp->flags & XEN_NETTXF_extra_info)
		RING_GET_RESPONSE(&vif->tx, ++i)->status = XEN_NETIF_RSP_NULL;

	vif->tx.rsp_prod_pvt = ++i;
	RING_PUSH_RESPONSES_AND_CHECK_NOTIFY(&vif->tx, notify);
	if (notify)
		notify_remote_via_irq(vif->tx_irq);
}

static struct xen_netif_rx_response *make_rx_response(struct xenvif *vif,
					     u16      id,
					     s8       st,
					     u16      offset,
					     u16      size,
					     u16      flags)
{
	RING_IDX i = vif->rx.rsp_prod_pvt;
	struct xen_netif_rx_response *resp;

	resp = RING_GET_RESPONSE(&vif->rx, i);
	resp->offset     = offset;
	resp->flags      = flags;
	resp->id         = id;
	resp->status     = (s16)size;
	if (st < 0)
		resp->status = (s16)st;

	vif->rx.rsp_prod_pvt = ++i;

	return resp;
}

static inline int rx_work_todo(struct xenvif *vif)
{
	return !skb_queue_empty(&vif->rx_queue);
}

static inline int tx_work_todo(struct xenvif *vif)
{
	if (likely(RING_HAS_UNCONSUMED_REQUESTS(&vif->tx)) &&
	    (nr_pending_reqs(vif) + MAX_SKB_FRAGS) < MAX_PENDING_REQS)
		return 1;

	return 0;
}

void xenvif_unmap_frontend_ring(struct xenvif *vif, void *addr)
{
	if (addr)
		xenbus_unmap_ring_vfree(xenvif_to_xenbus_device(vif), addr);
}

int xenvif_map_frontend_ring(struct xenvif *vif,
			      void **vaddr,
			      int domid,
			      int ring_ref[],
			      unsigned int  ring_ref_count)
{
	int err = 0;

	err = xenbus_map_ring_valloc(xenvif_to_xenbus_device(vif),
				     ring_ref, ring_ref_count, vaddr);
	return err;
}

int xenvif_kthread(void *data)
{
	struct xenvif *vif = data;

	while (!kthread_should_stop()) {
		wait_event_interruptible(vif->wq,
					 rx_work_todo(vif) ||
					 kthread_should_stop());
		cond_resched();

		if (kthread_should_stop())
			break;

		if (rx_work_todo(vif))
			xenvif_rx_action(vif);
	}

	return 0;
}

static int __create_percpu_scratch_space(unsigned int cpu)
{
	if (per_cpu(tx_copy_ops, cpu) ||
	    per_cpu(grant_copy_op, cpu) ||
	    per_cpu(meta, cpu))
		return 0;

	per_cpu(tx_copy_ops, cpu) =
		vzalloc_node(sizeof(struct gnttab_copy) * MAX_PENDING_REQS,
			     cpu_to_node(cpu));

	if (!per_cpu(tx_copy_ops, cpu))
		per_cpu(tx_copy_ops, cpu) = vzalloc(sizeof(struct gnttab_copy)
						    * MAX_PENDING_REQS);

	per_cpu(grant_copy_op, cpu) =
		vzalloc_node(sizeof(struct gnttab_copy)
			     * 2 * NETBK_MAX_RX_RING_SIZE, cpu_to_node(cpu));

	if (!per_cpu(grant_copy_op, cpu))
		per_cpu(grant_copy_op, cpu) =
			vzalloc(sizeof(struct gnttab_copy)
				* 2 * NETBK_MAX_RX_RING_SIZE);

	per_cpu(meta, cpu) = vzalloc_node(sizeof(struct xenvif_rx_meta)
					  * 2 * NETBK_MAX_RX_RING_SIZE,
					  cpu_to_node(cpu));
	if (!per_cpu(meta, cpu))
		per_cpu(meta, cpu) = vzalloc(sizeof(struct xenvif_rx_meta)
					     * 2 * NETBK_MAX_RX_RING_SIZE);

	if (!per_cpu(tx_copy_ops, cpu) ||
	    !per_cpu(grant_copy_op, cpu) ||
	    !per_cpu(meta, cpu))
		return -ENOMEM;

	return 0;
}

static void __free_percpu_scratch_space(unsigned int cpu)
{
	/* freeing NULL pointer is legit */
	/* carefully work around race condition */
	void *tmp;
	tmp = per_cpu(tx_copy_ops, cpu);
	per_cpu(tx_copy_ops, cpu) = NULL;
	vfree(tmp);

	tmp = per_cpu(grant_copy_op, cpu);
	per_cpu(grant_copy_op, cpu) = NULL;
	vfree(tmp);

	tmp = per_cpu(meta, cpu);
	per_cpu(meta, cpu) = NULL;
	vfree(tmp);
}

static int __netback_percpu_callback(struct notifier_block *nfb,
				     unsigned long action, void *hcpu)
{
	unsigned int cpu = (unsigned long)hcpu;
	int rc = NOTIFY_DONE;

	switch (action) {
	case CPU_ONLINE:
	case CPU_ONLINE_FROZEN:
		pr_info("CPU %x online, creating scratch space\n", cpu);
		rc = __create_percpu_scratch_space(cpu);
		if (rc) {
			pr_alert("failed to create scratch space"
				 " for CPU %x\n", cpu);
			/* FIXME: nothing more we can do here, we will
			 * print out warning message when thread or
			 * NAPI runs on this cpu. Also stop getting
			 * called in the future.
			 */
			__free_percpu_scratch_space(cpu);
			rc = NOTIFY_BAD;
		} else {
			rc = NOTIFY_OK;
		}
		break;
	case CPU_DEAD:
	case CPU_DEAD_FROZEN:
		pr_info("CPU %x offline, destroying scratch space\n",
			cpu);
		__free_percpu_scratch_space(cpu);
		rc = NOTIFY_OK;
		break;
	default:
		break;
	}

	return rc;
}

static struct notifier_block netback_notifier_block = {
	.notifier_call = __netback_percpu_callback,
};

static int __init netback_init(void)
{
	int rc = -ENOMEM;
	int cpu;

	if (!xen_domain())
		return -ENODEV;

	/* Don't need to disable preempt here, since nobody else will
	 * touch these percpu areas during start up. */
	for_each_online_cpu(cpu) {
		rc = __create_percpu_scratch_space(cpu);

		if (rc)
			goto failed_init;
	}

	register_hotcpu_notifier(&netback_notifier_block);

	rc = page_pool_init();
	if (rc)
		goto failed_init_pool;

	rc = xenvif_xenbus_init();
	if (rc)
		goto failed_init_xenbus;

	return rc;

failed_init_xenbus:
	page_pool_destroy();
failed_init_pool:
	unregister_hotcpu_notifier(&netback_notifier_block);
failed_init:
	for_each_online_cpu(cpu)
		__free_percpu_scratch_space(cpu);
	return rc;
}

module_init(netback_init);

static void __exit netback_exit(void)
{
	int cpu;

	xenvif_xenbus_exit();
	page_pool_destroy();

	unregister_hotcpu_notifier(&netback_notifier_block);

	/* Since we're here, nobody else will touch per-cpu area. */
	for_each_online_cpu(cpu)
		__free_percpu_scratch_space(cpu);
}
module_exit(netback_exit);

MODULE_LICENSE("Dual BSD/GPL");
MODULE_ALIAS("xen-backend:vif");
