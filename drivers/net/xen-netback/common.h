/*
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

#ifndef __XEN_NETBACK__COMMON_H__
#define __XEN_NETBACK__COMMON_H__

#define pr_fmt(fmt) KBUILD_MODNAME ":%s: " fmt, __func__

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/ip.h>
#include <linux/in.h>
#include <linux/io.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/wait.h>
#include <linux/sched.h>

#include <xen/interface/io/netif.h>
#include <xen/interface/grant_table.h>
#include <xen/grant_table.h>
#include <xen/xenbus.h>

#define DRV_NAME "netback: "
#include "page_pool.h"

struct xenvif_rx_meta {
	int id;
	int size;
	int gso_size;
};

/* Discriminate from any valid pending_idx value. */
#define INVALID_PENDING_IDX 0xFFFF

#define MAX_BUFFER_OFFSET PAGE_SIZE

#define NETBK_TX_RING_SIZE(_nr_pages)					\
	(__CONST_RING_SIZE(xen_netif_tx, PAGE_SIZE * (_nr_pages)))
#define NETBK_RX_RING_SIZE(_nr_pages)					\
	(__CONST_RING_SIZE(xen_netif_rx, PAGE_SIZE * (_nr_pages)))

#define NETBK_MAX_RING_PAGE_ORDER XENBUS_MAX_RING_PAGE_ORDER
#define NETBK_MAX_RING_PAGES      (1U << NETBK_MAX_RING_PAGE_ORDER)

#define NETBK_MAX_TX_RING_SIZE NETBK_TX_RING_SIZE(NETBK_MAX_RING_PAGES)
#define NETBK_MAX_RX_RING_SIZE NETBK_RX_RING_SIZE(NETBK_MAX_RING_PAGES)

#define MAX_PENDING_REQS NETBK_MAX_TX_RING_SIZE

struct xenvif {
	/* Unique identifier for this interface. */
	domid_t          domid;
	unsigned int     handle;

	/* Use NAPI for guest TX */
	struct napi_struct napi;
	/* Use kthread for guest RX */
	struct task_struct *task;
	wait_queue_head_t wq;

	u8               fe_dev_addr[6];

	/* When feature-split-event-channels = 0, tx_irq = rx_irq */
	unsigned int tx_irq;
	unsigned int rx_irq;

	/* The shared rings and indexes. */
	struct xen_netif_tx_back_ring tx;
	struct xen_netif_rx_back_ring rx;
	int nr_tx_handles;
	int nr_rx_handles;

	/* Frontend feature information. */
	u8 can_sg:1;
	u8 gso:1;
	u8 gso_prefix:1;
	u8 csum:1;

	/* Internal feature information. */
	u8 can_queue:1;	    /* can queue packets for receiver? */

	/*
	 * Allow xenvif_start_xmit() to peek ahead in the rx request
	 * ring.  This is a prediction of what rx_req_cons will be
	 * once all queued skbs are put on the ring.
	 */
	RING_IDX rx_req_cons_peek;

	/* Transmit shaping: allow 'credit_bytes' every 'credit_usec'. */
	unsigned long   credit_bytes;
	unsigned long   credit_usec;
	unsigned long   remaining_credit;
	struct timer_list credit_timeout;

	/* Statistics */
	unsigned long rx_gso_checksum_fixup;

	/* Miscellaneous private stuff. */
	struct net_device *dev;

	struct sk_buff_head rx_queue;
	struct sk_buff_head tx_queue;

	idx_t mmap_pages[MAX_PENDING_REQS];

	pending_ring_idx_t pending_prod;
	pending_ring_idx_t pending_cons;

	u16 pending_ring[MAX_PENDING_REQS];
};

static inline struct xenbus_device *xenvif_to_xenbus_device(struct xenvif *vif)
{
	return to_xenbus_device(vif->dev->dev.parent);
}

struct xenvif *xenvif_alloc(struct device *parent,
			    domid_t domid,
			    unsigned int handle);

int xenvif_connect(struct xenvif *vif,
		   unsigned long tx_ring_ref[], unsigned int tx_ring_order,
		   unsigned long rx_ring_ref[], unsigned int rx_ring_order,
		   unsigned int tx_evtchn, unsigned int rx_evtchn);
void xenvif_disconnect(struct xenvif *vif);

int xenvif_xenbus_init(void);
void xenvif_xenbus_exit(void);

int xenvif_schedulable(struct xenvif *vif);

int xenvif_rx_ring_full(struct xenvif *vif);

int xenvif_must_stop_queue(struct xenvif *vif);

/* (Un)Map communication rings. */
void xenvif_unmap_frontend_ring(struct xenvif *vif, void *addr);
int xenvif_map_frontend_ring(struct xenvif *vif,
			     void **vaddr,
			     int domid,
			     int ring_ref[],
			     unsigned int  ring_ref_count);

/* Check for SKBs from frontend and schedule backend processing */
void xenvif_check_rx_xenvif(struct xenvif *vif);

/* Queue an SKB for transmission to the frontend */
void xenvif_queue_tx_skb(struct xenvif *vif, struct sk_buff *skb);
/* Notify xenvif that ring now has space to send an skb to the frontend */
void xenvif_notify_tx_completion(struct xenvif *vif);

/* Returns number of ring slots required to send an skb to the frontend */
unsigned int xenvif_count_skb_slots(struct xenvif *vif, struct sk_buff *skb);

int xenvif_tx_action(struct xenvif *vif, int budget);
void xenvif_rx_action(struct xenvif *vif);

int xenvif_kthread(void *data);

extern unsigned int MODPARM_netback_max_tx_ring_page_order;
extern unsigned int MODPARM_netback_max_rx_ring_page_order;

#endif /* __XEN_NETBACK__COMMON_H__ */
