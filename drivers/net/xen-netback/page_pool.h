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

#ifndef __PAGE_POOL_H__
#define __PAGE_POOL_H__

struct pending_tx_info {
	struct xen_netif_tx_request req;
};
typedef unsigned int pending_ring_idx_t;

typedef uint32_t idx_t;

#define ENTRIES_PER_CPU (1024)
#define INVALID_ENTRY 0xffffffff

struct page_pool_entry {
	struct page *page;
	struct pending_tx_info tx_info;
	union {
		struct xenvif *vif;
		idx_t          fl;
	} u;
};

union page_ext {
	idx_t idx;
	void *mapping;
};

int  page_pool_init(void);
void page_pool_destroy(void);


struct page *page_pool_get(struct xenvif *vif, int *pidx);
void         page_pool_put(int idx);
int          is_in_pool(struct page *page, int *pidx);

struct page            *to_page(int idx);
struct xenvif          *to_vif(int idx);
struct pending_tx_info *to_txinfo(int idx);

#endif /* __PAGE_POOL_H__ */
