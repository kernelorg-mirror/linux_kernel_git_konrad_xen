/*
 * Global page pool for netback.
 *
 * Wei Liu <wei.liu2@citrix.com>
 * Copyright (c) Citrix Systems
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
#include <asm/xen/page.h>

static idx_t free_head;
static int free_count;
static unsigned long pool_size;
static DEFINE_SPINLOCK(pool_lock);
static struct page_pool_entry *pool;

static int get_free_entry(void)
{
	int idx;

	spin_lock(&pool_lock);

	if (free_count == 0) {
		spin_unlock(&pool_lock);
		return -ENOSPC;
	}

	idx = free_head;
	free_count--;
	free_head = pool[idx].u.fl;
	pool[idx].u.fl = INVALID_ENTRY;

	spin_unlock(&pool_lock);

	return idx;
}

static void put_free_entry(idx_t idx)
{
	spin_lock(&pool_lock);

	pool[idx].u.fl = free_head;
	free_head = idx;
	free_count++;

	spin_unlock(&pool_lock);
}

static inline void set_page_ext(struct page *pg, unsigned int idx)
{
	union page_ext ext = { .idx = idx };

	BUILD_BUG_ON(sizeof(ext) > sizeof(ext.mapping));
	pg->mapping = ext.mapping;
}

static int get_page_ext(struct page *pg, unsigned int *pidx)
{
	union page_ext ext = { .mapping = pg->mapping };
	int idx;

	idx = ext.idx;

	if ((idx < 0) || (idx >= pool_size))
		return 0;

	if (pool[idx].page != pg)
		return 0;

	*pidx = idx;

	return 1;
}

int is_in_pool(struct page *page, int *pidx)
{
	return get_page_ext(page, pidx);
}

struct page *page_pool_get(struct xenvif *vif, int *pidx)
{
	int idx;
	struct page *page;

	idx = get_free_entry();
	if (idx < 0)
		return NULL;
	page = alloc_page(GFP_ATOMIC);

	if (page == NULL) {
		put_free_entry(idx);
		return NULL;
	}

	set_page_ext(page, idx);
	pool[idx].u.vif = vif;
	pool[idx].page = page;

	*pidx = idx;

	return page;
}

void page_pool_put(int idx)
{
	struct page *page = pool[idx].page;

	pool[idx].page = NULL;
	pool[idx].u.vif = NULL;
	page->mapping = 0;
	put_page(page);
	put_free_entry(idx);
}

int page_pool_init()
{
	int cpus = 0;
	int i;

	cpus = num_online_cpus();
	pool_size = cpus * ENTRIES_PER_CPU;

	pool = vzalloc(sizeof(struct page_pool_entry) * pool_size);

	if (!pool)
		return -ENOMEM;

	for (i = 0; i < pool_size - 1; i++)
		pool[i].u.fl = i+1;
	pool[pool_size-1].u.fl = INVALID_ENTRY;
	free_count = pool_size;
	free_head = 0;

	return 0;
}

void page_pool_destroy()
{
	int i;
	for (i = 0; i < pool_size; i++)
		if (pool[i].page)
			put_page(pool[i].page);

	vfree(pool);
}

struct page *to_page(int idx)
{
	return pool[idx].page;
}

struct xenvif *to_vif(int idx)
{
	return pool[idx].u.vif;
}

struct pending_tx_info *to_txinfo(int idx)
{
	return &pool[idx].tx_info;
}
