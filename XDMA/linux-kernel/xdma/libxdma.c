/*
 * This file is part of the Xilinx DMA IP Core driver for Linux
 *
 * Copyright (c) 2016-present,  Xilinx, Inc.
 * All rights reserved.
 *
 * This source code is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * The full GNU General Public License is included in this distribution in
 * the file called "COPYING".
 */

#define pr_fmt(fmt) KBUILD_MODNAME ":%s: " fmt, __func__

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/vmalloc.h>

#include "libxdma.h"
#include "xdma_cdev.h"



/* Module Parameters */
#ifdef XDMA_CONFIG_BAR_NUM
static int config_bar_num=XDMA_CONFIG_BAR_NUM;
#else 
static int config_bar_num=-1;
#endif
module_param(config_bar_num, int, 0444);/*no point to make it writable*/
MODULE_PARM_DESC(config_bar_num, "Input the config block BAR number");

static const unsigned int interrupt_mode=0;

#ifdef XDMA_C2H_CREDITS
static unsigned int enable_st_c2h_credit = XDMA_C2H_CREDITS;
#else
static unsigned int enable_st_c2h_credit = 0;
#endif
module_param(enable_st_c2h_credit, uint, 0644);
MODULE_PARM_DESC(enable_st_c2h_credit,
	"Set 1 to enable ST C2H engine credit feature, default is 0 ( credit control disabled)");

#ifdef XDMA_H2C_TIMEOUT
unsigned int h2c_timeout_ms = XDMA_H2C_TIMEOUT;
#else
unsigned int h2c_timeout_ms = 5000;
#endif
module_param(h2c_timeout_ms, uint, 0644);
MODULE_PARM_DESC(h2c_timeout_ms, "H2C sgdma timeout in milliseconds, default is 10 seconds.");

#ifdef XDMA_C2H_TIMEOUT
unsigned int c2h_timeout_ms = XDMA_C2H_TIMEOUT;
#else 
unsigned int c2h_timeout_ms = 5000;
#endif
module_param(c2h_timeout_ms, uint, 0644);
MODULE_PARM_DESC(c2h_timeout_ms, "C2H sgdma timeout in milliseconds, default is 10 seconds.");



#define XDMA_PERF_NUM_DESC 128

#define const_cast(type, var) *((type*) &(var))

#if !LINUX_VERSION_CHECK(4,13,0)
#define __GFP_RETRY_MAYFAIL __GFP_RETRY
#elif !LINUX_VERSION_CHECK(4,12,0)
#error "Minimum supported Linux version is 4.12"
#endif

/*
 * xdma device management
 * maintains a list of the xdma devices
 */
static LIST_HEAD(xdev_list);
static DEFINE_MUTEX(xdev_mutex);

static LIST_HEAD(xdev_rcu_list);
static DEFINE_SPINLOCK(xdev_rcu_lock);

#ifndef list_last_entry
#define list_last_entry(ptr, type, member) list_entry((ptr)->prev, type, member)
#endif

static inline int xdev_list_add(struct xdma_dev *xdev)
{
	mutex_lock(&xdev_mutex);
	if (list_empty(&xdev_list)) {
		xdev->idx = 0;
		
	} else {
		struct xdma_dev *last;

		last = list_last_entry(&xdev_list, struct xdma_dev, list_head);
		xdev->idx = last->idx + 1;
	}
	list_add_tail(&xdev->list_head, &xdev_list);
	mutex_unlock(&xdev_mutex);

	dbg_init("dev %s, xdev 0x%p, xdma idx %d.\n",
		 dev_name(&xdev->pdev->dev), xdev, xdev->idx);

	spin_lock(&xdev_rcu_lock);
	list_add_tail_rcu(&xdev->rcu_node, &xdev_rcu_list);
	spin_unlock(&xdev_rcu_lock);

	return 0;
}

#undef list_last_entry

static inline void xdev_list_remove(struct xdma_dev *xdev)
{
	mutex_lock(&xdev_mutex);
	list_del(&xdev->list_head);
	mutex_unlock(&xdev_mutex);

	spin_lock(&xdev_rcu_lock);
	list_del_rcu(&xdev->rcu_node);
	spin_unlock(&xdev_rcu_lock);
	synchronize_rcu();
}

struct xdma_dev *xdev_find_by_pdev(struct pci_dev *pdev)
{
	struct xdma_dev *xdev, *tmp;

	mutex_lock(&xdev_mutex);
	list_for_each_entry_safe(xdev, tmp, &xdev_list, list_head) {
		if (xdev->pdev == pdev) {
			mutex_unlock(&xdev_mutex);
			return xdev;
		}
	}
	mutex_unlock(&xdev_mutex);
	return NULL;
}

static inline int debug_check_dev_hndl(const char *fname, struct pci_dev *pdev,
				       void *hndl)
{
	struct xdma_dev *xdev;

	if (!pdev)
		return -EINVAL;

	xdev = xdev_find_by_pdev(pdev);
	if (!xdev) {
		pr_info("%s pdev 0x%p, hndl 0x%p, NO match found!\n", fname,
			pdev, hndl);
		return -EINVAL;
	}
	if (xdev != hndl) {
		pr_err("%s pdev 0x%p, hndl 0x%p != 0x%p!\n", fname, pdev, hndl,
		       xdev);
		return -EINVAL;
	}

	return 0;
}

#ifdef __LIBXDMA_DEBUG__
/* SECTION: Function definitions */
#define write_register(v, mem, off) \
do {\
pr_info("w reg %s: 0x%lx(0x%p), 0x%x.\n", #mem, off, mem, v); \
	iowrite32(v, mem);\
}while(0)
#else
#define write_register(v, mem, off) iowrite32(v, mem)
#endif

inline u32 read_register(void *iomem)
{
	return ioread32(iomem);
}

static inline u32 build_u32(u32 hi, u32 lo)
{
	return ((hi & 0xFFFFUL) << 16) | (lo & 0xFFFFUL);
}

static inline u64 build_u64(u64 hi, u64 lo)
{
	return ((hi & 0xFFFFFFFULL) << 32) | (lo & 0xFFFFFFFFULL);
}

static void check_nonzero_interrupt_status(struct xdma_dev *xdev)
{
	struct interrupt_regs *reg =
		(struct interrupt_regs *)(xdev->bar[xdev->config_bar_idx] +
					  XDMA_OFS_INT_CTRL);
	u32 w;

	w = read_register(&reg->user_int_enable);
	if (w)
		pr_info("%s xdma%d user_int_enable = 0x%08x\n",
			dev_name(&xdev->pdev->dev), xdev->idx, w);

	w = read_register(&reg->channel_int_enable);
	if (w)
		pr_info("%s xdma%d channel_int_enable = 0x%08x\n",
			dev_name(&xdev->pdev->dev), xdev->idx, w);

	w = read_register(&reg->user_int_request);
	if (w)
		pr_info("%s xdma%d user_int_request = 0x%08x\n",
			dev_name(&xdev->pdev->dev), xdev->idx, w);
	w = read_register(&reg->channel_int_request);
	if (w)
		pr_info("%s xdma%d channel_int_request = 0x%08x\n",
			dev_name(&xdev->pdev->dev), xdev->idx, w);

	w = read_register(&reg->user_int_pending);
	if (w)
		pr_info("%s xdma%d user_int_pending = 0x%08x\n",
			dev_name(&xdev->pdev->dev), xdev->idx, w);
	w = read_register(&reg->channel_int_pending);
	if (w)
		pr_info("%s xdma%d channel_int_pending = 0x%08x\n",
			dev_name(&xdev->pdev->dev), xdev->idx, w);
}

/* channel_interrupts_enable -- Enable interrupts we are interested in */
static void channel_interrupts_enable(struct xdma_dev *xdev, u32 mask)
{
	struct interrupt_regs *reg =
		(struct interrupt_regs *)(xdev->bar[xdev->config_bar_idx] +
					  XDMA_OFS_INT_CTRL);

	write_register(mask, &reg->channel_int_enable_w1s, XDMA_OFS_INT_CTRL);
}

/* channel_interrupts_disable -- Disable interrupts we not interested in */
static void channel_interrupts_disable(struct xdma_dev *xdev, u32 mask)
{
	struct interrupt_regs *reg =
		(struct interrupt_regs *)(xdev->bar[xdev->config_bar_idx] +
					  XDMA_OFS_INT_CTRL);

	write_register(mask, &reg->channel_int_enable_w1c, XDMA_OFS_INT_CTRL);
}

/* user_interrupts_enable -- Enable interrupts we are interested in */
static void user_interrupts_enable(struct xdma_dev *xdev, u32 mask)
{
	struct interrupt_regs *reg =
		(struct interrupt_regs *)(xdev->bar[xdev->config_bar_idx] +
					  XDMA_OFS_INT_CTRL);

	write_register(mask, &reg->user_int_enable_w1s, XDMA_OFS_INT_CTRL);
}

/* user_interrupts_disable -- Disable interrupts we not interested in */
static void user_interrupts_disable(struct xdma_dev *xdev, u32 mask)
{
	struct interrupt_regs *reg =
		(struct interrupt_regs *)(xdev->bar[xdev->config_bar_idx] +
					  XDMA_OFS_INT_CTRL);

	write_register(mask, &reg->user_int_enable_w1c, XDMA_OFS_INT_CTRL);
}

/* read_interrupts -- Print the interrupt controller status */
static u32 read_interrupts(struct xdma_dev *xdev)
{
	struct interrupt_regs *reg =
		(struct interrupt_regs *)(xdev->bar[xdev->config_bar_idx] +
					  XDMA_OFS_INT_CTRL);
	u32 lo;
	u32 hi;

	/* extra debugging; inspect complete engine set of registers */
	hi = read_register(&reg->user_int_request);
	dbg_io("ioread32(0x%p) returned 0x%08x (user_int_request).\n",
	       &reg->user_int_request, hi);
	lo = read_register(&reg->channel_int_request);
	dbg_io("ioread32(0x%p) returned 0x%08x (channel_int_request)\n",
	       &reg->channel_int_request, lo);

	/* return interrupts: user in upper 16-bits, channel in lower 16-bits */
	return build_u32(hi, lo);
}

void enable_perf(struct xdma_engine *engine)
{
	u32 w;

	w = XDMA_PERF_CLEAR;
	write_register(w, &engine->regs->perf_ctrl,
		       (unsigned long)(&engine->regs->perf_ctrl) -
			       (unsigned long)(&engine->regs));
	read_register(&engine->regs->identifier);
	w = XDMA_PERF_AUTO | XDMA_PERF_RUN;
	write_register(w, &engine->regs->perf_ctrl,
		       (unsigned long)(&engine->regs->perf_ctrl) -
			       (unsigned long)(&engine->regs));
	read_register(&engine->regs->identifier);

	dbg_perf("XDMA_IOCTL_PERF_START\n");
}

void get_perf_stats(struct xdma_engine *engine)
{
	u32 hi;
	u32 lo;

	if (!engine) {
		pr_err("dma engine NULL\n");
		return;
	}

		
	hi = read_register(&engine->regs->perf_cyc_hi);
	lo = read_register(&engine->regs->perf_cyc_lo);

	engine->xdma_perf.clock_cycle_count = build_u64(hi, lo);

	hi = read_register(&engine->regs->perf_dat_hi);
	lo = read_register(&engine->regs->perf_dat_lo);
	engine->xdma_perf.data_cycle_count = build_u64(hi, lo);

	
}


/*Checks status and returns if an error occured, in which case its prints errors*/
static bool engine_process_status(struct xdma_engine *engine, u32 status)
{
	
	const u32 mask_good=(XDMA_STAT_DESC_STOPPED|XDMA_STAT_DESC_COMPLETED);
	const u32 mask_error=~(mask_good|XDMA_STAT_BUSY|XDMA_STAT_IDLE_STOPPED);

	dbg_tfr("XDMA engine %s status: 0x%08x \n", engine->name, status);
	
	/* check if an error caused the interrupt*/
	
	if(unlikely(status & mask_error))
	{
		pr_err("XDMA engine %s reports following error(s):\n", engine->name);
	
		if (status & XDMA_STAT_ALIGN_MISMATCH)
			pr_err("Source and destination adress are not properly" 
			"aligned to each other\n");
		if (status & XDMA_STAT_MAGIC_STOPPED)
			pr_err("A descriptor has invalid magic bit field\n");
		if (status & XDMA_STAT_INVALID_LEN)
			pr_err("Length of a descriptor is not multiple of datapath width\n");

		if (engine->dir == DMA_TO_DEVICE) 
		{
			/* H2C only */
			if (status & XDMA_STAT_H2C_R_ERR_MASK) 
			{
				pr_err("PCI-E read error(s): ");
				if (status & XDMA_STAT_H2C_R_UNSUPP_REQ)
					pr_cont("unsupported request ");
				if (status & XDMA_STAT_H2C_R_COMPL_ABORT)
					pr_cont("completer abort ");
				if (status & XDMA_STAT_H2C_R_PARITY_ERR)
					pr_cont("parity error ");
				if (status & XDMA_STAT_H2C_R_HEADER_EP)
					pr_cont("header EP ");
				if (status & XDMA_STAT_H2C_R_UNEXP_COMPL)
					pr_cont("unexpected completeion ");
				
				pr_cont("\n");

			}

			if ((status & XDMA_STAT_H2C_W_ERR_MASK)) 
			{
				pr_err("PCI-E write error: ");
				if (status & XDMA_STAT_H2C_W_DECODE_ERR)
					pr_cont("decode error ");
				if (status & XDMA_STAT_H2C_W_SLAVE_ERR)
					pr_cont("slave error ");
				
				pr_cont("\n");
			}

		} 
		else 
		{
			/* C2H only */
			if ((status & XDMA_STAT_C2H_R_ERR_MASK)) 
			{
				pr_err("PCI-E read error(s): ");
			
				if (status & XDMA_STAT_C2H_R_DECODE_ERR)
					pr_cont("decode error ");
				if (status & XDMA_STAT_C2H_R_SLAVE_ERR)
					pr_cont("slave error ");
				
				pr_cont("\n");				
			}
		}

		/* common H2C & C2H */
		if ((status & XDMA_STAT_DESC_ERR_MASK)) 
		{
			pr_err("Descriptor error(s): ");
			if (status & XDMA_STAT_DESC_UNSUPP_REQ)
				pr_cont("unsupported request ");
			if (status & XDMA_STAT_DESC_COMPL_ABORT)
				pr_cont("completer abort ");
			if (status & XDMA_STAT_DESC_PARITY_ERR)
				pr_cont("parity error ");
			if (status & XDMA_STAT_DESC_HEADER_EP)
				pr_cont("header EP ");
			if (status & XDMA_STAT_DESC_UNEXP_COMPL)
				pr_cont("unexpected completeion ");
				
			pr_cont("\n");
		}
		dbg_tfr("%u descriptors were completed.\n", ioread32( &(engine->regs->completed_desc_count)));
		return false;
	}
	else if( likely(status & mask_good))
	{
		dbg_tfr("Transfer on engine %s completed successfully\n", engine->name);
		return true;
	}
	else
	{
		pr_err("unexpected status %x on engine %s\n", status, engine->name);
		return false;
	}
	
}

static int xdma_engine_stop(struct xdma_engine *engine)
{
	u32 w=0x1;

	xdma_debug_assert_ptr(engine);
	/* make no sense to write all the flags again. Just clear the Run bit*/

	dbg_tfr("Stopping SG DMA %s engine; writing 0x%08x to 0x%p.\n",
		engine->name, w, (u32 *)&engine->regs->control_w1c);
	write_register(w, &engine->regs->control_w1c,
			(unsigned long)(&engine->regs->control_w1c) -
				(unsigned long)(&engine->regs));
	/* dummy read of status register to flush all previous writes */
	dbg_tfr("%s(%s) done\n", __func__, engine->name);
	engine->running = 0;
	return 0;
}

static irqreturn_t user_irq_service(int irq, struct xdma_user_irq *user_irq)
{
	unsigned long flags;

	if (!user_irq) {
		pr_err("Invalid user_irq\n");
		return IRQ_NONE;
	}

	if (user_irq->handler)
		return user_irq->handler(user_irq->user_idx, user_irq->dev);

	spin_lock_irqsave(&(user_irq->events_lock), flags);
	if (!user_irq->events_irq) {
		user_irq->events_irq = 1;
		wake_up_interruptible(&(user_irq->events_wq));
	}
	spin_unlock_irqrestore(&(user_irq->events_lock), flags);

	return IRQ_HANDLED;
}

/*
 * xdma_isr() - Interrupt handler
 *
 * @dev_id pointer to xdma_dev
 */
static irqreturn_t xdma_isr(int irq, void *dev_id)
{
	u32 ch_irq;
	u32 user_irq;
	u32 mask;
	struct xdma_dev *xdev;
	struct interrupt_regs *irq_regs;

	dbg_irq("(irq=%d, dev 0x%p) <<<< ISR.\n", irq, dev_id);
	if (!dev_id) {
		pr_err("Invalid dev_id on irq line %d\n", irq);
		return -IRQ_NONE;
	}
	xdev = (struct xdma_dev *)dev_id;

	if (!xdev) {
		WARN_ON(!xdev);
		dbg_irq("%s(irq=%d) xdev=%p ??\n", __func__, irq, xdev);
		return IRQ_NONE;
	}

	irq_regs = (struct interrupt_regs *)(xdev->bar[xdev->config_bar_idx] +
					     XDMA_OFS_INT_CTRL);

	/* read channel interrupt requests */
	ch_irq = read_register(&irq_regs->channel_int_request);
	dbg_irq("ch_irq = 0x%08x\n", ch_irq);


	/* read user interrupts  */
	user_irq = read_register(&irq_regs->user_int_request);
	dbg_irq("user_irq = 0x%08x\n", user_irq);

	if (user_irq) {
		int user = 0;
		u32 mask = 1;
		int max = xdev->user_max;

		for (; user < max && user_irq; user++, mask <<= 1) {
			if (user_irq & mask) {
				user_irq &= ~mask;
				user_irq_service(irq, &xdev->user_irq[user]);
			}
		}
	}

	mask = ch_irq & xdev->mask_irq_h2c;
	if (mask) {
		int channel = 0;
		int max = xdev->h2c_channel_num;

		/* iterate over H2C (PCIe read) */
		for (channel = 0; channel < max && mask; channel++) {
			struct xdma_engine *engine = &xdev->engine_h2c[channel];

			/* engine present and its interrupt fired? */
			if ((engine->irq_bitmask & mask) &&
			    (engine->magic == MAGIC_ENGINE)) {
				mask &= ~engine->irq_bitmask;
				dbg_tfr("complete, %s.\n", engine->name);
				complete(&(engine->engine_compl));
			}
		}
	}

	mask = ch_irq & xdev->mask_irq_c2h;
	if (mask) {
		int channel = 0;
		int max = xdev->c2h_channel_num;

		/* iterate over C2H (PCIe write) */
		for (channel = 0; channel < max && mask; channel++) {
			struct xdma_engine *engine = &xdev->engine_c2h[channel];

			/* engine present and its interrupt fired? */
			if ((engine->irq_bitmask & mask) &&
			    (engine->magic == MAGIC_ENGINE)) {
				mask &= ~engine->irq_bitmask;
				dbg_tfr("complete, %s.\n", engine->name);
				complete(&(engine->engine_compl));
			}
		}
	}

	xdev->irq_count++;
	return IRQ_HANDLED;
}

/*
 * xdma_user_irq() - Interrupt handler for user interrupts in MSI-X mode
 *
 * @dev_id pointer to xdma_dev
 */
static irqreturn_t xdma_user_irq(int irq, void *dev_id)
{
	struct xdma_user_irq *user_irq;

	dbg_irq("(irq=%d) <<<< INTERRUPT SERVICE ROUTINE\n", irq);

	if (!dev_id) {
		pr_err("Invalid dev_id on irq line %d\n", irq);
		return IRQ_NONE;
	}
	user_irq = (struct xdma_user_irq *)dev_id;

	return user_irq_service(irq, user_irq);
}

/*
 * xdma_channel_irq() - Interrupt handler for channel interrupts in MSI-X mode
 *
 * @dev_id pointer to xdma_dev
 */
static irqreturn_t xdma_channel_irq(int irq, void *dev_id)
{
	struct xdma_dev *xdev;
	struct xdma_engine *engine;

	dbg_irq("(irq=%d) <<<< INTERRUPT service ROUTINE\n", irq);
	if (!dev_id) {
		pr_err("Invalid dev_id on irq line %d\n", irq);
		return IRQ_NONE;
	}

	engine = (struct xdma_engine *)dev_id;
	xdev = engine->xdev;

	if (!xdev) {
		WARN_ON(!xdev);
		dbg_irq("%s(irq=%d) xdev=%p ??\n", __func__, irq, xdev);
		return IRQ_NONE;
	}


	complete(&(engine->engine_compl));

	/*
	 * need to protect access here if multiple MSI-X are used for
	 * user interrupts
	 */
	xdev->irq_count++;
	return IRQ_HANDLED;
}

/*
 * Unmap the BAR regions that had been mapped earlier using map_bars()
 */
static void unmap_bars(struct xdma_dev *xdev, struct pci_dev *dev)
{
	int i;

	for (i = 0; i < XDMA_BAR_NUM; i++) {
		/* is this BAR mapped? */
		if (xdev->bar[i]) {
			/* unmap BAR */
			pci_iounmap(dev, xdev->bar[i]);
			/* mark as unmapped */
			xdev->bar[i] = NULL;
		}
	}
}

static int map_single_bar(struct xdma_dev *xdev, struct pci_dev *dev, int idx, resource_size_t *bar_len)
{
	resource_size_t bar_start;

	bar_start = pci_resource_start(dev, idx);
	*bar_len = pci_resource_len(dev, idx);

	xdev->bar[idx] = NULL;

	/* do not map BARs with length 0. Note that start MAY be 0! */
	if (*bar_len==0) {
		//pr_info("BAR #%d is not present - skipping\n", idx);
		return 0;
	}

	/*
	 * map the full device memory or IO region into kernel virtual
	 * address space
	 */
	dbg_init("BAR%d: %llu bytes to be mapped.\n", idx, *bar_len);
	xdev->bar[idx] = pci_iomap(dev, idx, *bar_len);

	if (unlikely(!xdev->bar[idx])) {
		pr_info("Could not map BAR %d.\n", idx);
		return -1;
	}
	xdev->bar_size[idx] = *bar_len;
	pr_info("BAR%d at 0x%llx mapped at 0x%p, length=%llu\n", idx,
		(u64)bar_start, xdev->bar[idx],  *bar_len);

	return 0;
}

static bool is_config_bar(struct xdma_dev *xdev, int idx, resource_size_t len)
{
	u32 irq_id = 0;
	u32 cfg_id = 0;
	u32 mask = 0xffff0000; /* Compare only XDMA ID's not Version number */
	
	struct interrupt_regs *irq_regs =
		(struct interrupt_regs *)(xdev->bar[idx] + XDMA_OFS_INT_CTRL);
	struct config_regs *cfg_regs =
		(struct config_regs *)(xdev->bar[idx] + XDMA_OFS_CONFIG);
		
	if (len<XDMA_BAR_SIZE)
	{
		dbg_init("BAR %d is NOT the XDMA config BAR: 0x%x, 0x%x, len=%llu\n",
			 idx, irq_id, cfg_id, len);
		dbg_init("Hardware was not probed."); 
		return false; 
	}

	irq_id = read_register(&irq_regs->identifier);
	cfg_id = read_register(&cfg_regs->identifier);

	if (((irq_id & mask) == IRQ_BLOCK_ID) &&
	    ((cfg_id & mask) == CONFIG_BLOCK_ID)) {
		dbg_init("BAR %d is the XDMA config BAR\n", idx);
		return true;
	} else {
		dbg_init("BAR %d is NOT the XDMA config BAR: 0x%x, 0x%x, len=%llu\n",
			 idx, irq_id, cfg_id, len);
		return false;
	}

}


static int identify_bars(struct xdma_dev *xdev, int *bar_id_list, int num_bars,
			 int config_bar_pos)
{
	/*
	 * The following logic identifies which BARs contain what functionality
	 * based on the position of the XDMA config BAR and the number of BARs
	 * detected. The rules are that the user logic and bypass logic BARs
	 * are optional.  When both are present, the XDMA config BAR will be the
	 * 2nd BAR detected (config_bar_pos = 1), with the user logic being
	 * detected first and the bypass being detected last. When one is
	 * omitted, the type of BAR present can be identified by whether the
	 * XDMA config BAR is detected first or last.  When both are omitted,
	 * only the XDMA config BAR is present.  This somewhat convoluted
	 * approach is used instead of relying on BAR numbers in order to work
	 * correctly with both 32-bit and 64-bit BARs.
	 */

	if (!xdev) {
		pr_err("Invalid xdev\n");
		return -EINVAL;
	}

	if (!bar_id_list) {
		pr_err("Invalid bar id list.\n");
		return -EINVAL;
	}

	dbg_init("xdev 0x%p, bars %d, config at %d.\n", xdev, num_bars,
		 config_bar_pos);

	switch (num_bars) {
	case 0:
	/* no bars were identified - probably wrong device*/
		pr_crit("No BARs were detected!");
		return -ENODEV;
	case 1:
		/* Only one BAR present - it must be config bar, but verify */
									
		if(config_bar_pos<0)
		{					/*     \/ temporary workaround, should pass real bar length*/
			if(is_config_bar(xdev, bar_id_list[0], XDMA_BAR_SIZE ))
				xdev->config_bar_idx=bar_id_list[0];
			else
				goto no_config_bar;
		}
		
		break;

	case 2:
		if(config_bar_pos<0)
		{
			pr_notice("Config bar num was invalid or not set. Falling back to autodetection");
			/*  \/config bar 0 was already probed                       \/ temporary workaround, should pass real bar length*/
			if((config_bar_num!=0)&&is_config_bar(xdev, bar_id_list[0], XDMA_BAR_SIZE ))
			{
				xdev->config_bar_idx=bar_id_list[0];
				config_bar_pos=0;
			}					/*     \/ temporary workaround, should pass real bar length*/
			else if(is_config_bar(xdev, bar_id_list[1], XDMA_BAR_SIZE ))
			{
				xdev->config_bar_idx=bar_id_list[1];
				config_bar_pos=1;
			}
			else
				goto no_config_bar;
		}
		
		if (config_bar_pos == 0) {
			xdev->bypass_bar_idx = bar_id_list[1];
		} else if (config_bar_pos == 1) {
			xdev->user_bar_idx = bar_id_list[0];
		} else {
			pr_warn("2 BARs, XDMA config BAR unexpected %d.\n",
				config_bar_pos);
		}
		break;

	case 3:
	case 4:
		if(config_bar_pos<0)
		{//config bar must be in the middle - verify by probing expected position
			if ((num_bars==3)&&is_config_bar(xdev, bar_id_list[1], XDMA_BAR_SIZE))
			{
				xdev->config_bar_idx=bar_id_list[1];
				config_bar_pos=1;
			}
			else if ((num_bars==4)&&is_config_bar(xdev, bar_id_list[2], XDMA_BAR_SIZE))
			{
				xdev->config_bar_idx=bar_id_list[2];
				config_bar_pos=2;
			}
			else 
				goto no_config_bar;
		}
		if ((config_bar_pos == 1) || (config_bar_pos == 2)) {
			/* user bar at bar #0 */
			xdev->user_bar_idx = bar_id_list[0];
			/* bypass bar at the last bar */
			xdev->bypass_bar_idx = bar_id_list[num_bars - 1];
		} else {
			pr_warn("3/4 BARs, XDMA config BAR unexpected %d.\n",
				config_bar_pos);
		}
		break;

	default:
		if(xdev->config_bar_idx<0)
			goto no_config_bar;
		/* Should not occur - warn user but safe to continue */
		pr_warn("Unexpected # BARs (%d), XDMA config BAR only.\n",
			num_bars);
		break;
	}
	pr_info("%d BARs: config %d, user %d, bypass %d.\n", num_bars,
		xdev->config_bar_idx, xdev->user_bar_idx, xdev->bypass_bar_idx);
	return 0;
	
no_config_bar:
	pr_err("Failed to detect XDMA config BAR\n");
	return -EINVAL;
}


/* map_bars() -- map device regions into kernel virtual address space
 *
 * Map the device memory regions into kernel virtual address space after
 * verifying their sizes respect the minimum sizes needed
 */
static int map_bars(struct xdma_dev *xdev, struct pci_dev *dev)
{
	int rv;

	int i;
	int bar_id_list[XDMA_BAR_NUM];
	int bar_id_idx = 0;
	int config_bar_pos = -1;

	/* iterate through all the BARs */
	for (i = 0; i < XDMA_BAR_NUM; i++) {
		resource_size_t bar_len =0;
		rv=map_single_bar(xdev, dev, i, &bar_len);
		/* if config_bar_num was set, make sure that the input was really set to config BAR*/
		if( (i==config_bar_num)&&is_config_bar(xdev, config_bar_num, bar_len) )
		{
			xdev->config_bar_idx = config_bar_num;
			config_bar_pos = bar_id_idx;
		}
		
		
		if (bar_len == 0) {
			continue;
		} else if (rv< 0) {
			rv = -EINVAL;
			goto fail;
		}

		bar_id_list[bar_id_idx] = i;
		bar_id_idx++;
	}

	rv = identify_bars(xdev, bar_id_list, bar_id_idx, config_bar_pos);
	if (unlikely(rv < 0)) {
		pr_err("Failed to identify bars\n");
		goto fail;
	}

	/* successfully mapped all required BAR regions */
	return 0;

fail:
	/* unwind; unmap any BARs that we did map */
	unmap_bars(xdev, dev);
	return rv;
}

/*
 * MSI-X interrupt:
 *	<h2c+c2h channel_max> vectors, followed by <user_max> vectors
 */

/*
 * code to detect if MSI/MSI-X capability exists is derived
 * from linux/pci/msi.c - pci_msi_check_device
 */

#ifndef arch_msi_check_device
static int arch_msi_check_device(struct pci_dev *dev, int nvec, int type)
{
	return 0;
}
#endif

/* type = PCI_CAP_ID_MSI or PCI_CAP_ID_MSIX */
static int msi_msix_capable(struct pci_dev *dev, int type)
{
	struct pci_bus *bus;
	int ret;

	if (!dev || dev->no_msi)
		return 0;

	for (bus = dev->bus; bus; bus = bus->parent)
		if (bus->bus_flags & PCI_BUS_FLAGS_NO_MSI)
			return 0;

	ret = arch_msi_check_device(dev, 1, type);
	if (ret)
		return 0;

	if (!pci_find_capability(dev, type))
		return 0;

	return 1;
}

static void disable_msi_msix(struct xdma_dev *xdev, struct pci_dev *pdev)
{
	if (xdev->msix_enabled) {
		pci_disable_msix(pdev);
		xdev->msix_enabled = 0;
	} else if (xdev->msi_enabled) {
		pci_disable_msi(pdev);
		xdev->msi_enabled = 0;
	}
}

static int enable_msi_msix(struct xdma_dev *xdev, struct pci_dev *pdev)
{
	int rv = 0;

	if (!xdev) {
		pr_err("Invalid xdev\n");
		return -EINVAL;
	}

	if (!pdev) {
		pr_err("Invalid pdev\n");
		return -EINVAL;
	}

	if ((interrupt_mode == 3 || !interrupt_mode) && msi_msix_capable(pdev, PCI_CAP_ID_MSIX)) {
		int req_nvec = xdev->c2h_channel_num + xdev->h2c_channel_num +
			       xdev->user_max;


		dbg_init("Enabling MSI-X\n");
		/*avilable since 4.8*/
		rv = pci_alloc_irq_vectors(pdev, req_nvec, req_nvec, PCI_IRQ_MSIX);

		if (rv < 0)
			dbg_init("Couldn't enable MSI-X mode: %d\n", rv);

		xdev->msix_enabled = 1;

	} else if ((interrupt_mode == 1 || !interrupt_mode) &&
		   msi_msix_capable(pdev, PCI_CAP_ID_MSI)) {
		/* enable message signalled interrupts */
		dbg_init("pci_enable_msi()\n");
		rv = pci_enable_msi(pdev);
		if (rv < 0)
			dbg_init("Couldn't enable MSI mode: %d\n", rv);
		xdev->msi_enabled = 1;

	} else {
		dbg_init("MSI/MSI-X not detected - using legacy interrupts\n");
	}

	return rv;
}

static void pci_check_intr_pend(struct pci_dev *pdev)
{
	u16 v;

	pci_read_config_word(pdev, PCI_STATUS, &v);
	if (v & PCI_STATUS_INTERRUPT) {
		pr_info("%s PCI STATUS Interrupt pending 0x%x.\n",
			dev_name(&pdev->dev), v);
		pci_write_config_word(pdev, PCI_STATUS, PCI_STATUS_INTERRUPT);
	}
}

static void pci_keep_intx_enabled(struct pci_dev *pdev)
{
	/* workaround to a h/w bug:
	 * when msix/msi become unavaile, default to legacy.
	 * However the legacy enable was not checked.
	 * If the legacy was disabled, no ack then everything stuck
	 */
	u16 pcmd, pcmd_new;

	pci_read_config_word(pdev, PCI_COMMAND, &pcmd);
	pcmd_new = pcmd & ~PCI_COMMAND_INTX_DISABLE;
	if (pcmd_new != pcmd) {
		pr_info("%s: clear INTX_DISABLE, 0x%x -> 0x%x.\n",
			dev_name(&pdev->dev), pcmd, pcmd_new);
		pci_write_config_word(pdev, PCI_COMMAND, pcmd_new);
	}
}

static void prog_irq_msix_user(struct xdma_dev *xdev, bool clear)
{
	/* user */
	struct interrupt_regs *int_regs =
		(struct interrupt_regs *)(xdev->bar[xdev->config_bar_idx] +
					  XDMA_OFS_INT_CTRL);
	u32 i = xdev->c2h_channel_num + xdev->h2c_channel_num;
	u32 max = i + xdev->user_max;
	int j;

	for (j = 0; i < max; j++) {
		u32 val = 0;
		int k;
		int shift = 0;

		if (clear)
			i += 4;
		else
			for (k = 0; k < 4 && i < max; i++, k++, shift += 8)
				val |= (i & 0x1f) << shift;

		write_register(
			val, &int_regs->user_msi_vector[j],
			XDMA_OFS_INT_CTRL +
				((unsigned long)&int_regs->user_msi_vector[j] -
				 (unsigned long)int_regs));

		dbg_init("vector %d, 0x%x.\n", j, val);
	}
}

static void prog_irq_msix_channel(struct xdma_dev *xdev, bool clear)
{
	struct interrupt_regs *int_regs =
		(struct interrupt_regs *)(xdev->bar[xdev->config_bar_idx] +
					  XDMA_OFS_INT_CTRL);
	u32 max = xdev->c2h_channel_num + xdev->h2c_channel_num;
	u32 i;
	int j;

	/* engine */
	for (i = 0, j = 0; i < max; j++) {
		u32 val = 0;
		int k;
		int shift = 0;

		if (clear)
			i += 4;
		else
			for (k = 0; k < 4 && i < max; i++, k++, shift += 8)
				val |= (i & 0x1f) << shift;

		write_register(val, &int_regs->channel_msi_vector[j],
			       XDMA_OFS_INT_CTRL +
				       ((unsigned long)&int_regs
						->channel_msi_vector[j] -
					(unsigned long)int_regs));
		dbg_init("vector %d, 0x%x.\n", j, val);
	}
}

static void irq_msix_channel_teardown(struct xdma_dev *xdev)
{
	struct xdma_engine *engine;
	int j = 0;
	int i = 0;

	if (!xdev->msix_enabled)
		return;

	prog_irq_msix_channel(xdev, 1);

	engine = xdev->engine_h2c;
	for (i = 0; i < xdev->h2c_channel_num; i++, j++, engine++) {
		if (!engine->msix_irq_line)
			break;
		dbg_sg("Release IRQ#%d for engine %p\n", engine->msix_irq_line,
		       engine);
		free_irq(engine->msix_irq_line, engine);
	}

	engine = xdev->engine_c2h;
	for (i = 0; i < xdev->c2h_channel_num; i++, j++, engine++) {
		if (!engine->msix_irq_line)
			break;
		dbg_sg("Release IRQ#%d for engine %p\n", engine->msix_irq_line,
		       engine);
		free_irq(engine->msix_irq_line, engine);
	}
}

static int irq_msix_channel_setup(struct xdma_dev *xdev)
{
	int i;
	int j;
	int rv = 0;
	u32 vector;
	struct xdma_engine *engine;

	if (!xdev) {
		pr_err("dma engine NULL\n");
		return -EINVAL;
	}

	if (!xdev->msix_enabled)
		return 0;

	j = xdev->h2c_channel_num;
	engine = xdev->engine_h2c;
	for (i = 0; i < xdev->h2c_channel_num; i++, engine++) {
		vector = pci_irq_vector(xdev->pdev, i);

		rv = request_irq(vector, xdma_channel_irq, 0, xdev->mod_name,
				 engine);
		if (rv) {
			pr_info("requesti irq#%d failed %d, engine %s.\n",
				vector, rv, engine->name);
			return rv;
		}
		pr_info("engine %s, irq#%d.\n", engine->name, vector);
		engine->msix_irq_line = vector;
	}

	engine = xdev->engine_c2h;
	for (i = 0; i < xdev->c2h_channel_num; i++, j++, engine++) {
		vector = pci_irq_vector(xdev->pdev, j);

		rv = request_irq(vector, xdma_channel_irq, 0, xdev->mod_name,
				 engine);
		if (rv) {
			pr_info("requesti irq#%d failed %d, engine %s.\n",
				vector, rv, engine->name);
			return rv;
		}
		pr_info("engine %s, irq#%d.\n", engine->name, vector);
		engine->msix_irq_line = vector;
	}

	return 0;
}

static void irq_msix_user_teardown(struct xdma_dev *xdev)
{
	int i;
	int j;

	if (!xdev) {
		pr_err("Invalid xdev\n");
		return;
	}

	if (!xdev->msix_enabled)
		return;

	j = xdev->h2c_channel_num + xdev->c2h_channel_num;

	prog_irq_msix_user(xdev, 1);

	for (i = 0; i < xdev->user_max; i++, j++) {
		u32 vector = pci_irq_vector(xdev->pdev, j);

		dbg_init("user %d, releasing IRQ#%d\n", i, vector);
		free_irq(vector, &xdev->user_irq[i]);
	}
}

static int irq_msix_user_setup(struct xdma_dev *xdev)
{
	int i;
	int j = xdev->h2c_channel_num + xdev->c2h_channel_num;
	int rv = 0;

	/* vectors set in probe_scan_for_msi() */
	for (i = 0; i < xdev->user_max; i++, j++) {
		u32 vector = pci_irq_vector(xdev->pdev, j);

		rv = request_irq(vector, xdma_user_irq, 0, xdev->mod_name,
				 &xdev->user_irq[i]);
		if (rv) {
			pr_info("user %d couldn't use IRQ#%d, %d\n", i, vector,
				rv);
			break;
		}
		pr_info("%d-USR-%d, IRQ#%d with 0x%p\n", xdev->idx, i, vector,
			&xdev->user_irq[i]);
	}

	/* If any errors occur, free IRQs that were successfully requested */
	if (rv) {
		for (i--, j--; i >= 0; i--, j--) {
			u32 vector = pci_irq_vector(xdev->pdev, j);

			free_irq(vector, &xdev->user_irq[i]);
		}
	}

	return rv;
}

static int irq_msi_setup(struct xdma_dev *xdev, struct pci_dev *pdev)
{
	int rv;

	xdev->irq_line = (int)pdev->irq;
	rv = request_irq(pdev->irq, xdma_isr, 0, xdev->mod_name, xdev);
	if (rv)
		dbg_init("Couldn't use IRQ#%d, %d\n", pdev->irq, rv);
	else
		dbg_init("Using IRQ#%d with 0x%p\n", pdev->irq, xdev);

	return rv;
}

static int irq_legacy_setup(struct xdma_dev *xdev, struct pci_dev *pdev)
{
	u32 w;
	u8 val;
	void *reg;
	int rv;

	pci_read_config_byte(pdev, PCI_INTERRUPT_PIN, &val);
	if (val == 0) {
		dbg_init("Legacy interrupt not supported\n");
		return -EINVAL;
	}

	dbg_init("Legacy Interrupt register value = %d\n", val);
	if (val > 1) {
		val--;
		w = (val << 24) | (val << 16) | (val << 8) | val;
		/* Program IRQ Block Channel vector and IRQ Block User vector
		 * with Legacy interrupt value
		 */
		reg = xdev->bar[xdev->config_bar_idx] + 0x2080; // IRQ user
		write_register(w, reg, 0x2080);
		write_register(w, reg + 0x4, 0x2084);
		write_register(w, reg + 0x8, 0x2088);
		write_register(w, reg + 0xC, 0x208C);
		reg = xdev->bar[xdev->config_bar_idx] + 0x20A0; // IRQ Block
		write_register(w, reg, 0x20A0);
		write_register(w, reg + 0x4, 0x20A4);
	}

	xdev->irq_line = (int)pdev->irq;
	rv = request_irq(pdev->irq, xdma_isr, IRQF_SHARED, xdev->mod_name,
			 xdev);
	if (rv)
		dbg_init("Couldn't use IRQ#%d, %d\n", pdev->irq, rv);
	else
		dbg_init("Using IRQ#%d with 0x%p\n", pdev->irq, xdev);

	return rv;
}

static void irq_teardown(struct xdma_dev *xdev)
{
	if (xdev->msix_enabled) {
		irq_msix_channel_teardown(xdev);
		irq_msix_user_teardown(xdev);
	} else if (xdev->irq_line != -1) {
		dbg_init("Releasing IRQ#%d\n", xdev->irq_line);
		free_irq(xdev->irq_line, xdev);
	}
}

static int irq_setup(struct xdma_dev *xdev, struct pci_dev *pdev)
{
	pci_keep_intx_enabled(pdev);

	if (xdev->msix_enabled) {
		int rv = irq_msix_channel_setup(xdev);

		if (rv)
			return rv;
		rv = irq_msix_user_setup(xdev);
		if (rv)
			return rv;
		prog_irq_msix_channel(xdev, 0);
		prog_irq_msix_user(xdev, 0);

		return 0;
	} else if (xdev->msi_enabled)
		return irq_msi_setup(xdev, pdev);

	return irq_legacy_setup(xdev, pdev);
}

#ifdef __LIBXDMA_DEBUG__
static void dump_desc(const struct xdma_desc *desc_virt)
{
	int j;
	u32 *p = (u32 *)desc_virt;
	static char *const field_name[] = { "magic|extra_adjacent|control",
					    "bytes",
					    "src_addr_lo",
					    "src_addr_hi",
					    "dst_addr_lo",
					    "dst_addr_hi",
					    "next_addr",
					    "next_addr_pad" };
	char *dummy;

	/* remove warning about unused variable when debug printing is off */
	dummy = field_name[0];

	for (j = 0; j < 8; j += 1) {
		pr_info("0x%08lx/0x%02lx: 0x%08x 0x%08x %s\n", (uintptr_t)p,
			(uintptr_t)p & 15, (int)*p, le32_to_cpu(*p),
			field_name[j]);
		p++;
	}
	pr_info("\n");
}

static const char* direction_to_string(enum dma_data_direction dir)
{

	switch(dir)
	{
	case DMA_BIDIRECTIONAL:
	 	return "DMA_BIDIRECTIONAL";
	case DMA_TO_DEVICE:
	 	return "DMA_TO_DEVICE";
	case DMA_FROM_DEVICE:
	 	return "DMA_FROM_DEVICE";
	case DMA_NONE:
	 	return "DMA_NONE";
	default:
		return "*ERROR* invalid DMA direction";
	}
	 	 
}

#else 
#define dump_desc(...)  
#endif /* __LIBXDMA_DEBUG__ */


static void engine_alignments(struct xdma_engine *engine)
{
	u32 w;
	u32 align_bytes;
	u32 granularity_bytes;
	u32 address_bits;

	w = read_register(&engine->regs->alignments);
	dbg_init("engine %p name %s alignments=0x%08x\n", engine, engine->name,
		 (int)w);

	align_bytes = (w & 0x00ff0000U) >> 16;
	granularity_bytes = (w & 0x0000ff00U) >> 8;
	address_bits = (w & 0x000000ffU);

	dbg_init("align_bytes = %d\n", align_bytes);
	dbg_init("granularity_bytes = %d\n", granularity_bytes);
	dbg_init("address_bits = %d\n", address_bits);
/*cast away const just for initialisation)*/
	if (w) {
		const_cast(u8, engine->addr_align) = align_bytes;
		const_cast(u8, engine->len_granularity) = granularity_bytes;
		const_cast(u8, engine->addr_bits) = address_bits;
	} else {
		/* Some default values if alignments are unspecified */
		const_cast(u8, engine->addr_align) = 1;
		const_cast(u8, engine->len_granularity) = 1;
		const_cast(u8, engine->addr_bits) = 64;
	}
}

static void engine_free_resource(struct xdma_engine *engine)
{
	dma_pool_destroy(engine->desc_pool);
	 
#ifdef XDMA_POLL_MODE
        dma_free_coherent( &(engine->xdev->pdev->dev), engine->poll_mode_wb.length, 
        		engine->poll_mode_wb.virtual_addr, engine->poll_mode_wb.dma_addr);
#endif
}

static int engine_destroy(struct xdma_dev *xdev, struct xdma_engine *engine)
{
	if (!xdev) {
		pr_err("Invalid xdev\n");
		return -EINVAL;
	}

	if (!engine) {
		pr_err("dma engine NULL\n");
		return -EINVAL;
	}

	dbg_sg("Shutting down engine %s%d", engine->name, engine->channel);

	/* Disable interrupts to stop processing new events during shutdown */
	write_register(0x0, &engine->regs->interrupt_enable_mask,
		       (unsigned long)(&engine->regs->interrupt_enable_mask) -
			       (unsigned long)(&engine->regs));

	if (enable_st_c2h_credit && engine->streaming &&
	    engine->dir == DMA_FROM_DEVICE) {
		u32 reg_value = (0x1 << engine->channel) << 16;
		struct sgdma_common_regs *reg =
			(struct sgdma_common_regs
				 *)(xdev->bar[xdev->config_bar_idx] +
				    (0x6 * TARGET_SPACING));
		write_register(reg_value, &reg->credit_mode_enable_w1c, 0);
	}


	/* Release memory use for descriptor writebacks */
	engine_free_resource(engine);

	memset(engine, 0, sizeof(struct xdma_engine));
	/* Decrement the number of engines available */
	xdev->engines_num--;
	return 0;
}





/* engine_create() - Create an SG DMA engine bookkeeping data structure
 *
 * An SG DMA engine consists of the resources for a single-direction transfer
 * queue; the SG DMA hardware, the software queue and interrupt handling.
 *
 * @dev Pointer to pci_dev
 * @offset byte address offset in BAR[xdev->config_bar_idx] resource for the
 * SG DMA * controller registers.
 * @dir: DMA_TO/FROM_DEVICE
 * @streaming Whether the engine is attached to AXI ST (rather than MM)
 */
static int engine_init_regs(struct xdma_engine *engine)
{
	/*configure both control and interrupt registers just once*/
	u32 interrupt_reg_value=0;
	u32 control_reg_value=
			(XDMA_CTRL_IE_DESC_ERROR
			|XDMA_CTRL_IE_READ_ERROR
			|XDMA_CTRL_IE_WRITE_ERROR
			|XDMA_CTRL_IE_INVALID_LENGTH
			|XDMA_CTRL_IE_MAGIC_STOPPED
			|XDMA_CTRL_IE_DESC_ALIGN_MISMATCH
			|XDMA_CTRL_IE_DESC_COMPLETED
			|XDMA_CTRL_IE_DESC_STOPPED);
	
#ifdef XDMA_POLL_MODE
/* if using polled mode,  enable writeback and configure its address, disable interrupts */
	engine->poll_mode_wb.virtual_addr->completed_desc_count=0;
	write_register((u32) engine->poll_mode_wb.dma_addr, &(engine->regs->poll_mode_wb_lo), 0);
	write_register((u32) (engine->poll_mode_wb.dma_addr>>32), &(engine->regs->poll_mode_wb_hi), 0);
	control_reg_value|=XDMA_CTRL_POLL_MODE_WB;
#else/
	/* Othwerwise configure error interrupts to match control register by default,
	thus all events trigger interrupt  */
	interrupt_reg_value=control_reg_value;
#endif
	/*disable channel writeback*/
	control_reg_value|=XDMA_CTRL_STM_MODE_WB;
	/* Apply engine configurations */
	write_register(control_reg_value, &engine->regs->control,
		       (unsigned long)(&engine->regs->control) -
			       (unsigned long)(&engine->regs));
	
	write_register(interrupt_reg_value, &engine->regs->interrupt_enable_mask,
		       (unsigned long)(&engine->regs->interrupt_enable_mask) -
			       (unsigned long)(&engine->regs));

	engine->interrupt_enable_mask_value = interrupt_reg_value;
	
	engine_alignments(engine);
	/* only enable credit mode for AXI-ST C2H */
	if (enable_st_c2h_credit && engine->streaming &&
	    engine->dir == DMA_FROM_DEVICE) {
		struct xdma_dev *xdev = engine->xdev;
		u32 reg_value = (0x1 << engine->channel) << 16;
		struct sgdma_common_regs *reg =
			(struct sgdma_common_regs
				 *)(xdev->bar[xdev->config_bar_idx] +
				    (0x6 * TARGET_SPACING));

		write_register(reg_value, &reg->credit_mode_enable_w1s, 0);
	}

	return 0;

}

static int engine_alloc_resource(struct xdma_engine *engine)
{
	struct xdma_dev *xdev = engine->xdev;
	char pool_name[32]="Descriptor pool for ";/*20 characters*/
	
	engine->desc_pool= dma_pool_create(strncat(pool_name, engine->name, sizeof(pool_name)-20-1), &(xdev->pdev->dev),
                engine->adj_block_len * sizeof(struct xdma_desc), sizeof(struct xdma_desc), XDMA_PAGE_SIZE);
        if(unlikely(engine->desc_pool==NULL))
        	{
        		pr_err("Failed to create descriptor pool for engine %s", engine->name);
        		return -ENOMEM;
        	}
        	
#ifdef XDMA_POLL_MODE
	engine->poll_mode_wb.virtual_addr=dma_alloc_coherent( &(xdev->pdev->dev), sizeof(struct xdma_poll_wb), 
					&(engine->poll_mode_wb.dma_addr), GFP_KERNEL|__GFP_RETRY_MAYFAIL);
	if(unlikely(engine->poll_mode_wb.virtual_addr==NULL))
	{
		pr_err("Failed to alloacate structure for poll mode.\n");
		return -ENOMEM;
	}
	engine->poll_mode_wb.length= sizeof(struct xdma_poll_wb);
	dbg_init("Poll mode wb: virt %p, DMA addr %llx\n", engine->poll_mode_wb.virtual_addr, engine->poll_mode_wb.dma_addr);
#endif


	return 0;
/*
err_out:
	engine_free_resource(engine);
	return -ENOMEM;*/
}
/* register offset for the engine */
static unsigned int get_engine_offset(enum dma_data_direction dir, int channel)
{
	/* read channels at 0x0000, write channels at 0x1000,
	 * channels at 0x100 interval
	 */
	unsigned int offset =channel * CHANNEL_SPACING;
	if (dir == DMA_FROM_DEVICE)
		offset+=H2C_CHANNEL_OFFSET;
	return offset;
}

static int engine_init(struct xdma_dev *xdev, enum dma_data_direction dir, int channel)
{
	int rv;
	u32 val;
	unsigned int desc_max;
	unsigned int offset=get_engine_offset(dir, channel);
	struct xdma_engine *engine= dir==DMA_TO_DEVICE? &(xdev->engine_h2c[channel]): &(xdev->engine_c2h[channel]);
	dbg_init("channel %d, offset 0x%x, dir %s.\n", channel, offset,direction_to_string( dir));
	

	/* set magic */
	engine->magic = MAGIC_ENGINE;

	engine->channel = channel;
	
#ifndef XDMA_POLL_MODE
	/* engine interrupt request bit */
	engine->irq_bitmask = (1 << XDMA_ENG_IRQ_NUM) - 1;
	engine->irq_bitmask <<= (xdev->engines_num * XDMA_ENG_IRQ_NUM);
#endif
	/* parent */
	engine->xdev = xdev;
	/* register address */
	engine->regs = (xdev->bar[xdev->config_bar_idx] + offset);
	engine->sgdma_regs = xdev->bar[xdev->config_bar_idx] + offset +
			     SGDMA_OFFSET_FROM_CHANNEL;
	val = read_register(&engine->regs->identifier);
	if (val & 0x8000U)
		engine->streaming = 1;

	/* remember SG DMA direction */
	const_cast(enum dma_data_direction, engine->dir) = dir;
	snprintf(engine->name, sizeof(engine->name), "%d-%s%d-%s", xdev->idx,
		(dir == DMA_TO_DEVICE) ? "H2C" : "C2H", channel,
		engine->streaming ? "ST" : "MM");

	/*calculate maximum usable register for engine, so that hardware descriptor FIFO would not overflow
	First clculate size of the FIFO, then total descriptors that fit into it, and divide by number of channels,
	separately for each diraction. */
	const_cast(unsigned int, engine->desc_max) = (XDMA_DESC_FIFO_DEPTH * xdev->datapath_width) /sizeof(struct xdma_desc)
	    				/(dir==DMA_TO_DEVICE? xdev->h2c_channel_num: xdev->c2h_channel_num);
	
	    	
	const_cast(unsigned int, engine->adj_block_len)=engine->xdev->max_read_request_size /sizeof(struct xdma_desc);
	dbg_init("engine %p name %s irq_bitmask=0x%08x\n", engine, engine->name,
		 (unsigned int) (int)engine->irq_bitmask);



	if (dir == DMA_TO_DEVICE)
		xdev->mask_irq_h2c |= engine->irq_bitmask;
	else
		xdev->mask_irq_c2h |= engine->irq_bitmask;
	

	rv = engine_alloc_resource(engine);
	if (unlikely(rv))
		return rv;
		
	xdev->engines_num++;
	rv = engine_init_regs(engine);
	if (unlikely(rv))
		return rv;
	/*init completion was renamed twice */
	#if LINUX_VERSION_CHECK(5,11,0)
	init_completion(&(engine->engine_compl));
	#elif LINUX_VERSION_CHECK(4,14,0)
	__init_completion(&(engine->engine_compl));
	#else
	init_completion(&(engine->engine_compl));
	#endif
	/*pr_info("XDMA engine %s can use up to %u descriptors with length of block of adjacent descriptors up to %u", 
		engine->name,engine->desc_max, engine->adj_block_len);*/

	return 0;
}





/*check transfer parameters for validity */
static int xdma_validate_transfer(const struct xdma_engine *engine)
{	
	int rv=0;
	const struct xdma_transfer_params *transfer_params=&(engine->transfer_params);
	const u64 addr_align_mask= ((u64) engine->addr_align)-1;
	const u64 granularity_mask= ((u64) engine->len_granularity)-1;
	dbg_fops("Transfer request on engine %s: buf 0x%px, length %zu, AXI address 0x%llx, direction %s\n",
		engine->name, transfer_params->buf, transfer_params->length, transfer_params->ep_addr, 
		direction_to_string(transfer_params->dir));
	if(unlikely(transfer_params->buf==NULL))
		return -EINVAL;
	if(unlikely(transfer_params->length==0))
		return -EINVAL;
	rv=position_check(MAX_RESOURCE_SIZE, transfer_params->ep_addr, engine->addr_align);
	if(unlikely(rv<0))
		return rv;
	if (engine->non_incr_addr)
	{
		if((((uintptr_t) transfer_params->buf) &addr_align_mask)!=0|| (transfer_params->ep_addr&addr_align_mask)!=0)
		{
			
			pr_err("The address of transfer buffer 0x%px or AXI address %lld are not multiple of %u\n", 
				transfer_params->buf, transfer_params->ep_addr, engine->addr_align);
			return -EINVAL;
		}
		if((transfer_params->length & granularity_mask)!=0)
		{
			pr_err("Transfer length %zu is not multiple of %u\n", 
			transfer_params->length, engine->len_granularity);
			return -EINVAL;
		}
		
	}
	/* else is not neccessary since there supposed to be no limitations
	according to PG195*/
		
	return rv;	
}

static int xdma_sgtable_to_descriptors(struct xdma_engine *engine)
{
	struct xdma_transfer *transfer=&(engine->transfer);
	const unsigned int sg_nents=transfer->sgt.nents;
	struct scatterlist *sg_iter=transfer->sgt.sgl;
	struct scatterlist *sg_prev=NULL;
	loff_t ep_addr= engine->streaming? : engine->transfer_params.ep_addr;
	unsigned int block_num=0;
	unsigned int processed_sg_entries=0;
	u32 control_flags=(XDMA_DESC_STOPPED|XDMA_DESC_COMPLETED);
	
	if(engine->streaming&& (engine->dir==DMA_TO_DEVICE) && engine->eop_flush)
		control_flags|=XDMA_DESC_EOP;	
	
	
	
		
	/*calculate number of adjasent descriptor blocks, for now limited to 1
	transfer->num_adj_blocks= divide_roundup(transfer->sgt.nents, engine->adj_block_len);*/
	transfer->num_adj_blocks=1;
	/*dbg_sg("%u adjacent descriptor blocks are required for transfer on engine %s\n",
		 transfer->num_adj_blocks, engine->name);*/
		 
	/*Dynamically allocated as an array as a provision for possible future splitting into multiple transfers.*/
	transfer->adj_desc_blocks=kmalloc_array(transfer->num_adj_blocks, 
		sizeof(*(transfer->adj_desc_blocks)), GFP_KERNEL|__GFP_NORETRY|__GFP_ZERO);
	if(unlikely(transfer->adj_desc_blocks==NULL))
	{
		dbg_sg("Failed to allocate memory for descriptor DMA records on engine %s.\n", engine->name);
		return -ENOMEM; 
	}
	transfer->cleanup_flags|=XFER_FLAG_DMA_RECORD_ALLOC;
	/* step through blocks of adjacent descriptors*/
	for(block_num; (block_num<transfer->num_adj_blocks) && (processed_sg_entries < sg_nents); ++block_num)
	{
		unsigned int i=0;
		unsigned desc_in_block=0;
		struct xdma_desc *current_desc;
		dma_addr_t desc_dma_addr;
				
		transfer->adj_desc_blocks[block_num].virtual_addr=dma_pool_zalloc(engine->desc_pool,
		/*memory is allocated at the first call and then stays in the pool,therefore it is acceptable to wait once*/
						GFP_KERNEL|__GFP_RETRY_MAYFAIL, &desc_dma_addr);
		if(unlikely(transfer->adj_desc_blocks[block_num].virtual_addr==NULL))
		{
			dbg_sg("Failed to allocate memory from descriptor pool on engune %s\n", engine->name);		
			return -ENOMEM;
		}
		transfer->adj_desc_blocks[block_num].dma_addr=desc_dma_addr;
		transfer->adj_desc_blocks[block_num].length=engine->adj_block_len*sizeof(struct xdma_desc);
		transfer->cleanup_flags|=XFER_FLAG_DESC_DMA_ALLOC;
		
		dbg_sg("Descriptor DMA record %u: virtual address %p, DMA address %#016llx\n",
			block_num, transfer->adj_desc_blocks[block_num].virtual_addr, transfer->adj_desc_blocks[block_num].dma_addr);
		
		for(i; (processed_sg_entries < sg_nents) && (i<engine->adj_block_len); ++i,  desc_dma_addr+=sizeof(struct xdma_desc))
		{
			current_desc=&(transfer->adj_desc_blocks[block_num].virtual_addr[i]);
			dma_addr_t desc_start= sg_dma_address(sg_iter);
			unsigned int desc_length=0;
			unsigned int desc_length_accum=0;
							
			dbg_sg("SG entries:\n");
			/*merge entries that are contiguous in DMA (bus) address space*/
			while((processed_sg_entries < sg_nents) && ((sg_prev==NULL) || ((sg_dma_address(sg_prev)+sg_dma_len(sg_prev))==sg_dma_address(sg_iter))) )
			{
				desc_length_accum+=sg_dma_len(sg_iter);
				if(likely(desc_length_accum<=XDMA_DESC_BLEN_MAX))
				{
					dbg_sg("%u: 0x%p, pg 0x%p,%u+%u, dma %#016llx,%u.\n", processed_sg_entries, sg_iter, sg_page((struct scatterlist *) sg_iter), 
					sg_iter->offset, sg_iter->length, sg_dma_address(sg_iter),sg_dma_len(sg_iter));
					desc_length=desc_length_accum;
					++processed_sg_entries;
					sg_prev=sg_iter;
					sg_iter=sg_next(sg_iter);
				}
				else 
					break;
					
				
			}
			dbg_sg("were merged into descriptor %u with start DMA addr %llx, length %u:\n", i, desc_start, desc_length);
		
		
		
		
			current_desc->bytes=cpu_to_le32(desc_length);
			if(engine->dir== DMA_TO_DEVICE)
			{
				split_into_val32(desc_start, current_desc->src_addr_hi,
						 current_desc->src_addr_lo);
				if(!engine->streaming)
				{
					split_into_val32(ep_addr, current_desc->dst_addr_hi, current_desc->dst_addr_lo);
				}
			}
			else
			{
				split_into_val32(desc_start, current_desc->dst_addr_hi, current_desc->dst_addr_lo);
				if(!engine->streaming)
				{
					split_into_val32(ep_addr, current_desc->src_addr_hi,
						 current_desc->src_addr_lo);
				}	
			}
			
			if(i>0) /*link to previuos descriptor*/
			{		
				split_into_val32(desc_dma_addr, (current_desc-1)->next_hi, (current_desc-1)->next_lo);
									
				//dump_sg_with_desc(sg_prev, current_desc-1);
			
				
			}
			
			if(!engine->streaming && !engine->non_incr_addr)
				ep_addr+=desc_length;
			
			dump_desc(current_desc);
					
			
		}
		/*there are still unprocessed sg entries. Transfer doesn't fit into single adjacent block*/
		if ( processed_sg_entries<sg_nents)
		{
			pr_err("Transfer requires more than a single block of %u adjacent descripors."
			"Splitting into multiple transfers is required, but currently not implemented", 
			engine->adj_block_len);
			return -EFBIG;
		}
		/*set control flags on the very last descriptor*/
		current_desc->control|=cpu_to_le32(control_flags);
		desc_in_block=i;
		for(i=0; i<desc_in_block; ++i)/*desc magic and next adjacent*/
		{
			transfer->adj_desc_blocks[block_num].virtual_addr[i].control|= cpu_to_le32(DESC_MAGIC|((desc_in_block-i-1)<<DESC_ADJ_SHIFT));
		}
		//dump_desc(current_desc);
	}
	
	

	return 0;
}


#if !LINUX_VERSION_CHECK(5,8,0)
/*steal the code from newer kernel versions. Kind of local backport.
Both easier and safer than add to the mess below with fences*/
static int dma_map_sgtable(struct device *dev, struct sg_table *sgt,
		enum dma_data_direction dir, unsigned long attrs)
{
	int nents;

	nents = dma_map_sg_attrs(dev, sgt->sgl, sgt->orig_nents, dir, attrs);
	if (nents <= 0)
		return -EINVAL;
	sgt->nents = nents;
	return 0;
}

static void dma_unmap_sgtable(struct device *dev, struct sg_table *sgt,
		enum dma_data_direction dir, unsigned long attrs)
{
	dma_unmap_sg_attrs(dev, sgt->sgl, sgt->orig_nents, dir, attrs);
}
#endif


/*all transfer initalisation stages. Linux kernel API changes
cause quite a mess*/ 
static int xdma_prepare_transfer(struct xdma_engine *engine)
{
	int rv=0;
	const struct xdma_transfer_params *transfer_params=&(engine->transfer_params);
	struct xdma_transfer *transfer=&(engine->transfer);
	transfer->num_pages=(((uintptr_t)transfer_params->buf + transfer_params->length + PAGE_SIZE - 1) -
				 ((uintptr_t)transfer_params->buf & PAGE_MASK))>> PAGE_SHIFT;
	/*Here is where kvmalloc is really neccessary, because the size of transfer and
	therefore number of pages can vary widely, from just a few to millions.
	Save time by ommitting zeroing, since they are going to be overwritten 
	in the next step anyway*/
	transfer->pages=kvmalloc_array(transfer->num_pages, sizeof(struct page*), GFP_KERNEL);
	if(unlikely(transfer->pages==NULL))
	{
		dbg_sg("Failed to allocate memory for pages on engine %s\n", engine->name);
		return -ENOMEM;
	}
	dbg_sg("Pages for transfer on engine %s were allocated with %cmalloc\n", engine->name,
		is_vmalloc_addr(transfer->pages)? 'v' :'k');
	transfer->cleanup_flags|=XFER_FLAG_PAGES_ALLOC;
	#if LINUX_VERSION_CHECK(5,6,0)
	/*pin_user_pages (not get_...) should be used in DMA application. see Linux docs*/
	rv=pin_user_pages_fast((unsigned long)transfer_params->buf, transfer->num_pages,
				FOLL_WRITE, transfer->pages);
	#else
	rv=get_user_pages_fast((unsigned long)transfer_params->buf, transfer->num_pages,
				FOLL_WRITE, transfer->pages);
	#endif
	if(unlikely(rv<0))
	{
		dbg_sg("Unable to pin user pages on engine %s\n", engine->name);
		return rv; 
	}
	transfer->cleanup_flags|=XFER_FLAG_PAGES_PINNED;
	if(unlikely(rv<transfer->num_pages))
	{
		dbg_sg("Not all pages could be pinned on engine %s\n", engine->name);
		transfer->num_pages=rv;/*to be able correctly unpin pages*/
		return -EFAULT;
	}
	/*the functions allocate small chunks of memory. If that fails, there is 
	no point trying, hence __NO_RETRY*/
	#if LINUX_VERSION_CHECK(5,15,0)
	rv=sg_alloc_table_from_pages_segment(&(transfer->sgt), transfer->pages, 
			transfer->num_pages, offset_in_page(transfer_params->buf),
			transfer_params->length, XDMA_DESC_BLEN_MAX, GFP_KERNEL|__GFP_NORETRY);
	#elif LINUX_VERSION_CHECK(5,10,0)
	rv=PTR_ERR_OR_ZERO(__sg_alloc_table_from_pages(&(transfer->sgt), 
			transfer->pages, transfer->num_pages, 
			offset_in_page(transfer_params->buf), transfer_params->length,
			XDMA_DESC_BLEN_MAX, NULL, 0, GFP_KERNEL|__GFP_NORETRY ));
	#elif LINUX_VERSION_CHECK(4,15,0)
	rv=__sg_alloc_table_from_pages(&(transfer->sgt), transfer->pages, 
			transfer->num_pages, offset_in_page(transfer_params->buf),
			transfer_params->length, XDMA_DESC_BLEN_MAX & PAGE_MASK,
			GFP_KERNEL|__GFP_NORETRY);
	#else
	/*this version has increased chance to fail,
	 because of absent limit for the length of scatterlist entries.
	 to be on safe side, it advisable to limit transfer length to XDMA_DESC_BLEN_MAX*/
	rv=sg_alloc_table_from_pages(&(transfer->sgt), transfer->pages, transfer->num_pages,
				offset_in_page(transfer_params->buf), transfer_params->length,
				GFP_KERNEL|__GFP_NORETRY);
	#endif
	if (unlikely(rv<0))
	{
		dbg_sg("Failed to allocate SG table for engine %s\n", engine->name);
		return rv;
	}
	transfer->cleanup_flags|=XFER_FLAG_SGTABLE_ALLOC;
	
	rv=dma_map_sgtable(&(engine->xdev->pdev->dev), &(transfer->sgt), engine->dir, 0);
	if (unlikely(rv<0))
	{
		dbg_sg("Failed to map sg table for engine %s\n", engine->name);
		return rv; 
	}
	transfer->cleanup_flags|=XFER_FLAG_SGTABLE_MAPPED;
	dbg_sg("Num pages %u, sg entries after allocation %u, after mapping %u\n", 
		transfer->num_pages, transfer->sgt.orig_nents, transfer->sgt.nents);
	
	rv=xdma_sgtable_to_descriptors(engine);
	
	return rv;	
}

/*retrieve first adjacent count from an adjacent block. can be used for setting up transfer as well
as to figure out the number of descriptors in the block */
static inline u32 get_initial_adj_count(struct xdma_engine *engine, unsigned int adj_block_num)
{
	return (le32_to_cpu(engine->transfer.adj_desc_blocks[adj_block_num].virtual_addr[0].control) & DESC_ADJ_MASK)>>DESC_ADJ_SHIFT;
}

static void xdma_launch_transfer(struct xdma_engine *engine)
{
	dma_addr_t first_desc_addr=engine->transfer.adj_desc_blocks[0].dma_addr;
	
	if((engine->dir==DMA_FROM_DEVICE)&&(engine->streaming)&&(enable_st_c2h_credit>0))
		write_register(min_t(unsigned int, enable_st_c2h_credit, XDMA_MAX_C2H_CREDITS), 
		&(engine->sgdma_regs->credits), 0);
	reinit_completion(&(engine->engine_compl));	
#ifdef XDMA_POLL_MODE
	engine->poll_mode_wb.virtual_addr->completed_desc_count=0;
#endif
	write_register((u32) first_desc_addr, &(engine->sgdma_regs->first_desc_lo), 0);
	write_register((u32) (first_desc_addr>>32), &(engine->sgdma_regs->first_desc_hi), 0);
	write_register(get_initial_adj_count(engine, 0), &(engine->sgdma_regs->first_desc_adjacent), 0);
	
	
	write_register(XDMA_CTRL_RUN_STOP, &(engine->regs->control_w1s), 0);

								
}

/*In case of timeout the function checks, if some progress has been made, that means descriptors were completed.
	If so wait longer for a timeout period to allow transfer to proceed*/
static long xdma_wait_for_transfer(struct xdma_engine *engine)
{
	long rv;
	u32 last_completed_descriptors=0;
	unsigned int timeout=(engine->dir==DMA_TO_DEVICE)? h2c_timeout_ms: c2h_timeout_ms;
	unsigned long timeout_jiffies=(timeout==0)? MAX_SCHEDULE_TIMEOUT : msecs_to_jiffies(timeout);
#ifdef XDMA_POLL_MODE
	unsigned long jiffies_limit= jiffies + timeout_jiffies;
	unsigned int descriptors_count= get_initial_adj_count(engine, 0)+1;/*this gives the number of descriptors in an adjacent block*/
	/*replicates behaviour of wait_for_completion for unified handling of transfer result*/
	do
	{	
		u32 poll_wb=engine->poll_mode_wb.virtual_addr->completed_desc_count;
		u32 current_completed_descriptors=poll_wb & WB_COUNT_MASK;
		/*return a positive value. xdma_finalise_transfer will deal with it appropriately*/
		if((current_completed_descriptors >= descriptors_count) || (poll_wb & WB_ERR_MASK))
		{
			dbg_tfr("Poll mode readback: %x\n", poll_wb);
			return 1;
		}
		/* return 0 to signify timeout*/
		if (timeout>0  && time_after_eq(jiffies, jiffies_limit))
		{
			if(current_completed_descriptors > last_completed_descriptors)
			{
				jiffies_limit= jiffies + timeout_jiffies;//reset timer;
				last_completed_descriptors=current_completed_descriptors;
				
			}
			else
				return 0;
				
		}
	/*catch signals*/
	} while(!signal_pending(current));
	/*like wait for completion*/
	return -ERESTARTSYS;
	
#else/* wait for interrupt with completion*/
	
	while((rv=wait_for_completion_interruptible_timeout( &(engine->engine_compl), timeout_jiffies))==0)
	{
		u32 current_completed_descriptors=ioread32( &(engine->regs->completed_desc_count));
		if(current_completed_descriptors > last_completed_descriptors)
			last_completed_descriptors=current_completed_descriptors;
		else
			break;
	}
	dbg_tfr("Wait for completion on engine %s returned %ld\n", engine->name, rv);
	
	return rv;
#endif
}

static ssize_t calculate_completed_length(const struct xdma_engine *engine, u32 num_descriptors)
{
	ssize_t completed_length=0;
	unsigned int i=0;
	for(i; i< num_descriptors; ++i)
		completed_length += engine->transfer.adj_desc_blocks[0].virtual_addr[i].bytes;
		
	return completed_length;

}

static ssize_t xdma_finalise_transfer(struct xdma_engine *engine, ssize_t transfer_result)
{
#ifndef XDMA_POLL_MODE
	channel_interrupts_disable(engine->xdev, engine->irq_bitmask);
#endif
	if(transfer_result >0)/*interrupt was recieved*/
	{
		u32 status=ioread32( &(engine->regs->status_rc));
		xdma_engine_stop(engine);
		if(engine_process_status(engine, status))
			transfer_result=engine->transfer_params.length;
		else
			transfer_result= -EIO;
		
		
	}
	else/* timeout or signal*/
	{
		
		u32 completed_descriptors=ioread32( &(engine->regs->completed_desc_count));
		xdma_engine_stop(engine);
		dbg_tfr("%u descriptors were completed on engine %s\n", completed_descriptors, engine->name);
		if(transfer_result==0)
		{
			pr_warn("Transfer on engine %s has timed out.\n", engine->name);
			if(completed_descriptors==0)
			{
				transfer_result=-ETIMEDOUT;
				goto exit;
			}
		}
		else/*transfer result < 0: signal was recieved*/
		{
			pr_warn("Transfer on engine %s has been interrupted by a signal.\n", engine->name);
			if(completed_descriptors==0)
			{
				 transfer_result=-EINTR;
				 goto exit;
			}
		}
		
		/*calculate and return total length of completed descriptors*/ 
		transfer_result=calculate_completed_length(engine, completed_descriptors);
	}
	
	exit:
#ifndef XDMA_POLL_MODE
	channel_interrupts_enable(engine->xdev, engine->irq_bitmask);
#endif	
	return transfer_result;
}
static void xdma_cleanup_transfer(struct xdma_engine *engine, bool transfer_ok)
{
	struct xdma_transfer *transfer=&(engine->transfer);
	dbg_tfr("Cleanup flags: %x\n", transfer->cleanup_flags);
	
	if(transfer->cleanup_flags & XFER_FLAG_DESC_DMA_ALLOC)
	{
		unsigned int i=0;
		for(i; (i <transfer->num_adj_blocks)&& (transfer->adj_desc_blocks[i].length>0); ++i)
			dma_pool_free(engine->desc_pool, transfer->adj_desc_blocks[transfer->num_adj_blocks-i-1].virtual_addr,
					 transfer->adj_desc_blocks[transfer->num_adj_blocks-i-1].dma_addr);
	}
	
	if(transfer->cleanup_flags & XFER_FLAG_DMA_RECORD_ALLOC)
		kfree(transfer->adj_desc_blocks);
	
	if(transfer->cleanup_flags & XFER_FLAG_SGTABLE_MAPPED)
		dma_unmap_sgtable(&(engine->xdev->pdev->dev), &(transfer->sgt), engine->dir, 0);
	
	if(transfer->cleanup_flags & XFER_FLAG_SGTABLE_ALLOC)
		sg_free_table(&(transfer->sgt));
	
	if(transfer->cleanup_flags & XFER_FLAG_PAGES_PINNED)
	#if LINUX_VERSION_CHECK(5,6,0)/*Mark pages dirty for successful C2H transfers*/
		unpin_user_pages_dirty_lock(transfer->pages, transfer->num_pages,
					 (engine->dir==DMA_FROM_DEVICE) && transfer_ok);
	#else
	{
		struct page **page_iter=transfer->pages;
		struct page **pages_end=transfer->pages+transfer->num_pages;
		for(page_iter; page_iter!=pages_end; ++page_iter)
		{
			if((engine->dir==DMA_FROM_DEVICE) && transfer_ok)
				set_page_dirty_lock(*page_iter);
			
			put_page(*page_iter);
		}
	}	
	#endif
	if(transfer->cleanup_flags & XFER_FLAG_PAGES_ALLOC)
		kvfree(transfer->pages);
	/*clear for the next transfer*/	
	memset(transfer, 0, sizeof(struct xdma_transfer));
	
}


ssize_t xdma_xfer_submit(struct xdma_engine *engine)
{

	ssize_t rv=0;
	rv=xdma_validate_transfer(engine);
	if (rv<0)
		return rv; 
		
	rv=xdma_prepare_transfer(engine);
	if(rv<0)
		goto cleanup;
	xdma_launch_transfer(engine);
	rv=xdma_wait_for_transfer(engine);
	rv=xdma_finalise_transfer(engine, rv);
	
	cleanup:
	xdma_cleanup_transfer(engine, rv>0);
	
	return rv;
}


int xdma_performance_submit(struct xdma_engine *engine)
{
	u64 ep_addr = engine->streaming? : engine->xdma_perf.axi_address;
	int i=0;
	int rv = 0;
	struct xdma_desc *current_desc;
	/*array of DMA records for performance buffers*/
	generic_dma_record(u8) *perf_bufs=NULL;
	unsigned int num_bufs= divide_roundup(engine->xdma_perf.transfer_size, KMALLOC_MAX_SIZE);
	unsigned int next_adj=num_bufs - 1;
	dma_addr_t desc_dma_addr;
	if(num_bufs > engine->adj_block_len)
		return -EFBIG;
		
	if(engine->xdma_perf.transfer_size==0 ||
	engine->xdma_perf.transfer_size & (engine->xdev->datapath_width-1))
	{
		pr_err("Invalid performance transfer length. It must be positive and aligned to datapath width %u\n", 
			engine->xdev->datapath_width);
		return -EINVAL;
	}
	/*neccessary for correct function of xdma_finalise_transfer*/
	engine->transfer_params.length=engine->xdma_perf.transfer_size;
	perf_bufs=kcalloc(num_bufs, sizeof(*perf_bufs), GFP_KERNEL|__GFP_RETRY_MAYFAIL);
	if (perf_bufs==NULL)
		return -ENOMEM;
	/*in order to use xdma_launch_transfer*/
	engine->transfer.adj_desc_blocks=kzalloc(sizeof( *(engine->transfer.adj_desc_blocks)),
					 GFP_KERNEL|__GFP_RETRY_MAYFAIL);
	if(engine->transfer.adj_desc_blocks==NULL)
	{
		rv=-ENOMEM;
		goto free_record;
	}
	engine->transfer.adj_desc_blocks->virtual_addr=dma_pool_zalloc(engine->desc_pool, GFP_KERNEL|__GFP_RETRY_MAYFAIL, &desc_dma_addr);
	if(engine->transfer.adj_desc_blocks->virtual_addr==NULL)
	{
		dbg_sg("Failed to allocate memory from descriptor pool on engune %s\n", engine->name);	
		rv=-ENOMEM;	
		goto free_adj_blocks;
	}
	engine->transfer.adj_desc_blocks->dma_addr=desc_dma_addr;
	engine->transfer.adj_desc_blocks->length=engine->adj_block_len*sizeof(struct xdma_desc);
	
	for(i; i<num_bufs; ++i, desc_dma_addr+=sizeof(struct xdma_desc), --next_adj)
	{
		current_desc=&(engine->transfer.adj_desc_blocks->virtual_addr[i]);
		unsigned int buf_length= (i==(num_bufs-1))? engine->xdma_perf.transfer_size & (KMALLOC_MAX_SIZE-1) : KMALLOC_MAX_SIZE;
		perf_bufs[i].virtual_addr=kmalloc(buf_length, GFP_KERNEL|__GFP_RETRY_MAYFAIL);
		if(perf_bufs[i].virtual_addr==NULL)
		{	
			rv=-ENOMEM;
			goto unmap_bufs;
		}
					
		perf_bufs[i].dma_addr=dma_map_single( &(engine->xdev->pdev->dev), perf_bufs[i].virtual_addr, buf_length, engine->dir);
		rv=dma_mapping_error( &(engine->xdev->pdev->dev), perf_bufs[i].dma_addr);
		if(rv<0)
		{
			goto unmap_bufs;		
		}
		perf_bufs[i].length=buf_length;
		
		current_desc->control= cpu_to_le32(DESC_MAGIC|(next_adj<<DESC_ADJ_SHIFT));/*desc magic and next adjacent*/
		current_desc->bytes=buf_length;
		if(engine->dir== DMA_TO_DEVICE)
		{
			split_into_val32(perf_bufs[i].dma_addr, current_desc->src_addr_hi,
					 current_desc->src_addr_lo);
			if(!engine->streaming)
			{
				split_into_val32(ep_addr, current_desc->dst_addr_hi, current_desc->dst_addr_lo);
			}
		}
		else
		{
			split_into_val32(perf_bufs[i].dma_addr, current_desc->dst_addr_hi, current_desc->dst_addr_lo);
			if(!engine->streaming)
			{
				split_into_val32(ep_addr, current_desc->src_addr_hi,
					 current_desc->src_addr_lo);
			}	
		}
		
		if(i>0) /*link to previuos descriptor*/
		{		
			split_into_val32(desc_dma_addr, (current_desc-1)->next_hi, (current_desc-1)->next_lo);
								
			dump_desc(current_desc-1);
		
			
		}
		
		if(!engine->streaming && !engine->non_incr_addr)
			ep_addr+=buf_length;
		
		
		
	}
	
	current_desc->control|=(XDMA_DESC_STOPPED|XDMA_DESC_COMPLETED);
	dump_desc(current_desc);
	
	enable_perf(engine);	
	xdma_launch_transfer(engine);
	rv=xdma_wait_for_transfer(engine);
	rv=xdma_finalise_transfer(engine, rv);
	if(rv!=engine->xdma_perf.transfer_size)
	{
		pr_err("Perfomance test failed on engine %s\n", engine->name);
		if(rv>=0)
			rv= -EIO;
		goto unmap_bufs;
			
	}
	get_perf_stats(engine);
	
	
	
unmap_bufs:/*\/back to last index*/
	for(--i; i>=0; --i)
	{
		if(perf_bufs[i].length>0)
			dma_unmap_single( &(engine->xdev->pdev->dev), perf_bufs[i].dma_addr,
					 perf_bufs[i].length, engine->dir);
		
		kfree(perf_bufs[i].virtual_addr);
	
	}
	dma_pool_free(engine->desc_pool, engine->transfer.adj_desc_blocks->virtual_addr, 
				engine->transfer.adj_desc_blocks->dma_addr);
free_adj_blocks:
	kfree(engine->transfer.adj_desc_blocks);
free_record:
	kfree(perf_bufs);
	return rv;
}

static struct xdma_dev *alloc_dev_instance(struct pci_dev *pdev)
{
	int i;
	struct xdma_dev *xdev;
	struct xdma_engine *engine;

	if (!pdev) {
		pr_err("Invalid pdev\n");
		return NULL;
	}

	/* allocate zeroed device book keeping structure */
	xdev = kzalloc(sizeof(struct xdma_dev), GFP_KERNEL);
	if (!xdev) {
		pr_info("OOM, xdma_dev.\n");
		return NULL;
	}
	spin_lock_init(&xdev->lock);

	xdev->magic = MAGIC_DEVICE;
	xdev->config_bar_idx = -1;
	xdev->user_bar_idx = -1;
	xdev->bypass_bar_idx = -1;
	xdev->irq_line = -1;

	/* create a driver to device reference */
	xdev->pdev = pdev;
	dbg_init("xdev = 0x%p\n", xdev);

	/* Set up data user IRQ data structures */
	for (i = 0; i < 16; i++) {
		xdev->user_irq[i].xdev = xdev;
		spin_lock_init(&xdev->user_irq[i].events_lock);
		init_waitqueue_head(&xdev->user_irq[i].events_wq);
		xdev->user_irq[i].handler = NULL;
		xdev->user_irq[i].user_idx = i; /* 0 based */
	}


	return xdev;
}

static int request_regions(struct xdma_dev *xdev, struct pci_dev *pdev)
{
	int rv;

	if (!xdev) {
		pr_err("Invalid xdev\n");
		return -EINVAL;
	}

	if (!pdev) {
		pr_err("Invalid pdev\n");
		return -EINVAL;
	}

	dbg_init("pci_request_regions()\n");
	rv = pci_request_regions(pdev, xdev->mod_name);
	/* could not request all regions? */
	if (rv) {
		dbg_init("pci_request_regions() = %d, device in use?\n", rv);
		/* assume device is in use so do not disable it later */
		xdev->regions_in_use = 1;
	} else {
		xdev->got_regions = 1;
	}

	return rv;
}

static int set_dma_mask(struct pci_dev *pdev)
{
	if (!pdev) {
		pr_err("Invalid pdev\n");
		return -EINVAL;
	}

	dbg_init("sizeof(dma_addr_t) == %ld\n", sizeof(dma_addr_t));
	/* 64-bit addressing capability for XDMA? avilible since 3.13*/
	if (!dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64))) 
	{
		/* query for DMA transfer */
		/* @see Documentation/DMA-mapping.txt */
		dbg_init("set_dma_mask(64)\n");

		/* use 64-bit DMA */
		dbg_init("Using a 64-bit DMA mask.\n");
	} else if (!dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32))) 

	{
		dbg_init("Could not set 64-bit DMA mask.\n");
		/* use 32-bit DMA */
		dbg_init("Using a 32-bit DMA mask.\n");
	} else {
		dbg_init("No suitable DMA possible.\n");
		return -EINVAL;
	}

	return 0;
}

static int get_engine_channel_id(struct engine_regs *regs)
{
	int value;

	if (!regs) {
		pr_err("Invalid engine registers\n");
		return -EINVAL;
	}

	value = read_register(&regs->identifier);

	return (value & 0x00000f00U) >> 8;
}

static int get_engine_id(struct engine_regs *regs)
{
	int value;

	if (!regs) {
		pr_err("Invalid engine registers\n");
		return -EINVAL;
	}

	value = read_register(&regs->identifier);
	return (value & 0xffff0000U) >> 16;
}

static void remove_engines(struct xdma_dev *xdev)
{
	struct xdma_engine *engine;
	int i;
	int rv;

	if (!xdev) {
		pr_err("Invalid xdev\n");
		return;
	}

	/* iterate over channels */
	for (i = 0; i < xdev->h2c_channel_num; i++) {
		engine = &xdev->engine_h2c[i];
		if (engine->magic == MAGIC_ENGINE) {
			dbg_sg("Remove %s, %d", engine->name, i);
			rv = engine_destroy(xdev, engine);
			if (rv < 0)
				pr_err("Failed to destroy H2C engine %d\n", i);
			dbg_sg("%s, %d removed", engine->name, i);
		}
	}
	kfree(xdev->engine_h2c);

	for (i = 0; i < xdev->c2h_channel_num; i++) {
		engine = &xdev->engine_c2h[i];
		if (engine->magic == MAGIC_ENGINE) {
			dbg_sg("Remove %s, %d", engine->name, i);
			rv = engine_destroy(xdev, engine);
			if (rv < 0)
				pr_err("Failed to destroy C2H engine %d\n", i);
			dbg_sg("%s, %d removed", engine->name, i);
		}
	}
	kfree(xdev->engine_c2h);
}

/*read max read request size register from the device and set parameter accordingly*/
static inline void set_max_read_request_size(struct xdma_dev *xdev)
{
	unsigned int max_read_request_size=128;
	struct config_regs *cfg_regs= (struct config_regs *)(xdev->bar[xdev->config_bar_idx] + XDMA_OFS_CONFIG);	
	u32 mrrs_reg=ioread32( &(cfg_regs->max_read_request_size));
	switch(mrrs_reg)
	{
		case 0:
			max_read_request_size= 128;
			break;
		case 1:
			max_read_request_size= 256;
			break;
		case 2:
			max_read_request_size= 512;
			break;
		case 3:
			max_read_request_size= 1024;
			break;
		case 4:
			max_read_request_size= 2048;
			break;
		case 5:
			max_read_request_size= 4096;
			break;
		default:
			pr_warn("Unexpected value for MRRS register 0x%x. Falling back to 128 bytes", mrrs_reg);
			max_read_request_size= 128;
			break;
	}
	
	const_cast(unsigned int, xdev->max_read_request_size)=max_read_request_size;
}
/*read datapath width register from the device and set parameter accordingly*/
static inline void set_datapath_width(struct xdma_dev *xdev)
{
	unsigned int datapath_width=64/8;
	struct config_regs *cfg_regs= (struct config_regs *)(xdev->bar[xdev->config_bar_idx] + XDMA_OFS_CONFIG);
	u32 datapath_reg=ioread32( &(cfg_regs->datapath_width));
	switch(datapath_reg)
	{
		case 0:
			datapath_width= 64/8;
			break;
		case 1:
			datapath_width= 128/8;
			break;	
		case 2:
			datapath_width= 256/8;
			break;	
		case 3:
			datapath_width= 512/8;
			break;
		default:
			pr_warn("Unexpected value for datapath width register 0x%x. Falling back to 64 bits bytes", datapath_reg);
			datapath_width= 64/8;
			break;		
	}
	const_cast(unsigned int, xdev->datapath_width)=datapath_width;
}


static bool probe_for_engine(struct xdma_dev *xdev, enum dma_data_direction dir,
			    int channel)
{
	struct engine_regs *regs;
	int offset = get_engine_offset(dir, channel);
	u32 engine_id;
	u32 engine_id_expected;
	u32 channel_id;
	struct xdma_engine *engine;
	int rv;

	if (dir == DMA_TO_DEVICE) {
		engine_id_expected = XDMA_ID_H2C;
		engine = &xdev->engine_h2c[channel];
	} else {
		engine_id_expected = XDMA_ID_C2H;
		engine = &xdev->engine_c2h[channel];
	}
	regs = xdev->bar[xdev->config_bar_idx] + offset;
	engine_id = get_engine_id(regs);
	channel_id = get_engine_channel_id(regs);

	if ((engine_id != engine_id_expected) || (channel_id != channel)) {
		dbg_init(
			"%s %d engine, reg off 0x%x, id mismatch 0x%x,0x%x,exp 0x%x,0x%x, SKIP.\n",
			dir == DMA_TO_DEVICE ? "H2C" : "C2H", channel, offset,
			engine_id, channel_id, engine_id_expected,
			channel_id != channel);
		return false;
	}

	dbg_init("found AXI %s %d engine, reg. off 0x%x, id 0x%x,0x%x.\n",
		 dir == DMA_TO_DEVICE ? "H2C" : "C2H", channel, offset,
		 engine_id, channel_id);

	return true;
}

static int probe_engines(struct xdma_dev *xdev)
{
	int i=0;
	int rv = 0;

	xdma_debug_assert_ptr(xdev);
	
	/*probe engines first to find number of engines. 
	this allows to correctly calculate max descriptors as well as dynamic allocation*/
	for(xdev->h2c_channel_num=0; (xdev->h2c_channel_num<XDMA_CHANNEL_NUM_MAX)&&probe_for_engine(xdev, DMA_TO_DEVICE, xdev->h2c_channel_num); ++xdev->h2c_channel_num);
	
	xdev->c2h_channel_num = 0;/* set to 0  already hereto allow correct destruction in case of error*/ 
	/* allocate and initialize engines */
	if(xdev->h2c_channel_num>0)
	{
		
		xdev->engine_h2c=kcalloc(xdev->h2c_channel_num, sizeof(struct xdma_engine), GFP_KERNEL|__GFP_RETRY_MAYFAIL);
		if(unlikely(xdev->engine_h2c==NULL))
		{
			pr_err("Failed to allocate memory for %s engines", "H2C");
			return -ENOMEM;
		}
		
		for (i=0;i<xdev->h2c_channel_num;i++)
		{
			rv = engine_init(xdev, DMA_TO_DEVICE, i);
			if (rv != 0) 
			{
				pr_err("failed to create AXI %s %d engine.\n", "H2C", i);
				return rv;
			}
		}
	}
	
	
	for (xdev->c2h_channel_num = 0;(xdev->c2h_channel_num<XDMA_CHANNEL_NUM_MAX)&&probe_for_engine(xdev, DMA_FROM_DEVICE, xdev->c2h_channel_num); ++xdev->c2h_channel_num);
	
	if (xdev->c2h_channel_num>0)
	{
		xdev->engine_c2h=kcalloc(xdev->c2h_channel_num, sizeof(struct xdma_engine), GFP_KERNEL|__GFP_RETRY_MAYFAIL);
		if(unlikely(xdev->engine_c2h==NULL))
		{
			pr_err("Failed to allocate memory for %s engines", "C2H");
			return -ENOMEM;
		}
			
		for (i=0;i<xdev->c2h_channel_num;i++)
		{
			rv = engine_init( xdev, DMA_FROM_DEVICE, i);
			if (rv != 0) 
			{
				pr_err("failed to create AXI %s %d engine.\n",
					"C2H", i);
				return rv;
			}
		}
	}
	return 0;
}


static void pci_enable_capability(struct pci_dev *pdev, int cap)
{
	u16 v;
	int pos;

	pos = pci_pcie_cap(pdev);
	if (pos > 0) {
		pci_read_config_word(pdev, pos + PCI_EXP_DEVCTL, &v);
		v |= cap;
		pci_write_config_word(pdev, pos + PCI_EXP_DEVCTL, v);
	}
}

void *xdma_device_open(const char *mname, struct pci_dev *pdev, int *user_max,
		       int *h2c_channel_num, int *c2h_channel_num)
{
	struct xdma_dev *xdev = NULL;
	int rv = 0;

	pr_info("%s device %s, 0x%p.\n", mname, dev_name(&pdev->dev), pdev);

	/* allocate zeroed device book keeping structure */
	xdev = alloc_dev_instance(pdev);
	if (unlikely(!xdev))
		goto err_alloc_dev_instance;
	xdev->mod_name = mname;
	xdev->user_max = *user_max;
	xdev->h2c_channel_num = *h2c_channel_num;
	xdev->c2h_channel_num = *c2h_channel_num;

	set_bit( XDEV_FLAG_OFFLINE_BIT, &(xdev->flags));

	if (xdev->user_max == 0 || xdev->user_max > MAX_USER_IRQ)
		xdev->user_max = MAX_USER_IRQ;
	if (xdev->h2c_channel_num == 0 ||
	    xdev->h2c_channel_num > XDMA_CHANNEL_NUM_MAX)
		xdev->h2c_channel_num = XDMA_CHANNEL_NUM_MAX;
	if (xdev->c2h_channel_num == 0 ||
	    xdev->c2h_channel_num > XDMA_CHANNEL_NUM_MAX)
		xdev->c2h_channel_num = XDMA_CHANNEL_NUM_MAX;

	rv = xdev_list_add(xdev);
	if (unlikely(rv < 0))
		goto err_xdev_list_add;

	rv = pci_enable_device(pdev);
	if (unlikely(rv)) {
		dbg_init("pci_enable_device() failed, %d.\n", rv);
		goto err_pci_enable_device;
	}

	/* keep INTx enabled */
	pci_check_intr_pend(pdev);

	/* enable relaxed ordering */
	pci_enable_capability(pdev, PCI_EXP_DEVCTL_RELAX_EN);

	/* enable extended tag */
	pci_enable_capability(pdev, PCI_EXP_DEVCTL_EXT_TAG);

	/* enable bus master capability */
	pci_set_master(pdev);
	/*limit sg entries after possible merging by DMA mapping 
		to max descriptor length rounded down to page boundary*/
	dma_set_max_seg_size( &(pdev->dev), XDMA_DESC_BLEN_MAX & PAGE_MASK);
	
	rv = request_regions(xdev, pdev);
	if (unlikely(rv))
		goto err_request_regions;

	rv = map_bars(xdev, pdev);
	if (unlikely(rv))
		goto err_map_bars;
	
	rv = set_dma_mask(pdev);
	if (unlikely(rv))
		goto err_set_dma_mask;

	check_nonzero_interrupt_status(xdev);
	/* explicitely zero all interrupt enable masks */
	channel_interrupts_disable(xdev, ~0);
	user_interrupts_disable(xdev, ~0);
	read_interrupts(xdev);

	set_max_read_request_size(xdev);
	set_datapath_width(xdev);
	dbg_init("XDMA MRRS is %u byte, datapath width is %u bit/%u byte", 
		xdev->max_read_request_size, xdev->datapath_width*8, xdev->datapath_width);
	rv = probe_engines(xdev);
	if (unlikely(rv))
		goto err_probe_engines;

	rv = enable_msi_msix(xdev, pdev);
	if (unlikely(rv < 0))
		goto err_enable_msi_msix;

	rv = irq_setup(xdev, pdev);
	if (unlikely(rv < 0))
		goto err_irq_setup;

#ifndef XDMA_POLL_MODE
	channel_interrupts_enable(xdev, xdev->mask_irq_h2c | xdev->mask_irq_c2h);
#endif

	/* Flush writes */
	read_interrupts(xdev);

	*user_max = xdev->user_max;
	*h2c_channel_num = xdev->h2c_channel_num;
	*c2h_channel_num = xdev->c2h_channel_num;

	clear_bit(XDEV_FLAG_OFFLINE_BIT, &(xdev->flags));
	return (void *)xdev;

err_irq_setup:
	disable_msi_msix(xdev, pdev);
err_enable_msi_msix:
	remove_engines(xdev);
err_probe_engines:
err_set_dma_mask:
	unmap_bars(xdev, pdev);
err_map_bars:
	if (xdev->got_regions)
		pci_release_regions(pdev);
err_request_regions:
	if (!xdev->regions_in_use)
		pci_disable_device(pdev);
err_pci_enable_device:
	xdev_list_remove(xdev);
err_xdev_list_add:
	kfree(xdev);
err_alloc_dev_instance:
	return NULL;
}

void xdma_device_close(struct pci_dev *pdev, void *dev_hndl)
{
	struct xdma_dev *xdev = (struct xdma_dev *)dev_hndl;

	dbg_init("pdev 0x%p, xdev 0x%p.\n", pdev, dev_hndl);

	if (!dev_hndl)
		return;

	if (debug_check_dev_hndl(__func__, pdev, dev_hndl) < 0)
		return;

	dbg_sg("remove(dev = 0x%p) where pdev->dev.driver_data = 0x%p\n", pdev,
	       xdev);
	if (xdev->pdev != pdev) {
		dbg_sg("pci_dev(0x%lx) != pdev(0x%lx)\n",
		       (unsigned long)xdev->pdev, (unsigned long)pdev);
	}

	channel_interrupts_disable(xdev, ~0);
	user_interrupts_disable(xdev, ~0);
	read_interrupts(xdev);

	irq_teardown(xdev);
	disable_msi_msix(xdev, pdev);

	remove_engines(xdev);
	unmap_bars(xdev, pdev);

	if (xdev->got_regions) {
		dbg_init("pci_release_regions 0x%p.\n", pdev);
		pci_release_regions(pdev);
	}

	if (!xdev->regions_in_use) {
		dbg_init("pci_disable_device 0x%p.\n", pdev);
		pci_disable_device(pdev);
	}

	xdev_list_remove(xdev);

	kfree(xdev);
}

void xdma_device_offline(struct pci_dev *pdev, void *dev_hndl)
{
	struct xdma_dev *xdev = (struct xdma_dev *)dev_hndl;
	struct xdma_engine *engine;
	int i;
	int rv;

	if (!dev_hndl)
		return;

	if (debug_check_dev_hndl(__func__, pdev, dev_hndl) < 0)
		return;

	pr_info("pdev 0x%p, xdev 0x%p.\n", pdev, xdev);
	set_bit( XDEV_FLAG_OFFLINE_BIT, &(xdev->flags));

	/* wait for all engines to be idle */
	for (i = 0; i < xdev->h2c_channel_num; i++) {
		unsigned long flags;

		engine = &xdev->engine_h2c[i];

		if (engine->magic == MAGIC_ENGINE) {
			
			rv = xdma_engine_stop(engine);
			if (rv < 0)
				pr_err("Failed to stop engine\n");
			
		}
	}

	for (i = 0; i < xdev->c2h_channel_num; i++) {
		unsigned long flags;

		engine = &xdev->engine_c2h[i];
		if (engine->magic == MAGIC_ENGINE) {
			
			/*engine->shutdown |= ENGINE_SHUTDOWN_REQUEST;*/

			rv = xdma_engine_stop(engine);
			if (rv < 0)
				pr_err("Failed to stop engine\n");
			
		}
	}

	/* turn off interrupts */
	channel_interrupts_disable(xdev, ~0);
	user_interrupts_disable(xdev, ~0);
	read_interrupts(xdev);
	irq_teardown(xdev);

	pr_info("xdev 0x%p, done.\n", xdev);
}

void xdma_device_online(struct pci_dev *pdev, void *dev_hndl)
{
	struct xdma_dev *xdev = (struct xdma_dev *)dev_hndl;
	struct xdma_engine *engine;
	unsigned long flags;
	int i;

	if (!dev_hndl)
		return;

	if (debug_check_dev_hndl(__func__, pdev, dev_hndl) < 0)
		return;

	pr_info("pdev 0x%p, xdev 0x%p.\n", pdev, xdev);

	for (i = 0; i < xdev->h2c_channel_num; i++) {
		engine = &xdev->engine_h2c[i];
		if (engine->magic == MAGIC_ENGINE) {
			engine_init_regs(engine);

		}
	}

	for (i = 0; i < xdev->c2h_channel_num; i++) {
		engine = &xdev->engine_c2h[i];
		if (engine->magic == MAGIC_ENGINE) {
			engine_init_regs(engine);

		}
	}

	/* re-write the interrupt table */
#ifndef XDMA_POLL_PODE
	irq_setup(xdev, pdev);

	channel_interrupts_enable(xdev, xdev->mask_irq_h2c | xdev->mask_irq_c2h);
	user_interrupts_enable(xdev, xdev->mask_irq_user);
	read_interrupts(xdev);
#endif

	clear_bit( XDEV_FLAG_OFFLINE_BIT, &(xdev->flags));
	pr_info("xdev 0x%p, done.\n", xdev);
}

int xdma_device_restart(struct pci_dev *pdev, void *dev_hndl)
{
	struct xdma_dev *xdev = (struct xdma_dev *)dev_hndl;

	if (!dev_hndl)
		return -EINVAL;

	if (debug_check_dev_hndl(__func__, pdev, dev_hndl) < 0)
		return -EINVAL;

	pr_info("NOT implemented, 0x%p.\n", xdev);
	return -EINVAL;
}

int xdma_user_isr_register(void *dev_hndl, unsigned int mask,
			   irq_handler_t handler, void *dev)
{
	struct xdma_dev *xdev = (struct xdma_dev *)dev_hndl;
	int i;

	if (!dev_hndl)
		return -EINVAL;

	if (debug_check_dev_hndl(__func__, xdev->pdev, dev_hndl) < 0)
		return -EINVAL;

	for (i = 0; i < xdev->user_max && mask; i++) {
		unsigned int bit = (1 << i);

		if ((bit & mask) == 0)
			continue;

		mask &= ~bit;
		xdev->user_irq[i].handler = handler;
		xdev->user_irq[i].dev = dev;
	}

	return 0;
}

int xdma_user_isr_enable(void *dev_hndl, unsigned int mask)
{
	struct xdma_dev *xdev = (struct xdma_dev *)dev_hndl;

	if (!dev_hndl)
		return -EINVAL;

	if (debug_check_dev_hndl(__func__, xdev->pdev, dev_hndl) < 0)
		return -EINVAL;

	xdev->mask_irq_user |= mask;
	/* enable user interrupts */
	user_interrupts_enable(xdev, mask);
	read_interrupts(xdev);

	return 0;
}

int xdma_user_isr_disable(void *dev_hndl, unsigned int mask)
{
	struct xdma_dev *xdev = (struct xdma_dev *)dev_hndl;

	if (!dev_hndl)
		return -EINVAL;

	if (debug_check_dev_hndl(__func__, xdev->pdev, dev_hndl) < 0)
		return -EINVAL;

	xdev->mask_irq_user &= ~mask;
	user_interrupts_disable(xdev, mask);
	read_interrupts(xdev);

	return 0;
}

void engine_addrmode_set(struct xdma_engine *engine, bool set)
{
	u32 w = XDMA_CTRL_NON_INCR_ADDR;
	dbg_perf("XDMA_IOCTL_ADDRMODE_SET\n");
	if(engine->non_incr_addr!=set)
	{	
		engine->non_incr_addr = set;
		if (set)
			write_register(
				w, &engine->regs->control_w1s,
				(unsigned long)(&engine->regs->control_w1s) -
					(unsigned long)(&engine->regs));
		else
			write_register(
				w, &engine->regs->control_w1c,
				(unsigned long)(&engine->regs->control_w1c) -
					(unsigned long)(&engine->regs));
		
		engine_alignments(engine);
	}
	
}
