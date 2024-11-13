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
#include "libxdma_api.h"
#include "cdev_sgdma.h"



/* Module Parameters */
#ifdef XDMA_CONFIG_BAR_NUM
static int config_bar_num=XDMA_CONFIG_BAR_NUM;
#else 
static int config_bar_num=-1;
#endif
module_param(config_bar_num, int, 0444);/*no point to make it writable*/
MODULE_PARM_DESC(config_bar_num, "Input the config block BAR number");

static unsigned int interrupt_mode;
module_param(interrupt_mode, uint, 0644);
MODULE_PARM_DESC(interrupt_mode, "0 - Auto , 1 - MSI, 2 - Legacy, 3 - MSI-x");

static unsigned int enable_st_c2h_credit = 0;
module_param(enable_st_c2h_credit, uint, 0644);
MODULE_PARM_DESC(enable_st_c2h_credit,
	"Set 1 to enable ST C2H engine credit feature, default is 0 ( credit control disabled)");


#define XDMA_PERF_NUM_DESC 128




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
inline void __write_register(const char *fn, u32 value, void *iomem,
			     unsigned long off)
{
	pr_info("%s: w reg 0x%lx(0x%p), 0x%x.\n", fn, off, iomem, value);
	iowrite32(value, iomem);
}
#define write_register(v, mem, off) __write_register(__func__, v, mem, off)
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

	dbg_perf("IOCTL_XDMA_PERF_START\n");
}

void get_perf_stats(struct xdma_engine *engine)
{
	u32 hi;
	u32 lo;

	if (!engine) {
		pr_err("dma engine NULL\n");
		return;
	}

	if (!engine->xdma_perf) {
		pr_info("%s perf struct not set up.\n", engine->name);
		return;
	}

	hi = 0;
	lo = read_register(&engine->regs->completed_desc_count);
	engine->xdma_perf->iterations = build_u64(hi, lo);

	hi = read_register(&engine->regs->perf_cyc_hi);
	lo = read_register(&engine->regs->perf_cyc_lo);

	engine->xdma_perf->clock_cycle_count = build_u64(hi, lo);

	hi = read_register(&engine->regs->perf_dat_hi);
	lo = read_register(&engine->regs->perf_dat_lo);
	engine->xdma_perf->data_cycle_count = build_u64(hi, lo);

	hi = read_register(&engine->regs->perf_pnd_hi);
	lo = read_register(&engine->regs->perf_pnd_lo);
	engine->xdma_perf->pending_count = build_u64(hi, lo);
}

static int engine_reg_dump(struct xdma_engine *engine)
{
	u32 w;

	if (!engine) {
		pr_err("dma engine NULL\n");
		return -EINVAL;
	}

	w = read_register(&engine->regs->identifier);
	pr_info("%s: ioread32(0x%p) = 0x%08x (id).\n", engine->name,
		&engine->regs->identifier, w);
	w &= BLOCK_ID_MASK;
	if (w != BLOCK_ID_HEAD) {
		pr_err("%s: engine id missing, 0x%08x exp. & 0x%x = 0x%x\n",
		       engine->name, w, BLOCK_ID_MASK, BLOCK_ID_HEAD);
		return -EINVAL;
	}
	/* extra debugging; inspect complete engine set of registers */
	w = read_register(&engine->regs->status);
	pr_info("%s: ioread32(0x%p) = 0x%08x (status).\n", engine->name,
		&engine->regs->status, w);
	w = read_register(&engine->regs->control);
	pr_info("%s: ioread32(0x%p) = 0x%08x (control)\n", engine->name,
		&engine->regs->control, w);
	w = read_register(&engine->sgdma_regs->first_desc_lo);
	pr_info("%s: ioread32(0x%p) = 0x%08x (first_desc_lo)\n", engine->name,
		&engine->sgdma_regs->first_desc_lo, w);
	w = read_register(&engine->sgdma_regs->first_desc_hi);
	pr_info("%s: ioread32(0x%p) = 0x%08x (first_desc_hi)\n", engine->name,
		&engine->sgdma_regs->first_desc_hi, w);
	w = read_register(&engine->sgdma_regs->first_desc_adjacent);
	pr_info("%s: ioread32(0x%p) = 0x%08x (first_desc_adjacent).\n",
		engine->name, &engine->sgdma_regs->first_desc_adjacent, w);
	w = read_register(&engine->regs->completed_desc_count);
	pr_info("%s: ioread32(0x%p) = 0x%08x (completed_desc_count).\n",
		engine->name, &engine->regs->completed_desc_count, w);
	w = read_register(&engine->regs->interrupt_enable_mask);
	pr_info("%s: ioread32(0x%p) = 0x%08x (interrupt_enable_mask)\n",
		engine->name, &engine->regs->interrupt_enable_mask, w);

	return 0;
}

static void engine_status_dump(struct xdma_engine *engine)
{
	u32 v = engine->status;
	char buffer[256];
	char *buf = buffer;
	int len = 0;

	len = sprintf(buf, "SG engine %s status: 0x%08x: ", engine->name, v);

	if ((v & XDMA_STAT_BUSY))
		len += sprintf(buf + len, "BUSY,");
	if ((v & XDMA_STAT_DESC_STOPPED))
		len += sprintf(buf + len, "DESC_STOPPED,");
	if ((v & XDMA_STAT_DESC_COMPLETED))
		len += sprintf(buf + len, "DESC_COMPL,");

	/* common H2C & C2H */
	if ((v & XDMA_STAT_COMMON_ERR_MASK)) {
		if ((v & XDMA_STAT_ALIGN_MISMATCH))
			len += sprintf(buf + len, "ALIGN_MISMATCH ");
		if ((v & XDMA_STAT_MAGIC_STOPPED))
			len += sprintf(buf + len, "MAGIC_STOPPED ");
		if ((v & XDMA_STAT_INVALID_LEN))
			len += sprintf(buf + len, "INVLIAD_LEN ");
		if ((v & XDMA_STAT_IDLE_STOPPED))
			len += sprintf(buf + len, "IDLE_STOPPED ");
		buf[len - 1] = ',';
	}

	if (engine->dir == DMA_TO_DEVICE) {
		/* H2C only */
		if ((v & XDMA_STAT_H2C_R_ERR_MASK)) {
			len += sprintf(buf + len, "R:");
			if ((v & XDMA_STAT_H2C_R_UNSUPP_REQ))
				len += sprintf(buf + len, "UNSUPP_REQ ");
			if ((v & XDMA_STAT_H2C_R_COMPL_ABORT))
				len += sprintf(buf + len, "COMPL_ABORT ");
			if ((v & XDMA_STAT_H2C_R_PARITY_ERR))
				len += sprintf(buf + len, "PARITY ");
			if ((v & XDMA_STAT_H2C_R_HEADER_EP))
				len += sprintf(buf + len, "HEADER_EP ");
			if ((v & XDMA_STAT_H2C_R_UNEXP_COMPL))
				len += sprintf(buf + len, "UNEXP_COMPL ");
			buf[len - 1] = ',';
		}

		if ((v & XDMA_STAT_H2C_W_ERR_MASK)) {
			len += sprintf(buf + len, "W:");
			if ((v & XDMA_STAT_H2C_W_DECODE_ERR))
				len += sprintf(buf + len, "DECODE_ERR ");
			if ((v & XDMA_STAT_H2C_W_SLAVE_ERR))
				len += sprintf(buf + len, "SLAVE_ERR ");
			buf[len - 1] = ',';
		}

	} else {
		/* C2H only */
		if ((v & XDMA_STAT_C2H_R_ERR_MASK)) {
			len += sprintf(buf + len, "R:");
			if ((v & XDMA_STAT_C2H_R_DECODE_ERR))
				len += sprintf(buf + len, "DECODE_ERR ");
			if ((v & XDMA_STAT_C2H_R_SLAVE_ERR))
				len += sprintf(buf + len, "SLAVE_ERR ");
			buf[len - 1] = ',';
		}
	}

	/* common H2C & C2H */
	if ((v & XDMA_STAT_DESC_ERR_MASK)) {
		len += sprintf(buf + len, "DESC_ERR:");
		if ((v & XDMA_STAT_DESC_UNSUPP_REQ))
			len += sprintf(buf + len, "UNSUPP_REQ ");
		if ((v & XDMA_STAT_DESC_COMPL_ABORT))
			len += sprintf(buf + len, "COMPL_ABORT ");
		if ((v & XDMA_STAT_DESC_PARITY_ERR))
			len += sprintf(buf + len, "PARITY ");
		if ((v & XDMA_STAT_DESC_HEADER_EP))
			len += sprintf(buf + len, "HEADER_EP ");
		if ((v & XDMA_STAT_DESC_UNEXP_COMPL))
			len += sprintf(buf + len, "UNEXP_COMPL ");
		buf[len - 1] = ',';
	}

	buf[len - 1] = '\0';
	pr_info("%s\n", buffer);
}

/**
 * engine_status_read() - read status of SG DMA engine (optionally reset)
 *
 * Stores status in engine->status.
 *
 * @return error value on failure, 0 otherwise
 */
static int engine_status_read(struct xdma_engine *engine, bool clear, bool dump)
{
	int rv = 0;

	if (!engine) {
		pr_err("dma engine NULL\n");
		return -EINVAL;
	}

	if (dump) {
		rv = engine_reg_dump(engine);
		if (rv < 0) {
			pr_err("Failed to dump register\n");
			return rv;
		}
	}

	/* read status register */
	if (clear)
		engine->status = read_register(&engine->regs->status_rc);
	else
		engine->status = read_register(&engine->regs->status);

	if (dump)
		engine_status_dump(engine);

	return rv;
}

static int xdma_engine_stop(struct xdma_engine *engine)
{
	u32 w;

	if (!engine) {
		pr_err("dma engine NULL\n");
		return -EINVAL;
	}
	dbg_tfr("%s(engine=%p)\n", __func__, engine);

	if (enable_st_c2h_credit && engine->streaming &&
	    engine->dir == DMA_FROM_DEVICE)
		write_register(0, &engine->sgdma_regs->credits, 0);

	w = 0;
	w |= (u32)XDMA_CTRL_IE_DESC_ALIGN_MISMATCH;
	w |= (u32)XDMA_CTRL_IE_MAGIC_STOPPED;
	w |= (u32)XDMA_CTRL_IE_READ_ERROR;
	w |= (u32)XDMA_CTRL_IE_DESC_ERROR;

	/*if (poll_mode) {
		w |= (u32)XDMA_CTRL_POLL_MODE_WB;
	} else*/ {
		w |= (u32)XDMA_CTRL_IE_DESC_STOPPED;
		w |= (u32)XDMA_CTRL_IE_DESC_COMPLETED;
	}

	dbg_tfr("Stopping SG DMA %s engine; writing 0x%08x to 0x%p.\n",
		engine->name, w, (u32 *)&engine->regs->control);
	write_register(w, &engine->regs->control,
			(unsigned long)(&engine->regs->control) -
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

	/*
	 * disable all interrupts that fired; these are re-enabled individually
	 * after the causing module has been fully serviced.
	 */
	if (ch_irq)
		channel_interrupts_disable(xdev, ch_irq);

	/* read user interrupts - this read also flushes the above write */
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
		int max = xdev->h2c_channel_max;

		/* iterate over H2C (PCIe read) */
		for (channel = 0; channel < max && mask; channel++) {
			struct xdma_engine *engine = &xdev->engine_h2c[channel];

			/* engine present and its interrupt fired? */
			if ((engine->irq_bitmask & mask) &&
			    (engine->magic == MAGIC_ENGINE)) {
				mask &= ~engine->irq_bitmask;
				dbg_tfr("schedule_work, %s.\n", engine->name);
				//schedule_work(&engine->work);
			}
		}
	}

	mask = ch_irq & xdev->mask_irq_c2h;
	if (mask) {
		int channel = 0;
		int max = xdev->c2h_channel_max;

		/* iterate over C2H (PCIe write) */
		for (channel = 0; channel < max && mask; channel++) {
			struct xdma_engine *engine = &xdev->engine_c2h[channel];

			/* engine present and its interrupt fired? */
			if ((engine->irq_bitmask & mask) &&
			    (engine->magic == MAGIC_ENGINE)) {
				mask &= ~engine->irq_bitmask;
				dbg_tfr("schedule_work, %s.\n", engine->name);
				//schedule_work(&engine->work);
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
	struct interrupt_regs *irq_regs;

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

	irq_regs = (struct interrupt_regs *)(xdev->bar[xdev->config_bar_idx] +
					     XDMA_OFS_INT_CTRL);

	/* Disable the interrupt for this engine */
	write_register(
		engine->interrupt_enable_mask_value,
		&engine->regs->interrupt_enable_mask_w1c,
		(unsigned long)(&engine->regs->interrupt_enable_mask_w1c) -
			(unsigned long)(&engine->regs));
	/* Dummy read to flush the above write */
	read_register(&irq_regs->channel_int_pending);
	/* Schedule the bottom half */
	//schedule_work(&engine->work);

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

	if (!xdev->bar[idx]) {
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
	if (rv < 0) {
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
		int req_nvec = xdev->c2h_channel_max + xdev->h2c_channel_max +
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
	u32 i = xdev->c2h_channel_max + xdev->h2c_channel_max;
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
	u32 max = xdev->c2h_channel_max + xdev->h2c_channel_max;
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
	for (i = 0; i < xdev->h2c_channel_max; i++, j++, engine++) {
		if (!engine->msix_irq_line)
			break;
		dbg_sg("Release IRQ#%d for engine %p\n", engine->msix_irq_line,
		       engine);
		free_irq(engine->msix_irq_line, engine);
	}

	engine = xdev->engine_c2h;
	for (i = 0; i < xdev->c2h_channel_max; i++, j++, engine++) {
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

	j = xdev->h2c_channel_max;
	engine = xdev->engine_h2c;
	for (i = 0; i < xdev->h2c_channel_max; i++, engine++) {
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
	for (i = 0; i < xdev->c2h_channel_max; i++, j++, engine++) {
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

	j = xdev->h2c_channel_max + xdev->c2h_channel_max;

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
	int j = xdev->h2c_channel_max + xdev->c2h_channel_max;
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
static void dump_desc(struct xdma_desc *desc_virt)
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

static void transfer_dump(struct xdma_transfer *transfer)
{

}
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

	if (w) {
		engine->addr_align = align_bytes;
		engine->len_granularity = granularity_bytes;
		engine->addr_bits = address_bits;
	} else {
		/* Some default values if alignments are unspecified */
		engine->addr_align = 1;
		engine->len_granularity = 1;
		engine->addr_bits = 64;
	}
}

static void engine_free_resource(struct xdma_engine *engine)
{
	struct xdma_dev *xdev = engine->xdev;

	/* Release memory use for descriptor writebacks */
	if (engine->poll_mode_addr_virt) {
		dbg_sg("Releasing memory for descriptor writeback\n");
		dma_free_coherent(&xdev->pdev->dev, sizeof(struct xdma_poll_wb),
				  engine->poll_mode_addr_virt,
				  engine->poll_mode_bus);
		dbg_sg("Released memory for descriptor writeback\n");
		engine->poll_mode_addr_virt = NULL;
	}

	if (engine->desc) {
		dbg_init("device %s, engine %s pre-alloc desc 0x%p,0x%llx.\n",
			 dev_name(&xdev->pdev->dev), engine->name, engine->desc,
			 engine->desc_bus);
		dma_free_coherent(&xdev->pdev->dev,
				  engine->desc_max * sizeof(struct xdma_desc),
				  engine->desc, engine->desc_bus);
		engine->desc = NULL;
	}

	if (engine->cyclic_result) {
		dma_free_coherent(
			&xdev->pdev->dev,
			engine->desc_max * sizeof(struct xdma_result),
			engine->cyclic_result, engine->cyclic_result_bus);
		engine->cyclic_result = NULL;
	}
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
	u32 reg_value;
	int rv = 0;

	write_register(XDMA_CTRL_NON_INCR_ADDR, &engine->regs->control_w1c,
		       (unsigned long)(&engine->regs->control_w1c) -
			       (unsigned long)(&engine->regs));

	engine_alignments(engine);

	/* Configure error interrupts by default */
	reg_value = XDMA_CTRL_IE_DESC_ALIGN_MISMATCH;
	reg_value |= XDMA_CTRL_IE_MAGIC_STOPPED;
	reg_value |= XDMA_CTRL_IE_MAGIC_STOPPED;
	reg_value |= XDMA_CTRL_IE_READ_ERROR;
	reg_value |= XDMA_CTRL_IE_DESC_ERROR;

	/* if using polled mode, configure writeback address */
	/*if (poll_mode) {
		rv = engine_writeback_setup(engine);
		if (rv) {
			dbg_init("%s descr writeback setup failed.\n",
				 engine->name);
			goto fail_wb;
		}
	} else*/ {
		/* enable the relevant completion interrupts */
		reg_value |= XDMA_CTRL_IE_DESC_STOPPED;
		reg_value |= XDMA_CTRL_IE_DESC_COMPLETED;
	}

	/* Apply engine configurations */
	write_register(reg_value, &engine->regs->interrupt_enable_mask,
		       (unsigned long)(&engine->regs->interrupt_enable_mask) -
			       (unsigned long)(&engine->regs));

	engine->interrupt_enable_mask_value = reg_value;

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

fail_wb:
	return rv;
}

static int engine_alloc_resource(struct xdma_engine *engine)
{
	struct xdma_dev *xdev = engine->xdev;

	engine->desc = dma_alloc_coherent(&xdev->pdev->dev,
					  engine->desc_max *
						  sizeof(struct xdma_desc),
					  &engine->desc_bus, GFP_KERNEL);
	if (!engine->desc) {
		pr_warn("dev %s, %s pre-alloc desc OOM.\n",
			dev_name(&xdev->pdev->dev), engine->name);
		goto err_out;
	}



	if (engine->streaming && engine->dir == DMA_FROM_DEVICE) {
		engine->cyclic_result = dma_alloc_coherent(
			&xdev->pdev->dev,
			engine->desc_max * sizeof(struct xdma_result),
			&engine->cyclic_result_bus, GFP_KERNEL);

		if (!engine->cyclic_result) {
			pr_warn("%s, %s pre-alloc result OOM.\n",
				dev_name(&xdev->pdev->dev), engine->name);
			goto err_out;
		}
	}

	return 0;

err_out:
	engine_free_resource(engine);
	return -ENOMEM;
}

static int engine_init(struct xdma_engine *engine, struct xdma_dev *xdev,
		       int offset, enum dma_data_direction dir, int channel)
{
	int rv;
	u32 val;

	dbg_init("channel %d, offset 0x%x, dir %d.\n", channel, offset, dir);

	/* set magic */
	engine->magic = MAGIC_ENGINE;

	engine->channel = channel;

	/* engine interrupt request bit */
	engine->irq_bitmask = (1 << XDMA_ENG_IRQ_NUM) - 1;
	engine->irq_bitmask <<= (xdev->engines_num * XDMA_ENG_IRQ_NUM);
	engine->bypass_offset = xdev->engines_num * BYPASS_MODE_SPACING;

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
	engine->dir = dir;
	snprintf(engine->name, sizeof(engine->name), "%d-%s%d-%s", xdev->idx,
		(dir == DMA_TO_DEVICE) ? "H2C" : "C2H", channel,
		engine->streaming ? "ST" : "MM");

	if (enable_st_c2h_credit && engine->streaming &&
	    engine->dir == DMA_FROM_DEVICE)
	    	engine->desc_max = XDMA_ENGINE_CREDIT_XFER_MAX_DESC;
	else
	    	engine->desc_max = XDMA_ENGINE_XFER_MAX_DESC;

	dbg_init("engine %p name %s irq_bitmask=0x%08x\n", engine, engine->name,
		 (int)engine->irq_bitmask);

	/* initialize the deferred work for transfer completion */


	if (dir == DMA_TO_DEVICE)
		xdev->mask_irq_h2c |= engine->irq_bitmask;
	else
		xdev->mask_irq_c2h |= engine->irq_bitmask;
	xdev->engines_num++;

	rv = engine_alloc_resource(engine);
	if (rv)
		return rv;

	rv = engine_init_regs(engine);
	if (rv)
		return rv;



	return 0;
}




#ifdef __LIBXDMA_DEBUG__
static void sgt_dump(struct sg_table *sgt)
{
	int i;
	struct scatterlist *sg = sgt->sgl;

	pr_info("sgt 0x%p, sgl 0x%p, nents %u/%u.\n", sgt, sgt->sgl, sgt->nents,
		sgt->orig_nents);

	for (i = 0; i < sgt->orig_nents; i++, sg = sg_next(sg))
		pr_info("%d, 0x%p, pg 0x%p,%u+%u, dma 0x%llx,%u.\n", i, sg,
			sg_page(sg), sg->offset, sg->length, sg_dma_address(sg),
			sg_dma_len(sg));
}


#endif


ssize_t xdma_xfer_aperture(struct xdma_engine *engine, bool write, u64 ep_addr,
			unsigned int aperture, struct sg_table *sgt,
			bool dma_mapped, int timeout_ms)
{

	struct xdma_dev *xdev;
	struct xdma_request_cb *req = NULL;
	struct scatterlist *sg;
	enum dma_data_direction dir = write ? DMA_TO_DEVICE : DMA_FROM_DEVICE;
	//unsigned int maxlen = min_t(unsigned int, aperture, desc_blen_max);
	unsigned int sg_max;
	unsigned int tlen = 0;
	u64 ep_addr_max = ep_addr + aperture - 1;
	ssize_t done = 0;
	int i, rv = 0;
#if 0
	if (!engine) {
		pr_err("dma engine NULL\n");
		return -EINVAL;
	}

	if (engine->magic != MAGIC_ENGINE) {
		pr_err("%s has invalid magic number %lx\n", engine->name,
		       engine->magic);
		return -EINVAL;
	}

	xdev = engine->xdev;
	if (xdma_device_flag_check(xdev, XDEV_FLAG_OFFLINE)) {
		pr_info("xdev 0x%p, offline.\n", xdev);
		return -EBUSY;
	}

	/* check the direction */
	if (engine->dir != dir) {
		pr_info("0x%p, %s, W %d, 0x%x/0x%x mismatch.\n", engine,
			engine->name, write, engine->dir, dir);
		return -EINVAL;
	}

	if (engine->streaming) {
		pr_info("%s aperture not supported in ST.\n", engine->name);
		return -EINVAL;
	}

	if (!dma_mapped) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 16, 0)
		sgt->nents = pci_map_sg(xdev->pdev, sgt->sgl, sgt->orig_nents,
					dir);
#else
		sgt->nents = dma_map_sg(&xdev->pdev->dev, sgt->sgl,
					sgt->orig_nents, dir);
#endif
		if (!sgt->nents) {
			pr_info("map sgl failed, sgt 0x%p.\n", sgt);
			return -EIO;
		}
	} else if (!sgt->nents) {
		pr_err("sg table has invalid number of entries 0x%p.\n", sgt);
		return -EIO;
	}
	sg_max = sgt->nents;

	req = kzalloc(sizeof(struct xdma_request_cb), GFP_KERNEL);
	if (!req) {
		rv = -ENOMEM;
		goto unmap_sgl;
	}
	memset(req, 0, sizeof(struct xdma_request_cb));
	req->sgt = sgt;
	req->ep_addr = ep_addr;
	req->aperture = aperture;
	req->sg = sgt->sgl;

	for (i = 0, sg = sgt->sgl; i < sgt->nents; i++, sg = sg_next(sg))
		tlen += sg_dma_len(sg);
	req->total_len = tlen;

	dbg_tfr("%s, aperture: sg cnt %u.\n", engine->name, sgt->nents);

	mutex_lock(&engine->desc_lock);

	while (req->offset < req->total_len) {
		unsigned long flags;
		struct xdma_transfer *xfer = &req->tfer[0];
		unsigned int sg_offset = req->sg_offset;
		unsigned int desc_idx, desc_max, desc_cnt = 0;
		struct xdma_desc *desc_virt;
		dma_addr_t desc_bus;

		/* initialize transfer */
		memset(xfer, 0, sizeof(struct xdma_transfer));
#if HAS_SWAKE_UP
		init_swait_queue_head(&xfer->wq);
#else
		init_waitqueue_head(&xfer->wq);
#endif
		xfer->dir = engine->dir;
		if (!dma_mapped)
			xfer->flags = XFER_FLAG_NEED_UNMAP;

		spin_lock_irqsave(&engine->lock, flags);

		desc_idx = engine->desc_idx;
		desc_max = engine->desc_max;

		xfer->desc_virt = desc_virt = engine->desc + desc_idx;
		xfer->res_virt = engine->cyclic_result + desc_idx;
		xfer->desc_bus = desc_bus = engine->desc_bus +
					(sizeof(struct xdma_desc) * desc_idx);
		xfer->res_bus = engine->cyclic_result_bus +
					(sizeof(struct xdma_result) * desc_idx);
		xfer->desc_index = desc_idx;

		/* build transfer */
		sg = req->sg;
		ep_addr = req->ep_addr + (req->offset & (aperture - 1));
		i = req->sg_idx;
		
		for (sg = req->sg; i < sg_max && desc_idx < desc_max;
			i++, sg = sg_next(sg)) {
			dma_addr_t addr = sg_dma_address(sg);

			tlen = sg_dma_len(sg);
			if (sg_offset) {
				tlen -= sg_offset;
				addr += sg_offset;
			}

			while (tlen) {
				unsigned int len = min_t(unsigned int, tlen,
								 maxlen);

				if (ep_addr > ep_addr_max)
					ep_addr = req->ep_addr;

				if ((ep_addr + len) > ep_addr_max)
					len = ep_addr_max - ep_addr + 1;

				xdma_desc_set(engine->desc + desc_idx, addr,
						ep_addr, len, dir);

	                	dbg_desc("sg %d -> desc %u: ep 0x%llx, 0x%llx + %u \n",
					i, desc_idx, ep_addr, addr, len);

				sg_offset += len;
				req->offset += len;
				xfer->len += len;
				ep_addr += len;
				addr += len;
				tlen -= len;
				
				desc_idx++;
				desc_cnt++;
				if (desc_idx == desc_max)
					break;
			}

			if (!tlen)
				sg_offset = 0;
			else
				break;
		}
		
		req->sg_offset = sg_offset;
		req->sg_idx = i;

		xfer->desc_adjacent = desc_cnt;
		xfer->desc_num = desc_cnt;
		engine->desc_used += desc_cnt;

		/* create the desc linked list */
		for (i = 0; i < (desc_cnt - 1); i++, desc_virt++) {
			desc_bus += sizeof(struct xdma_desc);
			/* singly-linked list uses bus addresses */
			desc_virt->next_lo = cpu_to_le32(PCI_DMA_L(desc_bus));
                	desc_virt->next_hi = cpu_to_le32(PCI_DMA_H(desc_bus));
                	desc_virt->control = cpu_to_le32(DESC_MAGIC);
		}
		/* terminate the last descriptor next pointer */
		desc_virt->next_lo = cpu_to_le32(0);
		desc_virt->next_hi = cpu_to_le32(0);
		desc_virt->control = cpu_to_le32(DESC_MAGIC |
					XDMA_DESC_STOPPED | XDMA_DESC_EOP |
					XDMA_DESC_COMPLETED);

		xfer->desc_cmpl_th = desc_cnt;

		/* fill in adjacent numbers */
		for (i = 0; i < desc_cnt; i++) {
			u32 next_adj = xdma_get_next_adj(desc_cnt - i - 1,
					(xfer->desc_virt + i)->next_lo);

	                dbg_desc("set next adj at idx %d to %u\n", i, next_adj);
        	        xdma_desc_adjacent(xfer->desc_virt + i, next_adj);
        	}

		engine->desc_idx = (engine->desc_idx + desc_cnt) % desc_max;
		spin_unlock_irqrestore(&engine->lock, flags);

		/* last transfer for the given request? */
		if (req->offset == req->total_len) {
			xfer->last_in_request = 1;
			xfer->sgt = sgt;
		}

		dbg_tfr("xfer %u,%u/%u, ep 0x%llx/0x%x, done %ld, sg %u/%u, desc %u.\n",
			xfer->len, req->offset, req->total_len, req->ep_addr,
			req->aperture, done, req->sg_idx, sg_max, desc_cnt);

		rv = transfer_queue(engine, xfer);
		if (rv < 0) {
			mutex_unlock(&engine->desc_lock);
			pr_info("unable to submit %s, %d.\n", engine->name, rv);
			goto unmap_sgl;
		}

		if (engine->cmplthp)
			xdma_kthread_wakeup(engine->cmplthp);

		if (timeout_ms > 0)
			xlx_wait_event_interruptible_timeout(xfer->wq,
				(xfer->state != TRANSFER_STATE_SUBMITTED),
				msecs_to_jiffies(timeout_ms));
		else
			xlx_wait_event_interruptible(xfer->wq,
				(xfer->state != TRANSFER_STATE_SUBMITTED));

		spin_lock_irqsave(&engine->lock, flags);

		switch (xfer->state) {
		case TRANSFER_STATE_COMPLETED:
			spin_unlock_irqrestore(&engine->lock, flags);

			rv = 0;
			dbg_tfr("transfer %p, %u, ep 0x%llx compl, +%lu.\n",
				xfer, xfer->len, req->ep_addr - xfer->len,
				done);

			done += xfer->len;

			break;
		case TRANSFER_STATE_FAILED:
			pr_info("xfer 0x%p,%u, failed, ep 0x%llx.\n", xfer,
				xfer->len, req->ep_addr - xfer->len);
			spin_unlock_irqrestore(&engine->lock, flags);

#ifdef __LIBXDMA_DEBUG__
			transfer_dump(xfer);
			sgt_dump(sgt);
#endif
			rv = -EIO;
			break;
		default:
			/* transfer can still be in-flight */
			pr_info("xfer 0x%p,%u, s 0x%x timed out, ep 0x%llx.\n",
				xfer, xfer->len, xfer->state, req->ep_addr);
			rv = engine_status_read(engine, 0, 1);
			if (rv < 0) {
				pr_err("Failed to read engine status\n");
			} else if (rv == 0) {
				//engine_status_dump(engine);
				rv = transfer_abort(engine, xfer);
				if (rv < 0) {
					pr_err("Failed to stop engine\n");
				} else if (rv == 0) {
					rv = xdma_engine_stop(engine);
					if (rv < 0)
						pr_err("Failed to stop engine\n");
				}
			}
			spin_unlock_irqrestore(&engine->lock, flags);

#ifdef __LIBXDMA_DEBUG__
			transfer_dump(xfer);
			sgt_dump(sgt);
#endif
			rv = -ERESTARTSYS;
			break;
		}

		engine->desc_used -= xfer->desc_num;
		transfer_destroy(xdev, xfer);

		if (rv < 0) {
			mutex_unlock(&engine->desc_lock);
			goto unmap_sgl;
		}
	} /* while (sg) */
	mutex_unlock(&engine->desc_lock);

unmap_sgl:
	if (!dma_mapped && sgt->nents) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 16, 0)
		pci_unmap_sg(xdev->pdev, sgt->sgl, sgt->orig_nents, dir);
#else
		dma_unmap_sg(&xdev->pdev->dev, sgt->sgl, sgt->orig_nents, dir);
#endif
		sgt->nents = 0;
	}

	if (req)
		xdma_request_free(req);
#endif
	/* as long as some data is processed, return the count */
	return done ? done : rv;
}

ssize_t xdma_xfer_submit(struct xdma_engine *engine)
{

	int rv = 0, tfer_idx = 0, i;
	ssize_t done = 0;
	
	/* as long as some data is processed, return the count */
	return done ? done : rv;
}


int xdma_performance_submit(struct xdma_dev *xdev, struct xdma_engine *engine)
{
	u32 max_consistent_size = XDMA_PERF_NUM_DESC * 32 * 1024; /* 4MB */
	struct xdma_transfer *transfer;
	u64 ep_addr = 0;
	int num_desc_in_a_loop = XDMA_PERF_NUM_DESC;
	int size_in_desc = engine->xdma_perf->transfer_size;
	int size = size_in_desc * num_desc_in_a_loop;
	int i;
	int rv = -ENOMEM;
	unsigned char free_desc = 0;
#if 0
	if (size_in_desc > max_consistent_size) {
		pr_err("%s max consistent size %d is more than supported %d\n",
		       engine->name, size_in_desc, max_consistent_size);
		return -EINVAL;
	}

	if (size > max_consistent_size) {
		size = max_consistent_size;
		num_desc_in_a_loop = size / size_in_desc;
	}

	engine->perf_buf_virt = dma_alloc_coherent(&xdev->pdev->dev,
						size_in_desc,
						&engine->perf_buf_bus,
						GFP_KERNEL);
	if (!engine->perf_buf_virt) {
		pr_err("dev %s, %s DMA allocation OOM.\n",
		       dev_name(&xdev->pdev->dev), engine->name);
		return rv;
	}

	/* allocate transfer data structure */
	transfer = kzalloc(sizeof(struct xdma_transfer), GFP_KERNEL);
	if (!transfer) {
		pr_err("dev %s, %s transfer request OOM.\n",
		       dev_name(&xdev->pdev->dev), engine->name);
		goto err_engine_transfer;
	}
	/* 0 = write engine (to_dev=0) , 1 = read engine (to_dev=1) */
	transfer->dir = engine->dir;
	/* set number of descriptors */
	transfer->desc_num = num_desc_in_a_loop;

	/* allocate descriptor list */
	if (!engine->desc) {
		engine->desc = dma_alloc_coherent(
			&xdev->pdev->dev,
			num_desc_in_a_loop * sizeof(struct xdma_desc),
			&engine->desc_bus, GFP_KERNEL);
		if (!engine->desc) {
			pr_err("%s DMA memory allocation for descriptors failed\n",
			       engine->name);
			goto err_engine_desc;
		}
		dbg_init("device %s, engine %s pre-alloc desc 0x%p,0x%llx.\n",
			 dev_name(&xdev->pdev->dev), engine->name, engine->desc,
			 engine->desc_bus);
		free_desc = 1;
	}
	transfer->desc_virt = engine->desc;
	transfer->desc_bus = engine->desc_bus;

	rv = transfer_desc_init(transfer, transfer->desc_num);
	if (rv < 0) {
		pr_err("Failed to initialize descriptors\n");
		goto err_dma_desc;
	}

	dbg_sg("transfer->desc_bus = 0x%llx.\n", (u64)transfer->desc_bus);

	for (i = 0; i < transfer->desc_num; i++) {
		struct xdma_desc *desc = transfer->desc_virt + i;
		dma_addr_t rc_bus_addr = engine->perf_buf_bus;

		/* fill in descriptor entry with transfer details */
		xdma_desc_set(desc, rc_bus_addr, ep_addr, size_in_desc,
			      engine->dir);
	}

	/* stop engine and request interrupt on last descriptor */
	rv = xdma_desc_control_set(transfer->desc_virt, 0);
	if (rv < 0) {
		pr_err("Failed to set desc control\n");
		goto err_dma_desc;
	}
	/* create a linked loop */
	xdma_desc_link(transfer->desc_virt + transfer->desc_num - 1,
		       transfer->desc_virt, transfer->desc_bus);

	transfer->cyclic = 1;

	/* initialize wait queue */
#if HAS_SWAKE_UP
	init_swait_queue_head(&transfer->wq);
#else
	init_waitqueue_head(&transfer->wq);
#endif

	//printk("=== Descriptor print for PERF\n");
	//transfer_dump(transfer);

	dbg_perf("Queueing XDMA I/O %s request for performance measurement.\n",
		 engine->dir ? "write (to dev)" : "read (from dev)");
	rv = transfer_queue(engine, transfer);
	if (rv < 0) {
		pr_err("Failed to queue transfer\n");
		goto err_dma_desc;
	}
	return 0;

err_dma_desc:
	if (free_desc && engine->desc)
		dma_free_coherent(&xdev->pdev->dev,
				num_desc_in_a_loop * sizeof(struct xdma_desc),
				engine->desc, engine->desc_bus);
	engine->desc = NULL;
err_engine_desc:
	if (transfer)
		list_del(&transfer->entry);
	kfree(transfer);
	transfer = NULL;
err_engine_transfer:
	if (engine->perf_buf_virt)
		dma_free_coherent(&xdev->pdev->dev, size_in_desc,
				engine->perf_buf_virt, engine->perf_buf_bus);
	engine->perf_buf_virt = NULL;
#endif
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

	engine = xdev->engine_h2c;
	for (i = 0; i < XDMA_CHANNEL_NUM_MAX; i++, engine++) {
		spin_lock_init(&engine->lock);
		mutex_init(&engine->desc_lock);
		INIT_LIST_HEAD(&engine->transfer_list);
#if HAS_SWAKE_UP
		init_swait_queue_head(&engine->shutdown_wq);
		init_swait_queue_head(&engine->xdma_perf_wq);
#else
		init_waitqueue_head(&engine->shutdown_wq);
		init_waitqueue_head(&engine->xdma_perf_wq);
#endif
	}

	engine = xdev->engine_c2h;
	for (i = 0; i < XDMA_CHANNEL_NUM_MAX; i++, engine++) {
		spin_lock_init(&engine->lock);
		mutex_init(&engine->desc_lock);
		INIT_LIST_HEAD(&engine->transfer_list);
#if HAS_SWAKE_UP
		init_swait_queue_head(&engine->shutdown_wq);
		init_swait_queue_head(&engine->xdma_perf_wq);
#else
		init_waitqueue_head(&engine->shutdown_wq);
		init_waitqueue_head(&engine->xdma_perf_wq);
#endif
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
	for (i = 0; i < xdev->h2c_channel_max; i++) {
		engine = &xdev->engine_h2c[i];
		if (engine->magic == MAGIC_ENGINE) {
			dbg_sg("Remove %s, %d", engine->name, i);
			rv = engine_destroy(xdev, engine);
			if (rv < 0)
				pr_err("Failed to destroy H2C engine %d\n", i);
			dbg_sg("%s, %d removed", engine->name, i);
		}
	}

	for (i = 0; i < xdev->c2h_channel_max; i++) {
		engine = &xdev->engine_c2h[i];
		if (engine->magic == MAGIC_ENGINE) {
			dbg_sg("Remove %s, %d", engine->name, i);
			rv = engine_destroy(xdev, engine);
			if (rv < 0)
				pr_err("Failed to destroy C2H engine %d\n", i);
			dbg_sg("%s, %d removed", engine->name, i);
		}
	}
}

static int probe_for_engine(struct xdma_dev *xdev, enum dma_data_direction dir,
			    int channel)
{
	struct engine_regs *regs;
	int offset = channel * CHANNEL_SPACING;
	u32 engine_id;
	u32 engine_id_expected;
	u32 channel_id;
	struct xdma_engine *engine;
	int rv;

	/* register offset for the engine */
	/* read channels at 0x0000, write channels at 0x1000,
	 * channels at 0x100 interval
	 */
	if (dir == DMA_TO_DEVICE) {
		engine_id_expected = XDMA_ID_H2C;
		engine = &xdev->engine_h2c[channel];
	} else {
		offset += H2C_CHANNEL_OFFSET;
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
		return -EINVAL;
	}

	dbg_init("found AXI %s %d engine, reg. off 0x%x, id 0x%x,0x%x.\n",
		 dir == DMA_TO_DEVICE ? "H2C" : "C2H", channel, offset,
		 engine_id, channel_id);

	/* allocate and initialize engine */
	rv = engine_init(engine, xdev, offset, dir, channel);
	if (rv != 0) {
		pr_info("failed to create AXI %s %d engine.\n",
			dir == DMA_TO_DEVICE ? "H2C" : "C2H", channel);
		return rv;
	}

	return 0;
}

static int probe_engines(struct xdma_dev *xdev)
{
	int i;
	int rv = 0;

	if (!xdev) {
		pr_err("Invalid xdev\n");
		return -EINVAL;
	}

	/* iterate over channels */
	for (i = 0; i < xdev->h2c_channel_max; i++) {
		rv = probe_for_engine(xdev, DMA_TO_DEVICE, i);
		if (rv)
			break;
	}
	xdev->h2c_channel_max = i;

	for (i = 0; i < xdev->c2h_channel_max; i++) {
		rv = probe_for_engine(xdev, DMA_FROM_DEVICE, i);
		if (rv)
			break;
	}
	xdev->c2h_channel_max = i;

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
		       int *h2c_channel_max, int *c2h_channel_max)
{
	struct xdma_dev *xdev = NULL;
	int rv = 0;

	pr_info("%s device %s, 0x%p.\n", mname, dev_name(&pdev->dev), pdev);

	/* allocate zeroed device book keeping structure */
	xdev = alloc_dev_instance(pdev);
	if (!xdev)
		goto err_alloc_dev_instance;
	xdev->mod_name = mname;
	xdev->user_max = *user_max;
	xdev->h2c_channel_max = *h2c_channel_max;
	xdev->c2h_channel_max = *c2h_channel_max;

	xdma_device_flag_set(xdev, XDEV_FLAG_OFFLINE);

	if (xdev->user_max == 0 || xdev->user_max > MAX_USER_IRQ)
		xdev->user_max = MAX_USER_IRQ;
	if (xdev->h2c_channel_max == 0 ||
	    xdev->h2c_channel_max > XDMA_CHANNEL_NUM_MAX)
		xdev->h2c_channel_max = XDMA_CHANNEL_NUM_MAX;
	if (xdev->c2h_channel_max == 0 ||
	    xdev->c2h_channel_max > XDMA_CHANNEL_NUM_MAX)
		xdev->c2h_channel_max = XDMA_CHANNEL_NUM_MAX;

	rv = xdev_list_add(xdev);
	if (rv < 0)
		goto err_xdev_list_add;

	rv = pci_enable_device(pdev);
	if (rv) {
		dbg_init("pci_enable_device() failed, %d.\n", rv);
		goto err_pci_enable_device;
	}

	/* keep INTx enabled */
	pci_check_intr_pend(pdev);

	/* enable relaxed ordering */
	pci_enable_capability(pdev, PCI_EXP_DEVCTL_RELAX_EN);

	/* enable extended tag */
	pci_enable_capability(pdev, PCI_EXP_DEVCTL_EXT_TAG);

	/* force MRRS to be 512 */
	rv = pcie_set_readrq(pdev, 512);
	if (rv)
		pr_info("device %s, error set PCI_EXP_DEVCTL_READRQ: %d.\n",
			dev_name(&pdev->dev), rv);

	/* enable bus master capability */
	pci_set_master(pdev);

	rv = request_regions(xdev, pdev);
	if (rv)
		goto err_request_regions;

	rv = map_bars(xdev, pdev);
	if (rv)
		goto err_map_bars;

	rv = set_dma_mask(pdev);
	if (rv)
		goto err_set_dma_mask;

	check_nonzero_interrupt_status(xdev);
	/* explicitely zero all interrupt enable masks */
	channel_interrupts_disable(xdev, ~0);
	user_interrupts_disable(xdev, ~0);
	read_interrupts(xdev);

	rv = probe_engines(xdev);
	if (rv)
		goto err_probe_engines;

	rv = enable_msi_msix(xdev, pdev);
	if (rv < 0)
		goto err_enable_msi_msix;

	rv = irq_setup(xdev, pdev);
	if (rv < 0)
		goto err_irq_setup;

	/*if (!poll_mode)*/
		channel_interrupts_enable(xdev, ~0);

	/* Flush writes */
	read_interrupts(xdev);

	*user_max = xdev->user_max;
	*h2c_channel_max = xdev->h2c_channel_max;
	*c2h_channel_max = xdev->c2h_channel_max;

	xdma_device_flag_clear(xdev, XDEV_FLAG_OFFLINE);
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
	xdma_device_flag_set(xdev, XDEV_FLAG_OFFLINE);

	/* wait for all engines to be idle */
	for (i = 0; i < xdev->h2c_channel_max; i++) {
		unsigned long flags;

		engine = &xdev->engine_h2c[i];

		if (engine->magic == MAGIC_ENGINE) {
			spin_lock_irqsave(&engine->lock, flags);
			engine->shutdown |= ENGINE_SHUTDOWN_REQUEST;

			rv = xdma_engine_stop(engine);
			if (rv < 0)
				pr_err("Failed to stop engine\n");
			spin_unlock_irqrestore(&engine->lock, flags);
		}
	}

	for (i = 0; i < xdev->c2h_channel_max; i++) {
		unsigned long flags;

		engine = &xdev->engine_c2h[i];
		if (engine->magic == MAGIC_ENGINE) {
			spin_lock_irqsave(&engine->lock, flags);
			engine->shutdown |= ENGINE_SHUTDOWN_REQUEST;

			rv = xdma_engine_stop(engine);
			if (rv < 0)
				pr_err("Failed to stop engine\n");
			spin_unlock_irqrestore(&engine->lock, flags);
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

	for (i = 0; i < xdev->h2c_channel_max; i++) {
		engine = &xdev->engine_h2c[i];
		if (engine->magic == MAGIC_ENGINE) {
			engine_init_regs(engine);
			spin_lock_irqsave(&engine->lock, flags);
			engine->shutdown &= ~ENGINE_SHUTDOWN_REQUEST;
			spin_unlock_irqrestore(&engine->lock, flags);
		}
	}

	for (i = 0; i < xdev->c2h_channel_max; i++) {
		engine = &xdev->engine_c2h[i];
		if (engine->magic == MAGIC_ENGINE) {
			engine_init_regs(engine);
			spin_lock_irqsave(&engine->lock, flags);
			engine->shutdown &= ~ENGINE_SHUTDOWN_REQUEST;
			spin_unlock_irqrestore(&engine->lock, flags);
		}
	}

	/* re-write the interrupt table */
	/*if (!poll_mode) */{
		irq_setup(xdev, pdev);

		channel_interrupts_enable(xdev, ~0);
		user_interrupts_enable(xdev, xdev->mask_irq_user);
		read_interrupts(xdev);
	}

	xdma_device_flag_clear(xdev, XDEV_FLAG_OFFLINE);
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

int engine_addrmode_set(struct xdma_engine *engine, unsigned long arg)
{
	int rv;
	unsigned long dst;
	u32 w = XDMA_CTRL_NON_INCR_ADDR;

	dbg_perf("IOCTL_XDMA_ADDRMODE_SET\n");
	rv = get_user(dst, (int __user *)arg);

	if (rv == 0) {
		engine->non_incr_addr = !!dst;
		if (engine->non_incr_addr)
			write_register(
				w, &engine->regs->control_w1s,
				(unsigned long)(&engine->regs->control_w1s) -
					(unsigned long)(&engine->regs));
		else
			write_register(
				w, &engine->regs->control_w1c,
				(unsigned long)(&engine->regs->control_w1c) -
					(unsigned long)(&engine->regs));
	}
	engine_alignments(engine);

	return rv;
}
