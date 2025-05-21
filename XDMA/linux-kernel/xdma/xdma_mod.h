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

#ifndef __XDMA_MODULE_H__
#define __XDMA_MODULE_H__

#include <linux/types.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/poll.h>
#include <linux/pci.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/workqueue.h>
#include <linux/aio.h>
#include <linux/splice.h>
#include <linux/version.h>
#include <linux/uio.h>
#include <linux/spinlock_types.h>

#include "libxdma.h"


#define MAGIC_CHAR	0xCCCCCCCCU
#define MAGIC_BITSTREAM 0xBBBBBBBBU

extern unsigned int h2c_timeout_ms;
extern unsigned int c2h_timeout_ms;


enum cdev_type {
	CHAR_USER,
	CHAR_CTRL,
	CHAR_XVC,
	CHAR_EVENTS,
	CHAR_XDMA_H2C,
	CHAR_XDMA_C2H,
	CHAR_BYPASS_H2C,
	CHAR_BYPASS_C2H,
	CHAR_BYPASS,
};

struct xdma_cdev {
	unsigned int magic;		/* structure ID for sanity checks */
	struct xdma_pci_dev *xpdev;
	struct xdma_dev *xdev;
	dev_t cdevno;			/* character device major:minor */
	struct cdev cdev;		/* character device embedded struct */
	enum cdev_type type;
	int bar;			/* PCIe BAR for HW access, if needed, or xdma channel namber */
	unsigned long base;		/* bar access offset */
	struct xdma_engine *engine;	/* engine instance, if needed */
	struct xdma_user_irq *user_irq;	/* IRQ value, if needed */
	struct device *sys_device;	/* sysfs device */
	spinlock_t lock;
};

/* XDMA PCIe device specific book-keeping */
struct xdma_pci_dev {
	unsigned int magic;		/* structure ID for sanity checks */
	struct pci_dev *pdev;	/* pci device struct from probe() */
	struct xdma_dev *xdev;
	int major;		/* major number */
	int instance;		/* instance number */
	int user_max;
	int c2h_channel_num;
	int h2c_channel_num;

	unsigned int flags;
	/* character device structures */
	struct xdma_cdev ctrl_cdev;
	struct xdma_cdev sgdma_c2h_cdev[XDMA_CHANNEL_NUM_MAX];
	struct xdma_cdev sgdma_h2c_cdev[XDMA_CHANNEL_NUM_MAX];
	struct xdma_cdev events_cdev[16];

	struct xdma_cdev user_cdev;
	struct xdma_cdev bypass_c2h_cdev[XDMA_CHANNEL_NUM_MAX];
	struct xdma_cdev bypass_h2c_cdev[XDMA_CHANNEL_NUM_MAX];
	struct xdma_cdev bypass_cdev_base;

	struct xdma_cdev xvc_cdev;

	void *data;
};

#endif /* ifndef __XDMA_MODULE_H__ */
