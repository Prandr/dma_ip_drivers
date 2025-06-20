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

#define pr_fmt(fmt)     KBUILD_MODNAME ":%s: " fmt, __func__

#include "xdma_cdev.h"

/*
 * character device file operations for events
 */
static ssize_t char_events_read(struct file *filp, char __user *buf,
		size_t count, loff_t *pos)
{
	int rv;
	struct xdma_user_irq *user_irq;
	struct xdma_cdev *xcdev = (struct xdma_cdev *)filp->private_data;
	u32 events_user;
	unsigned long flags;

	rv = xcdev_check(__func__, xcdev, 0);
	if (rv < 0)
		return rv;
	user_irq = xcdev->user_irq;
	xdma_debug_assert_ptr(user_irq);
	
	if (count != 4)
		return -EPROTO;

	if (*pos & 3)
		return -EPROTO;

	/*
	 * sleep until any interrupt events have occurred,
	 * or a signal arrived
	 */
	rv = wait_event_interruptible(user_irq->events_wq,
			user_irq->events_irq != 0);
	if (rv)
		dbg_sg("wait_event_interruptible=%d\n", rv);

	/* wait_event_interruptible() was interrupted by a signal */
	if (rv == -ERESTARTSYS)
		return -ERESTARTSYS;

	/* atomically decide which events are passed to the user */
	spin_lock_irqsave(&user_irq->events_lock, flags);
	events_user = user_irq->events_irq;
	user_irq->events_irq = 0;
	spin_unlock_irqrestore(&user_irq->events_lock, flags);

	rv = copy_to_user(buf, &events_user, 4);
	if (rv)
		dbg_sg("Copy to user failed but continuing\n");

	return 4;
}

static unsigned int char_events_poll(struct file *filp, poll_table *wait)
{
	struct xdma_user_irq *user_irq;
	struct xdma_cdev *xcdev = (struct xdma_cdev *)filp->private_data;
	unsigned long flags;
	unsigned int mask = 0;
	int rv;

	rv = xcdev_check(__func__, xcdev, 0);
	if (rv < 0)
		return rv;
	user_irq = xcdev->user_irq;
	xdma_debug_assert_ptr(user_irq);

	poll_wait(filp, &user_irq->events_wq,  wait);

	spin_lock_irqsave(&user_irq->events_lock, flags);
	if (user_irq->events_irq)
		mask = POLLIN | POLLRDNORM;	/* readable */

	spin_unlock_irqrestore(&user_irq->events_lock, flags);

	return mask;
}

/*
 * character device file operations for the irq events
 */
static const struct file_operations events_fops = {
	.owner = THIS_MODULE,
	.open = char_open,
	.release = char_close,
	.read = char_events_read,
	.poll = char_events_poll,
};

void cdev_event_init(struct xdma_cdev *xcdev)
{
	xcdev->user_irq = &(xcdev->xdev->user_irq[xcdev->bar]);
	cdev_init(&xcdev->cdev, &events_fops);
}
