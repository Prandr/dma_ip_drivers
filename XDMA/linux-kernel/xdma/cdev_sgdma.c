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

#include <linux/types.h>
#include <asm/cacheflush.h>
#include <linux/slab.h>
#include <linux/aio.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/kthread.h>
#include <linux/version.h>
#include <linux/uio.h>
#include "libxdma_api.h"
#include "xdma_cdev.h"
#include "cdev_sgdma.h"


/* Module Parameters */
unsigned int h2c_timeout_ms = 10000;
module_param(h2c_timeout_ms, uint, 0644);
MODULE_PARM_DESC(h2c_timeout_ms, "H2C sgdma timeout in milliseconds, default is 10 seconds.");

unsigned int c2h_timeout_ms = 10000;
module_param(c2h_timeout_ms, uint, 0644);
MODULE_PARM_DESC(c2h_timeout_ms, "C2H sgdma timeout in milliseconds, default is 10 seconds.");

extern struct kmem_cache *cdev_cache;
static void char_sgdma_unmap_user_buf(struct xdma_io_cb *cb, bool write);



/*
 * character device file operations for SG DMA engine
 */


/* char_sgdma_read_write() -- Read from or write to the device
 *
 * @buf userspace buffer
 * @count number of bytes in the userspace buffer
 * @pos byte-address in device
 * @dir_to_device If !0, a write to the device is performed
 *
 * Iterate over the userspace buffer, taking at most 255 * PAGE_SIZE bytes for
 * each DMA transfer.
 *
 * For each transfer, get the user pages, build a sglist, map, build a
 * descriptor table. submit the transfer. wait for the interrupt handler
 * to wake us on completion.
 */

static int check_transfer_align(struct xdma_engine *engine,
	const char __user *buf, size_t count, loff_t pos, int sync)
{
	if (!engine) {
		pr_err("Invalid DMA engine\n");
		return -EINVAL;
	}

	/* AXI ST or AXI MM non-incremental addressing mode? */
	if (engine->non_incr_addr) {
		int buf_lsb = (int)((uintptr_t)buf) & (engine->addr_align - 1);
		size_t len_lsb = count & ((size_t)engine->len_granularity - 1);
		int pos_lsb = (int)pos & (engine->addr_align - 1);

		dbg_tfr("AXI ST or MM non-incremental\n");
		dbg_tfr("buf_lsb = %d, pos_lsb = %d, len_lsb = %ld\n", buf_lsb,
			pos_lsb, len_lsb);

		if (buf_lsb != 0) {
			dbg_tfr("FAIL: non-aligned buffer address %p\n", buf);
			return -EINVAL;
		}

		if ((pos_lsb != 0) && (sync)) {
			dbg_tfr("FAIL: non-aligned AXI MM FPGA addr 0x%llx\n",
				(unsigned long long)pos);
			return -EINVAL;
		}

		if (len_lsb != 0) {
			dbg_tfr("FAIL: len %d is not a multiple of %d\n",
				(int)count,
				(int)engine->len_granularity);
			return -EINVAL;
		}
		/* AXI MM incremental addressing mode */
	} else {
		int buf_lsb = (int)((uintptr_t)buf) & (engine->addr_align - 1);
		int pos_lsb = (int)pos & (engine->addr_align - 1);

		if (buf_lsb != pos_lsb) {
			dbg_tfr("FAIL: Misalignment error\n");
			dbg_tfr("host addr %p, FPGA addr 0x%llx\n", buf, pos);
			return -EINVAL;
		}
	}

	return 0;
}

static ssize_t char_sgdma_read_write(struct file *file, const char __user *buf,
		size_t count, loff_t *pos, bool write)
{
	int rv;
	ssize_t res = 0;
	struct xdma_cdev *xcdev = (struct xdma_cdev *)file->private_data;
	struct xdma_dev *xdev;
	struct xdma_engine *engine;

	res = xdma_xfer_submit(engine);

	return res;
}


static ssize_t char_sgdma_write(struct file *file, const char __user *buf,
		size_t count, loff_t *pos)
{
	return char_sgdma_read_write(file, buf, count, pos, 1);
}

static ssize_t char_sgdma_read(struct file *file, char __user *buf,
				size_t count, loff_t *pos)
{
	return char_sgdma_read_write(file, buf, count, pos, 0);
}





static int ioctl_do_perf_start(struct xdma_engine *engine, unsigned long arg)
{

	int rv;
	struct xdma_dev *xdev;
#if 0
	if (!engine) {
		pr_err("Invalid DMA engine\n");
		return -EINVAL;
	}

	xdev = engine->xdev;
	if (!xdev) {
		pr_err("Invalid xdev\n");
		return -EINVAL;
	}

	/* performance measurement already running on this engine? */
	if (engine->xdma_perf) {
		dbg_perf("IOCTL_XDMA_PERF_START failed!\n");
		dbg_perf("Perf measurement already seems to be running!\n");
		return -EBUSY;
	}
	engine->xdma_perf = kzalloc(sizeof(struct xdma_performance_ioctl),
		GFP_KERNEL);

	if (!engine->xdma_perf)
		return -ENOMEM;

	rv = copy_from_user(engine->xdma_perf,
		(struct xdma_performance_ioctl __user *)arg,
		sizeof(struct xdma_performance_ioctl));

	if (rv < 0) {
		dbg_perf("Failed to copy from user space 0x%lx\n", arg);
		return -EINVAL;
	}
	if (engine->xdma_perf->version != IOCTL_XDMA_PERF_V1) {
		dbg_perf("Unsupported IOCTL version %d\n",
			engine->xdma_perf->version);
		return -EINVAL;
	}

	enable_perf(engine);
	dbg_perf("transfer_size = %d\n", engine->xdma_perf->transfer_size);
	/* initialize wait queue */
#if HAS_SWAKE_UP
	init_swait_queue_head(&engine->xdma_perf_wq);
#else
	init_waitqueue_head(&engine->xdma_perf_wq);
#endif
	rv = xdma_performance_submit(xdev, engine);
#endif
	if (rv < 0)
		pr_err("Failed to submit dma performance\n");
	return rv;
}

static int ioctl_do_perf_stop(struct xdma_engine *engine, unsigned long arg)
{
	struct xdma_transfer *transfer = NULL;
	int rv;
#if 0
	if (!engine) {
		pr_err("Invalid DMA engine\n");
		return -EINVAL;
	}

	dbg_perf("IOCTL_XDMA_PERF_STOP\n");

	/* no performance measurement running on this engine? */
	if (!engine->xdma_perf) {
		dbg_perf("No measurement in progress\n");
		return -EINVAL;
	}

	/* stop measurement */
	transfer = engine_cyclic_stop(engine);
	if (!transfer) {
		pr_err("Failed to stop cyclic transfer\n");
		return -EINVAL;
	}
	dbg_perf("Waiting for measurement to stop\n");

	get_perf_stats(engine);

	rv = copy_to_user((void __user *)arg, engine->xdma_perf,
			sizeof(struct xdma_performance_ioctl));
	if (rv) {
		dbg_perf("Error copying result to user\n");
		return rv;
	}

	kfree(transfer);

	kfree(engine->xdma_perf);
	engine->xdma_perf = NULL;
#endif
	return 0;
}

static int ioctl_do_perf_get(struct xdma_engine *engine, unsigned long arg)
{
	int rc;
#if 0
	if (!engine) {
		pr_err("Invalid DMA engine\n");
		return -EINVAL;
	}

	dbg_perf("IOCTL_XDMA_PERF_GET\n");

	if (engine->xdma_perf) {
		get_perf_stats(engine);

		rc = copy_to_user((void __user *)arg, engine->xdma_perf,
			sizeof(struct xdma_performance_ioctl));
		if (rc) {
			dbg_perf("Error copying result to user\n");
			return rc;
		}
	} else {
		dbg_perf("engine->xdma_perf == NULL?\n");
		return -EPROTO;
	}
#endif
	return 0;
}

static int ioctl_do_addrmode_set(struct xdma_engine *engine, unsigned long arg)
{
	return engine_addrmode_set(engine, arg);
}

static int ioctl_do_addrmode_get(struct xdma_engine *engine, unsigned long arg)
{
	int rv;
	unsigned long src;

	if (!engine) {
		pr_err("Invalid DMA engine\n");
		return -EINVAL;
	}
	src = !!engine->non_incr_addr;

	dbg_perf("IOCTL_XDMA_ADDRMODE_GET\n");
	rv = put_user(src, (int __user *)arg);

	return rv;
}

static int ioctl_do_align_get(struct xdma_engine *engine, unsigned long arg)
{
	if (!engine) {
		pr_err("Invalid DMA engine\n");
		return -EINVAL;
	}

	dbg_perf("IOCTL_XDMA_ALIGN_GET\n");
	return put_user(engine->addr_align, (int __user *)arg);
}


static int ioctl_do_aperture_dma(struct xdma_engine *engine, unsigned long arg,
				bool write)
{
	struct xdma_aperture_ioctl io;
#if 0
	struct xdma_io_cb cb;
	ssize_t res;
	int rv;

	rv = copy_from_user(&io, (struct xdma_aperture_ioctl __user *)arg,
				sizeof(struct xdma_aperture_ioctl));
	if (rv < 0) {
		dbg_tfr("%s failed to copy from user space 0x%lx\n",
			engine->name, arg);
		return -EINVAL;
	}

	dbg_tfr("%s, W %d, buf 0x%lx,%lu, ep %llu, aperture %u.\n",
		engine->name, write, io.buffer, io.len, io.ep_addr,
		io.aperture);

	if ((write && engine->dir != DMA_TO_DEVICE) ||
	    (!write && engine->dir != DMA_FROM_DEVICE)) {
		pr_err("r/w mismatch. W %d, dir %d.\n", write, engine->dir);
		return -EINVAL;
	}

	rv = check_transfer_align(engine, (char *)io.buffer, io.len,
				io.ep_addr, 1);
	if (rv) {
		pr_info("Invalid transfer alignment detected\n");
		return rv;
	}

	memset(&cb, 0, sizeof(struct xdma_io_cb));
	cb.buf = (char __user *)io.buffer;
	cb.len = io.len;
	cb.ep_addr = io.ep_addr;
	cb.write = write;
	rv = char_sgdma_map_user_buf_to_sgl(&cb, write);
	if (rv < 0)
		return rv;

	io.error = 0;
	res = xdma_xfer_aperture(engine, write, io.ep_addr, io.aperture,
				&cb.sgt, 0, write ? h2c_timeout_ms : c2h_timeout_ms);

	char_sgdma_unmap_user_buf(&cb, write);
	if (res < 0)
		io.error = res;
	else
		io.done = res;

	rv = copy_to_user((struct xdma_aperture_ioctl __user *)arg, &io,
				sizeof(struct xdma_aperture_ioctl));
	if (rv < 0) {
		dbg_tfr("%s failed to copy to user space 0x%lx, %ld\n",
			engine->name, arg, res);
		return -EINVAL;
	}
#endif
	return io.error;
}
	
static long char_sgdma_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	struct xdma_cdev *xcdev = (struct xdma_cdev *)file->private_data;
	struct xdma_dev *xdev;
	struct xdma_engine *engine;

	int rv = 0;

	rv = xcdev_check(__func__, xcdev, 1);
	if (rv < 0)
		return rv;

	xdev = xcdev->xdev;
	engine = xcdev->engine;

	switch (cmd) {
	case IOCTL_XDMA_PERF_START:
		rv = ioctl_do_perf_start(engine, arg);
		break;
	case IOCTL_XDMA_PERF_STOP:
		rv = ioctl_do_perf_stop(engine, arg);
		break;
	case IOCTL_XDMA_PERF_GET:
		rv = ioctl_do_perf_get(engine, arg);
		break;
	case IOCTL_XDMA_ADDRMODE_SET:
		rv = ioctl_do_addrmode_set(engine, arg);
		break;
	case IOCTL_XDMA_ADDRMODE_GET:
		rv = ioctl_do_addrmode_get(engine, arg);
		break;
	case IOCTL_XDMA_ALIGN_GET:
		rv = ioctl_do_align_get(engine, arg);
		break;
	case IOCTL_XDMA_APERTURE_R:
		rv = ioctl_do_aperture_dma(engine, arg, 0);
		break;
	case IOCTL_XDMA_APERTURE_W:
		rv = ioctl_do_aperture_dma(engine, arg, 1);
		break;
	default:
		dbg_perf("Unsupported operation\n");
		rv = -EINVAL;
		break;
	}

	return rv;
}

static int char_sgdma_open(struct inode *inode, struct file *file)
{
	struct xdma_cdev *xcdev;
	struct xdma_engine *engine;

	char_open(inode, file);

	xcdev = (struct xdma_cdev *)file->private_data;
	engine = xcdev->engine;

	if (engine->streaming && engine->dir == DMA_FROM_DEVICE) {
		if (engine->device_open == 1)
			return -EBUSY;
		engine->device_open = 1;

		engine->eop_flush = (file->f_flags & O_TRUNC) ? 1 : 0;
	}

	return 0;
}

static int char_sgdma_close(struct inode *inode, struct file *file)
{
	struct xdma_cdev *xcdev = (struct xdma_cdev *)file->private_data;
	struct xdma_engine *engine;
	int rv;

	rv = xcdev_check(__func__, xcdev, 1);
	if (rv < 0)
		return rv;

	engine = xcdev->engine;

	if (engine->streaming && engine->dir == DMA_FROM_DEVICE)
		engine->device_open = 0;

	return 0;
}
static const struct file_operations sgdma_fops = {
	.owner = THIS_MODULE,
	.open = char_sgdma_open,
	.release = char_sgdma_close,
	.write = char_sgdma_write,
	.read = char_sgdma_read,
	.unlocked_ioctl = char_sgdma_ioctl,
	.llseek = char_llseek,
};

void cdev_sgdma_init(struct xdma_cdev *xcdev)
{
	cdev_init(&xcdev->cdev, &sgdma_fops);
}
