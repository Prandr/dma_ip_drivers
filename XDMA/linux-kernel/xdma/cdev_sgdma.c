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
#include <linux/version.h>
#include "cdev_sgdma.h"



/*
 * character device file operations for SG DMA engine
 */


/* char_sgdma_read_write() -- Read from or write to the device
 *
 * @buf userspace buffer
 * @count number of bytes in the userspace buffer
 * @pos byte-address in device
 *  
 *
 * For each transfer, pin the user pages, build a sgtable, map, build a
 * descriptor table. submit the transfer. wait for the interrupt handler
 * to wake us on completion.
 */



static ssize_t char_sgdma_read_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *pos)
{
	ssize_t rv = 0;
	struct xdma_cdev *xcdev = (struct xdma_cdev *)filp->private_data;
	struct xdma_dev *xdev;
	struct xdma_engine *engine=xcdev->engine;
	/*guard against attempts for simultaneous transfer*/
	if(test_and_set_bit(XENGINE_BUSY_BIT, &(engine->flags)))
		return -EBUSY;
	/*just fill transfer params. checks are performed later inside xdma_xfer_submit*/
	engine->transfer_params.buf=buf;
	engine->transfer_params.length=count;
	if(!engine->streaming)
		engine->transfer_params.ep_addr=*pos;
	/*doesn't really matter in this case. setup is performed according engine direction*/
	engine->transfer_params.dir=engine->dir;

	rv = xdma_xfer_submit(engine);
	
	if(!engine->streaming && !engine->non_incr_addr &&(rv>0))
		*pos+=rv;
	
	clear_bit(XENGINE_BUSY_BIT, &(engine->flags));
	
	return rv;
}


static ssize_t char_sgdma_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *pos)
{
	return char_sgdma_read_write(filp, buf, count, pos);
}

static ssize_t char_sgdma_read(struct file *filp, char __user *buf,
				size_t count, loff_t *pos)
{
	return char_sgdma_read_write(filp, buf, count, pos);
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
	int set;
	int rv = get_user(set, (int __user *) arg);
	if(unlikely(rv<0))
		return rv;

	engine_addrmode_set(engine, !!set);
	return 0;
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


static int ioctl_do_submit_transfer(struct xdma_engine *engine, unsigned long arg)
{
	struct xdma_transfer_params __user *user_transfer_params=(struct xdma_transfer_params __user *) arg;
	ssize_t transfer_res;
	int rv=access_assert(user_transfer_params, sizeof(struct xdma_transfer_params));
	if (unlikely(rv<0))
		return rv;
	rv=access_assert(user_transfer_params->buf, user_transfer_params->length);
	if (unlikely(rv<0))
		return rv;
	/*to verify user intention, otherwise not really necessary*/
	if(user_transfer_params->dir!= engine->dir)
			return -ENOTSUPP;
			
	if(test_and_set_bit(XENGINE_BUSY_BIT, &(engine->flags)))
		return -EBUSY;					
	/*we already checked the access*/
	rv=raw_copy_from_user( &(engine->transfer_params), user_transfer_params, sizeof(struct xdma_transfer_params));
	if (unlikely(rv<0))
	{
		clear_bit(XENGINE_BUSY_BIT, &(engine->flags));
		return -EFAULT;
	}
	transfer_res=xdma_xfer_submit(engine);
	if(transfer_res<0)
	{
		rv=__put_user( 0, &(user_transfer_params->length));
		if (unlikely(rv<0))
			return rv;
		rv= (int) transfer_res;
	}
	else
	{
		rv=__put_user(transfer_res, &(user_transfer_params->length));
		if (unlikely(rv<0))
			return rv;
		rv=0;
	}
	return rv;	
}
	
static long char_sgdma_ioctl(struct file *filp, unsigned int cmd,
		unsigned long arg)
{
	struct xdma_cdev *xcdev = (struct xdma_cdev *)filp->private_data;
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
	case IOCTL_XDMA_SUBMIT_TRANSFER:
		rv = ioctl_do_submit_transfer(engine, arg);
		break;
	default:
		dbg_perf("Unsupported operation\n");
		rv = -ENOTTY;
		break;
	}

	return rv;
}

static int char_sgdma_open(struct inode *inode, struct file *filp)
{
	int ret_val=0;
	struct xdma_cdev *xcdev;
	struct xdma_engine *engine;
	
	char_open(inode, filp);

	xcdev = (struct xdma_cdev *)filp->private_data;
	engine = xcdev->engine;
	
	//don't allow to open the engine more than once
	if(test_and_set_bit(XENGINE_OPEN_BIT, &(engine->flags)))
		return -EBUSY;
	/*Should never ever happen otherwise something went horribly wrong*/
	xdma_debug_assert_msg((engine->dir==DMA_TO_DEVICE)||(engine->dir==DMA_FROM_DEVICE), 
		"Unexpected direction of XDMA engine", -ENODEV);
	/* make sure that file access mode matches direction of engine and otherwise deny access  */
	if(engine->dir==DMA_TO_DEVICE)
	{
		if((filp->f_flags & O_ACCMODE)!=O_WRONLY)
		{
			ret_val= -EACCES;
			goto not_open;
		}
		filp->f_mode&= ~FMODE_READ;
		
	}
	else
	{
		if((filp->f_flags & O_ACCMODE)!=O_RDONLY)
		{
			ret_val= -EACCES;
			goto not_open;
		}
		filp->f_mode&= ~FMODE_WRITE;
	}
	
	if (engine->streaming)
	{	/*mark dev file as streaming device*/
		stream_open(inode, filp);
		engine->eop_flush=(filp->f_flags& O_TRUNC)? 1: 0;
		
			
	}else /*MM DMA engine*/
	{
		if ((ret_val=generic_file_open(inode, filp ))<0)
			goto not_open;
		
		engine_addrmode_set(engine, (filp->f_flags& O_TRUNC)? 1: 0);
		
	}
	print_fmode(filp->f_path.dentry->d_iname, filp->f_mode);
	
	not_open:
	if (ret_val<0)/*clear busy bit again, if file can't be allowed to open*/
		clear_bit(XENGINE_OPEN_BIT, &(engine->flags));
	
	return ret_val;
}

static int char_sgdma_close(struct inode *inode, struct file *filp)
{
	struct xdma_cdev *xcdev = (struct xdma_cdev *)filp->private_data;
	struct xdma_engine *engine;
	int rv;

	rv = xcdev_check(__func__, xcdev, 1);
	if (rv < 0)
		return rv;

	engine = xcdev->engine;
	
	clear_bit(XENGINE_OPEN_BIT, &(engine->flags));

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
