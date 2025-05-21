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
#include "xdma_cdev.h"
#include "xdma_ioctl.h"




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
	struct xdma_engine *engine=xcdev->engine;
	/*guard against attempts for simultaneous transfer*/
	if(test_and_set_bit(XENGINE_BUSY_BIT, &(engine->flags)))
		return -EBUSY;
	/*just fill transfer params. checks are performed later inside xdma_xfer_submit*/
	engine->transfer_params.buf=buf;
	engine->transfer_params.length=count;
	if(!engine->streaming)
		engine->transfer_params.ep_addr=*pos;
#ifdef ___LIBXDMA_DEBUG__
	/*doesn't really matter in this case. setup is performed according engine direction*/
	engine->transfer_params.dir=engine->dir;
#endif
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





static int ioctl_do_perf_test(struct xdma_engine *engine, unsigned long arg)
{

	int rv;
	
	xdma_debug_assert_ptr(engine);

	if (test_and_set_bit(XENGINE_BUSY_BIT, &(engine->flags))) 		
		return -EBUSY;
	
	rv = copy_from_user( &(engine->xdma_perf),
		(struct xdma_performance_ioctl __user *)arg,
		sizeof(struct xdma_performance_ioctl));

	if (rv < 0) {
		dbg_perf("Failed to copy from user space 0x%lx\n", arg);
		goto exit;
	}
	
	dbg_perf("Performance test transfer_size = %u\n", engine->xdma_perf.transfer_size);
	rv = xdma_performance_submit(engine);
	if (rv < 0)
		goto exit;
	rv = copy_to_user((void __user *)arg, &(engine->xdma_perf),
			sizeof(struct xdma_performance_ioctl));
	if (rv<0) 
		dbg_perf("Error copying result to user\n");
	
	exit:
	clear_bit(XENGINE_BUSY_BIT, &(engine->flags));
	return rv;
}



static int ioctl_do_addrmode_set(struct xdma_engine *engine, unsigned long arg)
{
	bool set;
	int rv = get_user(set, (int __user *) arg);
	if(unlikely(rv<0))
		return rv;
	if (test_and_set_bit(XENGINE_BUSY_BIT, &(engine->flags))) 		
		return -EBUSY;
	engine_addrmode_set(engine, set);
	clear_bit(XENGINE_BUSY_BIT, &(engine->flags));
	return 0;
}

static int ioctl_do_addrmode_get(struct xdma_engine *engine, unsigned long arg)
{
	int rv;
	bool src;

	xdma_debug_assert_ptr(engine);
	src = (bool) engine->non_incr_addr;

	dbg_perf("XDMA_IOCTL_ADDRMODE_GET\n");
	rv = put_user(src, (int __user *)arg);

	return rv;
}

static int ioctl_do_align_get(struct xdma_engine *engine, unsigned long arg)
{
	xdma_debug_assert_ptr(engine);

	dbg_perf("XDMA_IOCTL_ALIGN_GET\n");
	return put_user(engine->addr_align, (int __user *)arg);
}


static int ioctl_do_submit_transfer(struct xdma_engine *engine, unsigned long arg)
{
	struct xdma_transfer_request __user *user_transfer_request=(struct xdma_transfer_request __user *) arg;
	ssize_t transfer_res;
	enum xdma_transfer_mode transfer_mode;
	int rv=access_assert(user_transfer_request, sizeof(struct xdma_transfer_request));
	if (unlikely(rv<0))
		return rv;
	
	rv=__get_user(transfer_mode, &(user_transfer_request->mode));
	if (unlikely(rv<0))
		return rv;
	/*to verify user intention, otherwise not really necessary*/
	if(!((transfer_mode==XDMA_H2C)&& (engine->dir==DMA_TO_DEVICE)) && 
		!((transfer_mode==XDMA_C2H) && (engine->dir==DMA_FROM_DEVICE)))
	{
		pr_err("Improper XDMA transfer mode\n");
		return -ENOTSUPP;
	}		
	if(test_and_set_bit(XENGINE_BUSY_BIT, &(engine->flags)))
		return -EBUSY;					
	/*we already checked the access*/
	rv=__get_user( engine->transfer_params.buf, &(user_transfer_request->buf));
	if (unlikely(rv<0))
			goto exit;
		rv=__get_user( engine->transfer_params.length, &(user_transfer_request->length));
	if (unlikely(rv<0))
			goto exit;
	/*                 \/ avoids compiler warning*/
	rv=access_assert((char*) engine->transfer_params.buf, engine->transfer_params.length);
	if (unlikely(rv<0))
	{
		engine->transfer_params.buf=NULL;
		engine->transfer_params.length=0;
		goto exit;
	}
	if(!engine->streaming)
	{
		rv=__get_user( engine->transfer_params.ep_addr, &(user_transfer_request->axi_address));
		if (unlikely(rv<0))
				goto exit;
	}
#ifdef ___LIBXDMA_DEBUG__
	engine->transfer_params.dir= (transfer_mode==XDMA_H2C) ? DMA_TO_DEVICE: DMA_FROM_DEVICE;
#endif
	transfer_res=xdma_xfer_submit(engine);
	
	if(transfer_res<0)
	{
		rv=__put_user( 0, &(user_transfer_request->length));
		if (unlikely(rv<0))
			goto exit;
		rv= (int) transfer_res;
	}
	else
	{
		rv=__put_user(transfer_res, &(user_transfer_request->length));
		if (unlikely(rv<0))
			goto exit;
		rv=0;
	}
	
	exit:
	clear_bit(XENGINE_BUSY_BIT, &(engine->flags));
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
	case XDMA_IOCTL_PERF_TEST:
		rv = ioctl_do_perf_test(engine, arg);
		break;
	case XDMA_IOCTL_ADDRMODE_SET:
		rv = ioctl_do_addrmode_set(engine, arg);
		break;
	case XDMA_IOCTL_ADDRMODE_GET:
		rv = ioctl_do_addrmode_get(engine, arg);
		break;
	case XDMA_IOCTL_ALIGN_GET:
		rv = ioctl_do_align_get(engine, arg);
		break;
	case XDMA_IOCTL_SUBMIT_TRANSFER:
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
