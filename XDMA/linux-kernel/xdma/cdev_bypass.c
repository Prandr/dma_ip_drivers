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
#define pr_fmt(fmt)	KBUILD_MODNAME ":%s: " fmt, __func__

#include "xdma_cdev.h"
 
#define write_register(v, mem, off) iowrite32(v, mem)


/*Use 64-bit IO if available, otherwise 32 bit*/
#ifdef CONFIG_64BIT
typedef u64 iotype;
#define write_func(val, addr) writeq(val, addr)
#define read_func(addr) readq(addr)
#define USING64 true

#else
typedef u32 iotype;
#define write_func(val, addr) iowrite32(val, addr)
#define read_func(addr) ioread32(addr)
#define USING64 false
#endif



static ssize_t char_bypass_read(struct file *filp, char __user *buf,
		size_t count, loff_t *pos)
{
	struct xdma_dev *xdev;
	struct xdma_cdev *xcdev = (struct xdma_cdev *)filp->private_data;
	int rc = 0;
	iotype __iomem *ep_addr;
	iotype __user *user_buf;
	u8 __iomem *ep_addr_8;
	size_t i;
	
	rc = xcdev_check(__func__, xcdev, false);
	if (unlikely(rc < 0))
		return rc;
	xdev = xcdev->xdev;


	/*sanity checks for offsets*/
	rc=position_check(xdev->bar_size[xcdev->bar], *pos, 1);
	if (unlikely(rc < 0))
		return rc;	

	/*check for multiple of datapth?*/
	ep_addr=xdev->bar[xcdev->bar] +*pos;
	user_buf= (iotype *)buf;
	dbg_fops("buf: %px,  bypass BAR: %p, pos: %llx (%lld), ep_addr: %p, count %zu, 64 bit IO: %s\n", 
			buf, xdev->bar[xcdev->bar], *pos, *pos, ep_addr, count, USING64? "true" : "false");
	/*Get data in largest chunks possible first (8 or 4 bytes)*/
	for (i=count/sizeof(iotype);i; --i,  ++user_buf, ++ep_addr)
	{	
		iotype val=read_func(ep_addr);
		rc=__put_user(val, user_buf);
		if(unlikely(rc<0))
			return rc;
	}
 	
 	/*then get remaining data byte by byte*/
 	buf= (char __user *) user_buf;
 	ep_addr_8=(u8 __iomem *) ep_addr; 
 	for(i=count & (sizeof(iotype)-1); i; --i, ++buf, ++ep_addr_8)
 	{
 		u8 val=ioread8(ep_addr_8);
		rc=__put_user(val, buf);
		if(unlikely(rc<0))
			return rc;
 	} 

	*pos+=count;
	return count;
	
}

static ssize_t char_bypass_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *pos)
{
	struct xdma_dev *xdev;
	struct xdma_cdev *xcdev = (struct xdma_cdev *)filp->private_data;
	iotype __iomem *ep_addr;
	iotype __user *user_buf;
	u8 __iomem *ep_addr_8;
	size_t i;

	int rc = 0;
	
	rc = xcdev_check(__func__, xcdev, false);
	if (unlikely(rc < 0))
		return rc;
	xdev = xcdev->xdev;
		

	/*sanity checks for offsets*/
	rc=position_check(xdev->bar_size[xcdev->bar], *pos, 1);
	if (unlikely(rc < 0))
		return rc;
	
	ep_addr=xdev->bar[xcdev->bar] +*pos;
	user_buf= (iotype *)buf;
	dbg_fops("buf: %px,  bypass BAR: %p, pos: %llx (%lld), ep_addr: %p, count %zu, 64 bit IO: %s\n", 
			buf, xdev->bar[xcdev->bar], *pos, *pos, ep_addr, count, USING64? "true" : "false");	
	
	for (i=count/sizeof(iotype);i; --i,  ++user_buf, ++ep_addr)
	{
		iotype val;
		rc=__get_user(val, user_buf);
		if(unlikely(rc<0))
			return rc;
		write_func(val, ep_addr);
	
	}
	
	buf= (char __user *) user_buf;
 	ep_addr_8=(u8 __iomem *) ep_addr; 
 	for(i=count & (sizeof(iotype)-1); i; --i, ++buf, ++ep_addr_8)
 	{
 		u8 val;
		rc=__get_user(val, buf);
		if(unlikely(rc<0))
			return rc;
		write_func(val, ep_addr_8);
 	}
	*pos+=count;
	return count;
}


/*
 * character device file operations for bypass operation
 */

static const struct file_operations bypass_fops = {
	.owner = THIS_MODULE,
	.open = char_open,
	.release = char_close,
	.read = char_bypass_read,
	.write = char_bypass_write,
	.mmap = bridge_mmap,
	.llseek = char_llseek,
};

void cdev_bypass_init(struct xdma_cdev *xcdev)
{
	cdev_init(&xcdev->cdev, &bypass_fops);
}
