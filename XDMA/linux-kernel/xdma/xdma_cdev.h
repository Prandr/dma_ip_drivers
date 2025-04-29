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

#ifndef __XDMA_CHRDEV_H__
#define __XDMA_CHRDEV_H__

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/errno.h>
#include "xdma_mod.h"

#define XDMA_NODE_NAME	"xdma"
#define XDMA_MINOR_BASE (0)
#define XDMA_MINOR_COUNT (255)
#define MAX_RESOURCE_SIZE   (~((resource_size_t) 0))//max positive value of loff_t

void xdma_cdev_cleanup(void);
int xdma_cdev_init(void);

int char_open(struct inode *inode, struct file *filp);
loff_t char_llseek(struct file *filp, loff_t off, int whence);
int char_close(struct inode *inode, struct file *filp);
int xcdev_check(const char *fname, struct xdma_cdev *xcdev, bool check_engine);
int position_check(resource_size_t max_pos, loff_t pos, loff_t align);
void cdev_ctrl_init(struct xdma_cdev *xcdev);
void cdev_xvc_init(struct xdma_cdev *xcdev);
void cdev_event_init(struct xdma_cdev *xcdev);
void cdev_sgdma_init(struct xdma_cdev *xcdev);
void cdev_bypass_init(struct xdma_cdev *xcdev);
long char_ctrl_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);

void xpdev_destroy_interfaces(struct xdma_pci_dev *xpdev);
int xpdev_create_interfaces(struct xdma_pci_dev *xpdev);

int bridge_mmap(struct file *filp, struct vm_area_struct *vma);
#ifdef __LIBXDMA_DEBUG__
void print_fmode(const unsigned char *file_name, unsigned int f_mode);
#else
#define print_fmode(...)
#endif

#endif /* __XDMA_CHRDEV_H__ */
