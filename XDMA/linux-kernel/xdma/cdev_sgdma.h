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

#ifndef _XDMA_IOCALLS_POSIX_H_
#define _XDMA_IOCALLS_POSIX_H_

#include <linux/ioctl.h>
#include "xdma_cdev.h"

#define IOCTL_XDMA_PERF_V1 (1)
#define XDMA_ADDRMODE_MEMORY (0)
#define XDMA_ADDRMODE_FIXED (1)

/*
 * _IO(type,nr)		    no arguments
 * _IOR(type,nr,datatype)   read data from driver
 * _IOW(type,nr,datatype)   write data to driver
 * _IORW(type,nr,datatype)  read/write data
 *
 * _IOC_DIR(nr)		    returns direction
 * _IOC_TYPE(nr)	    returns magic
 * _IOC_NR(nr)		    returns number
 * _IOC_SIZE(nr)	    returns size
 */

struct xdma_performance_ioctl {
	/* IOCTL_XDMA_IOCTL_Vx */
	uint32_t version;
	uint32_t transfer_size;
	/* measurement */
	uint32_t stopped;
	uint32_t iterations;
	uint64_t clock_cycle_count;
	uint64_t data_cycle_count;
	uint64_t pending_count;
};


/* IOCTL codes */

#define IOCTL_XDMA_PERF_START   _IOW(XDMA_IOC_MAGIC, 1, struct xdma_performance_ioctl )
#define IOCTL_XDMA_PERF_STOP    _IOW(XDMA_IOC_MAGIC, 2, struct xdma_performance_ioctl )
#define IOCTL_XDMA_PERF_GET     _IOR(XDMA_IOC_MAGIC, 3, struct xdma_performance_ioctl )
#define IOCTL_XDMA_ADDRMODE_SET _IOW(XDMA_IOC_MAGIC, 4, int)
#define IOCTL_XDMA_ADDRMODE_GET _IOR(XDMA_IOC_MAGIC, 5, int)
#define IOCTL_XDMA_ALIGN_GET    _IOR(XDMA_IOC_MAGIC, 6, int)
#define IOCTL_XDMA_SUBMIT_TRANSFER   _IOWR(XDMA_IOC_MAGIC, 7, struct xdma_transfer_params )

#endif /* _XDMA_IOCALLS_POSIX_H_ */
