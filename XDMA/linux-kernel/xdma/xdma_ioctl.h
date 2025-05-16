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
 
 /* Use 'x' as magic number */
 #define XDMA_IOC_MAGIC	'x'
 
 /*Operations and structures definitions for control devices*/
/* XL OpenCL X->58(ASCII), L->6C(ASCII), O->0 C->C L->6C(ASCII); */
#define XDMA_XCL_MAGIC 0X586C0C6C 

enum XDMA_IOC_TYPES {
	XDMA_IOC_NOP,
	XDMA_IOC_INFO,
	XDMA_IOC_OFFLINE,
	XDMA_IOC_ONLINE,
	XDMA_IOC_MAX
};

struct xdma_ioc_base {
	unsigned int magic;
	unsigned int command;
};

struct xdma_ioc_info {
	struct xdma_ioc_base	base;
	unsigned short		vendor;
	unsigned short		device;
	unsigned short		subsystem_vendor;
	unsigned short		subsystem_device;
	unsigned int		dma_engine_version;
	unsigned int		driver_version;
	unsigned long long	feature_id;
	unsigned short		domain;
	unsigned char		bus;
	unsigned char		dev;
	unsigned char		func;
};

/* IOCTL codes */
#define XDMA_IOCINFO		_IOWR(XDMA_IOC_MAGIC, XDMA_IOC_INFO, \
					struct xdma_ioc_info)
#define XDMA_IOCOFFLINE		_IO(XDMA_IOC_MAGIC, XDMA_IOC_OFFLINE)
#define XDMA_IOCONLINE		_IO(XDMA_IOC_MAGIC, XDMA_IOC_ONLINE)

 /*Operation and structures definitions for XDMA engines*/
struct xdma_performance_ioctl {
	uint32_t transfer_size;
	/* measurement */
	uint64_t clock_cycle_count;
	uint64_t data_cycle_count;
};

enum xdma_transfer_mode
{XDMA_READ, XDMA_WRITE };

struct xdma_transfer_request {
	char *buf;
	size_t length; 
	off_t ep_addr;
	enum xdma_transfer_mode mode;
};
/* IOCTL codes */

#define XDMA_IOCTL_PERF_TEST   _IOWR(XDMA_IOC_MAGIC, 1, struct xdma_performance_ioctl )
#define XDMA_IOCTL_ADDRMODE_SET _IOW(XDMA_IOC_MAGIC, 4, int)
#define XDMA_IOCTL_ADDRMODE_GET _IOR(XDMA_IOC_MAGIC, 5, int)
#define XDMA_IOCTL_ALIGN_GET    _IOR(XDMA_IOC_MAGIC, 6, int)
#define XDMA_IOCTL_SUBMIT_TRANSFER   _IOWR(XDMA_IOC_MAGIC, 7, struct xdma_transfer_request )

#endif /* _XDMA_IOCALLS_POSIX_H_ */
