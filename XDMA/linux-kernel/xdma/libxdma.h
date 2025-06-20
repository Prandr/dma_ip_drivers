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

#ifndef XDMA_LIB_H
#define XDMA_LIB_H

#include <linux/version.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/completion.h>
#include <linux/atomic.h>
#include "xdma_ioctl.h"

/* SECTION: Preprocessor macros/constants */
#define XDMA_BAR_NUM (6)

/* maximum amount of register space to map */
#define XDMA_BAR_SIZE (0x8000U)

#define XDMA_CHANNEL_NUM_MAX (4)
/*
 * interrupts per engine, rad2_vul.sv:237
 * .REG_IRQ_OUT	(reg_irq_from_ch[(channel*2) +: 2]),
 */
#define XDMA_ENG_IRQ_NUM	(1)
#define XDMA_PAGE_SIZE		0x1000
#define RX_STATUS_EOP		(1)

/* Target internal components on XDMA control BAR */
#define XDMA_OFS_INT_CTRL	(0x2000U)
#define XDMA_OFS_CONFIG		(0x3000U)

#define XDMA_DESC_FIFO_DEPTH 512


/* maximum size of a single DMA transfer descriptor */
#define XDMA_DESC_BLEN_BITS	28
#define XDMA_DESC_BLEN_MAX	((1 << (XDMA_DESC_BLEN_BITS)) - 1)


#define XDMA_MAX_C2H_CREDITS  ((1<<10)-1)
/* bits of the SG DMA control register */
#define XDMA_CTRL_RUN_STOP			(1U << 0)
#define XDMA_CTRL_IE_DESC_STOPPED		(1U << 1)
#define XDMA_CTRL_IE_DESC_COMPLETED		(1U << 2)
#define XDMA_CTRL_IE_DESC_ALIGN_MISMATCH	(1U << 3)
#define XDMA_CTRL_IE_MAGIC_STOPPED		(1U << 4)
#define XDMA_CTRL_IE_INVALID_LENGTH		(1U << 5)
#define XDMA_CTRL_IE_IDLE_STOPPED		(1U << 6)
#define XDMA_CTRL_IE_READ_ERROR			(0x1FU << 9)
#define XDMA_CTRL_IE_WRITE_ERROR		(0x1FU << 14)
#define XDMA_CTRL_IE_DESC_ERROR			(0x1FU << 19)
#define XDMA_CTRL_NON_INCR_ADDR			(1U << 25)
#define XDMA_CTRL_POLL_MODE_WB			(1U << 26)
#define XDMA_CTRL_STM_MODE_WB			(1U << 27)

/* bits of the SG DMA status register */
#define XDMA_STAT_BUSY			(1U << 0)
#define XDMA_STAT_DESC_STOPPED		(1U << 1)
#define XDMA_STAT_DESC_COMPLETED	(1U << 2)
#define XDMA_STAT_ALIGN_MISMATCH	(1U << 3)
#define XDMA_STAT_MAGIC_STOPPED		(1U << 4)
#define XDMA_STAT_INVALID_LEN		(1U << 5)
#define XDMA_STAT_IDLE_STOPPED		(1U << 6)

#define XDMA_STAT_COMMON_ERR_MASK \
	(XDMA_STAT_ALIGN_MISMATCH | XDMA_STAT_MAGIC_STOPPED | \
	 XDMA_STAT_INVALID_LEN)

/* desc_error, C2H & H2C */
#define XDMA_STAT_DESC_UNSUPP_REQ	(1U << 19)
#define XDMA_STAT_DESC_COMPL_ABORT	(1U << 20)
#define XDMA_STAT_DESC_PARITY_ERR	(1U << 21)
#define XDMA_STAT_DESC_HEADER_EP	(1U << 22)
#define XDMA_STAT_DESC_UNEXP_COMPL	(1U << 23)

#define XDMA_STAT_DESC_ERR_MASK	\
	(XDMA_STAT_DESC_UNSUPP_REQ | XDMA_STAT_DESC_COMPL_ABORT | \
	 XDMA_STAT_DESC_PARITY_ERR | XDMA_STAT_DESC_HEADER_EP | \
	 XDMA_STAT_DESC_UNEXP_COMPL)

/* read error: H2C */
#define XDMA_STAT_H2C_R_UNSUPP_REQ	(1U << 9)
#define XDMA_STAT_H2C_R_COMPL_ABORT	(1U << 10)
#define XDMA_STAT_H2C_R_PARITY_ERR	(1U << 11)
#define XDMA_STAT_H2C_R_HEADER_EP	(1U << 12)
#define XDMA_STAT_H2C_R_UNEXP_COMPL	(1U << 13)

#define XDMA_STAT_H2C_R_ERR_MASK	\
	(XDMA_STAT_H2C_R_UNSUPP_REQ | XDMA_STAT_H2C_R_COMPL_ABORT | \
	 XDMA_STAT_H2C_R_PARITY_ERR | XDMA_STAT_H2C_R_HEADER_EP | \
	 XDMA_STAT_H2C_R_UNEXP_COMPL)

/* write error, H2C only */
#define XDMA_STAT_H2C_W_DECODE_ERR	(1U << 14)
#define XDMA_STAT_H2C_W_SLAVE_ERR	(1U << 15)

#define XDMA_STAT_H2C_W_ERR_MASK	\
	(XDMA_STAT_H2C_W_DECODE_ERR | XDMA_STAT_H2C_W_SLAVE_ERR)

/* read error: C2H */
#define XDMA_STAT_C2H_R_DECODE_ERR	(1U << 9)
#define XDMA_STAT_C2H_R_SLAVE_ERR	(1U << 10)

#define XDMA_STAT_C2H_R_ERR_MASK	\
	(XDMA_STAT_C2H_R_DECODE_ERR | XDMA_STAT_C2H_R_SLAVE_ERR)

/* all combined */
#define XDMA_STAT_H2C_ERR_MASK	\
	(XDMA_STAT_COMMON_ERR_MASK | XDMA_STAT_DESC_ERR_MASK | \
	 XDMA_STAT_H2C_R_ERR_MASK | XDMA_STAT_H2C_W_ERR_MASK)

#define XDMA_STAT_C2H_ERR_MASK	\
	(XDMA_STAT_COMMON_ERR_MASK | XDMA_STAT_DESC_ERR_MASK | \
	 XDMA_STAT_C2H_R_ERR_MASK)

/* bits of the SGDMA descriptor control field */
#define XDMA_DESC_STOPPED	(1U << 0)
#define XDMA_DESC_COMPLETED	(1U << 1)
#define XDMA_DESC_EOP		(1U << 4)

#define XDMA_PERF_RUN	(1U << 0)
#define XDMA_PERF_CLEAR	(1U << 1)
#define XDMA_PERF_AUTO	(1U << 2)

#define MAGIC_ENGINE	0xEEEEEEEEU
#define MAGIC_DEVICE	0xDDDDDDDDU

/* upper 16-bits of engine identifier register */
#define XDMA_ID_H2C 0x1fc0U
#define XDMA_ID_C2H 0x1fc1U

#define LS_BYTE_MASK 0x000000FFU

#define BLOCK_ID_MASK 0xFFF00000
#define BLOCK_ID_HEAD 0x1FC00000

#define IRQ_BLOCK_ID 0x1fc20000U
#define CONFIG_BLOCK_ID 0x1fc30000U

#define WB_COUNT_MASK 0x00ffffffU
#define WB_ERR_MASK (1U << 31)

#define MAX_USER_IRQ 16

#define DESC_MAGIC 0xAD4B0000U
#define DESC_ADJ_SHIFT 8
#define DESC_ADJ_MASK (0x3FU<<DESC_ADJ_SHIFT)

#define C2H_WB 0x52B4U

#define MAX_NUM_ENGINES (XDMA_CHANNEL_NUM_MAX * 2)
#define H2C_CHANNEL_OFFSET 0x1000
#define SGDMA_OFFSET_FROM_CHANNEL 0x4000
#define CHANNEL_SPACING 0x100
#define TARGET_SPACING 0x1000


/* obtain the 32 most significant (high) bits of a 32-bit or 64-bit address */
#define PCI_DMA_H(addr) ((addr >> 16) >> 16)
/* obtain the 32 least significant (low) bits of a 32-bit or 64-bit address */
#define PCI_DMA_L(addr) (addr & 0xffffffffU)
/*split a 64-bit value into 32-bit values. To be used with descriptors*/
#define split_into_val32(val64, val32_high, val32_low) \
	val32_high=cpu_to_le32((typeof(val32_high)) ((val64)>>32));\
	val32_low=cpu_to_le32((typeof(val32_low)) (val64));
#define divide_roundup( x, y) ((x+(y-1))/y)

#ifndef VM_RESERVED
	#define VMEM_FLAGS (VM_IO | VM_DONTEXPAND | VM_DONTDUMP)
#else
	#define VMEM_FLAGS (VM_IO | VM_RESERVED)
#endif

#ifdef __LIBXDMA_DEBUG__
#define DEBUG
#define dbg_io		pr_info
#define dbg_fops	pr_info
#define dbg_perf	pr_info
#define dbg_sg		pr_info
#define dbg_tfr		pr_info
#define dbg_irq		pr_info
#define dbg_init	pr_info
#define dbg_desc	pr_info

#define xdma_debug_assert_msg(cond, msg, ret_val) \
		if (!(cond)){\
			pr_err("Debug assertion %s failed. %s", #cond, msg); \
			return (ret_val);\
			}
#define xdma_debug_assert_ptr(ptr) xdma_debug_assert_msg(ptr!=NULL, "Pointer " #ptr" is NULL.", -EINVAL)
#else
/* disable debugging */
#define dbg_io(...)
#define dbg_fops(...)
#define dbg_perf(...)
#define dbg_sg(...)
#define dbg_tfr(...)
#define dbg_irq(...)
#define dbg_init(...)
#define dbg_desc(...)

#define xdma_debug_assert_msg(...)
#define xdma_debug_assert_ptr(...)
#endif


/*define for non RHEL as a workaround to avoid preprocor error*/
#ifndef RHEL_RELEASE_VERSION/*set to int max to make comparision false on non RHEL*/
#define RHEL_RELEASE_VERSION(a,b) (0x7fffffff) 
#endif
#define LINUX_VERSION_CHECK(major, minor, rev) (LINUX_VERSION_CODE >= KERNEL_VERSION(major, minor, rev))
/*checks for RHEL and general Linux version in single macro */
#define KERNEL_VERSION_CHECK(rhel_major, rhel_minor, linux_major, linux_minor, linux_rev) \
    (((defined(RHEL_RELEASE_CODE) && (RHEL_RELEASE_CODE >= RHEL_RELEASE_VERSION(rhel_major, rhel_minor)))|| \
    (!defined(RHEL_RELEASE_CODE) && LINUX_VERSION_CHECK(linux_major, linux_minor, linux_rev)))) 
    

#if KERNEL_VERSION_CHECK(8, 0, 5, 0, 0)
	#define _access_check(ptr, size) access_ok(ptr, size)
#else 
	#define _access_check(ptr, size) access_ok(VERIFY_WRITE, ptr, size)//write includes read. has no impact anyway
#endif

static inline int __access_assert(void __user *ptr, unsigned long size, const char * arg_name, const char *func)
{
	if (!_access_check(ptr, size)) 
	{
		pr_err("Bad pointer %s (%px) in function %s",arg_name,  ptr, func);
		return -EFAULT; 
	}
	return 0;
}

#define access_assert(ptr, size) __access_assert(ptr, size, #ptr, __func__)

/* SECTION: Enum definitions */
enum dev_capabilities {
	CAP_64BIT_DMA = 2,
	CAP_64BIT_DESC = 4,
	CAP_ENGINE_WRITE = 8,
	CAP_ENGINE_READ = 16
};

/* SECTION: Structure definitions */


struct config_regs {
	u32 identifier;
	u32 block_busdev;
	u32 max_payload_size;
	u32 max_read_request_size;
	u32 block_system_id;
	u32 msi_enable;
	u32 datapath_width;
} __packed;

/**
 * SG DMA Controller status and control registers
 *
 * These registers make the control interface for DMA transfers.
 *
 * It sits in End Point (FPGA) memory BAR[0] for 32-bit or BAR[0:1] for 64-bit.
 * It references the first descriptor which exists in Root Complex (PC) memory.
 *
 * @note The registers must be accessed using 32-bit (PCI DWORD) read/writes,
 * and their values are in little-endian byte ordering.
 */
struct engine_regs {
	u32 identifier;
	u32 control;
	u32 control_w1s;
	u32 control_w1c;
	u32 reserved_1[12];	/* padding */

	u32 status;
	u32 status_rc;
	u32 completed_desc_count;
	u32 alignments;
	u32 reserved_2[14];	/* padding */

	u32 poll_mode_wb_lo;
	u32 poll_mode_wb_hi;
	u32 interrupt_enable_mask;
	u32 interrupt_enable_mask_w1s;
	u32 interrupt_enable_mask_w1c;
	u32 reserved_3[9];	/* padding */

	u32 perf_ctrl;
	u32 perf_cyc_lo;
	u32 perf_cyc_hi;
	u32 perf_dat_lo;
	u32 perf_dat_hi;
} __packed;

struct engine_sgdma_regs {
	u32 identifier;
	u32 reserved_1[31];	/* padding */

	/* bus address to first descriptor in Root Complex Memory */
	u32 first_desc_lo;
	u32 first_desc_hi;
	/* number of adjacent descriptors at first_desc */
	u32 first_desc_adjacent;
	u32 credits;
} __packed;

struct msix_vec_table_entry {
	u32 msi_vec_addr_lo;
	u32 msi_vec_addr_hi;
	u32 msi_vec_data_lo;
	u32 msi_vec_data_hi;
} __packed;

struct msix_vec_table {
	struct msix_vec_table_entry entry_list[32];
} __packed;

struct interrupt_regs {
	u32 identifier;
	u32 user_int_enable;
	u32 user_int_enable_w1s;
	u32 user_int_enable_w1c;
	u32 channel_int_enable;
	u32 channel_int_enable_w1s;
	u32 channel_int_enable_w1c;
	u32 reserved_1[9];	/* padding */

	u32 user_int_request;
	u32 channel_int_request;
	u32 user_int_pending;
	u32 channel_int_pending;
	u32 reserved_2[12];	/* padding */

	u32 user_msi_vector[8];
	u32 channel_msi_vector[8];
} __packed;

struct sgdma_common_regs {
	u32 padding[4];
	u32 credit_mode_enable;
	u32 credit_mode_enable_w1s;
	u32 credit_mode_enable_w1c;
} __packed;


/* Structure for polled mode descriptor writeback */
struct xdma_poll_wb {
	u32 completed_desc_count;
	//u32 reserved_1[7];
} __packed;


/**
 * Descriptor for a single contiguous memory block transfer.
 *
 * Multiple descriptors are linked by means of the next pointer. An additional
 * extra adjacent number gives the amount of extra contiguous descriptors.
 *
 * The descriptors are in root complex memory, and the bytes in the 32-bit
 * words must be in little-endian byte ordering.
 */
struct xdma_desc {
	u32 control;
	u32 bytes;		/* transfer length in bytes */
	u32 src_addr_lo;	/* source address (low 32-bit) */
	u32 src_addr_hi;	/* source address (high 32-bit) */
	u32 dst_addr_lo;	/* destination address (low 32-bit) */
	u32 dst_addr_hi;	/* destination address (high 32-bit) */
	/*
	 * next descriptor in the single-linked list of descriptors;
	 * this is the PCIe (bus) address of the next descriptor in the
	 * root complex memory
	 */
	u32 next_lo;		/* next desc address (low 32-bit) */
	u32 next_hi;		/* next desc address (high 32-bit) */
} __packed;

/* 32 bytes (four 32-bit words) or 64 bytes (eight 32-bit words) */
struct xdma_result {
	u32 status;
	u32 length;
	u32 reserved_1[6];	/* padding */
} __packed;

/*#define DMA_RECORD_TYPE(type) */
/*generic DMA record for flexible bookkeeping
inspired by C++ templates*/
#define generic_dma_record(enclosed_type)\
struct  {\
	enclosed_type *virtual_addr;\
	dma_addr_t dma_addr;\
	size_t length;\
} 
/*holds transfer parameters*/
struct xdma_transfer_params {
	const char __user *buf;
	size_t length; 
	loff_t ep_addr;
#ifdef __LIBXDMA_DEBUG__
	enum dma_data_direction dir;
#endif
};

#define XFER_FLAG_PAGES_ALLOC (1U<<0)
#define XFER_FLAG_PAGES_PINNED (1U<<1)
#define XFER_FLAG_SGTABLE_ALLOC (1U<<2)
#define XFER_FLAG_SGTABLE_MAPPED (1U<<3)
#define XFER_FLAG_DMA_RECORD_ALLOC (1U<<4)
#define XFER_FLAG_DESC_DMA_ALLOC (1U<<5)

/* holds data necessary to perform a transfer*/
struct xdma_transfer {
	struct page **pages;
	unsigned int num_pages;
	struct sg_table sgt;
	generic_dma_record(struct xdma_desc) *adj_desc_blocks;/*bookkeeping for descriptors grouped in adjacent blocks*/
	unsigned int num_adj_blocks;
	unsigned int cleanup_flags;/*track initialisation stages of a transfer for cleanup*/
};

#define XENGINE_OPEN_BIT 0L
#define XENGINE_BUSY_BIT 1L

struct xdma_engine {
	unsigned int magic;	/* structure ID for sanity checks */
	struct xdma_dev *xdev;	/* parent device */
	char name[16];		/* name of this engine */
	int version;		/* version of this engine */
	volatile unsigned long flags;/*keeps device state of the engine*/

	/* HW register address offsets */
	struct engine_regs *regs;		/* Control reg BAR offset */
	struct engine_sgdma_regs *sgdma_regs;	/* SGDAM reg BAR offset */
	
	/* Engine state, configuration and flags */
	//enum shutdown_state shutdown;	/* engine shutdown mode */
	/*intrinsic properties of engine declared const*/
	const enum dma_data_direction dir;
	const u8 addr_align;		/* source/dest alignment in bytes */
	const u8 len_granularity;	/* transfer length multiple */
	const u8 addr_bits;		/* HW datapath address width */
	u8 channel:2;		/* engine indices */
	u8 streaming:1;
	const u8 filler1:1;	
	u8 running:1;		/* flag if the driver started engine */
	u8 non_incr_addr:1;	/* flag if non-incremental addressing used */
	u8 eop_flush:1;		/* st c2h only, flush up the data with eop */
	const u8 filler2:1;

	const unsigned int adj_block_len;	/* max length of adjacent block of descriptors */
	const unsigned desc_max;		/* max # descriptors per xfer */
	struct xdma_transfer_params transfer_params;
	struct xdma_transfer transfer;
	struct dma_pool *desc_pool;/*DMA pool for descriptors*/
	struct completion engine_compl;	
	/* only used for MSIX mode to store per-engine interrupt mask value */
	u32 interrupt_enable_mask_value;

#ifdef XDMA_POLL_MODE
	/* Members associated with polled mode support */
	generic_dma_record(volatile struct xdma_poll_wb) poll_mode_wb;
#endif

	/* Members associated with interrupt mode support */
	int msix_irq_line;		/* MSI-X vector for this engine */
	u32 irq_bitmask;		/* IRQ bit mask for this engine */

	/* for performance test support */
	struct xdma_performance_ioctl xdma_perf;	/* perf test control */
};

struct xdma_user_irq {
	struct xdma_dev *xdev;		/* parent device */
	u8 user_idx;			/* 0 ~ 15 */
	u8 events_irq;			/* accumulated IRQs */
	spinlock_t events_lock;		/* lock to safely update events_irq */
	wait_queue_head_t events_wq;	/* wait queue to sync waiting threads */
	irq_handler_t handler;

	void *dev;
};

/* XDMA PCIe device specific book-keeping */
#define XDEV_FLAG_OFFLINE_BIT	0L
struct xdma_dev {
	struct list_head list_head;
	struct list_head rcu_node;

	unsigned int magic;		/* structure ID for sanity checks */
	struct pci_dev *pdev;	/* pci device struct from probe() */
	int idx;		/* dev index */

	const char *mod_name;		/* name of module owning the dev */

	spinlock_t lock;		/* protects concurrent access */
	volatile unsigned long flags;

	/* PCIe BAR management */
	void __iomem *bar[XDMA_BAR_NUM];	/* addresses for mapped BARs */
	resource_size_t bar_size[XDMA_BAR_NUM];		/* mapped size of BARs	*/ 
	int user_bar_idx;	/* BAR index of user logic */
	int config_bar_idx;	/* BAR index of XDMA config logic */
	int bypass_bar_idx;	/* BAR index of XDMA bypass logic */
	int regions_in_use;	/* flag if dev was in use during probe() */
	int got_regions;	/* flag if probe() obtained the regions */

	int user_max;
	int c2h_channel_num;
	int h2c_channel_num;
	
	const unsigned int max_read_request_size;
	const unsigned int datapath_width;

	/* Interrupt management */
	int irq_count;		/* interrupt counter */
	int irq_line;		/* flag if irq allocated successfully */
	int msi_enabled;	/* flag if msi was enabled for the device */
	int msix_enabled;	/* flag if msi-x was enabled for the device */
	struct xdma_user_irq user_irq[16];	/* user IRQ management */
	unsigned int mask_irq_user;

	/* XDMA engine management */
	int engines_num;	/* Total engine count */
	u32 mask_irq_h2c;
	u32 mask_irq_c2h;
	struct xdma_engine *engine_h2c;
	struct xdma_engine *engine_c2h;

	/* SD_Accel specific */
	enum dev_capabilities capabilities;
	u64 feature_id;
};


void write_register(u32 value, void *iomem);
u32 read_register(void *iomem);

struct xdma_dev *xdev_find_by_pdev(struct pci_dev *pdev);
void xdma_device_close(struct pci_dev *pdev, void *dev_hndl);
void *xdma_device_open(const char *mname, struct pci_dev *pdev, int *user_max,
		       int *h2c_channel_num, int *c2h_channel_num);
int xdma_user_isr_enable(void *dev_hndl, unsigned int mask);
void xdma_device_offline(struct pci_dev *pdev, void *dev_handle);
void xdma_device_online(struct pci_dev *pdev, void *dev_handle);
ssize_t xdma_xfer_submit(struct xdma_engine *engine);
int xdma_performance_submit(struct xdma_engine *engine);
struct xdma_transfer *engine_cyclic_stop(struct xdma_engine *engine);
void enable_perf(struct xdma_engine *engine);
void get_perf_stats(struct xdma_engine *engine);

void engine_addrmode_set(struct xdma_engine *engine, bool set);

#endif /* XDMA_LIB_H */
