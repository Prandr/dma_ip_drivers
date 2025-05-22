
SHELL = /bin/bash
#
# optional makefile parameters:
# - DEBUG=<0|1>,	enable verbose debug print-out in the driver
# - config_bar_num=,	xdma pci config bar number
# - xvc_bar_num=,	xvc pci bar #
# - xvc_bar_offset=,	xvc register base offset
# 
ifneq ($(xvc_bar_num),)
	XVC_FLAGS += -D__XVC_BAR_NUM__=$(xvc_bar_num)
endif

ifneq ($(xvc_bar_offset),)
	XVC_FLAGS += -D__XVC_BAR_OFFSET__=$(xvc_bar_offset)
endif

#$(warning XVC_FLAGS: $(XVC_FLAGS).)

topdir := $(shell cd $(src)/.. && pwd)

TARGET_MODULE:=xdma-chr

EXTRA_CFLAGS := -I$(topdir)/include $(XVC_FLAGS)
ifeq ($(DEBUG),1)
	EXTRA_CFLAGS += -D__LIBXDMA_DEBUG__
endif
ifneq ($(config_bar_num),)
	EXTRA_CFLAGS += -DXDMA_CONFIG_BAR_NUM=$(config_bar_num)
endif

ifneq ($(c2h_credits),)
	EXTRA_CFLAGS += -DXDMA_C2H_CREDITS=$(c2h_credits)
endif
ifneq ($(h2c_timeout),)
	EXTRA_CFLAGS += -DXDMA_H2C_TIMEOUT=$(h2c_timeout)
endif
ifneq ($(c2h_timeout),)
	EXTRA_CFLAGS += -DXDMA_C2H_TIMEOUT=$(c2h_timeout)
endif
ifeq ($(POLLING),1)
	EXTRA_CFLAGS += -DXDMA_POLL_MODE
endif

ifneq ($(PREFIX),)
	EXTRA_CFLAGS += -DXDMA_PREFIX='"$(PREFIX)"'
endif
#EXTRA_CFLAGS += -DINTERNAL_TESTING

ifneq ($(KERNELRELEASE),)
	$(TARGET_MODULE)-objs := libxdma.o xdma_cdev.o cdev_ctrl.o cdev_events.o cdev_sgdma.o cdev_xvc.o cdev_bypass.o xdma_mod.o 
	obj-m := $(TARGET_MODULE).o
else
	MODLIB:=/lib/modules/$(shell uname -r)
	BUILDSYSTEM_DIR:=$(MODLIB)/build
	PWD:=$(shell pwd)
	DEPMOD:=depmod
all :
	$(MAKE) -C $(BUILDSYSTEM_DIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(BUILDSYSTEM_DIR) M=$(PWD) clean
	@/bin/rm -f *.ko modules.order *.mod.c *.o *.o.ur-safe .*.o.cmd

install: modules_install
	@rm -f /lib/modules/5.15.0-67-generic/extra/xdma.ko

modules_install: all
	@echo "installing kernel modules to $(MODLIB)/xdma ..."
	@mkdir -p -m 755 $(MODLIB)/xdma
	@install -v -m 644 *.ko $(MODLIB)/xdma
	@$(DEPMOD) -a || true
	@install -v -m 644 xdma_ioctl.h /usr/local/include

uninstall:
	@echo "Un-installing $(MODLIB)/xdma ..."
	@/bin/rm -rf $(MODLIB)/xdma
	@$(DEPMOD) -a || true
	@/bin/rm -f /usr/local/include/xdma_ioctl.h
help:
	@cat build-help.txt



endif
