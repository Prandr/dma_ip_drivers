# Xilinx DMA IP Reference drivers
## Xilinx DMA (XDMA)

The driver for XDMA that was reworked from ground up is released publicly under FOSS priciple of _mutual_ benefit and cooperation. Please, be aware that **neither me nor my employer are in any way associated or affiliated with Advanced Micro Devices, Inc (formerly Xilinx, Inc).** Therefore, *demands* of support would serve no purpose, because you shouldn't regard yourself a customer. Instead regard yourself a member of an interest group, that has a need for high-speed communication over PCI-E between an FPGA and a host and wishes to use XDMA for that purpose. You are welcome to try and use the reworked driver, but be also encouraged provide feedback and otherwise contribute any way you feel capable.

This software is not guaranteed to be free of errors. Take a note of the disclaimer on top of each source file. You should thoroughly test before put it in operation, especially in safety or mission-critical applications. **Please carefully read the [README](./XDMA/linux-kernel/readme.md) before using the driver**

Issues are only for reports and discussion of the problems. Anything else: general feedback and discusssions, ideas and question should be posted in Discussions.

The drivers mentioned below are retained for repo consistency, but otherwise stale.


## Xilinx QDMA

The Xilinx PCI Express Multi Queue DMA (QDMA) IP provides high-performance direct memory access (DMA) via PCI Express. The PCIe QDMA can be implemented in UltraScale+ devices.

Both the linux kernel driver and the DPDK driver can be run on a PCI Express root port host PC to interact with the QDMA endpoint IP via PCI Express.

### Getting Started

* [QDMA Reference Drivers Comprehensive documentation](https://xilinx.github.io/dma_ip_drivers/)

## Xilinx-VSEC (XVSEC)

Xilinx-VSEC (XVSEC) are Xilinx supported VSECs. The XVSEC Driver helps creating and deploying designs that may include the Xilinx VSEC PCIe Features.

VSEC (Vendor Specific Extended Capability) is a feature of PCIe.

The VSEC itself is implemented in the PCIe extended capability register in the FPGA hardware (as either soft or hard IP). The drivers and SW are created to interface with and use this hardware implemented feature.

The XVSEC driver currently include the MCAP VSEC, but will be expanded to include the XVC VSEC and NULL VSEC.

### Getting Started

* [XVSEC Linux Kernel Reference Driver Comprehensive documentation](https://xilinx.github.io/dma_ip_drivers/)

### Support

Refer to Xilinx PCIe Forum for any queries/issues/support required w.r.t Xilinx's DMA IP Reference Drivers

Note: Issues are disabled in github for these drivers. All the queries shall be redirected through Xilinx PCIe Forum link given below.

* [Xilinx PCIe Forum](https://forums.xilinx.com/t5/PCIe-and-CPM/bd-p/PCIe)