## Why was this driver created and how is it different?
The mainline Xilinx DMA driver creates descriptors strictly on the boundaries of a page,
thus limiting them to only 4096 bytes. This way it makes very inefficient use of them.

In this driver the DMA routine has been reworked from ground up. It merges contiguous
addresses ranges both in physical and DMA (bus) address spaces into as few descriptors as
possible. This reduces transfer overhead, as larger transfers no longer have to be split
and interrupted to be iteratively submitted to the IP core. It is particulary optimised
for modern CPUs with an IOMMU, that may be able to map the whole data buffer into a
contiguous DMA address range.

Furthermore, handling of the transfer has been drastically slimmed down to bring down the
latency as far as possible. It aspires to implement Figures 5-7 from PG195 in
most efficient manner. Among the improvements:
- Length of the adjacent descriptor blocks gets adapted to the Maximum Request Read
Size (MRRS) as PG195 (p. 24) commands.
- The number of max descriptors per transfer could be adapted to FIFO size and 
number of channels (per p. 26 in PG195). However, due to a bug in the IP Core 
(see Known Issues) a transfer is limited to a single adjacent block. It is still
much larger than in the old driver: typically just under 4 GB, double of that if
the MRRS is 1024 bytes.
- The memory for engines is allocated dynamically, which saves a little bit of 
kernel memory.

In order to achieve these goals the driver has following limitations and for this reason
won't be submitted as PR:
- Removed transfer queues and support for asynchronous I/O. Allegedly it is
broken, and almost noone uses it anyway.
- The backward support for Linux kernels is limited to version 4.12
- The individual engines are each allowed to be opened and operated by a single thread. 
This permits to mostly eliminate locking.

This reworked version includes my PRs (in stable form) as well as relevant PRs by others
(like alonbl's patch set) that doesn't concern XDMA procedure (since it was thrown out in
full anyway).

### Other features
- Reworked poll mode as compile-only option. Everything gets done in a single thread, 
so there are no core migrations or context switches.
- Reworked `ioctl` operations. Added ability to submit transfer request over ioctl. This is
primarily intended to circumvent 2 GB limit for read and write operations in Linux. 
The `xdma_ioctl.h` header is installed system-wide.
- Reworked bypass BAR. It is now properly implemented, so it is possible to transmit 
data on it. It could be useful for small transfers, that require low and stable latency. 
See [bypass BAR](./docs/bypass_bar.md) for more detailed information. It is faster 
(on 64-byte systems significantly) than over user BAR.

Refer to [build help](./xdma/build-help.txt) or run `make help` for available 
configuration options.

Descriptor bypass is NOT supported and won't be until solutions for following obstacles 
are devised:
- There is no way for driver to figure out, if descriptor bypass was selected for a channel
- How the logic on FPGA is supposed to learn the DMA Addresses on the host?

### Known issues
- I have no ability to test on different kernel versions. That is why the driver may fail 
to compile because of typos and syntax errors in the version fenced sections.
- I don't follow the development of Linux kernel. The driver won't be updated for a new 
kernel until an issue is filed. When I have time, I will create a guide with whose help
you could do it yourself
- The IP core has a bug that makes it incapable to process more than 1 adjacent block of 
descriptors (+1 descriptor), when there are merged address ranges (that is length of a
descriptor is >4096 bytes). That is why the transfer is currently limited to a single 
adjacent block of descriptors, which still way larger than the old driver thanks to
merging. The workaround could be to execute transfer block-by-block and could be 
implemented if proven in demand. I will try to raise the issue with AMD.
-  The latency of XDMA may vary due to the kernel API functions that may sleep 
(for example memory allocation with kmalloc), therefore it is **not** RT capable. Consider
to use bypass BAR for such application.
### Changes in behaviour over mainline driver
- The driver strictly enforces appropriate access om DMA devices. For example, it is not 
possible to use `pwrite` on an AXI-Stream interface.
- Non-incremental (fixed) address mode can be also set by O_TRUNC flag while opening a 
MM XDMA device.
- The driver strives to conform to standard behavior of write and read operations, which 
states that in case operation times out or gets interrupted by a signal, it should return 
the amount of read or written bytes, unless no data could be processed. Therefore, it
returns the number of bytes of the completed descriptors. The actual transmitted data is 
likely larger than that, but unfortunately, there is no way for driver to find out exact
amount of transmitted data, therefore the data in the last incomplete descriptor is as good
as lost, unless you have means to differentiate between good and nonsense values.
- In case of timeout, driver checks first, if some progress has been made in terms of 
descriptors and if so it waits a further period, to give a transfer chance to complete.
- Default timeout is reduced to 5 ms.

### Where can programming guides and examples be found?
Check the 
[XDMA Communication tutorial](https://github.com/mwrnd/notes/tree/main/XDMA_Communication)
by mwrnd. For the most part it applies to this version of the driver as well.
## What about the in-tree XDMA driver?
The in-tree driver seem not to have device file interface and is available only since
version 6.3, while we needed the driver for an earlier version. There is some
(independent) overlap in implementation like using DMA pools for descriptors and
completions for waiting, but I am not aware, if that driver performs any merging address 
ranges.