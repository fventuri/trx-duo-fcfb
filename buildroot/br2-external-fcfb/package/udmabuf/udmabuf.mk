################################################################################
#
# udmabuf (ikwzm u-dma-buf)
#
################################################################################

UDMABUF_VERSION = 5.5.0
UDMABUF_SITE = $(call github,ikwzm,udmabuf,v$(UDMABUF_VERSION))
UDMABUF_LICENSE = BSD-2-Clause
UDMABUF_LICENSE_FILES = LICENSE

# u-dma-buf keys its object on CONFIG_U_DMA_BUF and its feature set on the
# U_DMA_BUF_* defines. Buildroot's kernel-module infra invokes the kernel's
# `modules` target directly (obj-$(CONFIG_U_DMA_BUF) += u-dma-buf.o), NOT the
# package Makefile's `all:` target, so pass the config in explicitly or nothing
# is built. U_DMA_BUF_CONFIG=1 enables the driver's compile-time config block.
#
# U_DMA_BUF_QUIRK_MMAP=1 is ESSENTIAL for fcfb: it makes mmap fault in real
# struct pages (vmf_insert_page) instead of remap_pfn_range/vmf_insert_pfn.
# Only page-backed VMAs let get_user_pages pin the buffer, which is what UDP
# MSG_ZEROCOPY needs to DMA straight from the ring. Without it the mmap is
# VM_PFNMAP and zero-copy send fails (the very obstacle we switched off /dev/mem
# to avoid). On ARM this also defaults quirk_mmap_mode to ALWAYS_ON.
UDMABUF_MODULE_MAKE_OPTS = CONFIG_U_DMA_BUF=m U_DMA_BUF_CONFIG=1 U_DMA_BUF_QUIRK_MMAP=1

# CMA is the page-backed allocator u-dma-buf hands out from (fcfb uses a
# dedicated reserved reusable pool in the device tree, referenced by the
# u-dma-buf node's memory-region); DMA_CMA lets the device draw from it. Both
# are already =y in the trx-duo build; assert them so the module never silently
# loses its backing allocator.
define UDMABUF_LINUX_CONFIG_FIXUPS
	$(call KCONFIG_ENABLE_OPT,CONFIG_CMA)
	$(call KCONFIG_ENABLE_OPT,CONFIG_DMA_CMA)
endef

$(eval $(kernel-module))
$(eval $(generic-package))
