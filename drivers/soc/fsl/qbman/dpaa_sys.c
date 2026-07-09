/* Copyright 2017 NXP Semiconductor, Inc.
 * Copyright 2020 Puresoftware Ltd.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *	 notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *	 notice, this list of conditions and the following disclaimer in the
 *	 documentation and/or other materials provided with the distribution.
 *     * Neither the name of NXP Semiconductor nor the
 *	 names of its contributors may be used to endorse or promote products
 *	 derived from this software without specific prior written permission.
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY NXP Semiconductor ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NXP Semiconductor BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/acpi.h>
#include <linux/dma-map-ops.h>
#include <linux/dma-mapping.h>
#include "dpaa_sys.h"

/* Currently, the Linux kernel port for arm64 performs the memory
 * reservations using memblock (the memory allocator for early boot) only for
 * device tree (arm64_memblock_init() -> early_init_fdt_scan_reserved_mem().
 * Since we cannot use that same mechanism on ACPI, we will perform CMA for the
 * BMan FBPR, QMan FQD and PFDR.
 */
static int qbman_init_private_mem_acpi(struct device *dev, int idx, dma_addr_t *addr,
				       size_t *size, int dev_id)
{
	unsigned long pool_size_order = 0;
	phys_addr_t mem_addr, mem_size;
	struct page *page = NULL;
	size_t page_sz_count = 0;
	int val_cnt;
	u32 val[2];
	int err;

	switch (dev_id) {
	case DPAA_BMAN_DEV:
		val_cnt = 1;
		break;
	case DPAA_QMAN_DEV:
		val_cnt = 2;
		break;
	default:
		return -ENODEV;
	}

	err = device_property_read_u32_array(dev, "size", val, val_cnt);
	if (err < 0)
		return err;

	mem_size = 2 * val[idx];
	page_sz_count = DIV_ROUND_UP(mem_size, PAGE_SIZE);
	pool_size_order = get_order(mem_size);

	page = dma_alloc_from_contiguous(dev, page_sz_count,
					 pool_size_order,
					 false);
	if (!page) {
		pr_info("dma_alloc_from_contiguous failed.\n");
		return -ENOMEM;
	}
	mem_addr = page_to_phys(page);
	mem_addr = ALIGN(mem_addr, mem_size);

	dev_dbg(dev, "%s %s base [%llx] size [%llx]\n",
		dev_id == DPAA_BMAN_DEV ? "BMan" : "QMan",
		dev_id == DPAA_BMAN_DEV ? (idx == 0 ? "FBPR" : "(unknown)") :
		idx == 0 ? "FQD" : idx == 1 ? "PFDR" : "(unknown)", mem_addr,
		mem_size);

	*addr = mem_addr;
	*size = val[idx];

	return 0;
}

/*
 * Initialize a devices private memory region
 */
int qbman_init_private_mem(struct device *dev, int idx, const char *compat,
			   dma_addr_t *addr, size_t *size, int dev_id)
{
	struct device_node *mem_node;
	struct reserved_mem *rmem;
	__be32 *res_array;
	int err;

	if (is_acpi_node(dev->fwnode))
		return qbman_init_private_mem_acpi(dev, idx, addr, size, dev_id);

	mem_node = of_parse_phandle(dev->of_node, "memory-region", idx);
	if (!mem_node) {
		mem_node = of_find_compatible_node(NULL, NULL, compat);
		if (!mem_node) {
			dev_err(dev, "No memory-region found for index %d or compatible '%s'\n",
				idx, compat);
			return -ENODEV;
		}
	}

	rmem = of_reserved_mem_lookup(mem_node);
	if (!rmem) {
		dev_err(dev, "of_reserved_mem_lookup() returned NULL\n");
		return -ENODEV;
	}
	*addr = rmem->base;
	*size = rmem->size;

	/*
	 * Check if the reg property exists - if not insert the node
	 * so upon kexec() the same memory region address will be preserved.
	 * This is needed because QBMan HW does not allow the base address/
	 * size to be modified once set.
	 */
	if (!of_property_present(mem_node, "reg")) {
		struct property *prop;

		prop = devm_kzalloc(dev, sizeof(*prop), GFP_KERNEL);
		if (!prop)
			return -ENOMEM;
		prop->value = res_array = devm_kzalloc(dev, sizeof(__be32) * 4,
						       GFP_KERNEL);
		if (!prop->value)
			return -ENOMEM;
		res_array[0] = cpu_to_be32(upper_32_bits(*addr));
		res_array[1] = cpu_to_be32(lower_32_bits(*addr));
		res_array[2] = cpu_to_be32(upper_32_bits(*size));
		res_array[3] = cpu_to_be32(lower_32_bits(*size));
		prop->length = sizeof(__be32) * 4;
		prop->name = devm_kstrdup(dev, "reg", GFP_KERNEL);
		if (!prop->name)
			return -ENOMEM;
		err = of_add_property(mem_node, prop);
		if (err)
			return err;
	}

	return 0;
}
