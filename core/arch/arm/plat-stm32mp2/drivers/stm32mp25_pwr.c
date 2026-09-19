// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) 2022-2026, STMicroelectronics
 * Copyright (c) 2026, Mateusz Nowicki
 */

#include <assert.h>
#include <io.h>
#include <kernel/dt.h>
#include <kernel/dt_driver.h>
#include <kernel/panic.h>
#include <libfdt.h>
#include <mm/core_memprot.h>
#include <platform_config.h>
#include <stm32mp25_pwr.h>
#include <trace.h>

static struct io_pa_va pwr_base = { .pa = PWR_BASE };
static size_t pwr_size = 1;

vaddr_t stm32mp25_pwr_base(void)
{
	return io_pa_or_va_secure(&pwr_base, pwr_size);
}

static TEE_Result stm32mp25_pwr_probe(const void *fdt, int node,
				      const void *compat_data __unused)
{
	struct dt_node_info info = { };
	TEE_Result res = TEE_ERROR_GENERIC;
	int subnode = 0;

	fdt_fill_device_info(fdt, &info, node);
	if (info.reg == DT_INFO_INVALID_REG ||
	    info.reg_size == DT_INFO_INVALID_REG_SIZE)
		return TEE_ERROR_BAD_PARAMETERS;

	assert(info.reg == PWR_BASE);
	pwr_size = info.reg_size;

	fdt_for_each_subnode(subnode, fdt, node) {
		res = dt_driver_maybe_add_probe_node(fdt, subnode);
		if (res) {
			EMSG("Failed on node %s with %#"PRIx32,
			     fdt_get_name(fdt, subnode, NULL), res);
			panic();
		}
	}

	return TEE_SUCCESS;
}

static const struct dt_device_match stm32mp25_pwr_match_table[] = {
	{ .compatible = "st,stm32mp25-pwr" },
	{ }
};

DEFINE_DT_DRIVER(stm32mp25_pwr_dt_driver) = {
	.name = "stm32mp25-pwr",
	.match_table = stm32mp25_pwr_match_table,
	.probe = stm32mp25_pwr_probe,
};
