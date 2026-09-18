// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) 2026, Mateusz Nowicki
 */

#include <assert.h>
#include <compiler.h>
#include <confine_array_index.h>
#include <drivers/rstctrl.h>
#include <drivers/scmi-msg.h>
#include <drivers/scmi.h>
#include <drivers/stm32mp2_rcc_util.h>
#include <drivers/stm32mp_dt_bindings.h>
#include <initcall.h>
#include <stdint.h>
#include <trace.h>

#define TIMEOUT_US_1MS		1000

struct stm32_scmi_rd {
	unsigned long reset_id;
	const char *name;
	struct rstctrl *rstctrl;
};

#define RESET_CELL(_scmi_id, _id, _name) \
	[(_scmi_id)] = { \
		.reset_id = (_id), \
		.name = (_name), \
	}

/* Same domains as the SCP-firmware config in stm32mp25-st-scmi-cfg.dtsi */
static struct stm32_scmi_rd stm32_scmi_reset_domain[] = {
	RESET_CELL(RST_SCMI_FMC, FMC_R, "fmc"),
	RESET_CELL(RST_SCMI_OSPI1, OSPI1_R, "ospi1"),
	RESET_CELL(RST_SCMI_OSPI1DLL, OSPI1DLL_R, "ospi1_ddl"),
	RESET_CELL(RST_SCMI_OSPI2, OSPI2_R, "ospi2"),
	RESET_CELL(RST_SCMI_OSPI2DLL, OSPI2DLL_R, "ospi2_ddl"),
};

struct channel_resources {
	struct scmi_msg_channel *channel;
	struct stm32_scmi_rd *rd;
	size_t rd_count;
};

static const struct channel_resources scmi_channel[] = {
	[0] = {
		.channel = &(struct scmi_msg_channel){ },
		.rd = stm32_scmi_reset_domain,
		.rd_count = ARRAY_SIZE(stm32_scmi_reset_domain),
	},
};

static const struct channel_resources *find_resource(unsigned int channel_id)
{
	const size_t max_id = ARRAY_SIZE(scmi_channel);
	const unsigned int confined_id = confine_array_index(channel_id,
							     max_id);

	if (channel_id >= max_id)
		return NULL;

	return &scmi_channel[confined_id];
}

struct scmi_msg_channel *plat_scmi_get_channel(unsigned int channel_id)
{
	const struct channel_resources *res = find_resource(channel_id);

	if (!res)
		return NULL;

	return res->channel;
}

static const uint8_t protocol_list[] = {
	SCMI_PROTOCOL_ID_CLOCK,
	SCMI_PROTOCOL_ID_RESET_DOMAIN,
	0,
};

size_t plat_scmi_protocol_count(void)
{
	return ARRAY_SIZE(protocol_list) - 1;
}

const uint8_t *plat_scmi_protocol_list(unsigned int channel_id __unused)
{
	return protocol_list;
}

static const char vendor[] = "ST";
static const char sub_vendor[] = "STM32MP2";

const char *plat_scmi_vendor_name(void)
{
	return vendor;
}

const char *plat_scmi_sub_vendor_name(void)
{
	return sub_vendor;
}

static struct stm32_scmi_rd *find_rd(unsigned int channel_id,
				     unsigned int scmi_id)
{
	const struct channel_resources *res = find_resource(channel_id);
	unsigned int confined_id = 0;

	if (!res || scmi_id >= res->rd_count)
		return NULL;

	confined_id = confine_array_index(scmi_id, res->rd_count);

	if (!res->rd[confined_id].name)
		return NULL;

	return res->rd + confined_id;
}

size_t plat_scmi_rd_count(unsigned int channel_id)
{
	const struct channel_resources *res = find_resource(channel_id);
	const size_t count = res ? res->rd_count : 0;

	return count;
}

const char *plat_scmi_rd_get_name(unsigned int channel_id,
				  unsigned int scmi_id)
{
	const struct stm32_scmi_rd *rd = find_rd(channel_id, scmi_id);
	const char *name = rd ? rd->name : NULL;

	return name;
}

int32_t plat_scmi_rd_autonomous(unsigned int channel_id, unsigned int scmi_id,
				unsigned int state)
{
	const struct stm32_scmi_rd *rd = find_rd(channel_id, scmi_id);
	int32_t status = SCMI_SUCCESS;

	FMSG("SCMI reset %u cycle", scmi_id);

	if (!rd)
		status = SCMI_NOT_FOUND;
	else if (!rd->rstctrl)
		status = SCMI_DENIED;
	else if (state)
		status = SCMI_NOT_SUPPORTED;
	else if (rstctrl_assert_to(rd->rstctrl, TIMEOUT_US_1MS) ||
		 rstctrl_deassert_to(rd->rstctrl, TIMEOUT_US_1MS))
		status = SCMI_HARDWARE_ERROR;

	return status;
}

int32_t plat_scmi_rd_set_state(unsigned int channel_id, unsigned int scmi_id,
			       bool assert_not_deassert)
{
	const struct stm32_scmi_rd *rd = find_rd(channel_id, scmi_id);
	int32_t status = SCMI_SUCCESS;
	TEE_Result res = TEE_SUCCESS;

	FMSG("SCMI reset %u %s", scmi_id,
	     assert_not_deassert ? "set" : "release");

	if (!rd) {
		status = SCMI_NOT_FOUND;
	} else if (!rd->rstctrl) {
		status = SCMI_DENIED;
	} else {
		if (assert_not_deassert)
			res = rstctrl_assert(rd->rstctrl);
		else
			res = rstctrl_deassert(rd->rstctrl);

		if (res)
			status = SCMI_HARDWARE_ERROR;
	}

	return status;
}

static TEE_Result stm32mp2_init_scmi_server(void)
{
	size_t i = 0;
	size_t j = 0;

	for (i = 0; i < ARRAY_SIZE(scmi_channel); i++) {
		const struct channel_resources *res = scmi_channel + i;

		for (j = 0; j < res->rd_count; j++) {
			struct stm32_scmi_rd *rd = res->rd + j;
			struct rstctrl *rstctrl = NULL;
			TEE_Result ret = TEE_SUCCESS;

			if (!rd->name)
				continue;

			rstctrl = stm32mp_rcc_reset_id_to_rstctrl(rd->reset_id);
			assert(rstctrl);

			ret = rstctrl_get_exclusive(rstctrl);
			if (ret) {
				EMSG("scmi: rd %zu %s: get_exclusive %#"PRIx32,
				     j, rd->name, ret);
				continue;
			}

			rd->rstctrl = rstctrl;
		}
	}

	return TEE_SUCCESS;
}

driver_init_late(stm32mp2_init_scmi_server);
