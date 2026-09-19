// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) 2026, Mateusz Nowicki
 */

#include <assert.h>
#include <compiler.h>
#include <confine_array_index.h>
#include <drivers/clk.h>
#include <drivers/regulator.h>
#include <drivers/rstctrl.h>
#include <drivers/scmi-msg.h>
#include <drivers/scmi.h>
#include <drivers/stm32_cpu_opp.h>
#include <drivers/stm32mp2_rcc_util.h>
#include <drivers/stm32mp_dt_bindings.h>
#include <initcall.h>
#include <kernel/dt.h>
#include <libfdt.h>
#include <stdint.h>
#include <stdlib.h>
#include <trace.h>

#define TIMEOUT_US_1MS		1000
#define SCMI_VOLTD_MAX_COUNT	64

struct stm32_scmi_clk {
	unsigned long clock_id;
	bool exposed;
};

#define CLOCK_CELL(_scmi_id, _id) \
	[(_scmi_id)] = { \
		.clock_id = (_id), \
		.exposed = true, \
	}

/* Same domains as the SCP-firmware config in stm32mp25-st-scmi-cfg.dtsi */
static const struct stm32_scmi_clk stm32_scmi_clock[] = {
	CLOCK_CELL(CK_SCMI_ICN_HS_MCU, CK_ICN_HS_MCU),
	CLOCK_CELL(CK_SCMI_ICN_SDMMC, CK_ICN_SDMMC),
	CLOCK_CELL(CK_SCMI_ICN_DDR, CK_ICN_DDR),
	CLOCK_CELL(CK_SCMI_ICN_DISPLAY, CK_ICN_DISPLAY),
	CLOCK_CELL(CK_SCMI_ICN_HSL, CK_ICN_HSL),
	CLOCK_CELL(CK_SCMI_ICN_NIC, CK_ICN_NIC),
	CLOCK_CELL(CK_SCMI_ICN_VID, CK_ICN_VID),
	CLOCK_CELL(CK_SCMI_FLEXGEN_07, CK_FLEXGEN_07),
	CLOCK_CELL(CK_SCMI_FLEXGEN_08, CK_FLEXGEN_08),
	CLOCK_CELL(CK_SCMI_FLEXGEN_09, CK_FLEXGEN_09),
	CLOCK_CELL(CK_SCMI_FLEXGEN_10, CK_FLEXGEN_10),
	CLOCK_CELL(CK_SCMI_FLEXGEN_11, CK_FLEXGEN_11),
	CLOCK_CELL(CK_SCMI_FLEXGEN_12, CK_FLEXGEN_12),
	CLOCK_CELL(CK_SCMI_FLEXGEN_13, CK_FLEXGEN_13),
	CLOCK_CELL(CK_SCMI_FLEXGEN_14, CK_FLEXGEN_14),
	CLOCK_CELL(CK_SCMI_FLEXGEN_15, CK_FLEXGEN_15),
	CLOCK_CELL(CK_SCMI_FLEXGEN_16, CK_FLEXGEN_16),
	CLOCK_CELL(CK_SCMI_FLEXGEN_17, CK_FLEXGEN_17),
	CLOCK_CELL(CK_SCMI_FLEXGEN_18, CK_FLEXGEN_18),
	CLOCK_CELL(CK_SCMI_FLEXGEN_19, CK_FLEXGEN_19),
	CLOCK_CELL(CK_SCMI_FLEXGEN_20, CK_FLEXGEN_20),
	CLOCK_CELL(CK_SCMI_FLEXGEN_21, CK_FLEXGEN_21),
	CLOCK_CELL(CK_SCMI_FLEXGEN_22, CK_FLEXGEN_22),
	CLOCK_CELL(CK_SCMI_FLEXGEN_23, CK_FLEXGEN_23),
	CLOCK_CELL(CK_SCMI_FLEXGEN_24, CK_FLEXGEN_24),
	CLOCK_CELL(CK_SCMI_FLEXGEN_25, CK_FLEXGEN_25),
	CLOCK_CELL(CK_SCMI_FLEXGEN_26, CK_FLEXGEN_26),
	CLOCK_CELL(CK_SCMI_FLEXGEN_27, CK_FLEXGEN_27),
	CLOCK_CELL(CK_SCMI_FLEXGEN_28, CK_FLEXGEN_28),
	CLOCK_CELL(CK_SCMI_FLEXGEN_29, CK_FLEXGEN_29),
	CLOCK_CELL(CK_SCMI_FLEXGEN_30, CK_FLEXGEN_30),
	CLOCK_CELL(CK_SCMI_FLEXGEN_31, CK_FLEXGEN_31),
	CLOCK_CELL(CK_SCMI_FLEXGEN_32, CK_FLEXGEN_32),
	CLOCK_CELL(CK_SCMI_FLEXGEN_33, CK_FLEXGEN_33),
	CLOCK_CELL(CK_SCMI_FLEXGEN_34, CK_FLEXGEN_34),
	CLOCK_CELL(CK_SCMI_FLEXGEN_35, CK_FLEXGEN_35),
	CLOCK_CELL(CK_SCMI_FLEXGEN_36, CK_FLEXGEN_36),
	CLOCK_CELL(CK_SCMI_FLEXGEN_37, CK_FLEXGEN_37),
	CLOCK_CELL(CK_SCMI_FLEXGEN_38, CK_FLEXGEN_38),
	CLOCK_CELL(CK_SCMI_FLEXGEN_39, CK_FLEXGEN_39),
	CLOCK_CELL(CK_SCMI_FLEXGEN_40, CK_FLEXGEN_40),
	CLOCK_CELL(CK_SCMI_FLEXGEN_41, CK_FLEXGEN_41),
	CLOCK_CELL(CK_SCMI_FLEXGEN_42, CK_FLEXGEN_42),
	CLOCK_CELL(CK_SCMI_FLEXGEN_43, CK_FLEXGEN_43),
	CLOCK_CELL(CK_SCMI_FLEXGEN_44, CK_FLEXGEN_44),
	CLOCK_CELL(CK_SCMI_FLEXGEN_45, CK_FLEXGEN_45),
	CLOCK_CELL(CK_SCMI_FLEXGEN_46, CK_FLEXGEN_46),
	CLOCK_CELL(CK_SCMI_FLEXGEN_47, CK_FLEXGEN_47),
	CLOCK_CELL(CK_SCMI_FLEXGEN_48, CK_FLEXGEN_48),
	CLOCK_CELL(CK_SCMI_FLEXGEN_49, CK_FLEXGEN_49),
	CLOCK_CELL(CK_SCMI_FLEXGEN_50, CK_FLEXGEN_50),
	CLOCK_CELL(CK_SCMI_FLEXGEN_51, CK_FLEXGEN_51),
	CLOCK_CELL(CK_SCMI_FLEXGEN_52, CK_FLEXGEN_52),
	CLOCK_CELL(CK_SCMI_FLEXGEN_53, CK_FLEXGEN_53),
	CLOCK_CELL(CK_SCMI_FLEXGEN_54, CK_FLEXGEN_54),
	CLOCK_CELL(CK_SCMI_FLEXGEN_55, CK_FLEXGEN_55),
	CLOCK_CELL(CK_SCMI_FLEXGEN_56, CK_FLEXGEN_56),
	CLOCK_CELL(CK_SCMI_FLEXGEN_57, CK_FLEXGEN_57),
	CLOCK_CELL(CK_SCMI_FLEXGEN_58, CK_FLEXGEN_58),
	CLOCK_CELL(CK_SCMI_FLEXGEN_59, CK_FLEXGEN_59),
	CLOCK_CELL(CK_SCMI_FLEXGEN_60, CK_FLEXGEN_60),
	CLOCK_CELL(CK_SCMI_FLEXGEN_61, CK_FLEXGEN_61),
	CLOCK_CELL(CK_SCMI_FLEXGEN_62, CK_FLEXGEN_62),
	CLOCK_CELL(CK_SCMI_FLEXGEN_63, CK_FLEXGEN_63),
	CLOCK_CELL(CK_SCMI_ICN_LS_MCU, CK_ICN_LS_MCU),
	CLOCK_CELL(CK_SCMI_HSE, HSE_CK),
	CLOCK_CELL(CK_SCMI_LSE, LSE_CK),
	CLOCK_CELL(CK_SCMI_HSI, HSI_CK),
	CLOCK_CELL(CK_SCMI_LSI, LSI_CK),
	CLOCK_CELL(CK_SCMI_MSI, MSI_CK),
	CLOCK_CELL(CK_SCMI_HSE_DIV2, HSE_DIV2_CK),
	CLOCK_CELL(CK_SCMI_PLL3, PLL3_CK),
	CLOCK_CELL(CK_SCMI_RTC, CK_BUS_RTC),
	CLOCK_CELL(CK_SCMI_RTCCK, RTC_CK),
	CLOCK_CELL(CK_SCMI_ICN_APB1, CK_ICN_APB1),
	CLOCK_CELL(CK_SCMI_ICN_APB2, CK_ICN_APB2),
	CLOCK_CELL(CK_SCMI_ICN_APB3, CK_ICN_APB3),
	CLOCK_CELL(CK_SCMI_ICN_APB4, CK_ICN_APB4),
	CLOCK_CELL(CK_SCMI_ICN_APBDBG, CK_ICN_APBDBG),
	CLOCK_CELL(CK_SCMI_TIMG1, TIMG1_CK),
	CLOCK_CELL(CK_SCMI_TIMG2, TIMG2_CK),
	CLOCK_CELL(CK_SCMI_BUS_ETR, CK_BUS_ETR),
	CLOCK_CELL(CK_SCMI_FMC, CK_KER_FMC),
	CLOCK_CELL(CK_SCMI_GPIOA, CK_BUS_GPIOA),
	CLOCK_CELL(CK_SCMI_GPIOB, CK_BUS_GPIOB),
	CLOCK_CELL(CK_SCMI_GPIOC, CK_BUS_GPIOC),
	CLOCK_CELL(CK_SCMI_GPIOD, CK_BUS_GPIOD),
	CLOCK_CELL(CK_SCMI_GPIOE, CK_BUS_GPIOE),
	CLOCK_CELL(CK_SCMI_GPIOF, CK_BUS_GPIOF),
	CLOCK_CELL(CK_SCMI_GPIOG, CK_BUS_GPIOG),
	CLOCK_CELL(CK_SCMI_GPIOH, CK_BUS_GPIOH),
	CLOCK_CELL(CK_SCMI_GPIOI, CK_BUS_GPIOI),
	CLOCK_CELL(CK_SCMI_GPIOJ, CK_BUS_GPIOJ),
	CLOCK_CELL(CK_SCMI_GPIOK, CK_BUS_GPIOK),
	CLOCK_CELL(CK_SCMI_GPIOZ, CK_BUS_GPIOZ),
	CLOCK_CELL(CK_SCMI_HPDMA1, CK_BUS_HPDMA1),
	CLOCK_CELL(CK_SCMI_HPDMA2, CK_BUS_HPDMA2),
	CLOCK_CELL(CK_SCMI_HPDMA3, CK_BUS_HPDMA3),
	CLOCK_CELL(CK_SCMI_IPCC1, CK_BUS_IPCC1),
	CLOCK_CELL(CK_SCMI_IPCC2, CK_BUS_IPCC2),
	CLOCK_CELL(CK_SCMI_OSPI1, CK_KER_OSPI1),
	CLOCK_CELL(CK_SCMI_OSPI2, CK_KER_OSPI2),
	CLOCK_CELL(CK_SCMI_TPIU, CK_KER_TPIU),
	CLOCK_CELL(CK_SCMI_SYSDBG, CK_SYSDBG),
	CLOCK_CELL(CK_SCMI_SYSATB, CK_BUS_SYSATB),
	CLOCK_CELL(CK_SCMI_BUS_STM, CK_BUS_STM),
	CLOCK_CELL(CK_SCMI_KER_STM, CK_KER_STM),
	CLOCK_CELL(CK_SCMI_KER_ETR, CK_KER_ETR),
};

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

struct stm32_scmi_voltd {
	const char *name;
	struct regulator *regulator;
	bool enabled;
};

struct stm32_scmi_perfd {
	const char *name;
};

static const struct stm32_scmi_perfd stm32_scmi_perf_domain[] = {
	[0] = { .name = "cpu0" },
};

struct channel_resources {
	struct scmi_msg_channel *channel;
	const char *agent_name;
	const struct stm32_scmi_clk *clock;
	size_t clock_count;
	struct stm32_scmi_rd *rd;
	size_t rd_count;
	struct stm32_scmi_voltd *voltd;
	size_t voltd_count;
	const struct stm32_scmi_perfd *perfd;
	size_t perfd_count;
};

/* Voltage domains are filled from DT at init, see scmi_voltd_init_from_dt() */
static struct channel_resources scmi_channel[] = {
	[0] = {
		.channel = &(struct scmi_msg_channel){ },
		.agent_name = "a35-nsec",
		.clock = stm32_scmi_clock,
		.clock_count = ARRAY_SIZE(stm32_scmi_clock),
		.rd = stm32_scmi_reset_domain,
		.rd_count = ARRAY_SIZE(stm32_scmi_reset_domain),
		.perfd = stm32_scmi_perf_domain,
		.perfd_count = ARRAY_SIZE(stm32_scmi_perf_domain),
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
#ifdef CFG_SCMI_MSG_PERF_DOMAIN
	SCMI_PROTOCOL_ID_PERF,
#endif
	SCMI_PROTOCOL_ID_CLOCK,
	SCMI_PROTOCOL_ID_RESET_DOMAIN,
	SCMI_PROTOCOL_ID_VOLTAGE_DOMAIN,
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

size_t plat_scmi_agent_count(void)
{
	return ARRAY_SIZE(scmi_channel);
}

const char *plat_scmi_agent_name(unsigned int agent_id)
{
	const struct channel_resources *res = find_resource(agent_id);

	return res ? res->agent_name : NULL;
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

static struct stm32_scmi_voltd *find_voltd(unsigned int channel_id,
					   unsigned int scmi_id)
{
	const struct channel_resources *res = find_resource(channel_id);
	unsigned int confined_id = 0;

	if (!res || scmi_id >= res->voltd_count)
		return NULL;

	confined_id = confine_array_index(scmi_id, res->voltd_count);

	if (!res->voltd[confined_id].regulator)
		return NULL;

	return res->voltd + confined_id;
}

size_t plat_scmi_voltd_count(unsigned int channel_id)
{
	const struct channel_resources *res = find_resource(channel_id);
	const size_t count = res ? res->voltd_count : 0;

	return count;
}

const char *plat_scmi_voltd_get_name(unsigned int channel_id,
				     unsigned int scmi_id)
{
	const struct stm32_scmi_voltd *voltd = find_voltd(channel_id, scmi_id);
	const char *name = voltd ? voltd->name : NULL;

	return name;
}

int32_t plat_scmi_voltd_levels_array(unsigned int channel_id,
				     unsigned int scmi_id, size_t start_index,
				     long *out_levels, size_t *nb_elts)
{
	const struct stm32_scmi_voltd *voltd = find_voltd(channel_id, scmi_id);
	struct regulator_voltages_desc *desc = NULL;
	TEE_Result res = TEE_ERROR_GENERIC;
	const int *levels = NULL;
	size_t n = 0;

	if (!voltd)
		return SCMI_NOT_FOUND;

	res = regulator_supported_voltages(voltd->regulator, &desc, &levels);
	if (res == TEE_ERROR_NOT_SUPPORTED)
		return SCMI_NOT_SUPPORTED;
	if (res)
		return SCMI_GENERIC_ERROR;

	/* Triplet min/max/step is served by plat_scmi_voltd_levels_by_step() */
	if (!desc || desc->type != VOLTAGE_TYPE_FULL_LIST)
		return SCMI_NOT_SUPPORTED;

	if (start_index >= desc->num_levels)
		return SCMI_OUT_OF_RANGE;

	if (!*nb_elts) {
		*nb_elts = desc->num_levels - start_index;
		return SCMI_SUCCESS;
	}

	*nb_elts = MIN(*nb_elts, desc->num_levels - start_index);
	for (n = 0; n < *nb_elts; n++)
		out_levels[n] = levels[start_index + n];

	return SCMI_SUCCESS;
}

int32_t plat_scmi_voltd_levels_by_step(unsigned int channel_id,
				       unsigned int scmi_id, long *min_max_step)
{
	const struct stm32_scmi_voltd *voltd = find_voltd(channel_id, scmi_id);
	struct regulator_voltages_desc *desc = NULL;
	TEE_Result res = TEE_ERROR_GENERIC;
	const int *levels = NULL;

	if (!voltd)
		return SCMI_NOT_FOUND;

	res = regulator_supported_voltages(voltd->regulator, &desc, &levels);
	if (res == TEE_ERROR_NOT_SUPPORTED)
		return SCMI_NOT_SUPPORTED;
	if (res)
		return SCMI_GENERIC_ERROR;

	if (!desc || desc->type != VOLTAGE_TYPE_INCREMENT)
		return SCMI_NOT_SUPPORTED;

	min_max_step[0] = levels[0];
	min_max_step[1] = levels[1];
	min_max_step[2] = levels[2];

	return SCMI_SUCCESS;
}

int32_t plat_scmi_voltd_get_level(unsigned int channel_id, unsigned int scmi_id,
				  long *level_uv)
{
	const struct stm32_scmi_voltd *voltd = find_voltd(channel_id, scmi_id);

	if (!voltd)
		return SCMI_NOT_FOUND;

	*level_uv = regulator_get_voltage(voltd->regulator);

	return SCMI_SUCCESS;
}

int32_t plat_scmi_voltd_set_level(unsigned int channel_id, unsigned int scmi_id,
				  long level_uv)
{
	const struct stm32_scmi_voltd *voltd = find_voltd(channel_id, scmi_id);
	int32_t status = SCMI_SUCCESS;

	FMSG("SCMI voltd %u level %ld", scmi_id, level_uv);

	if (!voltd)
		status = SCMI_NOT_FOUND;
	else if (level_uv < INT_MIN || level_uv > INT_MAX)
		status = SCMI_OUT_OF_RANGE;
	else if (regulator_set_voltage(voltd->regulator, level_uv))
		status = SCMI_GENERIC_ERROR;

	return status;
}

int32_t plat_scmi_voltd_get_config(unsigned int channel_id,
				   unsigned int scmi_id, uint32_t *config)
{
	const struct stm32_scmi_voltd *voltd = find_voltd(channel_id, scmi_id);

	if (!voltd)
		return SCMI_NOT_FOUND;

	if (voltd->enabled)
		*config = SCMI_VOLTAGE_DOMAIN_CONFIG_ARCH_ON;
	else
		*config = SCMI_VOLTAGE_DOMAIN_CONFIG_ARCH_OFF;

	return SCMI_SUCCESS;
}

int32_t plat_scmi_voltd_set_config(unsigned int channel_id,
				   unsigned int scmi_id, uint32_t config)
{
	struct stm32_scmi_voltd *voltd = find_voltd(channel_id, scmi_id);
	int32_t status = SCMI_SUCCESS;

	FMSG("SCMI voltd %u config %#"PRIx32, scmi_id, config);

	if (!voltd) {
		status = SCMI_NOT_FOUND;
	} else if (config == SCMI_VOLTAGE_DOMAIN_CONFIG_ARCH_ON) {
		if (!voltd->enabled) {
			if (regulator_enable(voltd->regulator))
				status = SCMI_GENERIC_ERROR;
			else
				voltd->enabled = true;
		}
	} else if (config == SCMI_VOLTAGE_DOMAIN_CONFIG_ARCH_OFF) {
		if (voltd->enabled) {
			if (regulator_disable(voltd->regulator))
				status = SCMI_GENERIC_ERROR;
			else
				voltd->enabled = false;
		}
	} else {
		status = SCMI_INVALID_PARAMETERS;
	}

	return status;
}

#ifdef CFG_SCMI_MSG_PERF_DOMAIN
static const struct stm32_scmi_perfd *find_perfd(unsigned int channel_id,
						 unsigned int domain_id)
{
	const struct channel_resources *res = find_resource(channel_id);
	unsigned int confined_id = 0;

	if (!res || domain_id >= res->perfd_count)
		return NULL;

	confined_id = confine_array_index(domain_id, res->perfd_count);

	return res->perfd + confined_id;
}

size_t plat_scmi_perf_count(unsigned int channel_id)
{
	const struct channel_resources *res = find_resource(channel_id);

	if (!res || !stm32_cpu_opp_count())
		return 0;

	return res->perfd_count;
}

const char *plat_scmi_perf_domain_name(unsigned int channel_id,
				       unsigned int domain_id)
{
	const struct stm32_scmi_perfd *perfd = NULL;

	perfd = find_perfd(channel_id, domain_id);

	return perfd ? perfd->name : NULL;
}

int32_t plat_scmi_perf_sustained_freq(unsigned int channel_id,
				      unsigned int domain_id,
				      unsigned int *freq)
{
	if (!find_perfd(channel_id, domain_id))
		return SCMI_NOT_FOUND;

	*freq = stm32_cpu_opp_sustained_level();

	return SCMI_SUCCESS;
}

int32_t plat_scmi_perf_level_latency(unsigned int channel_id,
				     unsigned int domain_id,
				     unsigned int level __unused,
				     unsigned int *latency)
{
	if (!find_perfd(channel_id, domain_id))
		return SCMI_NOT_FOUND;

	*latency = CFG_STM32MP_OPP_LATENCY_US;

	return SCMI_SUCCESS;
}

static bool cpu_opp_power_known(void)
{
	unsigned int n = 0;

	for (n = 0; n < stm32_cpu_opp_count(); n++)
		if (!stm32_cpu_opp_power_uw(n))
			return false;

	return stm32_cpu_opp_count() != 0;
}

unsigned int plat_scmi_perf_power_scale(unsigned int channel_id)
{
	if (plat_scmi_perf_count(channel_id) && cpu_opp_power_known())
		return SCMI_PERF_POWER_SCALE_UW;

	return SCMI_PERF_POWER_SCALE_ABSTRACT;
}

int32_t plat_scmi_perf_level_power_cost(unsigned int channel_id,
					unsigned int domain_id,
					unsigned int level,
					unsigned int *power_cost)
{
	unsigned int n = 0;

	if (!find_perfd(channel_id, domain_id))
		return SCMI_NOT_FOUND;

	if (!cpu_opp_power_known()) {
		*power_cost = 0;
		return SCMI_SUCCESS;
	}

	for (n = 0; n < stm32_cpu_opp_count(); n++) {
		if (stm32_cpu_opp_level(n) == level) {
			*power_cost = stm32_cpu_opp_power_uw(n);
			return SCMI_SUCCESS;
		}
	}

	return SCMI_NOT_FOUND;
}

int32_t plat_scmi_perf_levels_array(unsigned int channel_id,
				    unsigned int domain_id, size_t start_index,
				    unsigned int *levels, size_t *nb_elts)
{
	size_t full_count = stm32_cpu_opp_count();
	size_t out_count = 0;
	size_t n = 0;

	if (!find_perfd(channel_id, domain_id))
		return SCMI_NOT_FOUND;

	if (SUB_OVERFLOW(full_count, start_index, &out_count))
		return SCMI_OUT_OF_RANGE;

	if (!levels) {
		*nb_elts = out_count;
		return SCMI_SUCCESS;
	}

	out_count = MIN(out_count, *nb_elts);
	for (n = 0; n < out_count; n++)
		levels[n] = stm32_cpu_opp_level(start_index + n);

	*nb_elts = out_count;

	return SCMI_SUCCESS;
}

int32_t plat_scmi_perf_level_get(unsigned int channel_id,
				 unsigned int domain_id, unsigned int *level)
{
	if (!find_perfd(channel_id, domain_id))
		return SCMI_NOT_FOUND;

	if (stm32_cpu_opp_read_level(level))
		return SCMI_GENERIC_ERROR;

	return SCMI_SUCCESS;
}

int32_t plat_scmi_perf_level_set(unsigned int channel_id,
				 unsigned int domain_id, unsigned int level)
{
	FMSG("SCMI perf %u level %u", domain_id, level);

	if (!find_perfd(channel_id, domain_id))
		return SCMI_NOT_FOUND;

	switch (stm32_cpu_opp_set_level(level)) {
	case TEE_SUCCESS:
		return SCMI_SUCCESS;
	case TEE_ERROR_BAD_PARAMETERS:
		return SCMI_OUT_OF_RANGE;
	default:
		return SCMI_GENERIC_ERROR;
	}
}
#endif /* CFG_SCMI_MSG_PERF_DOMAIN */

/*
 * Voltage domains exposed to an agent are listed in a DT node:
 *
 *	scmi-regulators {
 *		compatible = "st,scmi-regulator-consumer";
 *		scmi-channel-id = <0>;
 *		regulator@N {
 *			reg = <N>;
 *			voltd-supply = <&some_regulator>;
 *		};
 *	};
 */
static TEE_Result scmi_voltd_init_from_dt(const void *fdt, int node)
{
	struct channel_resources *res = NULL;
	struct stm32_scmi_voltd *voltd = NULL;
	uint32_t channel_id = 0;
	size_t count = 0;
	int subnode = 0;

	if (fdt_read_uint32(fdt, node, "scmi-channel-id", &channel_id) ||
	    channel_id >= ARRAY_SIZE(scmi_channel)) {
		EMSG("scmi: %s: invalid scmi-channel-id",
		     fdt_get_name(fdt, node, NULL));
		return TEE_ERROR_BAD_PARAMETERS;
	}
	res = scmi_channel + channel_id;

	fdt_for_each_subnode(subnode, fdt, node) {
		paddr_t reg = fdt_reg_base_address(fdt, subnode);

		if (reg == DT_INFO_INVALID_REG || reg >= SCMI_VOLTD_MAX_COUNT)
			return TEE_ERROR_BAD_PARAMETERS;
		count = MAX(count, (size_t)reg + 1);
	}

	if (!count)
		return TEE_SUCCESS;

	voltd = calloc(count, sizeof(*voltd));
	if (!voltd)
		return TEE_ERROR_OUT_OF_MEMORY;

	fdt_for_each_subnode(subnode, fdt, node) {
		size_t id = fdt_reg_base_address(fdt, subnode);
		struct regulator *regulator = NULL;
		TEE_Result ret = TEE_SUCCESS;

		ret = regulator_dt_get_supply(fdt, subnode, "voltd",
					      &regulator);
		if (ret) {
			EMSG("scmi: voltd %zu: no supply (%#" PRIx32 ")", id,
			     ret);
			continue;
		}

		/* Keep boot-on supplies up until the agent takes them over */
		if (regulator_is_always_on(regulator) ||
		    regulator->flags & REGULATOR_BOOT_ON) {
			if (regulator_enable(regulator)) {
				EMSG("scmi: voltd %zu %s: enable failed", id,
				     regulator_name(regulator));
				continue;
			}
			voltd[id].enabled = true;
		}

		voltd[id].regulator = regulator;
		voltd[id].name = regulator_name(regulator);

		DMSG("SCMI voltd %zu: %s %duV%s", id, voltd[id].name,
		     regulator_get_voltage(regulator),
		     voltd[id].enabled ? " enabled" : "");
	}

	res->voltd = voltd;
	res->voltd_count = count;

	return TEE_SUCCESS;
}

static TEE_Result stm32mp2_init_scmi_server(void)
{
	const void *fdt = get_secure_dt();
	int node = -1;
	size_t i = 0;
	size_t j = 0;

	if (fdt) {
		while (true) {
			node = fdt_node_offset_by_compatible(fdt, node,
							     "st,scmi-regulator-consumer");
			if (node < 0)
				break;

			if (scmi_voltd_init_from_dt(fdt, node))
				panic();
		}
	}

	for (i = 0; i < ARRAY_SIZE(scmi_channel); i++) {
		const struct channel_resources *res = scmi_channel + i;

		for (j = 0; j < res->clock_count; j++) {
			const struct stm32_scmi_clk *scmi_clk = res->clock + j;
			struct clk *clk = NULL;
			TEE_Result ret = TEE_SUCCESS;

			if (!scmi_clk->exposed)
				continue;

			clk = stm32mp_rcc_clock_id_to_clk(scmi_clk->clock_id);
			if (!clk) {
				EMSG("scmi: clk %zu: no clock %lu", j,
				     scmi_clk->clock_id);
				continue;
			}

			ret = scmi_clk_add(clk, i, j);
			if (ret) {
				EMSG("scmi: clk %zu %s: scmi_clk_add %#"PRIx32,
				     j, clk_get_name(clk), ret);
				continue;
			}
		}

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
