// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) 2022-2026, STMicroelectronics
 * Copyright (c) 2026, Mateusz Nowicki
 */

#include <assert.h>
#include <compiler.h>
#include <drivers/regulator.h>
#include <io.h>
#include <kernel/delay.h>
#include <kernel/dt.h>
#include <kernel/dt_driver.h>
#include <kernel/panic.h>
#include <kernel/pm.h>
#include <libfdt.h>
#include <stm32_sysconf.h>
#include <stm32mp25_pwr.h>
#include <stm32mp_pm.h>
#include <stdio.h>
#include <string.h>
#include <trace.h>

#define PWR_CR1			U(0x00)
#define PWR_CR7			U(0x18)
#define PWR_CR8			U(0x1c)

#define PWR_CR1_VDDIO3VMEN	BIT(0)
#define PWR_CR1_VDDIO4VMEN	BIT(1)
#define PWR_CR1_UCPDVMEN	BIT(3)
#define PWR_CR1_AVMEN		BIT(4)
#define PWR_CR1_VDDIO3SV	BIT(8)
#define PWR_CR1_VDDIO4SV	BIT(9)
#define PWR_CR1_UCPDSV		BIT(11)
#define PWR_CR1_ASV		BIT(12)
#define PWR_CR1_VDDIO3RDY	BIT(16)
#define PWR_CR1_VDDIO4RDY	BIT(17)
#define PWR_CR1_UCPDRDY		BIT(19)
#define PWR_CR1_ARDY		BIT(20)
#define PWR_CR1_VDDIOVRSEL	BIT(24)
#define PWR_CR1_VDDIO3VRSEL	BIT(25)
#define PWR_CR1_VDDIO4VRSEL	BIT(26)

#define PWR_CR7_VDDIO2VMEN	BIT(0)
#define PWR_CR7_VDDIO2SV	BIT(8)
#define PWR_CR7_VDDIO2RDY	BIT(16)
#define PWR_CR7_VDDIO2VRSEL	BIT(24)

#define PWR_CR8_VDDIO1VMEN	BIT(0)
#define PWR_CR8_VDDIO1SV	BIT(8)
#define PWR_CR8_VDDIO1RDY	BIT(16)
#define PWR_CR8_VDDIO1VRSEL	BIT(24)

#define TIMEOUT_US_10MS		U(10000)
#define VRSEL_SETTLE_US		U(2000)

/* Below this supply level the IOs must use the 1.8V range (VRSEL set) */
#define IO_VOLTAGE_THRESHOLD_UV	2700000

#define IOCOMP_CODE_MAX		U(2)

struct pwr_regu {
	uint32_t enable_reg;
	uint32_t enable_mask;
	uint32_t ready_mask;
	uint32_t valid_mask;
	uint32_t vrsel_mask;
	enum syscfg_io_ids comp_idx;
	int suspend_uv;
	bool suspend_state;
	bool is_an_iod;
	bool vrsel_locked;
	bool iocomp_fixed;
	uint32_t iocomp_code[IOCOMP_CODE_MAX];
};

static TEE_Result pwr_set_low_volt(struct regulator *regulator, bool state)
{
	struct pwr_regu *pwr_regu = regulator->priv;
	vaddr_t reg = stm32mp25_pwr_base() + pwr_regu->enable_reg;

	if (!pwr_regu->vrsel_mask)
		return TEE_SUCCESS;

	DMSG("%s: %s voltage range", regulator_name(regulator),
	     state ? "1.8V" : "3.3V");

	if (!state) {
		io_clrbits32(reg, pwr_regu->vrsel_mask);
		return TEE_SUCCESS;
	}

	if (pwr_regu->vrsel_locked)
		return TEE_SUCCESS;

	io_setbits32(reg, pwr_regu->vrsel_mask);
	/* Write is locked unless the HSLV OTP bit is fused, IOs run degraded */
	if (!(io_read32(reg) & pwr_regu->vrsel_mask)) {
		IMSG("%s: VRSEL locked by HSLV OTP, 1.8V IOs run degraded",
		     regulator_name(regulator));
		pwr_regu->vrsel_locked = true;
	}

	return TEE_SUCCESS;
}

static TEE_Result pwr_update_low_volt(struct regulator *regulator)
{
	struct pwr_regu *pwr_regu = regulator->priv;
	int level_uv = 0;

	if (!pwr_regu->is_an_iod)
		return TEE_SUCCESS;

	level_uv = regulator_get_voltage(regulator->supply);

	return pwr_set_low_volt(regulator, level_uv < IO_VOLTAGE_THRESHOLD_UV);
}

static TEE_Result pwr_iocomp_enable(struct pwr_regu *pwr_regu)
{
	if (!pwr_regu->is_an_iod || pwr_regu->iocomp_fixed)
		return TEE_SUCCESS;

	return stm32mp25_syscfg_enable_iocomp(pwr_regu->comp_idx);
}

static TEE_Result pwr_iocomp_disable(struct pwr_regu *pwr_regu)
{
	if (!pwr_regu->is_an_iod || pwr_regu->iocomp_fixed)
		return TEE_SUCCESS;

	return stm32mp25_syscfg_disable_iocomp(pwr_regu->comp_idx);
}

static void pwr_iocomp_fixed(struct pwr_regu *pwr_regu)
{
	if (!pwr_regu->is_an_iod || !pwr_regu->iocomp_fixed)
		return;

	stm32mp25_syscfg_fixed_iocomp(pwr_regu->comp_idx,
				      pwr_regu->iocomp_code[0],
				      pwr_regu->iocomp_code[1]);
}

static TEE_Result pwr_enable_reg(struct pwr_regu *pwr_regu)
{
	vaddr_t reg = stm32mp25_pwr_base() + pwr_regu->enable_reg;
	uint64_t to = 0;

	if (!pwr_regu->enable_mask)
		return TEE_SUCCESS;

	io_setbits32(reg, pwr_regu->enable_mask);

	to = timeout_init_us(TIMEOUT_US_10MS);
	while (!timeout_elapsed(to))
		if (io_read32(reg) & pwr_regu->ready_mask)
			break;

	if (!(io_read32(reg) & pwr_regu->ready_mask)) {
		io_clrbits32(reg, pwr_regu->enable_mask);
		return TEE_ERROR_GENERIC;
	}

	io_setbits32(reg, pwr_regu->valid_mask);

	/* Monitor only checks the supply, disable it to save power */
	io_clrbits32(reg, pwr_regu->enable_mask);

	return TEE_SUCCESS;
}

static void pwr_disable_reg(struct pwr_regu *pwr_regu)
{
	vaddr_t reg = stm32mp25_pwr_base() + pwr_regu->enable_reg;

	if (!pwr_regu->enable_mask)
		return;

	dsb();

	io_clrbits32(reg, pwr_regu->valid_mask);
	io_clrbits32(reg, pwr_regu->enable_mask);
}

static TEE_Result pwr_set_state(struct regulator *regulator, bool enable)
{
	struct pwr_regu *pwr_regu = regulator->priv;
	TEE_Result res = TEE_ERROR_GENERIC;

	FMSG("%s: set state %u", regulator_name(regulator), enable);

	if (enable) {
		/* RM0457: select the voltage range before removing isolation */
		res = pwr_update_low_volt(regulator);
		if (res)
			return res;

		res = pwr_enable_reg(pwr_regu);
		if (res)
			return res;

		res = pwr_iocomp_enable(pwr_regu);
		if (res) {
			pwr_disable_reg(pwr_regu);
			return res;
		}
	} else {
		res = pwr_iocomp_disable(pwr_regu);
		if (res)
			return res;

		pwr_disable_reg(pwr_regu);
	}

	return TEE_SUCCESS;
}

static TEE_Result pwr_get_state(struct regulator *regulator, bool *enabled)
{
	struct pwr_regu *pwr_regu = regulator->priv;
	vaddr_t reg = stm32mp25_pwr_base() + pwr_regu->enable_reg;

	if (pwr_regu->enable_mask)
		*enabled = io_read32(reg) & pwr_regu->valid_mask;
	else
		*enabled = true;

	return TEE_SUCCESS;
}

static TEE_Result pwr_get_voltage(struct regulator *regulator, int *level_uv)
{
	*level_uv = regulator_get_voltage(regulator->supply);

	return TEE_SUCCESS;
}

static TEE_Result pwr_set_voltage(struct regulator *regulator, int level_uv)
{
	TEE_Result res = TEE_ERROR_GENERIC;
	TEE_Result result = TEE_ERROR_GENERIC;
	bool is_enabled = false;

	DMSG("%s: set volt to %d uV", regulator_name(regulator), level_uv);

	res = pwr_get_state(regulator, &is_enabled);
	if (res)
		return res;

	/* Isolate IOs and disable IOs compensation */
	if (is_enabled) {
		res = pwr_set_state(regulator, false);
		if (res)
			return res;
	}

	/* Leaving the 1.8V range with VRSEL set damages the IOs */
	if (level_uv >= IO_VOLTAGE_THRESHOLD_UV) {
		res = pwr_set_low_volt(regulator, false);
		if (res)
			return res;
	}

	result = regulator_set_voltage(regulator->supply, level_uv);
	if (result) {
		EMSG("%s: supply set voltage failed: %#"PRIx32,
		     regulator_name(regulator), result);
		/* Continue to restore IOs setting for current voltage */
		level_uv = regulator_get_voltage(regulator->supply);
	}

	if (level_uv < IO_VOLTAGE_THRESHOLD_UV) {
		/* Let the supply discharge below the 3.3V range first */
		udelay(VRSEL_SETTLE_US);
		res = pwr_set_low_volt(regulator, true);
		if (res)
			return res;
	}

	if (is_enabled) {
		res = pwr_set_state(regulator, true);
		if (res)
			return res;
	}

	dsb();

	return result;
}

static TEE_Result pwr_supported_voltages(struct regulator *regulator,
					 struct regulator_voltages_desc **desc,
					 const int **levels)
{
	return regulator_supported_voltages(regulator->supply, desc, levels);
}

/* Leave the 1.8V range before suspend to protect the IOs */
static TEE_Result pwr_regu_pm(enum pm_op op, unsigned int pm_hint,
			      const struct pm_callback_handle *hdl)
{
	struct regulator *regulator = hdl->handle;
	struct pwr_regu *pwr_regu = regulator->priv;
	TEE_Result res = TEE_ERROR_GENERIC;

	assert(op == PM_OP_SUSPEND || op == PM_OP_RESUME);

	/* Regulators are not changed by Stop modes */
	if (PM_HINT_PLATFORM_STATE(pm_hint) < PM_D1_LPLV_LEVEL)
		return TEE_SUCCESS;

	if (op == PM_OP_SUSPEND) {
		res = pwr_get_state(regulator, &pwr_regu->suspend_state);
		if (res)
			return res;

		if (pwr_regu->is_an_iod) {
			res = pwr_get_voltage(regulator, &pwr_regu->suspend_uv);
			if (res)
				return res;

			res = pwr_set_low_volt(regulator, false);
			if (res)
				return res;

			res = pwr_iocomp_disable(pwr_regu);
			if (res)
				return res;
		}
	} else {
		if (pwr_regu->is_an_iod) {
			pwr_iocomp_fixed(pwr_regu);

			res = pwr_set_voltage(regulator, pwr_regu->suspend_uv);
			if (res)
				return res;
		}

		res = pwr_set_state(regulator, pwr_regu->suspend_state);
		if (res)
			return res;
	}

	return TEE_SUCCESS;
}
DECLARE_KEEP_PAGER(pwr_regu_pm);

static TEE_Result pwr_supplied_init(struct regulator *regulator,
				    const void *fdt __unused, int node __unused)
{
	struct pwr_regu *pwr_regu = regulator->priv;
	TEE_Result res = TEE_ERROR_GENERIC;

	pwr_iocomp_fixed(pwr_regu);

	res = pwr_update_low_volt(regulator);
	if (res)
		return res;

	register_pm_driver_cb(pwr_regu_pm, regulator, "pwr-regu");

	return TEE_SUCCESS;
}

static const struct regulator_ops pwr_regu_ops = {
	.set_state = pwr_set_state,
	.get_state = pwr_get_state,
	.set_voltage = pwr_set_voltage,
	.get_voltage = pwr_get_voltage,
	.supported_voltages = pwr_supported_voltages,
	.supplied_init = pwr_supplied_init,
};
DECLARE_KEEP_PAGER(pwr_regu_ops);

static const struct regulator_ops pwr_regu_fixed_ops = {
	.set_state = pwr_set_state,
	.get_state = pwr_get_state,
	.supplied_init = pwr_supplied_init,
};
DECLARE_KEEP_PAGER(pwr_regu_fixed_ops);

enum pwr_regulator {
	IOD_VDDIO1,
	IOD_VDDIO2,
	IOD_VDDIO3,
	IOD_VDDIO4,
	IOD_VDDIO,
	REGU_UCPD,
	REGU_A,
	PWR_REGU_COUNT
};

static struct pwr_regu pwr_regulators[PWR_REGU_COUNT] = {
	[IOD_VDDIO1] = {
		.enable_reg = PWR_CR8,
		.enable_mask = PWR_CR8_VDDIO1VMEN,
		.ready_mask = PWR_CR8_VDDIO1RDY,
		.valid_mask = PWR_CR8_VDDIO1SV,
		.vrsel_mask = PWR_CR8_VDDIO1VRSEL,
		.is_an_iod = true,
		.comp_idx = SYSCFG_VDDIO1_ID,
	},
	[IOD_VDDIO2] = {
		.enable_reg = PWR_CR7,
		.enable_mask = PWR_CR7_VDDIO2VMEN,
		.ready_mask = PWR_CR7_VDDIO2RDY,
		.valid_mask = PWR_CR7_VDDIO2SV,
		.vrsel_mask = PWR_CR7_VDDIO2VRSEL,
		.is_an_iod = true,
		.comp_idx = SYSCFG_VDDIO2_ID,
	},
	[IOD_VDDIO3] = {
		.enable_reg = PWR_CR1,
		.enable_mask = PWR_CR1_VDDIO3VMEN,
		.ready_mask = PWR_CR1_VDDIO3RDY,
		.valid_mask = PWR_CR1_VDDIO3SV,
		.vrsel_mask = PWR_CR1_VDDIO3VRSEL,
		.is_an_iod = true,
		.comp_idx = SYSCFG_VDDIO3_ID,
	},
	[IOD_VDDIO4] = {
		.enable_reg = PWR_CR1,
		.enable_mask = PWR_CR1_VDDIO4VMEN,
		.ready_mask = PWR_CR1_VDDIO4RDY,
		.valid_mask = PWR_CR1_VDDIO4SV,
		.vrsel_mask = PWR_CR1_VDDIO4VRSEL,
		.is_an_iod = true,
		.comp_idx = SYSCFG_VDDIO4_ID,
	},
	/* VDD IOs have no monitor nor isolation, only the range selection */
	[IOD_VDDIO] = {
		.enable_reg = PWR_CR1,
		.vrsel_mask = PWR_CR1_VDDIOVRSEL,
		.is_an_iod = true,
		.comp_idx = SYSCFG_VDD_IO_ID,
	},
	[REGU_UCPD] = {
		.enable_reg = PWR_CR1,
		.enable_mask = PWR_CR1_UCPDVMEN,
		.ready_mask = PWR_CR1_UCPDRDY,
		.valid_mask = PWR_CR1_UCPDSV,
	},
	[REGU_A] = {
		.enable_reg = PWR_CR1,
		.enable_mask = PWR_CR1_AVMEN,
		.ready_mask = PWR_CR1_ARDY,
		.valid_mask = PWR_CR1_ASV,
	},
};

static struct regulator pwr_regu_device[PWR_REGU_COUNT];

#define DEFINE_REGUL(_id, _name, _supply, _ops) {	\
		.name = (_name),			\
		.supply_name = (_supply),		\
		.ops = (_ops),				\
		.priv = pwr_regulators + (_id),		\
		.regulator = pwr_regu_device + (_id),	\
	}

static const struct regu_dt_desc pwr_regu_desc[PWR_REGU_COUNT] = {
	[IOD_VDDIO1] = DEFINE_REGUL(IOD_VDDIO1, "vddio1", "vddio1",
				    &pwr_regu_ops),
	[IOD_VDDIO2] = DEFINE_REGUL(IOD_VDDIO2, "vddio2", "vddio2",
				    &pwr_regu_ops),
	[IOD_VDDIO3] = DEFINE_REGUL(IOD_VDDIO3, "vddio3", "vddio3",
				    &pwr_regu_ops),
	[IOD_VDDIO4] = DEFINE_REGUL(IOD_VDDIO4, "vddio4", "vddio4",
				    &pwr_regu_ops),
	[IOD_VDDIO] = DEFINE_REGUL(IOD_VDDIO, "vddio", "vdd", &pwr_regu_ops),
	[REGU_UCPD] = DEFINE_REGUL(REGU_UCPD, "vdd33ucpd", "vdd33ucpd",
				   &pwr_regu_fixed_ops),
	[REGU_A] = DEFINE_REGUL(REGU_A, "vdda18adc", "vdda18adc",
				&pwr_regu_fixed_ops),
};
DECLARE_KEEP_PAGER(pwr_regu_desc);

static TEE_Result pwr_regulator_probe(const void *fdt, int node,
				      const void *compat_data __unused)
{
	const struct regu_dt_desc *desc = NULL;
	const char *regu_name = fdt_get_name(fdt, node, NULL);
	struct pwr_regu *pwr_regu = NULL;
	TEE_Result res = TEE_ERROR_GENERIC;
	char prop[32] = { };
	size_t i = 0;

	for (i = 0; i < PWR_REGU_COUNT; i++) {
		if (!strcmp(pwr_regu_desc[i].name, regu_name)) {
			desc = pwr_regu_desc + i;
			break;
		}
	}
	if (!desc) {
		EMSG("No regulator found for node %s", regu_name);
		return TEE_ERROR_GENERIC;
	}

	snprintf(prop, sizeof(prop), "%s-supply", desc->supply_name);
	if (!fdt_getprop(fdt, node, prop, NULL)) {
		EMSG("Regulator %s has no %s", regu_name, prop);
		return TEE_ERROR_BAD_PARAMETERS;
	}

	pwr_regu = desc->priv;
	if (pwr_regu->is_an_iod) {
		int ret = fdt_read_uint32_array(fdt, node, "st,iocomp",
						pwr_regu->iocomp_code,
						IOCOMP_CODE_MAX);

		if (ret && ret != -FDT_ERR_NOTFOUND)
			return TEE_ERROR_BAD_PARAMETERS;

		pwr_regu->iocomp_fixed = !ret;
	}

	res = regulator_dt_register(fdt, node, node, desc);
	if (res)
		EMSG("Failed to register node %s: %#"PRIx32, regu_name, res);

	return res;
}

static const struct dt_device_match pwr_regulator_match_table[] = {
	{ .compatible = "st,stm32mp25-pwr-regu" },
	{ }
};

DEFINE_DT_DRIVER(stm32mp25_pwr_regulator_dt_driver) = {
	.name = "stm32mp25-pwr-regulator",
	.type = DT_DRIVER_REGULATOR,
	.match_table = pwr_regulator_match_table,
	.probe = pwr_regulator_probe,
};
