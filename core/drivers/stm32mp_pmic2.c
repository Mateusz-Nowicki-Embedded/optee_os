// SPDX-License-Identifier: BSD-3-Clause
/*
 * Copyright (c) 2022-2024, STMicroelectronics
 * Copyright (c) 2026, Mateusz Nowicki
 */

#include <drivers/i2c.h>
#include <drivers/regulator.h>
#include <drivers/stm32_i2c.h>
#include <drivers/stpmic2.h>
#include <keep.h>
#include <kernel/dt.h>
#include <kernel/dt_driver.h>
#include <kernel/mutex.h>
#include <kernel/panic.h>
#include <kernel/thread.h>
#include <libfdt.h>
#include <stdlib.h>
#include <string.h>
#include <trace.h>
#include <util.h>

#define PMIC_I2C_TRIALS			U(1)
#define PMIC_I2C_TIMEOUT_BUSY_MS	U(5)

/* DS14278 SRBCK: DVS slew rate min 1 mV/us for BUCK1/2/3/6 */
#define PMIC_RAMP_DELAY_UV_PER_US	U(1000)
#define PMIC_ENABLE_RAMP_DELAY_US	U(1000)

struct pmic_regu {
	struct stpmic2 *pmic;
	uint8_t id;
	int bypass_uv;
	struct regulator_voltages_desc levels_desc;
	int *levels;
};

struct pmic_compat_data {
	const struct regu_dt_desc *desc_table;
	size_t desc_len;
	uint8_t ref_id;
};

struct regu_dt_property {
	const char *name;
	enum stpmic2_prop_id prop;
};

static const struct regu_dt_property prop_table[] = {
	{ .name = "st,mask-reset", .prop = STPMIC2_MASK_RESET },
	{ .name = "st,pwrctrl-enable", .prop = STPMIC2_PWRCTRL_EN },
	{ .name = "st,pwrctrl-reset", .prop = STPMIC2_PWRCTRL_RS },
	{ .name = "st,pwrctrl-sel", .prop = STPMIC2_PWRCTRL_SEL },
	{
		.name = "st,alternate-input-source",
		.prop = STPMIC2_ALTERNATE_INPUT_SOURCE,
	},
};

static struct mutex pmic_mu = MUTEX_INITIALIZER;

static void lock_pmic_access(void)
{
	if (thread_get_id_may_fail() != THREAD_ID_INVALID)
		mutex_lock(&pmic_mu);
}

static void unlock_pmic_access(void)
{
	if (thread_get_id_may_fail() != THREAD_ID_INVALID)
		mutex_unlock(&pmic_mu);
}

static TEE_Result pmic_set_state(struct regulator *regulator, bool enable)
{
	struct pmic_regu *regu = regulator->priv;
	TEE_Result res = TEE_ERROR_GENERIC;

	FMSG("%s: set state to %u", regulator_name(regulator), enable);

	lock_pmic_access();
	res = stpmic2_regulator_set_state(regu->pmic, regu->id, enable);
	unlock_pmic_access();

	return res;
}

static TEE_Result pmic_get_state(struct regulator *regulator, bool *enabled)
{
	struct pmic_regu *regu = regulator->priv;
	TEE_Result res = TEE_ERROR_GENERIC;

	lock_pmic_access();
	res = stpmic2_regulator_get_state(regu->pmic, regu->id, enabled);
	unlock_pmic_access();

	return res;
}

static TEE_Result pmic_get_voltage(struct regulator *regulator, int *level_uv)
{
	struct pmic_regu *regu = regulator->priv;
	TEE_Result res = TEE_ERROR_GENERIC;
	uint16_t val = 0;

	lock_pmic_access();

	if (regu->bypass_uv) {
		uint8_t arg = 0;

		res = stpmic2_regulator_get_prop(regu->pmic, regu->id,
						 STPMIC2_BYPASS, &arg);
		if (res)
			goto out;

		if (arg == PROP_BYPASS_SET) {
			*level_uv = regu->bypass_uv;
			goto out;
		}
	}

	res = stpmic2_regulator_get_voltage(regu->pmic, regu->id, &val);
	if (res)
		goto out;

	*level_uv = (int)val * 1000;
out:
	unlock_pmic_access();

	return res;
}

static TEE_Result pmic_set_voltage(struct regulator *regulator, int level_uv)
{
	struct pmic_regu *regu = regulator->priv;
	TEE_Result res = TEE_ERROR_GENERIC;

	FMSG("%s: set volt to %d uV", regulator_name(regulator), level_uv);

	lock_pmic_access();

	if (regu->bypass_uv && level_uv == regu->bypass_uv) {
		res = stpmic2_regulator_set_prop(regu->pmic, regu->id,
						 STPMIC2_BYPASS,
						 PROP_BYPASS_SET);
		goto out;
	}

	res = stpmic2_regulator_set_voltage(regu->pmic, regu->id,
					    level_uv / 1000);
	if (res)
		goto out;

	if (regu->bypass_uv)
		res = stpmic2_regulator_set_prop(regu->pmic, regu->id,
						 STPMIC2_BYPASS,
						 PROP_BYPASS_RESET);
out:
	unlock_pmic_access();

	return res;
}

/* Sort, remove duplicates and clip to [min_uv, max_uv], return new count */
static size_t refine_levels_array(size_t count, int *levels_uv,
				  int min_uv, int max_uv)
{
	size_t n = 0;
	size_t m = 0;

	qsort_int(levels_uv, count);

	for (n = 1; n < count; n++) {
		if (levels_uv[m] != levels_uv[n]) {
			if (m + 1 != n)
				levels_uv[m + 1] = levels_uv[n];
			m++;
		}
	}
	count = m + 1;

	for (n = count; n; n--)
		if (levels_uv[n - 1] <= max_uv)
			break;
	count = n;

	for (n = 0; n < count; n++)
		if (levels_uv[n] >= min_uv)
			break;
	count -= n;

	memmove(levels_uv, levels_uv + n, count * sizeof(*levels_uv));

	return count;
}

static TEE_Result pmic_list_voltages(struct regulator *regulator,
				     struct regulator_voltages_desc **out_desc,
				     const int **out_levels)
{
	struct pmic_regu *regu = regulator->priv;

	if (!regu->levels) {
		TEE_Result res = TEE_ERROR_GENERIC;
		const uint16_t *level_ref = NULL;
		size_t level_count = 0;
		size_t count_ref = 0;
		int *levels2 = NULL;
		int *levels = NULL;
		size_t n = 0;

		res = stpmic2_regulator_levels_mv(regu->pmic, regu->id,
						  &level_ref, &count_ref);
		if (res)
			return res;

		level_count = count_ref;
		if (regu->bypass_uv)
			level_count++;

		levels = calloc(level_count, sizeof(*levels));
		if (!levels)
			return TEE_ERROR_OUT_OF_MEMORY;

		for (n = 0; n < count_ref; n++)
			levels[n] = level_ref[n] * 1000;

		if (regu->bypass_uv)
			levels[n] = regu->bypass_uv;

		level_count = refine_levels_array(level_count, levels,
						  regulator->min_uv,
						  regulator->max_uv);

		levels2 = realloc(levels, sizeof(*levels) * level_count);
		if (!levels2) {
			free(levels);
			return TEE_ERROR_OUT_OF_MEMORY;
		}

		regu->levels_desc.type = VOLTAGE_TYPE_FULL_LIST;
		regu->levels_desc.num_levels = level_count;
		regu->levels = levels2;
	}

	*out_desc = &regu->levels_desc;
	*out_levels = regu->levels;

	return TEE_SUCCESS;
}

static TEE_Result pmic_supplied_init(struct regulator *regulator,
				     const void *fdt, int node)
{
	struct pmic_regu *regu = regulator->priv;
	const struct regu_dt_property *p = NULL;
	TEE_Result res = TEE_ERROR_GENERIC;

	if (!regulator->ramp_delay_uv_per_us)
		regulator->ramp_delay_uv_per_us = PMIC_RAMP_DELAY_UV_PER_US;
	if (!regulator->enable_ramp_delay_us)
		regulator->enable_ramp_delay_us = PMIC_ENABLE_RAMP_DELAY_US;

	if (regulator->flags & REGULATOR_PULL_DOWN) {
		res = stpmic2_regulator_set_prop(regu->pmic, regu->id,
						 STPMIC2_PULL_DOWN, 0);
		if (res)
			return res;
	}

	if (regulator->flags & REGULATOR_OVER_CURRENT) {
		res = stpmic2_regulator_set_prop(regu->pmic, regu->id,
						 STPMIC2_OCP, 0);
		if (res)
			return res;
	}

	for (p = prop_table; p < (prop_table + ARRAY_SIZE(prop_table)); p++) {
		const fdt32_t *cuint = NULL;
		uint32_t value = 0;
		int len = 0;

		cuint = fdt_getprop(fdt, node, p->name, &len);
		if (!cuint)
			continue;

		if (len == sizeof(uint32_t))
			value = fdt32_to_cpu(*cuint);

		res = stpmic2_regulator_set_prop(regu->pmic, regu->id,
						 p->prop, value);
		if (res)
			return res;
	}

	return TEE_SUCCESS;
}

static const struct regulator_ops pmic2_regu_ops = {
	.set_state = pmic_set_state,
	.get_state = pmic_get_state,
	.set_voltage = pmic_set_voltage,
	.get_voltage = pmic_get_voltage,
	.supported_voltages = pmic_list_voltages,
	.supplied_init = pmic_supplied_init,
};
DECLARE_KEEP_PAGER(pmic2_regu_ops);

static const struct regulator_ops pmic2_switch_ops = {
	.set_state = pmic_set_state,
	.get_state = pmic_get_state,
	.supplied_init = pmic_supplied_init,
};
DECLARE_KEEP_PAGER(pmic2_switch_ops);

static TEE_Result register_pmic_regulator(const void *fdt, struct stpmic2 *pmic,
					  int node, int regulators_node)
{
	const struct pmic_compat_data *cdata = pmic->compat_data;
	const char *regu_name = fdt_get_name(fdt, node, NULL);
	const fdt32_t *cuint = NULL;
	struct regu_dt_desc desc = { };
	struct pmic_regu *regu = NULL;
	TEE_Result res = TEE_ERROR_GENERIC;
	size_t index = 0;
	uint8_t id = 0;
	int len = 0;

	for (index = 0; index < cdata->desc_len; index++)
		if (!strcmp(cdata->desc_table[index].name, regu_name))
			break;
	if (index == cdata->desc_len) {
		EMSG("Unknown regulator %s", regu_name);
		return TEE_ERROR_BAD_PARAMETERS;
	}

	desc = cdata->desc_table[index];

	res = stpmic2_regulator_get_id(regu_name, &id);
	if (res)
		return res;

	regu = calloc(1, sizeof(*regu));
	if (!regu)
		return TEE_ERROR_OUT_OF_MEMORY;

	regu->pmic = pmic;
	regu->id = id;

	cuint = fdt_getprop(fdt, node, "st,regulator-bypass-microvolt", &len);
	if (cuint && len == sizeof(uint32_t))
		regu->bypass_uv = fdt32_to_cpu(*cuint);

	desc.priv = regu;

	res = regulator_dt_register(fdt, node, regulators_node, &desc);
	if (res) {
		EMSG("Failed to register %s: %#"PRIx32, regu_name, res);
		free(regu);
	}

	return res;
}

static TEE_Result parse_regulator_fdt_nodes(const void *fdt, int node,
					    struct stpmic2 *pmic)
{
	int regulators_node = fdt_subnode_offset(fdt, node, "regulators");
	int regu_node = 0;

	if (regulators_node < 0)
		return TEE_ERROR_BAD_PARAMETERS;

	fdt_for_each_subnode(regu_node, fdt, regulators_node) {
		TEE_Result res = TEE_ERROR_GENERIC;

		if (fdt_get_status(fdt, regu_node) == DT_STATUS_DISABLED)
			continue;

		res = register_pmic_regulator(fdt, pmic, regu_node,
					      regulators_node);
		if (res)
			return res;
	}

	return TEE_SUCCESS;
}

static TEE_Result initialize_pmic2(const void *fdt, int node,
				   struct stpmic2 *pmic,
				   struct i2c_handle_s *i2c)
{
	const struct pmic_compat_data *cdata = pmic->compat_data;
	uint32_t i2c_addr = 0;
	uint8_t ver = 0;
	uint8_t pid = 0;

	if (fdt_read_uint32(fdt, node, "reg", &i2c_addr) ||
	    (i2c_addr << 1) > UINT16_MAX)
		return TEE_ERROR_BAD_PARAMETERS;

	pmic->pmic_i2c_handle = i2c;
	pmic->pmic_i2c_addr = i2c_addr << 1;

	if (!stm32_i2c_is_device_ready(i2c, pmic->pmic_i2c_addr,
				       PMIC_I2C_TRIALS,
				       PMIC_I2C_TIMEOUT_BUSY_MS)) {
		EMSG("PMIC not found at I2C address %#"PRIx32, i2c_addr);
		return TEE_ERROR_GENERIC;
	}

	if (stpmic2_get_product_id(pmic, &pid) ||
	    stpmic2_get_version(pmic, &ver))
		return TEE_ERROR_COMMUNICATION;

	pmic->ref_id = (pid & PMIC_REF_ID_MASK) >> PMIC_REF_ID_SHIFT;

	IMSG("PMIC STPMIC REFID:%"PRIu8" NVM:%#"PRIx8" V%"PRIu8".%"PRIu8,
	     pmic->ref_id, (uint8_t)(pid & PMIC_NVM_ID_MASK),
	     (uint8_t)((ver & MAJOR_VERSION_MASK) >> MAJOR_VERSION_SHIFT),
	     (uint8_t)(ver & MINOR_VERSION_MASK));

	if (pmic->ref_id != cdata->ref_id) {
		EMSG("PMIC REFID %"PRIu8" does not match compatible",
		     pmic->ref_id);
		return TEE_ERROR_BAD_STATE;
	}

	stpmic2_dump_regulators(pmic);

	return TEE_SUCCESS;
}

static TEE_Result stm32_pmic2_probe(const void *fdt, int node,
				    const void *compat_data)
{
	struct stm32_i2c_dev *stm32_i2c_dev = NULL;
	struct i2c_dev *i2c_dev = NULL;
	struct stpmic2 *pmic = NULL;
	TEE_Result res = TEE_SUCCESS;

	res = i2c_dt_get_dev(fdt, node, &i2c_dev);
	if (res)
		return res;

	stm32_i2c_dev = container_of(i2c_dev, struct stm32_i2c_dev, i2c_dev);

	pmic = calloc(1, sizeof(*pmic));
	if (!pmic)
		return TEE_ERROR_OUT_OF_MEMORY;

	pmic->compat_data = compat_data;

	res = initialize_pmic2(fdt, node, pmic, stm32_i2c_dev->handle);
	if (res) {
		free(pmic);
		return res;
	}

	/* Registered regulators keep a reference to pmic, do not free it */
	return parse_regulator_fdt_nodes(fdt, node, pmic);
}

#define DEFINE_REGU(_name) { .name = (_name), .ops = &pmic2_regu_ops }
#define DEFINE_SWITCH(_name) { .name = (_name), .ops = &pmic2_switch_ops }

static const struct regu_dt_desc pmic25_reguls[] = {
	DEFINE_REGU("buck1"),
	DEFINE_REGU("buck2"),
	DEFINE_REGU("buck3"),
	DEFINE_REGU("buck4"),
	DEFINE_REGU("buck5"),
	DEFINE_REGU("buck6"),
	DEFINE_REGU("buck7"),
	DEFINE_REGU("ldo1"),
	DEFINE_REGU("ldo2"),
	DEFINE_REGU("ldo3"),
	DEFINE_REGU("ldo4"),
	DEFINE_REGU("ldo5"),
	DEFINE_REGU("ldo6"),
	DEFINE_REGU("ldo7"),
	DEFINE_REGU("ldo8"),
	DEFINE_SWITCH("refddr"),
};
DECLARE_KEEP_PAGER(pmic25_reguls);

static const struct pmic_compat_data stm32_pmic25_cdata = {
	.desc_table = pmic25_reguls,
	.desc_len = ARRAY_SIZE(pmic25_reguls),
	.ref_id = PMIC_REF_ID_STPMIC25,
};

static const struct dt_device_match stm32_pmic2_match_table[] = {
	{ .compatible = "st,stpmic2", .compat_data = &stm32_pmic25_cdata },
	{ }
};

DEFINE_DT_DRIVER(stm32_pmic2_dt_driver) = {
	.name = "stm32_pmic2",
	.match_table = stm32_pmic2_match_table,
	.probe = stm32_pmic2_probe,
};
