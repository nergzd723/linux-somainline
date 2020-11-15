// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2018, The Linux Foundation. All rights reserved.
 *
 * OSM hardware initial programming addition
 * Copyright (C) 2020, AngeloGioacchino Del Regno <kholk11@gmail.com>
 */

#include <linux/bitfield.h>
#include <linux/cpufreq.h>
#include <linux/init.h>
#include <linux/interconnect.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/pm_domain.h>
#include <linux/pm_opp.h>
#include <linux/slab.h>

#include <linux/delay.h>
#include <linux/qcom_scm.h>

#define LUT_MAX_ENTRIES			40U
#define LUT_SRC_845			GENMASK(31, 30)
#define LUT_SRC_8998			GENMASK(27, 26)
#define LUT_L_VAL			GENMASK(7, 0)
#define LUT_CORE_COUNT			GENMASK(18, 16)
#define LUT_VOLT			GENMASK(11, 0)
#define CLK_HW_DIV			2
#define LUT_TURBO_IND			1

#define CYCLE_COUNTER_CLK_RATIO		GENMASK(5, 1)
#define OSM_XO_RATIO_VAL		(10 - 1)
#define CYCLE_COUNTER_USE_XO_EDGE	BIT(8)

/* FSM Boost Control */
#define CC_BOOST_EN			BIT(0)
#define PS_BOOST_EN			BIT(1)
#define DCVS_BOOST_EN			BIT(2)
#define BOOST_TIMER_REG_HI		GENMASK(31, 16)
#define BOOST_TIMER_REG_LO		GENMASK(15, 0)

#define PLL_WAIT_LOCK_TIME_NS		2000
#define SAFE_FREQ_WAIT_NS		1000
#define DEXT_DECREMENT_WAIT_NS		200

#define BOOST_SYNC_DELAY		5

#define HYSTERESIS_UP_MASK		GENMASK(31, 16)
#define HYSTERESIS_DN_MASK		GENMASK(15, 0)
#define HYSTERESIS_CC_NS		200
#define HYSTERESIS_LLM_NS		65535

/* FSM Droop Control */
#define PC_RET_EXIT_DROOP_EN		BIT(3)
#define WFX_DROOP_EN			BIT(4)
#define DCVS_DROOP_EN			BIT(5)
#define DROOP_TIMER1			GENMASK(31, 16)
#define DROOP_TIMER0			GENMASK(15, 0)
#define DROOP_CTRL_VAL			(BIT(3) | BIT(17) | BIT(31))
#define DROOP_TIMER_NS			100
#define DROOP_WAIT_RELEASE_TIMER_NS	50
#define DROOP_RELEASE_TIMER_NS		1

/* PLL Override Control */
#define PLL_OVERRIDE_DROOP_EN		BIT(0)

/* Sequencer */
#define SEQUENCER_REG(base, n)		(base + (n * 4))


struct qcom_cpufreq_soc_setup_data {
	/* OSM phys register offsets */
	u32 reg_osm_sequencer_base;

	/* Frequency domain register offsets */
	u32 reg_cc_zero_behav;
	u32 reg_spm_cc_hyst;
	u32 reg_spm_cc_dcvs_dis;
	u32 reg_spm_core_ret_map;
	u32 reg_llm_freq_vote_hyst;
	u32 reg_llm_volt_vote_hyst;
	u32 reg_llm_intf_dcvs_dis;
	u32 reg_pdn_fsm_ctrl;
	u32 reg_cc_boost_timer0;
	u32 reg_cc_boost_timer1;
	u32 reg_cc_boost_timer2;
	u32 reg_dcvs_boost_timer0;
	u32 reg_dcvs_boost_timer1;
	u32 reg_dcvs_boost_timer2;
	u32 reg_ps_boost_timer0;
	u32 reg_ps_boost_timer1;
	u32 reg_ps_boost_timer2;
	u32 reg_boost_sync_delay;
	u32 reg_droop_ctrl;
	u32 reg_droop_release_ctrl;
	u32 reg_droop_unstall_ctrl;
	u32 reg_droop_wait_release_ctrl;
	u32 reg_droop_timer_ctrl;
	u32 reg_droop_sync_delay;
	u32 reg_pll_override;
	u32 reg_cycle_counter;
};

struct qcom_cpufreq_soc_data {
	u32 reg_enable;
	u32 reg_index;
	u32 reg_freq_lut;
	u32 reg_freq_lut_mask;
	u32 reg_volt_lut;
	u32 reg_override;
	u32 reg_spare;
	u32 reg_perf_state;
	u8 lut_row_size;
	bool uses_tz;
	const struct qcom_cpufreq_soc_setup_data setup_regs;
};

struct qcom_cpufreq_data {
	void __iomem *osm_base;
	void __iomem *pll_base;
	void __iomem *apm_base;
	void __iomem *base;
	const struct qcom_cpufreq_soc_data *soc_data;
};

struct qcom_cpufreq_private {
	struct device *cpu_dev;
	struct opp_table *opp_table;
};

static const char *cprh_genpd_names[] = { "power-domain0", NULL };
static unsigned long cpu_hw_rate, xo_rate;
static bool icc_scaling_enabled;

static int qcom_cpufreq_set_bw(struct cpufreq_policy *policy,
			       unsigned long freq_khz)
{
	unsigned long freq_hz = freq_khz * 1000;
	struct dev_pm_opp *opp;
	struct device *dev;
	int ret;

	dev = get_cpu_device(policy->cpu);
	if (!dev)
		return -ENODEV;

	opp = dev_pm_opp_find_freq_exact(dev, freq_hz, true);
	if (IS_ERR(opp))
		return PTR_ERR(opp);

	ret = dev_pm_opp_set_bw(dev, opp);
	dev_pm_opp_put(opp);
	return ret;
}

static int qcom_cpufreq_update_opp(struct device *cpu_dev,
				   unsigned long freq_khz,
				   unsigned long volt)
{
	unsigned long freq_hz = freq_khz * 1000;
	int ret;

	/* Skip voltage update if the opp table is not available */
	if (!icc_scaling_enabled)
		return dev_pm_opp_add(cpu_dev, freq_hz, volt);

	ret = dev_pm_opp_adjust_voltage(cpu_dev, freq_hz, volt, volt, volt);
	if (ret) {
		dev_err(cpu_dev, "Voltage update failed freq=%ld\n", freq_khz);
		return ret;
	}

	return dev_pm_opp_enable(cpu_dev, freq_hz);
}

static int qcom_cpufreq_hw_target_index(struct cpufreq_policy *policy,
					unsigned int index)
{
	struct qcom_cpufreq_data *data = policy->driver_data;
	const struct qcom_cpufreq_soc_data *soc_data = data->soc_data;
	unsigned long freq = policy->freq_table[index].frequency;

	writel_relaxed(index, data->base + soc_data->reg_perf_state);

	if (icc_scaling_enabled)
		qcom_cpufreq_set_bw(policy, freq);

	arch_set_freq_scale(policy->related_cpus, freq,
			    policy->cpuinfo.max_freq);
	return 0;
}

static unsigned int qcom_cpufreq_hw_get(unsigned int cpu)
{
	struct qcom_cpufreq_data *data;
	const struct qcom_cpufreq_soc_data *soc_data;
	struct cpufreq_policy *policy;
	unsigned int index;

	policy = cpufreq_cpu_get_raw(cpu);
	if (!policy)
		return 0;

	data = policy->driver_data;
	soc_data = data->soc_data;

	index = readl_relaxed(data->base + soc_data->reg_perf_state);
	index = min(index, LUT_MAX_ENTRIES - 1);

	return policy->freq_table[index].frequency;
}

static unsigned int qcom_cpufreq_hw_fast_switch(struct cpufreq_policy *policy,
						unsigned int target_freq)
{
	struct qcom_cpufreq_data *data = policy->driver_data;
	const struct qcom_cpufreq_soc_data *soc_data = data->soc_data;
	unsigned int index;
	unsigned long freq;

	index = policy->cached_resolved_idx;
	writel_relaxed(index, data->base + soc_data->reg_perf_state);

	freq = policy->freq_table[index].frequency;
	arch_set_freq_scale(policy->related_cpus, freq,
			    policy->cpuinfo.max_freq);

	return freq;
}

#define SDM630_A53_MAX_FREQS	8
//static u32 sdm630_a53_voltages[SDM630_A53_MAX_FREQS] =
//		{650000, 654000 , 667000, 727000, 797000, 925000, 945000, 975000 };

static u32 sdm630_a53_voltages[SDM630_A53_MAX_FREQS] =
		{0x254, 0x10254, 0x20290, 0x302bc, 0x40324, 0x5035c, 0x603a0, 0x703c8 };
static u32 sdm630_a53_frequencies[SDM630_A53_MAX_FREQS] =
		{300000000,  633600000,  902400000,  1113600000,
		 1401600000, 1536000000, 1747200000, 1843200000};
static u32 sdm630_a53_freq_data[SDM630_A53_MAX_FREQS] =
		{0x0004000f, 0x05040020, 0x0404002e, 0x04040039,
		 0x04040048, 0x04040050, 0x0404005a, 0x04040060};
static u32 sdm630_a53_pll_overrides[SDM630_A53_MAX_FREQS] =
		{0x01200020, 0x03200020, 0x04250025, 0x052e002e,
		 0x07390039, 0x08400040, 0x09480048, 0x094c004c};
static u32 sdm630_a53_spare_data[SDM630_A53_MAX_FREQS] =
		{1, 1, 1, 2, 2, 2, 2, 3};
//static u32 sdm630_a53_virtual_corners[SDM630_A53_MAX_FREQS] =
//		{1, 2, 3, 4, 5, 6, 7, 8};

static u32 sdm630_a53_virtual_corners[SDM630_A53_MAX_FREQS] =
		{0, 1, 2, 3, 4, 5, 6, 7};

static int qcom_cpufreq_hw_write_lut(struct device *cpu_dev,
				     struct cpufreq_policy *policy)
{
	u32 real_voltage, corner;
	u32 tmp = 0;
	u32 lval, core_count, src;

	int i;
	struct qcom_cpufreq_data *drv_data = policy->driver_data;
	const struct qcom_cpufreq_soc_data *soc_data = drv_data->soc_data;

	for (i = 0; i < SDM630_A53_MAX_FREQS; i++) {
		writel_relaxed(i, drv_data->base + soc_data->reg_index +
				      (i * soc_data->lut_row_size));

		writel_relaxed(sdm630_a53_freq_data[i],
			       drv_data->base + soc_data->reg_freq_lut +
				      (i * soc_data->lut_row_size));

		writel_relaxed(sdm630_a53_voltages[i],
			       drv_data->base + soc_data->reg_volt_lut +
				      (i * soc_data->lut_row_size));

		writel_relaxed(sdm630_a53_pll_overrides[i],
			       drv_data->base + soc_data->reg_override +
				      (i * soc_data->lut_row_size));

		writel_relaxed(sdm630_a53_spare_data[i],
			       drv_data->base + soc_data->reg_spare +
				      (i * soc_data->lut_row_size));

		real_voltage = sdm630_a53_voltages[i] & 0x0fff;
		corner = sdm630_a53_voltages[i] >> 10;

		lval = FIELD_GET(LUT_L_VAL, sdm630_a53_freq_data[i]);
		core_count = FIELD_GET(LUT_CORE_COUNT, sdm630_a53_freq_data[i]);

		src = sdm630_a53_freq_data[i] >> 0x1a;

		dev_err(cpu_dev, "WRITE: index=%d src=%d, freq=%d, volt=%d, corner=%d    lval=%d corecnt=%d\n",
			i, src, sdm630_a53_frequencies[i], real_voltage, corner, lval, core_count);
	}
	wmb();
	tmp = readl_relaxed(drv_data->base + soc_data->reg_enable);






	for (i = SDM630_A53_MAX_FREQS-1; i < LUT_MAX_ENTRIES; i++) {
		writel_relaxed(i, drv_data->base + soc_data->reg_index +
				      i * soc_data->lut_row_size);

		writel_relaxed(sdm630_a53_freq_data[SDM630_A53_MAX_FREQS-1],
			       drv_data->base + soc_data->reg_freq_lut +
				      i * soc_data->lut_row_size);
		writel_relaxed(sdm630_a53_voltages[SDM630_A53_MAX_FREQS-1],
			       drv_data->base + soc_data->reg_volt_lut +
				      i * soc_data->lut_row_size);

		writel_relaxed(sdm630_a53_pll_overrides[SDM630_A53_MAX_FREQS-1],
			       drv_data->base + soc_data->reg_override +
				      i * soc_data->lut_row_size);
		writel_relaxed(sdm630_a53_spare_data[SDM630_A53_MAX_FREQS-1],
			       drv_data->base + soc_data->reg_spare +
				      i * soc_data->lut_row_size);

	}
	wmb();
	tmp = readl_relaxed(drv_data->base + soc_data->reg_enable);
	tmp = readl_relaxed(drv_data->base + soc_data->reg_enable);


	return 0;
}

static int qcom_cpufreq_hw_read_lut(struct device *cpu_dev,
				    struct cpufreq_policy *policy)
{
	u32 data, src, lval, i, core_count, prev_freq = 0, freq;
	u32 volt;
	struct cpufreq_frequency_table	*table;
	struct dev_pm_opp *opp;
	unsigned long rate;
	int ret;
	struct qcom_cpufreq_data *drv_data = policy->driver_data;
	const struct qcom_cpufreq_soc_data *soc_data = drv_data->soc_data;

	table = kcalloc(LUT_MAX_ENTRIES + 1, sizeof(*table), GFP_KERNEL);
	if (!table)
		return -ENOMEM;

	ret = dev_pm_opp_of_add_table(cpu_dev);
	if (!ret) {
		/* Disable all opps and cross-validate against LUT later */
		dev_err(cpu_dev, "Disable all opps and cross-validate against LUT later\n");
		icc_scaling_enabled = false;
		for (rate = 0; ; rate++) {
			opp = dev_pm_opp_find_freq_ceil(cpu_dev, &rate);
			if (IS_ERR(opp))
				break;

			dev_pm_opp_put(opp);
			dev_pm_opp_disable(cpu_dev, rate);
		}
	} else if (ret != -ENODEV) {
		dev_err(cpu_dev, "Invalid opp table in device tree\n");
		return ret;
	} else {
		policy->fast_switch_possible = true;
		icc_scaling_enabled = false;
	}

	for (i = 0; i < LUT_MAX_ENTRIES; i++) {
		data = readl_relaxed(drv_data->base + soc_data->reg_freq_lut +
				      i * soc_data->lut_row_size);
		src = data & soc_data->reg_freq_lut_mask;
		src >>= ffs(soc_data->reg_freq_lut_mask) - 1;

		lval = FIELD_GET(LUT_L_VAL, data);
		core_count = FIELD_GET(LUT_CORE_COUNT, data);

		data = readl_relaxed(drv_data->base + soc_data->reg_volt_lut +
				      i * soc_data->lut_row_size);
		volt = FIELD_GET(LUT_VOLT, data) * 1000;

		if (src)
			freq = xo_rate * lval / 1000;
		else
			freq = cpu_hw_rate / 1000;


		dev_err(cpu_dev, "READ: index=%d src=%d, lval=%d, freq=%d, volt=%d, core_count %d\n", i,
				src, lval, freq, volt, core_count);

		if (freq != prev_freq && core_count != LUT_TURBO_IND) {
			if (!qcom_cpufreq_update_opp(cpu_dev, freq, volt)) {
				table[i].frequency = freq;
				dev_dbg(cpu_dev, "index=%d freq=%d, core_count %d\n", i,
				freq, core_count);
			} else {
				dev_warn(cpu_dev, "failed to update OPP for freq=%d\n", freq);
				table[i].frequency = CPUFREQ_ENTRY_INVALID;
			}

		} else if (core_count == LUT_TURBO_IND) {
			table[i].frequency = CPUFREQ_ENTRY_INVALID;
		}

		/*
		 * Two of the same frequencies with the same core counts means
		 * end of table
		 */
		if (i > 0 && prev_freq == freq) {
			struct cpufreq_frequency_table *prev = &table[i - 1];

			/*
			 * Only treat the last frequency that might be a boost
			 * as the boost frequency
			 */
			if (prev->frequency == CPUFREQ_ENTRY_INVALID) {
				if (!qcom_cpufreq_update_opp(cpu_dev, prev_freq, volt)) {
					prev->frequency = prev_freq;
					prev->flags = CPUFREQ_BOOST_FREQ;
				} else {
					dev_warn(cpu_dev, "failed to update OPP for freq=%d\n",
						 freq);
				}
			}

			break;
		}

		prev_freq = freq;
	}

	table[i].frequency = CPUFREQ_TABLE_END;
	policy->freq_table = table;
	dev_pm_opp_set_sharing_cpus(cpu_dev, policy->cpus);

	return 0;
}

static void qcom_get_related_cpus(int index, struct cpumask *m)
{
	struct device_node *cpu_np;
	struct of_phandle_args args;
	int cpu, ret;

	for_each_possible_cpu(cpu) {
		cpu_np = of_cpu_device_node_get(cpu);
		if (!cpu_np)
			continue;

		ret = of_parse_phandle_with_args(cpu_np, "qcom,freq-domain",
						 "#freq-domain-cells", 0,
						 &args);
		of_node_put(cpu_np);
		if (ret < 0)
			continue;

		if (index == args.args[0])
			cpumask_set_cpu(cpu, m);
	}
}

static const struct qcom_cpufreq_soc_data qcom_soc_data = {
	.reg_enable = 0x0,
	.reg_freq_lut = 0x110,
	.reg_freq_lut_mask = LUT_SRC_845,
	.reg_volt_lut = 0x114,
	.reg_perf_state = 0x920,
	.lut_row_size = 32,
	.uses_tz = true,
};

static const struct qcom_cpufreq_soc_data msm8998_soc_data = {
	.reg_enable = 0x4,
	.reg_index = 0x150,
	.reg_freq_lut = 0x154,
	.reg_freq_lut_mask = LUT_SRC_8998,
	.reg_volt_lut = 0x158,
	.reg_override = 0x15c,
	.reg_spare = 0x164,
	.reg_perf_state = 0xf10,
	.lut_row_size = 32,
	.uses_tz = false,
	.setup_regs = {
		/* OSM phys */
		.reg_osm_sequencer_base = 0x300,

		/* Frequency domain offsets */
		.reg_cc_zero_behav = 0x0c,
		.reg_spm_cc_hyst = 0x1c,
		.reg_spm_cc_dcvs_dis = 0x20,
		.reg_spm_core_ret_map = 0x24,
		.reg_llm_freq_vote_hyst = 0x2c,
		.reg_llm_volt_vote_hyst = 0x30,
		.reg_llm_intf_dcvs_dis = 0x34,
		.reg_pdn_fsm_ctrl = 0x70,
		.reg_cc_boost_timer0 = 0x74,
		.reg_cc_boost_timer1 = 0x78,
		.reg_cc_boost_timer2 = 0x7c,
		.reg_dcvs_boost_timer0 = 0x84,
		.reg_dcvs_boost_timer1 = 0x88,
		.reg_dcvs_boost_timer2 = 0x8c,
		.reg_ps_boost_timer0 = 0x94,
		.reg_ps_boost_timer1 = 0x98,
		.reg_ps_boost_timer2 = 0x9c,
		.reg_boost_sync_delay = 0xa0,
		.reg_droop_ctrl = 0xa4,
		.reg_droop_release_ctrl = 0xa8,
		.reg_droop_unstall_ctrl = 0xac,
		.reg_droop_wait_release_ctrl = 0xb0,
		.reg_droop_timer_ctrl = 0xb8,
		.reg_droop_sync_delay = 0xbc,
		.reg_pll_override = 0xc0,
		.reg_cycle_counter = 0xf00,
	},
};

static const struct qcom_cpufreq_soc_data epss_soc_data = {
	.reg_enable = 0x0,
	.reg_freq_lut = 0x100,
	.reg_freq_lut_mask = LUT_SRC_845,
	.reg_volt_lut = 0x200,
	.reg_perf_state = 0x320,
	.lut_row_size = 4,
	.uses_tz = true,
};

static const struct of_device_id qcom_cpufreq_hw_match[] = {
	{ .compatible = "qcom,cpufreq-hw", .data = &qcom_soc_data },
	{ .compatible = "qcom,cpufreq-hw-8998", .data = &msm8998_soc_data },
	{ .compatible = "qcom,cpufreq-epss", .data = &epss_soc_data },
	{}
};
MODULE_DEVICE_TABLE(of, qcom_cpufreq_hw_match);

static void qcom_cpufreq_hw_boost_setup(void __iomem *timer0_addr)
{
	u32 val;

	val = FIELD_PREP(BOOST_TIMER_REG_LO, PLL_WAIT_LOCK_TIME_NS);
	val |= FIELD_PREP(BOOST_TIMER_REG_HI, SAFE_FREQ_WAIT_NS);
	writel_relaxed(val, timer0_addr);

	/* timer_reg1 */
	val = FIELD_PREP(BOOST_TIMER_REG_LO, PLL_WAIT_LOCK_TIME_NS);
	val |= FIELD_PREP(BOOST_TIMER_REG_HI, PLL_WAIT_LOCK_TIME_NS);
	writel_relaxed(val, timer0_addr + 0x4);

	/* timer_reg2 */
	val = FIELD_PREP(BOOST_TIMER_REG_LO, DEXT_DECREMENT_WAIT_NS);
	writel_relaxed(val, timer0_addr + 0x8);
}

static int qcom_cpufreq_hw_osm_setup(struct device *cpu_dev,
				     struct cpufreq_policy *policy,
				     u32 index)
{
	struct platform_device *pdev = cpufreq_get_driver_data();
	struct qcom_cpufreq_data *drv_data = policy->driver_data;
	const struct qcom_cpufreq_soc_setup_data *setup_regs;
	struct resource *osm_rsrc;
	struct device_node *cpu_np;
	struct opp_table *genpd_opp;
	u32 seq_phys_addr, val;
	const char *osm_resname;
	const char *genpd_name;
	int ret = 0;

	setup_regs = &drv_data->soc_data->setup_regs;

	osm_resname = kasprintf(GFP_KERNEL, "osm-domain%d", index);
	if (!osm_resname)
		return -ENOMEM;

	genpd_name = kasprintf(GFP_KERNEL, "power-domain%d", index);
	if (!genpd_name)
		return -ENOMEM;

	cpu_np = pdev->dev.of_node;
	if (!cpu_np)
		return -EINVAL;

	/*
	 * On some SoCs the OSM is not getting programmed from bootloader
	 * and needs to be done here: in this case, we need to retrieve
	 * the base physical address for the "Sequencer", so we will get
	 * the OSM base phys and apply the sequencer offset.
	 * 
	 * Note: We are not remapping this iospace because we are really
	 *       sending the physical address through SCM calls later.
	 */
	osm_rsrc = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						osm_resname);
	if (!osm_rsrc)
		return -ENODEV;

	seq_phys_addr = osm_rsrc->start + setup_regs->reg_osm_sequencer_base;


	/* TODO: Solve the following TODOs to remove sdm630_a53_{whatever} hardcoded values}
	/* TODO: Get open-loop voltages from CPRh */
	/* TODO: Get frequency data, pll overrides, frequencies from OPP */
	/* TODO: Get speed-bin info from QFPROM, restrict OPP */
/*
	genpd_opp = dev_pm_opp_attach_genpd(cpu_dev, &genpd_name, NULL);
	if (IS_ERR(genpd_opp)) {
		ret = PTR_ERR(genpd_opp);
		if (ret != -EPROBE_DEFER)
			dev_err(&pdev->dev,
				"Could not attach to pm_domain: %d\n", ret);
		return ret;
	}
*/


	/* Set OSM to XO clock ratio and use XO edge for the cycle counter */
	val = FIELD_PREP(CYCLE_COUNTER_CLK_RATIO, OSM_XO_RATIO_VAL);
	val |= CYCLE_COUNTER_USE_XO_EDGE;

	/* Enable the cycle counter ... all useless */
	val |= BIT(0);
	writel_relaxed(val, drv_data->base + setup_regs->reg_cycle_counter);


	/* Downstream, the LUT is written after enabling cycle counters... */
	qcom_cpufreq_hw_write_lut(cpu_dev, policy);

	/* CoreCount DCVS Policy: Wait time for frequency inc/decrement */
	val = FIELD_PREP(HYSTERESIS_UP_MASK, HYSTERESIS_CC_NS);
	val |= FIELD_PREP(HYSTERESIS_DN_MASK, HYSTERESIS_CC_NS);
	writel_relaxed(val, drv_data->base + setup_regs->reg_spm_cc_hyst);

	/* Set the frequency index for cluster power collapse */
	writel_relaxed(0, drv_data->base + setup_regs->reg_cc_zero_behav);
	wmb();

	/* Treat cores in retention as active */
	writel_relaxed(0, drv_data->base + setup_regs->reg_spm_core_ret_map);

	/* Enable CoreCount based DCVS */
	writel_relaxed(0, drv_data->base + setup_regs->reg_spm_cc_dcvs_dis);

	/* CoreCount DCVS-LLM Policy: Wait time for frequency inc/decrement */
	val = FIELD_PREP(HYSTERESIS_UP_MASK, HYSTERESIS_LLM_NS);
	val |= FIELD_PREP(HYSTERESIS_DN_MASK, HYSTERESIS_LLM_NS);
	writel_relaxed(val, drv_data->base + setup_regs->reg_llm_freq_vote_hyst);
	wmb();

	/* CoreCount DCVS-LLM Policy: Wait time for voltage inc/decrement */
	val = FIELD_PREP(HYSTERESIS_UP_MASK, HYSTERESIS_LLM_NS);
	val |= FIELD_PREP(HYSTERESIS_DN_MASK, HYSTERESIS_LLM_NS);
	writel_relaxed(val, drv_data->base + setup_regs->reg_llm_volt_vote_hyst);
	wmb();

	/* Enable LLM frequency+voltage voting */
	writel_relaxed(0, drv_data->base + setup_regs->reg_llm_intf_dcvs_dis);
	wmb();

	/* Setup Boost FSM Timers */
	qcom_cpufreq_hw_boost_setup(drv_data->base +
				    setup_regs->reg_cc_boost_timer0);
	qcom_cpufreq_hw_boost_setup(drv_data->base +
				    setup_regs->reg_dcvs_boost_timer0);
	qcom_cpufreq_hw_boost_setup(drv_data->base +
				    setup_regs->reg_ps_boost_timer0);

	/* PLL signal timing control for Boost */
	writel_relaxed(BOOST_SYNC_DELAY,
		       drv_data->base + setup_regs->reg_boost_sync_delay);

	/* Setup WFx and PC/RET droop unstall */
	val = readl_relaxed(drv_data->base + setup_regs->reg_droop_unstall_ctrl);
	val |= FIELD_PREP(DROOP_TIMER1, DROOP_TIMER_NS);
	val |= FIELD_PREP(DROOP_TIMER0, DROOP_TIMER_NS);
	writel_relaxed(val, drv_data->base + setup_regs->reg_droop_unstall_ctrl);

	/* Setup WFx and PC/RET droop wait-to-release */
	val = readl_relaxed(drv_data->base + setup_regs->reg_droop_wait_release_ctrl);
	val |= FIELD_PREP(DROOP_TIMER1, DROOP_WAIT_RELEASE_TIMER_NS);
	val |= FIELD_PREP(DROOP_TIMER0, DROOP_WAIT_RELEASE_TIMER_NS);
	writel_relaxed(val,
		       drv_data->base + setup_regs->reg_droop_wait_release_ctrl);

	/* PLL signal timing control for Droop */
	writel_relaxed(1, drv_data->base + setup_regs->reg_droop_sync_delay);

	/* Setup DCVS timers */
	writel_relaxed(DROOP_RELEASE_TIMER_NS,
		       drv_data->base + setup_regs->reg_droop_release_ctrl);
	writel_relaxed(DROOP_TIMER_NS,
		       drv_data->base + setup_regs->reg_droop_timer_ctrl);

	/* Setup Droop control */
	val = readl_relaxed(drv_data->base + setup_regs->reg_droop_ctrl);
	val |= DROOP_CTRL_VAL;
	writel_relaxed(val, drv_data->base + setup_regs->reg_droop_ctrl);

	/* Enable CC-Boost, DCVS-Boost, PS-Boost, WFx, PC/RET, DCVS FSM */
	val = readl_relaxed(drv_data->base + setup_regs->reg_pdn_fsm_ctrl);
	val |= CC_BOOST_EN | PS_BOOST_EN | DCVS_BOOST_EN;
	val |= WFX_DROOP_EN | PC_RET_EXIT_DROOP_EN | DCVS_DROOP_EN;
	writel_relaxed(val, drv_data->base + setup_regs->reg_pdn_fsm_ctrl);

	/* Enable PLL Droop Override */
	val = readl_relaxed(drv_data->base + setup_regs->reg_pll_override);
	val |= PLL_OVERRIDE_DROOP_EN;
	writel_relaxed(val, drv_data->base + setup_regs->reg_pll_override);

	/* Sequencer: MEM-ACC Programming */
	ret = qcom_scm_io_writel(SEQUENCER_REG(seq_phys_addr, 55), sdm630_a53_virtual_corners[3] - 1);
	if (ret) {
		dev_err(cpu_dev, "SCM write failed\n");
		return ret;
	}

	ret = qcom_scm_io_writel(SEQUENCER_REG(seq_phys_addr, 56), sdm630_a53_virtual_corners[3]);
	if (ret) {
		dev_err(cpu_dev, "SCM write failed\n");
		return ret;
	}

	ret = qcom_scm_io_writel(SEQUENCER_REG(seq_phys_addr, 57), sdm630_a53_virtual_corners[7] - 1);
	if (ret) {
		dev_err(cpu_dev, "SCM write failed\n");
		return ret;
	}


	ret = qcom_scm_io_writel(SEQUENCER_REG(seq_phys_addr, 58), sdm630_a53_virtual_corners[7]);
	if (ret) {
		dev_err(cpu_dev, "SCM write failed\n");
		return ret;
	}

	/* Set the L_VAL corresponding to THRESHOLD_VC==3 (in our case, virtual_corners[7]) */
	/* HACK!!! I know what is the right lval for this, search it into the table if this works!!!! */
	/*                    **** DO NOT UPSTREAM AS IT IS ****                     */
#define L_VAL_FOR_VC_EQUALS_7	96
	ret = qcom_scm_io_writel(SEQUENCER_REG(seq_phys_addr, 32), L_VAL_FOR_VC_EQUALS_7);
	if (ret) {
		dev_err(cpu_dev, "SCM write failed\n");
		return ret;
	}

	/* This is also supposed to be dynamic!!!!! */
#define APM_THRESHOLD_CORNER	6
#define APM_THRESHOLD_PRE_VC	(APM_THRESHOLD_CORNER - 1)
#define APM_CROSSOVER_CORNER	8
	/* APM Virtual Corner Setup */
	writel_relaxed(APM_THRESHOLD_CORNER, drv_data->base + 0x48); /* SEQ_REG1_OFFSET */
	ret = qcom_scm_io_writel(SEQUENCER_REG(seq_phys_addr, 72), APM_CROSSOVER_CORNER);
	if (ret) {
		dev_err(cpu_dev, "SCM write failed\n");
		return ret;
	}
	ret = qcom_scm_io_writel(SEQUENCER_REG(seq_phys_addr, 15), APM_THRESHOLD_CORNER);
	if (ret) {
		dev_err(cpu_dev, "SCM write failed\n");
		return ret;
	}
	ret = qcom_scm_io_writel(SEQUENCER_REG(seq_phys_addr, 31), APM_THRESHOLD_PRE_VC);
	if (ret) {
		dev_err(cpu_dev, "SCM write failed\n");
		return ret;
	}
	ret = qcom_scm_io_writel(SEQUENCER_REG(seq_phys_addr, 76), (0x39 | APM_THRESHOLD_CORNER << 6));
	if (ret) {
		dev_err(cpu_dev, "SCM write failed\n");
		return ret;
	}

	return ret;
}

static int qcom_cpufreq_hw_cpu_init(struct cpufreq_policy *policy)
{
	struct platform_device *pdev = cpufreq_get_driver_data();
	struct device *dev = &pdev->dev;
	struct of_phandle_args args;
	struct device_node *cpu_np;
	struct device *cpu_dev;
	void __iomem *base;
	struct qcom_cpufreq_data *data;
	int i, ret, index;

	u32 val;

	cpu_dev = get_cpu_device(policy->cpu);
	if (!cpu_dev) {
		pr_err("%s: failed to get cpu%d device\n", __func__,
		       policy->cpu);
		return -ENODEV;
	}

	cpu_np = of_cpu_device_node_get(policy->cpu);
	if (!cpu_np)
		return -EINVAL;

	ret = of_parse_phandle_with_args(cpu_np, "qcom,freq-domain",
					 "#freq-domain-cells", 0, &args);
	of_node_put(cpu_np);
	if (ret)
		return ret;

	index = args.args[0];

	base = devm_platform_ioremap_resource(pdev, index);
	if (IS_ERR(base))
		return PTR_ERR(base);

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto error;
	}

	data->soc_data = of_device_get_match_data(&pdev->dev);
	data->base = base;
	policy->driver_data = data;

#define DCVS_PERF_STATE_DEVIATION_INTR_EN		0xf18
#define DCVS_PERF_STATE_DEVIATION_CORRECTED_INTR_EN	0xf24
#define DCVS_PERF_STATE_MET_INTR_EN			0xf30
#define DCVS_OPERATION_IN_PROGRESS_REG			0xf0c
	if (!data->soc_data->uses_tz) {
		ret = qcom_cpufreq_hw_osm_setup(cpu_dev, policy, index);
		if (ret) {
			dev_err(dev, "Cannot setup the OSM controller: %d\n", ret);
			ret = -ENODEV;
			goto error;
		}

		/* Enable debug interrupts */
		writel_relaxed(1, base + DCVS_PERF_STATE_DEVIATION_INTR_EN);
		writel_relaxed(1, base + DCVS_PERF_STATE_DEVIATION_CORRECTED_INTR_EN);
		writel_relaxed(1, base + DCVS_PERF_STATE_MET_INTR_EN);

		val = readl_relaxed(data->base + DCVS_OPERATION_IN_PROGRESS_REG);
		dev_err(dev, "DCVS status BEFORE ENABLING is 0x%x\n", val);

		writel_relaxed(0, data->base + data->soc_data->reg_perf_state);
		val = readl_relaxed(data->base + DCVS_OPERATION_IN_PROGRESS_REG);
		val = readl_relaxed(data->base + DCVS_OPERATION_IN_PROGRESS_REG);
		dev_err(dev, "DCVS status BEFORE ENABLING AFTER SET P0 is 0x%x\n", val);

		udelay(100);
		writel_relaxed(1, base + data->soc_data->reg_enable);
		wmb();
		udelay(5);
	}

	/* HW should be in enabled state to proceed */
	if (!(readl_relaxed(base + data->soc_data->reg_enable) & 0x1)) {
		dev_err(dev, "Domain-%d cpufreq hardware not enabled (0x%x)\n", index, val);
		ret = -ENODEV;
		goto error;
	}


	qcom_get_related_cpus(index, policy->cpus);
	if (!cpumask_weight(policy->cpus)) {
		dev_err(dev, "Domain-%d failed to get related CPUs\n", index);
		ret = -ENOENT;
		goto error;
	}

	val = readl_relaxed(data->base);
	dev_err(dev, "OSM Version is %x\n", val);

	val = readl_relaxed(data->base + DCVS_OPERATION_IN_PROGRESS_REG);
	dev_err(dev, "DCVS status BEFORE FREQ SET is 0x%x\n", val);

	writel_relaxed(0, data->base + data->soc_data->reg_perf_state);

	ret = qcom_cpufreq_hw_read_lut(cpu_dev, policy);

	val = readl_relaxed(data->base + data->soc_data->reg_perf_state);
	dev_err(dev, "Setting frequency index. Current=0x%x\n", val);
	writel_relaxed(7, data->base + data->soc_data->reg_perf_state);
	wmb();



#define DOMAIN_PSTATE_STATUS	0xb00

	val = readl_relaxed(data->base + DOMAIN_PSTATE_STATUS);
	dev_err(dev, "P-State status before change: 0x%x\n", val);

	val = readl_relaxed(data->base + data->soc_data->reg_perf_state);
	dev_err(dev, "Current index is 0x%x\n", val);

	val = readl_relaxed(data->base + DOMAIN_PSTATE_STATUS);
	dev_err(dev, "P-State status after change: 0x%x\n", val);


	msleep(200);
	val = readl_relaxed(data->base + DCVS_OPERATION_IN_PROGRESS_REG);
	dev_err(dev, "Current DCVS status is 0x%x\n", val);

	val = readl_relaxed(data->base + DOMAIN_PSTATE_STATUS);
	dev_err(dev, "P-State status after wait: 0x%x\n", val);




#define DCVS_PERF_STATE_MET_INTR_CLEAR	0xf34
	for (i = data->soc_data->reg_perf_state; i < (DCVS_PERF_STATE_MET_INTR_CLEAR + 1); i +=4)
		dev_err(dev, "reg 0x%x = 0x%x\n", i, readl_relaxed(data->base + i));


	if (ret) {
		dev_err(dev, "Domain-%d failed to read LUT\n", index);
		goto error;
	}

	ret = dev_pm_opp_get_opp_count(cpu_dev);
	if (ret <= 0) {
		dev_err(cpu_dev, "Failed to add OPPs\n");
		ret = -ENODEV;
		goto error;
	}

	dev_pm_opp_of_register_em(cpu_dev, policy->cpus);

	return 0;
error:
	policy->driver_data = NULL;
	devm_iounmap(dev, base);
	return ret;
}

static int qcom_cpufreq_hw_cpu_exit(struct cpufreq_policy *policy)
{
	struct device *cpu_dev = get_cpu_device(policy->cpu);
	struct qcom_cpufreq_data *data = policy->driver_data;
	struct platform_device *pdev = cpufreq_get_driver_data();

	dev_pm_opp_remove_all_dynamic(cpu_dev);
	dev_pm_opp_of_cpumask_remove_table(policy->related_cpus);
	kfree(policy->freq_table);
	devm_iounmap(&pdev->dev, data->base);

	return 0;
}

static struct freq_attr *qcom_cpufreq_hw_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	&cpufreq_freq_attr_scaling_boost_freqs,
	NULL
};

static struct cpufreq_driver cpufreq_qcom_hw_driver = {
	.flags		= CPUFREQ_STICKY | CPUFREQ_NEED_INITIAL_FREQ_CHECK |
			  CPUFREQ_HAVE_GOVERNOR_PER_POLICY |
			  CPUFREQ_IS_COOLING_DEV,
	.verify		= cpufreq_generic_frequency_table_verify,
	.target_index	= qcom_cpufreq_hw_target_index,
	.get		= qcom_cpufreq_hw_get,
	.init		= qcom_cpufreq_hw_cpu_init,
	.exit		= qcom_cpufreq_hw_cpu_exit,
	.fast_switch    = qcom_cpufreq_hw_fast_switch,
	.name		= "qcom-cpufreq-hw",
	.attr		= qcom_cpufreq_hw_attr,
};

static int qcom_cpufreq_hw_driver_probe(struct platform_device *pdev)
{
	struct opp_table *genpd_opp;
	struct device *genpd_dev;
	struct device *cpu_dev;
	struct clk *clk;
	int ret;

	clk = clk_get(&pdev->dev, "xo");
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	xo_rate = clk_get_rate(clk);
	clk_put(clk);

	clk = clk_get(&pdev->dev, "alternate");
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	cpu_hw_rate = clk_get_rate(clk) / CLK_HW_DIV;
	clk_put(clk);

	cpufreq_qcom_hw_driver.driver_data = pdev;

	
	cpu_dev = get_cpu_device(0);
	if (!cpu_dev)
		return -EPROBE_DEFER;

	/*
	 * Check for CPRh genpd: if it's not present, then delay probing
	 * the driver, as we need it to initialize the OSM state machine
	 */
/*
	genpd_opp = dev_pm_opp_attach_genpd(&pdev->dev, &cprh_genpd_names[0], NULL);
	if (IS_ERR(genpd_opp)) {
		ret = PTR_ERR(genpd_opp);
		if (ret != -EPROBE_DEFER)
			dev_err(&pdev->dev,
				"Could not attach to pm_domain: %d\n", ret);
		else
			dev_err(&pdev->dev,
				"Deferring probe: cannot get genpd\n");
		return ret;
	}

	dev_pm_opp_detach_genpd(genpd_opp);
*/
/*
	genpd_dev = genpd_dev_pm_attach_by_id(&pdev->dev, 0);
	if (IS_ERR(genpd_dev)) {
		ret = PTR_ERR(genpd_dev);
		if (ret != -EPROBE_DEFER)
			dev_err(&pdev->dev,
				"Could not attach to pm_domain: %d\n", ret);
		else
			dev_err(&pdev->dev,
				"Deferring probe: cannot get genpd\n");
		return ret;
	}

	pm_genpd_remove_device(genpd_dev);
*/

	/* Check for optional interconnect paths on CPU0 */
//	ret = dev_pm_opp_of_find_icc_paths(cpu_dev, NULL);
//	if (ret)
//		return ret;

	ret = cpufreq_register_driver(&cpufreq_qcom_hw_driver);
	if (ret)
		dev_err(&pdev->dev, "CPUFreq HW driver failed to register\n");
	else
		dev_dbg(&pdev->dev, "QCOM CPUFreq HW driver initialized\n");

	return ret;
}

static int qcom_cpufreq_hw_driver_remove(struct platform_device *pdev)
{
	return cpufreq_unregister_driver(&cpufreq_qcom_hw_driver);
}

static struct platform_driver qcom_cpufreq_hw_driver = {
	.probe = qcom_cpufreq_hw_driver_probe,
	.remove = qcom_cpufreq_hw_driver_remove,
	.driver = {
		.name = "qcom-cpufreq-hw",
		.of_match_table = qcom_cpufreq_hw_match,
	},
};

static int __init qcom_cpufreq_hw_init(void)
{
	return platform_driver_register(&qcom_cpufreq_hw_driver);
}
postcore_initcall(qcom_cpufreq_hw_init);

static void __exit qcom_cpufreq_hw_exit(void)
{
	platform_driver_unregister(&qcom_cpufreq_hw_driver);
}
module_exit(qcom_cpufreq_hw_exit);

MODULE_DESCRIPTION("QCOM CPUFREQ HW Driver");
MODULE_LICENSE("GPL v2");
