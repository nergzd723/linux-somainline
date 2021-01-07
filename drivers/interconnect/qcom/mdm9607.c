// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021 Konrad Dybcio <konrad.dybcio@somainline.org>
 * Copyright (c) 2021, AngeloGioacchino Del Regno
 *                     <angelogioacchino.delregno@somainline.org>
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/interconnect-provider.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include <dt-bindings/interconnect/qcom,mdm9607.h>

#include "smd-rpm.h"

enum {
	MDM9607_BIMC_MASTER_APPS_PROC = 1,
	MDM9607_BIMC_MASTER_PCNOC_BIMC_1,
	MDM9607_BIMC_MASTER_TCU_0,
	MDM9607_BIMC_SLAVE_EBI,
	MDM9607_BIMC_SLAVE_BIMC_PCNOC,
	MDM9607_PCNOC_MASTER_QDSS_BAM,
	MDM9607_PCNOC_MASTER_BIMC_PCNOC,
	MDM9607_PCNOC_MASTER_QDSS_ETR,
	MDM9607_PCNOC_MASTER_AUDIO,
	MDM9607_PCNOC_MASTER_QPIC,
	MDM9607_PCNOC_MASTER_HSIC,
	MDM9607_PCNOC_MASTER_BLSP_1,
	MDM9607_PCNOC_MASTER_USB_HS1,
	MDM9607_PCNOC_MASTER_MASTER_CRYPTO,
	MDM9607_PCNOC_MASTER_SDCC_1,
	MDM9607_PCNOC_MASTER_SDCC_2,
	MDM9607_PCNOC_MASTER_XI_USB_HS1,
	MDM9607_PCNOC_MASTER_XI_HSIC,
	MDM9607_PCNOC_MASTER_SGMII,
	MDM9607_PCNOC_MASTER_PCNOC_M_0,
	MDM9607_PCNOC_SLAVE_PCNOC_M_0,
	MDM9607_PCNOC_MASTER_PCNOC_M_1,
	MDM9607_PCNOC_SLAVE_PCNOC_M_1,
	MDM9607_PCNOC_MASTER_QDSS_INT,
	MDM9607_PCNOC_SLAVE_QDSS_INT,
	MDM9607_PCNOC_MASTER_PCNOC_INT_0,
	MDM9607_PCNOC_SLAVE_PCNOC_INT_0,
	MDM9607_PCNOC_MASTER_PCNOC_INT_2,
	MDM9607_PCNOC_SLAVE_PCNOC_INT_2,
	MDM9607_PCNOC_MASTER_PCNOC_INT_3,
	MDM9607_PCNOC_SLAVE_PCNOC_INT_3,
	MDM9607_PCNOC_MASTER_PCNOC_S_0,
	MDM9607_PCNOC_SLAVE_PCNOC_S_0,
	MDM9607_PCNOC_MASTER_PCNOC_S_1,
	MDM9607_PCNOC_SLAVE_PCNOC_S_1,
	MDM9607_PCNOC_MASTER_PCNOC_S_2,
	MDM9607_PCNOC_SLAVE_PCNOC_S_2,
	MDM9607_PCNOC_MASTER_PCNOC_S_3,
	MDM9607_PCNOC_SLAVE_PCNOC_S_3,
	MDM9607_PCNOC_MASTER_PCNOC_S_4,
	MDM9607_PCNOC_SLAVE_PCNOC_S_4,
	MDM9607_PCNOC_MASTER_PCNOC_S_5,
	MDM9607_PCNOC_SLAVE_PCNOC_S_5,
	MDM9607_PCNOC_SLAVE_PCNOC_BIMC_1,
	MDM9607_PCNOC_SLAVE_QDSS_STM,
	MDM9607_PCNOC_SLAVE_CATS_0,
	MDM9607_PCNOC_SLAVE_IMEM,
	MDM9607_PCNOC_SLAVE_TCSR,
	MDM9607_PCNOC_SLAVE_SDCC_1,
	MDM9607_PCNOC_SLAVE_BLSP_1,
	MDM9607_PCNOC_SLAVE_SGMII,
	MDM9607_PCNOC_SLAVE_CRYPTO_0_CFG,
	MDM9607_PCNOC_SLAVE_MESSAGE_RAM,
	MDM9607_PCNOC_SLAVE_PDM,
	MDM9607_PCNOC_SLAVE_PRNG,
	MDM9607_PCNOC_SLAVE_USB2,
	MDM9607_PCNOC_SLAVE_SDCC_2,
	MDM9607_PCNOC_SLAVE_AUDIO,
	MDM9607_PCNOC_SLAVE_HSIC,
	MDM9607_PCNOC_SLAVE_USB_PHY,
	MDM9607_PCNOC_SLAVE_TLMM,
	MDM9607_PCNOC_SLAVE_IMEM_CFG,
	MDM9607_PCNOC_SLAVE_PMIC_ARB,
	MDM9607_PCNOC_SLAVE_TCU,
	MDM9607_PCNOC_SLAVE_QPIC,
};

#define RPM_BUS_MASTER_REQ      0x73616d62
#define RPM_BUS_SLAVE_REQ       0x766c7362

#define NOC_QOS_PRIORITYn_ADDR(n)	(0x8 + (n * 0x1000))
#define NOC_QOS_PRIORITY_MASK		0xf
#define NOC_QOS_PRIORITY_P1_SHIFT	0x2
#define NOC_QOS_PRIORITY_P0_SHIFT	0x3

#define NOC_QOS_MODEn_ADDR(n)		(0xc + (n * 0x1000))
#define NOC_QOS_MODEn_MASK		0x3

#define NOC_QOS_MODE_FIXED		0x0

#define to_qcom_provider(_provider) \
	container_of(_provider, struct qcom_icc_provider, provider)

static const struct clk_bulk_data bus_clocks[] = {
	{ .id = "bus" },
	{ .id = "bus_a" },
};

/**
 * struct qcom_icc_provider - Qualcomm specific interconnect provider
 * @provider: generic interconnect provider
 * @bus_clks: the clk_bulk_data table of bus clocks
 * @num_clks: the total number of clk_bulk_data entries
 * @mmio: NoC base iospace
 */
struct qcom_icc_provider {
	struct icc_provider provider;
	struct clk_bulk_data *bus_clks;
	int num_clks;
	struct regmap *regmap;
	void __iomem *mmio;
};

#define MDM9607_MAX_LINKS	15

/**
 * struct qcom_icc_qos - Qualcomm specific interconnect QoS parameters
 * @areq_prio: node requests priority
 * @prio_level: priority level for bus communication
 * @limit_commands: activate/deactivate limiter mode during runtime
 * @ap_owned: indicates if the node is owned by the AP or by the RPM
 * @qos_mode: default qos mode for this node
 * @qos_port: qos port number for finding qos registers of this node
 */
struct qcom_icc_qos {
	u32 areq_prio;
	u32 prio_level;
	bool limit_commands;
	bool ap_owned;
	int qos_mode;
	int qos_port;
};

/**
 * struct qcom_icc_node - Qualcomm specific interconnect nodes
 * @name: the node name used in debugfs
 * @id: a unique node identifier
 * @links: an array of nodes where we can go next while traversing
 * @num_links: the total number of @links
 * @buswidth: width of the interconnect between a node and the bus (bytes)
 * @mas_rpm_id:	RPM ID for devices that are bus masters
 * @slv_rpm_id:	RPM ID for devices that are bus slaves
 * @rate: current bus clock rate in Hz
 */
struct qcom_icc_node {
	unsigned char *name;
	u16 id;
	u16 links[MDM9607_MAX_LINKS];
	u16 num_links;
	u16 buswidth;
	int mas_rpm_id;
	int slv_rpm_id;
	struct qcom_icc_qos qos;
	u64 rate;
};

struct qcom_icc_desc {
	struct qcom_icc_node **nodes;
	size_t num_nodes;
	const struct regmap_config *regmap_cfg;
};

#define DEFINE_QNODE(_name, _id, _buswidth, _mas_rpm_id, _slv_rpm_id,	\
		     _ap_owned, _qos_mode, _qos_prio, _qos_port, ...)	\
		static struct qcom_icc_node _name = {			\
		.name = #_name,						\
		.id = _id,						\
		.buswidth = _buswidth,					\
		.mas_rpm_id = _mas_rpm_id,				\
		.slv_rpm_id = _slv_rpm_id,				\
		.qos.ap_owned = _ap_owned,				\
		.qos.qos_mode = _qos_mode,				\
		.qos.areq_prio = _qos_prio,				\
		.qos.prio_level = _qos_prio,				\
		.qos.qos_port = _qos_port,				\
		.num_links = ARRAY_SIZE(((int[]){ __VA_ARGS__ })),	\
		.links = { __VA_ARGS__ },				\
	}

#define NO_CONNECTION	0

/* TODO: verify qcom,blacklist downstream */

/* BIMC nodes */
DEFINE_QNODE(mas_apps_proc, MDM9607_BIMC_MASTER_APPS_PROC, 8, -1, -1, true, NOC_QOS_MODE_FIXED, 0, 0, MDM9607_BIMC_SLAVE_BIMC_PCNOC, MDM9607_BIMC_SLAVE_EBI);
DEFINE_QNODE(mas_pcnoc_bimc_1, MDM9607_BIMC_MASTER_PCNOC_BIMC_1, 8, 139, -1, false, -1, 0, -1, MDM9607_BIMC_SLAVE_EBI);
DEFINE_QNODE(mas_tcu_0, MDM9607_BIMC_MASTER_TCU_0, 8, -1, -1, true, NOC_QOS_MODE_FIXED, 2, 5, MDM9607_BIMC_SLAVE_EBI);
DEFINE_QNODE(slv_ebi, MDM9607_BIMC_SLAVE_EBI, 8, -1, 0, false, -1, 0, -1, NO_CONNECTION);
DEFINE_QNODE(slv_bimc_pcnoc, MDM9607_BIMC_SLAVE_BIMC_PCNOC, 8, -1, 202, false, -1, 0, -1, MDM9607_PCNOC_MASTER_BIMC_PCNOC);

static struct qcom_icc_node *mdm9607_bimc_nodes[] = {
	[MASTER_APPS_PROC] = &mas_apps_proc,
	[MASTER_PCNOC_BIMC_1] = &mas_pcnoc_bimc_1,
	[MASTER_TCU_0] = &mas_tcu_0,
	[SLAVE_EBI] = &slv_ebi,
	[SLAVE_BIMC_PCNOC] = &slv_bimc_pcnoc,
};

static const struct regmap_config mdm9607_bimc_regmap_config = {
	.reg_bits	= 32,
	.reg_stride	= 4,
	.val_bits	= 32,
	.max_register	= 0x58000,
	.fast_io	= true,
};

static struct qcom_icc_desc mdm9607_bimc = {
	.nodes = mdm9607_bimc_nodes,
	.num_nodes = ARRAY_SIZE(mdm9607_bimc_nodes),
	.regmap_cfg = &mdm9607_bimc_regmap_config,
};

/* PCNoC nodes */
DEFINE_QNODE(mas_qdss_bam, MDM9607_PCNOC_MASTER_QDSS_BAM, 4, 19, -1, false, -1, 0, -1, MDM9607_PCNOC_MASTER_QDSS_INT, MDM9607_PCNOC_SLAVE_QDSS_INT);
DEFINE_QNODE(mas_bimc_pcnoc, MDM9607_PCNOC_MASTER_BIMC_PCNOC, 8, 140, -1, false, -1, 0, -1, MDM9607_PCNOC_MASTER_PCNOC_INT_0, MDM9607_PCNOC_SLAVE_PCNOC_INT_0, MDM9607_PCNOC_MASTER_PCNOC_INT_2, MDM9607_PCNOC_SLAVE_PCNOC_INT_2, MDM9607_PCNOC_SLAVE_CATS_0);
DEFINE_QNODE(mas_qdss_etr, MDM9607_PCNOC_MASTER_QDSS_ETR, 8, 31, -1, false, -1, 0, -1, MDM9607_PCNOC_MASTER_QDSS_INT, MDM9607_PCNOC_SLAVE_QDSS_INT);
DEFINE_QNODE(mas_audio, MDM9607_PCNOC_MASTER_AUDIO, 4, 78, -1, false, -1, 0, -1, MDM9607_PCNOC_MASTER_PCNOC_M_0, MDM9607_PCNOC_SLAVE_PCNOC_M_0);
DEFINE_QNODE(mas_qpic, MDM9607_PCNOC_MASTER_QPIC, 4, 58, -1, false, -1, 0, -1, MDM9607_PCNOC_MASTER_PCNOC_M_0, MDM9607_PCNOC_SLAVE_PCNOC_M_0);
DEFINE_QNODE(mas_hsic, MDM9607_PCNOC_MASTER_HSIC, 4, 40, -1, false, -1, 0, -1, MDM9607_PCNOC_MASTER_PCNOC_M_0, MDM9607_PCNOC_SLAVE_PCNOC_M_0);
DEFINE_QNODE(mas_blsp_1, MDM9607_PCNOC_MASTER_BLSP_1, 4, 41, -1, false, -1, 0, -1, MDM9607_PCNOC_MASTER_PCNOC_M_1, MDM9607_PCNOC_SLAVE_PCNOC_M_1);
DEFINE_QNODE(mas_usb_hs1, MDM9607_PCNOC_MASTER_USB_HS1, 4, 42, -1, false, -1, 0, -1, MDM9607_PCNOC_MASTER_PCNOC_M_1, MDM9607_PCNOC_SLAVE_PCNOC_M_1);
DEFINE_QNODE(mas_crypto, MDM9607_PCNOC_MASTER_MASTER_CRYPTO, 8, -1, -1, true, NOC_QOS_MODE_FIXED, 2, 0, MDM9607_PCNOC_MASTER_PCNOC_INT_3, MDM9607_PCNOC_SLAVE_PCNOC_INT_3);
DEFINE_QNODE(mas_sdcc_1, MDM9607_PCNOC_MASTER_SDCC_1, 8, 33, -1, false, -1, 0, -1, MDM9607_PCNOC_MASTER_PCNOC_INT_3, MDM9607_PCNOC_SLAVE_PCNOC_INT_3);
DEFINE_QNODE(mas_sdcc_2, MDM9607_PCNOC_MASTER_SDCC_2, 8, 35, -1, false, -1, 0, -1, MDM9607_PCNOC_MASTER_PCNOC_INT_3, MDM9607_PCNOC_SLAVE_PCNOC_INT_3);
DEFINE_QNODE(mas_xi_usb_hs1, MDM9607_PCNOC_MASTER_XI_USB_HS1, 8, 138, -1, false, -1, 0, -1, MDM9607_PCNOC_SLAVE_PCNOC_BIMC_1, MDM9607_PCNOC_MASTER_PCNOC_INT_2, MDM9607_PCNOC_SLAVE_PCNOC_INT_2);
DEFINE_QNODE(mas_xi_hsic, MDM9607_PCNOC_MASTER_XI_HSIC, 8, 141, -1, false, -1, 0, -1, MDM9607_PCNOC_SLAVE_PCNOC_BIMC_1, MDM9607_PCNOC_MASTER_PCNOC_INT_2, MDM9607_PCNOC_SLAVE_PCNOC_INT_2);
/* TODO: fix QoS */
DEFINE_QNODE(mas_sgmii, MDM9607_PCNOC_MASTER_SGMII, 8, 142, -1, true, -1, 0, -1, MDM9607_PCNOC_SLAVE_PCNOC_BIMC_1, MDM9607_PCNOC_MASTER_PCNOC_INT_2, MDM9607_PCNOC_SLAVE_PCNOC_INT_2);

DEFINE_QNODE(slv_pcnoc_bimc_1, MDM9607_PCNOC_SLAVE_PCNOC_BIMC_1, 8, -1, 203, false, -1, 0, -1, MDM9607_BIMC_MASTER_PCNOC_BIMC_1);
DEFINE_QNODE(slv_qdss_stm, MDM9607_PCNOC_SLAVE_QDSS_STM, 4, -1, -1, true, -1, 0, -1, NO_CONNECTION);
DEFINE_QNODE(slv_cats_0, MDM9607_PCNOC_SLAVE_CATS_0, 8, -1, -1, true, -1, 0, -1, NO_CONNECTION);
DEFINE_QNODE(slv_imem, MDM9607_PCNOC_SLAVE_IMEM, 8, -1, -1, true, -1, 0, -1, NO_CONNECTION);
DEFINE_QNODE(slv_tcsr, MDM9607_PCNOC_SLAVE_TCSR, 4, -1, 50, false, -1, 0, -1, NO_CONNECTION);
DEFINE_QNODE(slv_sdcc_1, MDM9607_PCNOC_SLAVE_SDCC_1, 4, -1, 31, false, -1, 0, -1, NO_CONNECTION);
DEFINE_QNODE(slv_blsp_1, MDM9607_PCNOC_SLAVE_BLSP_1, 4, -1, 39, false, -1, 0, -1, NO_CONNECTION);
DEFINE_QNODE(slv_sgmii, MDM9607_PCNOC_SLAVE_SGMII, 4, -1, -1, true, -1, 0, -1, NO_CONNECTION);
DEFINE_QNODE(slv_crypto_0_cfg, MDM9607_PCNOC_SLAVE_CRYPTO_0_CFG, 4, -1, -1, true, -1, 0, -1, NO_CONNECTION);
DEFINE_QNODE(slv_message_ram, MDM9607_PCNOC_SLAVE_MESSAGE_RAM, 4, -1, 55, false, -1, 0, -1, NO_CONNECTION);
DEFINE_QNODE(slv_pdm, MDM9607_PCNOC_SLAVE_PDM, 4, -1, 41, false, -1, 0, -1, NO_CONNECTION);
DEFINE_QNODE(slv_prng, MDM9607_PCNOC_SLAVE_PRNG, 4, -1, 44, false, -1, 0, -1, NO_CONNECTION);
DEFINE_QNODE(slv_usb2, MDM9607_PCNOC_SLAVE_USB2, 4, -1, -1, true, -1, 0, -1, NO_CONNECTION);
DEFINE_QNODE(slv_sdcc_2, MDM9607_PCNOC_SLAVE_SDCC_2, 4, -1, 33, false, -1, 0, -1, NO_CONNECTION);
DEFINE_QNODE(slv_audio, MDM9607_PCNOC_SLAVE_AUDIO, 4, -1, 105, false, -1, 0, -1, NO_CONNECTION);
DEFINE_QNODE(slv_hsic, MDM9607_PCNOC_SLAVE_HSIC, 4, -1, 38, false, -1, 0, -1, NO_CONNECTION);
DEFINE_QNODE(slv_usb_phy, MDM9607_PCNOC_SLAVE_USB_PHY, 4, -1, -1, true, -1, 0, -1, NO_CONNECTION);
DEFINE_QNODE(slv_tlmm, MDM9607_PCNOC_SLAVE_TLMM, 4, -1, 51, false, -1, 0, -1, NO_CONNECTION);
DEFINE_QNODE(slv_imem_cfg, MDM9607_PCNOC_SLAVE_IMEM_CFG, 4, -1, -1, true, -1, 0, -1, NO_CONNECTION);
DEFINE_QNODE(slv_pmic_arb, MDM9607_PCNOC_SLAVE_PMIC_ARB, 4, -1, 59, false, -1, 0, -1, NO_CONNECTION);
DEFINE_QNODE(slv_tcu, MDM9607_PCNOC_SLAVE_TCU, 8, -1, -1, true, -1, 0, -1, NO_CONNECTION);
DEFINE_QNODE(slv_qpic, MDM9607_PCNOC_SLAVE_QPIC, 4, -1, 80, false, -1, 0, -1, NO_CONNECTION);

/* Internal nodes */
DEFINE_QNODE(mas_pcnoc_m_0, MDM9607_PCNOC_MASTER_PCNOC_M_0, 4, 87, -1, false, -1, 0, -1, MDM9607_PCNOC_SLAVE_PCNOC_BIMC_1, MDM9607_PCNOC_MASTER_PCNOC_INT_2, MDM9607_PCNOC_SLAVE_PCNOC_INT_2);
DEFINE_QNODE(mas_pcnoc_m_1, MDM9607_PCNOC_MASTER_PCNOC_M_1, 4, 88, -1, false, -1, 0, -1, MDM9607_PCNOC_SLAVE_PCNOC_BIMC_1, MDM9607_PCNOC_MASTER_PCNOC_INT_2, MDM9607_PCNOC_SLAVE_PCNOC_INT_2);
DEFINE_QNODE(mas_qdss_int, MDM9607_PCNOC_MASTER_QDSS_INT, 8, 98, -1, false, -1, 0, -1, MDM9607_PCNOC_SLAVE_PCNOC_BIMC_1, MDM9607_PCNOC_MASTER_PCNOC_INT_0, MDM9607_PCNOC_SLAVE_PCNOC_INT_0);
DEFINE_QNODE(mas_pcnoc_int_0, MDM9607_PCNOC_MASTER_PCNOC_INT_0, 8, 85, -1, true, -1, 0, -1, MDM9607_PCNOC_SLAVE_IMEM, MDM9607_PCNOC_SLAVE_QDSS_STM);
DEFINE_QNODE(mas_pcnoc_int_2, MDM9607_PCNOC_MASTER_PCNOC_INT_2, 8, 124, -1, false, -1, 0, -1, MDM9607_PCNOC_MASTER_PCNOC_S_1, MDM9607_PCNOC_SLAVE_PCNOC_S_1, MDM9607_PCNOC_MASTER_PCNOC_S_0, MDM9607_PCNOC_SLAVE_PCNOC_S_0, MDM9607_PCNOC_MASTER_PCNOC_S_4, MDM9607_PCNOC_SLAVE_PCNOC_S_4, MDM9607_PCNOC_MASTER_PCNOC_S_5, MDM9607_PCNOC_SLAVE_PCNOC_S_5, MDM9607_PCNOC_MASTER_PCNOC_S_3, MDM9607_PCNOC_SLAVE_PCNOC_S_3, MDM9607_PCNOC_SLAVE_TCU);
DEFINE_QNODE(mas_pcnoc_int_3, MDM9607_PCNOC_MASTER_PCNOC_INT_3, 8, 125, -1, false, -1, 0, -1, MDM9607_PCNOC_SLAVE_PCNOC_BIMC_1, MDM9607_PCNOC_MASTER_PCNOC_INT_2, MDM9607_PCNOC_SLAVE_PCNOC_INT_2);
DEFINE_QNODE(mas_pcnoc_s_0, MDM9607_PCNOC_MASTER_PCNOC_S_0, 4, 89, -1, false, -1, 0, -1, MDM9607_PCNOC_SLAVE_TCSR, MDM9607_PCNOC_SLAVE_SDCC_1, MDM9607_PCNOC_SLAVE_BLSP_1, MDM9607_PCNOC_SLAVE_SGMII);
DEFINE_QNODE(mas_pcnoc_s_1, MDM9607_PCNOC_MASTER_PCNOC_S_1, 4, 90, -1, false, -1, 0, -1, MDM9607_PCNOC_SLAVE_USB2, MDM9607_PCNOC_SLAVE_CRYPTO_0_CFG, MDM9607_PCNOC_SLAVE_PRNG, MDM9607_PCNOC_SLAVE_PDM, MDM9607_PCNOC_SLAVE_MESSAGE_RAM);
DEFINE_QNODE(mas_pcnoc_s_2, MDM9607_PCNOC_MASTER_PCNOC_S_2, 4, 91, -1, false, -1, 0, -1, MDM9607_PCNOC_SLAVE_HSIC, MDM9607_PCNOC_SLAVE_SDCC_2, MDM9607_PCNOC_SLAVE_AUDIO);
DEFINE_QNODE(mas_pcnoc_s_3, MDM9607_PCNOC_MASTER_PCNOC_S_3, 4, 92, -1, true, -1, 0, -1, MDM9607_PCNOC_SLAVE_USB_PHY);
DEFINE_QNODE(mas_pcnoc_s_4, MDM9607_PCNOC_MASTER_PCNOC_S_4, 4, 93, -1, false, -1, 0, -1, MDM9607_PCNOC_SLAVE_IMEM_CFG, MDM9607_PCNOC_SLAVE_PMIC_ARB);
DEFINE_QNODE(mas_pcnoc_s_5, MDM9607_PCNOC_MASTER_PCNOC_S_5, 4, 129, -1, false, -1, 0, -1, MDM9607_PCNOC_SLAVE_TLMM);

DEFINE_QNODE(slv_pcnoc_m_0, MDM9607_PCNOC_SLAVE_PCNOC_M_0, 4, -1, 116, false, -1, 0, -1, MDM9607_PCNOC_SLAVE_PCNOC_BIMC_1, MDM9607_PCNOC_MASTER_PCNOC_INT_2, MDM9607_PCNOC_SLAVE_PCNOC_INT_2);
DEFINE_QNODE(slv_pcnoc_m_1, MDM9607_PCNOC_SLAVE_PCNOC_M_1, 4, -1, 117, false, -1, 0, -1, MDM9607_PCNOC_SLAVE_PCNOC_BIMC_1, MDM9607_PCNOC_MASTER_PCNOC_INT_2, MDM9607_PCNOC_SLAVE_PCNOC_INT_2);
DEFINE_QNODE(slv_qdss_int, MDM9607_PCNOC_SLAVE_QDSS_INT, 8, -1, 128, false, -1, 0, -1, MDM9607_PCNOC_SLAVE_PCNOC_BIMC_1, MDM9607_PCNOC_MASTER_PCNOC_INT_0, MDM9607_PCNOC_SLAVE_PCNOC_INT_0);
DEFINE_QNODE(slv_pcnoc_int_0, MDM9607_PCNOC_SLAVE_PCNOC_INT_0, 8, -1, 114, true, -1, 0, -1, MDM9607_PCNOC_SLAVE_IMEM, MDM9607_PCNOC_SLAVE_QDSS_STM);
DEFINE_QNODE(slv_pcnoc_int_2, MDM9607_PCNOC_SLAVE_PCNOC_INT_2, 8, -1, 184, false, -1, 0, -1, MDM9607_PCNOC_MASTER_PCNOC_S_1, MDM9607_PCNOC_SLAVE_PCNOC_S_1, MDM9607_PCNOC_MASTER_PCNOC_S_0, MDM9607_PCNOC_SLAVE_PCNOC_S_0, MDM9607_PCNOC_MASTER_PCNOC_S_4, MDM9607_PCNOC_SLAVE_PCNOC_S_4, MDM9607_PCNOC_MASTER_PCNOC_S_5, MDM9607_PCNOC_SLAVE_PCNOC_S_5, MDM9607_PCNOC_MASTER_PCNOC_S_3, MDM9607_PCNOC_SLAVE_PCNOC_S_3, MDM9607_PCNOC_SLAVE_TCU);
DEFINE_QNODE(slv_pcnoc_int_3, MDM9607_PCNOC_SLAVE_PCNOC_INT_3, 8, -1, 185, false, -1, 0, -1, MDM9607_PCNOC_SLAVE_PCNOC_BIMC_1, MDM9607_PCNOC_MASTER_PCNOC_INT_2, MDM9607_PCNOC_SLAVE_PCNOC_INT_2);
DEFINE_QNODE(slv_pcnoc_s_0, MDM9607_PCNOC_SLAVE_PCNOC_S_0, 4, -1, 118, false, -1, 0, -1, MDM9607_PCNOC_SLAVE_TCSR, MDM9607_PCNOC_SLAVE_SDCC_1, MDM9607_PCNOC_SLAVE_BLSP_1, MDM9607_PCNOC_SLAVE_SGMII);
DEFINE_QNODE(slv_pcnoc_s_1, MDM9607_PCNOC_SLAVE_PCNOC_S_1, 4, -1, 119, false, -1, 0, -1, MDM9607_PCNOC_SLAVE_USB2, MDM9607_PCNOC_SLAVE_CRYPTO_0_CFG, MDM9607_PCNOC_SLAVE_PRNG, MDM9607_PCNOC_SLAVE_PDM, MDM9607_PCNOC_SLAVE_MESSAGE_RAM);
DEFINE_QNODE(slv_pcnoc_s_2, MDM9607_PCNOC_SLAVE_PCNOC_S_2, 4, -1, 120, false, -1, 0, -1, MDM9607_PCNOC_SLAVE_HSIC, MDM9607_PCNOC_SLAVE_SDCC_2, MDM9607_PCNOC_SLAVE_AUDIO);
DEFINE_QNODE(slv_pcnoc_s_3, MDM9607_PCNOC_SLAVE_PCNOC_S_3, 4, -1, 121, true, -1, 0, -1, MDM9607_PCNOC_SLAVE_USB_PHY);
DEFINE_QNODE(slv_pcnoc_s_4, MDM9607_PCNOC_SLAVE_PCNOC_S_4, 4, -1, 122, false, -1, 0, -1, MDM9607_PCNOC_SLAVE_IMEM_CFG, MDM9607_PCNOC_SLAVE_PMIC_ARB);
DEFINE_QNODE(slv_pcnoc_s_5, MDM9607_PCNOC_SLAVE_PCNOC_S_5, 4, -1, 189, false, -1, 0, -1, MDM9607_PCNOC_SLAVE_TLMM);

static struct qcom_icc_node *mdm9607_pcnoc_nodes[] = {
	[MASTER_QDSS_BAM] = &mas_qdss_bam,
	[MASTER_BIMC_PCNOC] = &mas_bimc_pcnoc,
	[MASTER_QDSS_ETR] = &mas_qdss_etr,
	[MASTER_AUDIO] = &mas_audio,
	[MASTER_QPIC] = &mas_qpic,
	[MASTER_HSIC] = &mas_hsic,
	[MASTER_BLSP_1] = &mas_blsp_1,
	[MASTER_USB_HS1] = &mas_usb_hs1,
	[MASTER_CRYPTO] = &mas_crypto,
	[MASTER_SDCC_1] = &mas_sdcc_1,
	[MASTER_SDCC_2] = &mas_sdcc_2,
	[MASTER_XI_USB_HS1] = &mas_xi_usb_hs1,
	[MASTER_XI_HSIC] = &mas_xi_hsic,
	[MASTER_SGMII] = &mas_sgmii,
	[SLAVE_PCNOC_BIMC_1] = &slv_pcnoc_bimc_1,
	[SLAVE_QDSS_STM] = &slv_qdss_stm,
	[SLAVE_CATS_0] = &slv_cats_0,
	[SLAVE_IMEM] = &slv_imem,
	[SLAVE_TCSR] = &slv_tcsr,
	[SLAVE_SDCC_1] = &slv_sdcc_1,
	[SLAVE_BLSP_1] = &slv_blsp_1,
	[SLAVE_SGMII] = &slv_sgmii,
	[SLAVE_CRYPTO_0_CFG] = &slv_crypto_0_cfg,
	[SLAVE_MESSAGE_RAM] = &slv_message_ram,
	[SLAVE_PDM] = &slv_pdm,
	[SLAVE_PRNG] = &slv_prng,
	[SLAVE_USB2] = &slv_usb2,
	[SLAVE_SDCC_2] = &slv_sdcc_2,
	[SLAVE_AUDIO] = &slv_audio,
	[SLAVE_HSIC] = &slv_hsic,
	[SLAVE_USB_PHY] = &slv_usb_phy,
	[SLAVE_TLMM] = &slv_tlmm,
	[SLAVE_IMEM_CFG] = &slv_imem_cfg,
	[SLAVE_PMIC_ARB] = &slv_pmic_arb,
	[SLAVE_TCU] = &slv_tcu,
	[SLAVE_QPIC] = &slv_qpic,

	/* Internal nodes */
	[MASTER_PCNOC_M_0] = &mas_pcnoc_m_0,
	[MASTER_PCNOC_M_1] = &mas_pcnoc_m_1,
	[MASTER_QDSS_INT] = &mas_qdss_int,
	[MASTER_PCNOC_INT_0] = &mas_pcnoc_int_0,
	[MASTER_PCNOC_INT_2] = &mas_pcnoc_int_2,
	[MASTER_PCNOC_INT_3] = &mas_pcnoc_int_3,
	[MASTER_PCNOC_S_0] = &mas_pcnoc_s_0,
	[MASTER_PCNOC_S_1] = &mas_pcnoc_s_1,
	[MASTER_PCNOC_S_2] = &mas_pcnoc_s_2,
	[MASTER_PCNOC_S_3] = &mas_pcnoc_s_3,
	[MASTER_PCNOC_S_4] = &mas_pcnoc_s_4,
	[MASTER_PCNOC_S_5] = &mas_pcnoc_s_5,

	[SLAVE_PCNOC_M_0] = &slv_pcnoc_m_0,
	[SLAVE_PCNOC_M_1] = &slv_pcnoc_m_1,
	[SLAVE_QDSS_INT] = &slv_qdss_int,
	[SLAVE_PCNOC_INT_0] = &slv_pcnoc_int_0,
	[SLAVE_PCNOC_INT_2] = &slv_pcnoc_int_2,
	[SLAVE_PCNOC_INT_3] = &slv_pcnoc_int_3,
	[SLAVE_PCNOC_S_0] = &slv_pcnoc_s_0,
	[SLAVE_PCNOC_S_1] = &slv_pcnoc_s_1,
	[SLAVE_PCNOC_S_2] = &slv_pcnoc_s_2,
	[SLAVE_PCNOC_S_3] = &slv_pcnoc_s_3,
	[SLAVE_PCNOC_S_4] = &slv_pcnoc_s_4,
	[SLAVE_PCNOC_S_5] = &slv_pcnoc_s_5,
};

static const struct regmap_config mdm9607_pcnoc_regmap_config = {
	.reg_bits	= 32,
	.reg_stride	= 4,
	.val_bits	= 32,
	.max_register	= 0x15080,
	.fast_io	= true,
};

static struct qcom_icc_desc mdm9607_pcnoc = {
	.nodes = mdm9607_pcnoc_nodes,
	.num_nodes = ARRAY_SIZE(mdm9607_pcnoc_nodes),
	.regmap_cfg = &mdm9607_pcnoc_regmap_config,
};

static int qcom_icc_noc_set_qos_priority(struct regmap *rmap, int qos_port,
					   u32 areq_prio, u32 prio_level)

{
	u32 val;
	int rc;

	/* Must be updated one at a time, P1 first, P0 last */
	val = areq_prio << NOC_QOS_PRIORITY_P1_SHIFT;
	rc = regmap_update_bits(rmap, NOC_QOS_PRIORITYn_ADDR(qos_port),
				NOC_QOS_PRIORITY_MASK, val);
	if (rc)
		return rc;

	val = prio_level << NOC_QOS_PRIORITY_P0_SHIFT;
	return regmap_update_bits(rmap, NOC_QOS_PRIORITYn_ADDR(qos_port),
				  NOC_QOS_PRIORITY_MASK, val);
}

static int qcom_icc_set_noc_qos(struct icc_node *src, u64 max_bw)
{
	struct qcom_icc_provider *qp;
	struct qcom_icc_node *qn;
	struct qcom_icc_qos *qos;
	struct icc_provider *provider;
	u32 mode;
	int rc = 0;

	qn = src->data;
	provider = src->provider;
	qp = to_qcom_provider(provider);
	qos = &qn->qos;

	if (qos->qos_port < 0) {
		dev_dbg(src->provider->dev,
			"NoC QoS: Skipping %s: vote aggregated on parent.\n",
			qn->name);
		return 0;
	}

	if (qos->qos_mode != -1)
		mode = qn->qos.qos_mode;

	if (mode == NOC_QOS_MODE_FIXED) {
		dev_dbg(src->provider->dev, "NoC QoS: %s: Set Fixed mode\n",
			qn->name);
		rc = qcom_icc_noc_set_qos_priority(qp->regmap, qos->qos_port,
					       qos->areq_prio,
					       qos->prio_level);
		if (rc)
			return rc;
	}

	return regmap_update_bits(qp->regmap,
				  NOC_QOS_MODEn_ADDR(qos->qos_port),
				  NOC_QOS_MODEn_MASK, mode);
}

static int qcom_icc_rpm_set(int mas_rpm_id, int slv_rpm_id, u64 sum_bw)
{
	int ret = 0;

	if (mas_rpm_id != -1) {
		ret = qcom_icc_rpm_smd_send(QCOM_SMD_RPM_ACTIVE_STATE,
					    RPM_BUS_MASTER_REQ,
					    mas_rpm_id,
					    sum_bw);
		if (ret) {
			pr_err("qcom_icc_rpm_smd_send mas %d error %d\n",
			       mas_rpm_id, ret);
			return ret;
		}
	}

	if (slv_rpm_id != -1) {
		ret = qcom_icc_rpm_smd_send(QCOM_SMD_RPM_ACTIVE_STATE,
					    RPM_BUS_SLAVE_REQ,
					    slv_rpm_id,
					    sum_bw);
		if (ret) {
			pr_err("qcom_icc_rpm_smd_send slv %d error %d\n",
			       slv_rpm_id, ret);
			return ret;
		}
	}

	return ret;
}

static int qcom_icc_set(struct icc_node *src, struct icc_node *dst)
{
	struct qcom_icc_provider *qp;
	struct qcom_icc_node *qn;
	struct icc_provider *provider;
	struct icc_node *n;
	u64 sum_bw;
	u64 max_peak_bw;
	u64 rate;
	u32 agg_avg = 0;
	u32 agg_peak = 0;
	int ret, i;

	qn = src->data;
	provider = src->provider;
	qp = to_qcom_provider(provider);

	list_for_each_entry(n, &provider->nodes, node_list)
		provider->aggregate(n, 0, n->avg_bw, n->peak_bw,
				    &agg_avg, &agg_peak);

	sum_bw = icc_units_to_bps(agg_avg);
	max_peak_bw = icc_units_to_bps(agg_peak);

	if (!qn->qos.ap_owned) {
		/* send bandwidth request message to the RPM processor */
		ret = qcom_icc_rpm_set(qn->mas_rpm_id, qn->slv_rpm_id, sum_bw);
		if (ret)
			return ret;
	} else if (qn->qos.qos_mode != -1) {
		/* set bandwidth directly from the AP */
		ret = qcom_icc_set_noc_qos(src, sum_bw);
		if (ret)
			return ret;
	}

	rate = max(sum_bw, max_peak_bw);

	do_div(rate, qn->buswidth);

	if (qn->rate == rate)
		return 0;

	for (i = 0; i < qp->num_clks; i++) {
		ret = clk_set_rate(qp->bus_clks[i].clk, rate);
		if (ret) {
			pr_err("%s clk_set_rate error: %d\n",
			       qp->bus_clks[i].id, ret);
			return ret;
		}
	}

	qn->rate = rate;

	return 0;
}

static int qnoc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct qcom_icc_desc *desc;
	struct icc_onecell_data *data;
	struct icc_provider *provider;
	struct qcom_icc_node **qnodes;
	struct qcom_icc_provider *qp;
	struct icc_node *node;
	struct resource *res;
	size_t num_nodes, i;
	int ret;

	/* wait for the RPM proxy */
	if (!qcom_icc_rpm_smd_available())
		return -EPROBE_DEFER;

	desc = of_device_get_match_data(dev);
	if (!desc)
		return -EINVAL;

	qnodes = desc->nodes;
	num_nodes = desc->num_nodes;

	qp = devm_kzalloc(dev, sizeof(*qp), GFP_KERNEL);
	if (!qp)
		return -ENOMEM;

	data = devm_kzalloc(dev, struct_size(data, nodes, num_nodes),
			    GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	qp->bus_clks = devm_kmemdup(dev, bus_clocks,
				    sizeof(bus_clocks), GFP_KERNEL);
	if (!qp->bus_clks)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	qp->mmio = devm_ioremap_resource(dev, res);
	if (IS_ERR(qp->mmio)) {
		dev_err(dev, "Cannot ioremap interconnect bus resource\n");
		return PTR_ERR(qp->mmio);
	}

	qp->regmap = devm_regmap_init_mmio(dev, qp->mmio, desc->regmap_cfg);
	if (IS_ERR(qp->regmap)) {
		dev_err(dev, "Cannot regmap interconnect bus resource\n");
		return PTR_ERR(qp->regmap);
	}

	ret = devm_clk_bulk_get(dev, qp->num_clks, qp->bus_clks);
	if (ret)
		return ret;

	ret = clk_bulk_prepare_enable(qp->num_clks, qp->bus_clks);
	if (ret)
		return ret;

	provider = &qp->provider;
	INIT_LIST_HEAD(&provider->nodes);
	provider->dev = dev;
	provider->set = qcom_icc_set;
	provider->aggregate = icc_std_aggregate;
	provider->xlate = of_icc_xlate_onecell;
	provider->data = data;

	ret = icc_provider_add(provider);
	if (ret) {
		dev_err(dev, "error adding interconnect provider: %d\n", ret);
		clk_bulk_disable_unprepare(qp->num_clks, qp->bus_clks);
		return ret;
	}

	for (i = 0; i < num_nodes; i++) {
		size_t j;

		node = icc_node_create(qnodes[i]->id);
		if (IS_ERR(node)) {
			ret = PTR_ERR(node);
			goto err;
		}

		node->name = qnodes[i]->name;
		node->data = qnodes[i];
		icc_node_add(node, provider);

		for (j = 0; j < qnodes[i]->num_links; j++)
			icc_link_create(node, qnodes[i]->links[j]);

		data->nodes[i] = node;
	}
	data->num_nodes = num_nodes;
	platform_set_drvdata(pdev, qp);

	return 0;
err:
	icc_nodes_remove(provider);
	clk_bulk_disable_unprepare(qp->num_clks, qp->bus_clks);
	icc_provider_del(provider);

	return ret;
}

static int qnoc_remove(struct platform_device *pdev)
{
	struct qcom_icc_provider *qp = platform_get_drvdata(pdev);

	icc_nodes_remove(&qp->provider);
	clk_bulk_disable_unprepare(qp->num_clks, qp->bus_clks);
	return icc_provider_del(&qp->provider);
}

static const struct of_device_id mdm9607_noc_of_match[] = {
	{ .compatible = "qcom,mdm9607-bimc", .data = &mdm9607_bimc },
	{ .compatible = "qcom,mdm9607-pcnoc", .data = &mdm9607_pcnoc },
	{ }
};
MODULE_DEVICE_TABLE(of, mdm9607_noc_of_match);

static struct platform_driver mdm9607_noc_driver = {
	.probe = qnoc_probe,
	.remove = qnoc_remove,
	.driver = {
		.name = "qnoc-mdm9607",
		.of_match_table = mdm9607_noc_of_match,
	},
};
module_platform_driver(mdm9607_noc_driver);
MODULE_AUTHOR("Konrad Dybcio <konrad.dybcio@somainline.org>");
MODULE_DESCRIPTION("Qualcomm MDM9607 NoC driver");
MODULE_LICENSE("GPL v2");
