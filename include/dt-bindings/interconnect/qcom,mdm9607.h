// SPDX-License-Identifier: GPL-2.0

#ifndef __DT_BINDINGS_INTERCONNECT_QCOM_MDM9607_H
#define __DT_BINDINGS_INTERCONNECT_QCOM_MDM9607_H

/* BIMC nodes */
#define MASTER_APPS_PROC		0
#define MASTER_PCNOC_BIMC_1		1
#define MASTER_TCU_0			2
#define SLAVE_EBI				3
#define SLAVE_BIMC_PCNOC		4

/* PCNoC nodes */
#define MASTER_QDSS_BAM			0
#define MASTER_BIMC_PCNOC		1
#define MASTER_QDSS_ETR			2
#define MASTER_AUDIO			3
#define MASTER_QPIC				4
#define MASTER_HSIC				5
#define MASTER_BLSP_1			6
#define MASTER_USB_HS1			7
#define MASTER_CRYPTO			8
#define MASTER_SDCC_1			9
#define MASTER_SDCC_2			10
#define MASTER_XI_USB_HS1		11
#define MASTER_XI_HSIC			12
#define MASTER_SGMII			13
#define SLAVE_PCNOC_BIMC_1		14
#define SLAVE_QDSS_STM			15
#define SLAVE_CATS_0			16
#define SLAVE_IMEM				17
#define SLAVE_TCSR				18
#define SLAVE_SDCC_1			19
#define SLAVE_BLSP_1			20
#define SLAVE_SGMII				21
#define SLAVE_CRYPTO_0_CFG		22
#define SLAVE_MESSAGE_RAM		23
#define SLAVE_PDM				24
#define SLAVE_PRNG				25
#define SLAVE_USB2				26
#define SLAVE_SDCC_2			27
#define SLAVE_AUDIO				28
#define SLAVE_HSIC				29
#define SLAVE_USB_PHY			30
#define SLAVE_TLMM				31
#define SLAVE_IMEM_CFG			32
#define SLAVE_PMIC_ARB			33
#define SLAVE_TCU				34
#define SLAVE_QPIC				35

/* Internal nodes */
#define MASTER_PCNOC_M_0		36
#define MASTER_PCNOC_M_1		37
#define MASTER_QDSS_INT			38
#define MASTER_PCNOC_INT_0		39
#define MASTER_PCNOC_INT_2		40
#define MASTER_PCNOC_INT_3		41
#define MASTER_PCNOC_S_0		42
#define MASTER_PCNOC_S_1		43
#define MASTER_PCNOC_S_2		44
#define MASTER_PCNOC_S_3		45
#define MASTER_PCNOC_S_4		46
#define MASTER_PCNOC_S_5		47
#define SLAVE_PCNOC_M_0			48
#define SLAVE_PCNOC_M_1			49
#define SLAVE_QDSS_INT			50
#define SLAVE_PCNOC_INT_0		51
#define SLAVE_PCNOC_INT_2		52
#define SLAVE_PCNOC_INT_3		53
#define SLAVE_PCNOC_S_0			54
#define SLAVE_PCNOC_S_1			55
#define SLAVE_PCNOC_S_2			56
#define SLAVE_PCNOC_S_3			57
#define SLAVE_PCNOC_S_4			58
#define SLAVE_PCNOC_S_5			59

#endif
