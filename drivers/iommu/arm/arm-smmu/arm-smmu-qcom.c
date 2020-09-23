// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2019, The Linux Foundation. All rights reserved.
 */

#include <linux/of_device.h>
#include <linux/qcom_scm.h>

#include "arm-smmu.h"

struct qcom_smmu {
	struct arm_smmu_device smmu;
};

static const struct of_device_id qcom_smmu_client_of_match[] __maybe_unused = {
	{ .compatible = "qcom,adreno" },
	{ .compatible = "qcom,mdp4" },
	{ .compatible = "qcom,mdss" },
	{ .compatible = "qcom,sc7180-mdss" },
	{ .compatible = "qcom,sc7180-mss-pil" },
	{ .compatible = "qcom,sdm845-mdss" },
	{ .compatible = "qcom,sdm845-mss-pil" },
	{ }
};

static int qcom_smmu_def_domain_type(struct device *dev)
{
	const struct of_device_id *match =
		of_match_device(qcom_smmu_client_of_match, dev);

	return match ? IOMMU_DOMAIN_IDENTITY : 0;
}

static int qcom_sdm845_smmu500_reset(struct arm_smmu_device *smmu)
{
	int ret;

	/*
	 * To address performance degradation in non-real time clients,
	 * such as USB and UFS, turn off wait-for-safe on sdm845 based boards,
	 * such as MTP and db845, whose firmwares implement secure monitor
	 * call handlers to turn on/off the wait-for-safe logic.
	 */
	ret = qcom_scm_qsmmu500_wait_safe_toggle(0);
	if (ret)
		dev_warn(smmu->dev, "Failed to turn off SAFE logic\n");

	return ret;
}

static int qcom_smmu500_reset(struct arm_smmu_device *smmu)
{
	const struct device_node *np = smmu->dev->of_node;

	arm_mmu500_reset(smmu);

	if (of_device_is_compatible(np, "qcom,sdm845-smmu-500"))
		return qcom_sdm845_smmu500_reset(smmu);

	return 0;
}

static const struct arm_smmu_impl qcom_smmu500_impl = {
	.def_domain_type = qcom_smmu_def_domain_type,
	.reset = qcom_smmu500_reset,
};

static int qcom_smmuv2_cfg_probe(struct arm_smmu_device *smmu)
{
	/*
	 * Some IOMMUs are getting set-up for Shared Virtual Address, but:
	 * 1. They are secured by the Hypervisor, so any configuration
	 *    change will generate a hyp-fault and crash the system
	 * 2. This 39-bits Virtual Address size deviates from the ARM
	 *    System MMU Architecture specification for SMMUv2, hence
	 *    it is non-standard. In this case, the only way to keep the
	 *    IOMMU as the firmware did configure it, is to hardcode a
	 *    maximum VA size of 39 bits (because of point 1).
	 */
	if (smmu->va_size > 39UL)
		dev_notice(smmu->dev,
			   "\tenabling workaround for QCOM SMMUv2 VA size\n");
	smmu->va_size = min(smmu->va_size, 39UL);

	return 0;
}

static const struct arm_smmu_impl qcom_smmuv2_impl = {
	.cfg_probe = qcom_smmuv2_cfg_probe,
};

struct arm_smmu_device *qcom_smmu_impl_init(struct arm_smmu_device *smmu)
{
	struct qcom_smmu *qsmmu;

	qsmmu = devm_kzalloc(smmu->dev, sizeof(*qsmmu), GFP_KERNEL);
	if (!qsmmu)
		return ERR_PTR(-ENOMEM);

	qsmmu->smmu = *smmu;

	if (of_device_is_compatible(np, "qcom,sdm660-smmu-v2")) {
		qsmmu->smmu.impl = &qcom_smmuv2_impl;
	} else {
		qsmmu->smmu.impl = &qcom_smmu_impl;
	}
	devm_kfree(smmu->dev, smmu);

	return &qsmmu->smmu;
}
