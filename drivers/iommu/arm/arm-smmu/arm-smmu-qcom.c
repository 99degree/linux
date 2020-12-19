// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2019, The Linux Foundation. All rights reserved.
 */

#include <linux/of_device.h>
#include <linux/qcom_scm.h>

#include "arm-smmu.h"

struct qcom_smmu {
	struct arm_smmu_device smmu;
	bool bypass_quirk;
	u8 bypass_cbndx;
};

static struct qcom_smmu *to_qcom_smmu(struct arm_smmu_device *smmu)
{
	return container_of(smmu, struct qcom_smmu, smmu);
}

static const struct of_device_id qcom_smmu_client_of_match[] __maybe_unused = {
	{ .compatible = "qcom,adreno" },
	{ .compatible = "qcom,mdp4" },
	{ .compatible = "qcom,mdss" },
        { .compatible = "qcom,mdp5" },
	{ .compatible = "qcom,sdm660-mdss" },
	{ .compatible = "qcom,sdm630-mdss" },
	{ .compatible = "qcom,sc7180-mdss" },
	{ .compatible = "qcom,sc7180-mss-pil" },
	{ .compatible = "qcom,sdm845-mdss" },
	{ .compatible = "qcom,sdm845-mss-pil" },
        { .compatible = "qcom,adreno-smmu" },
	{ }
};

static int qcom_smmu_cfg_probe(struct arm_smmu_device *smmu)
{
	unsigned int last_s2cr = ARM_SMMU_GR0_S2CR(smmu->num_mapping_groups - 1);
	struct qcom_smmu *qsmmu = to_qcom_smmu(smmu);
	u32 reg;
	u32 smr;
	int i;

	/*
	 * With some firmware versions writes to S2CR of type FAULT are
	 * ignored, and writing BYPASS will end up written as FAULT in the
	 * register. Perform a write to S2CR to detect if this is the case and
	 * if so reserve a context bank to emulate bypass streams.
	 */
	reg = FIELD_PREP(ARM_SMMU_S2CR_TYPE, S2CR_TYPE_BYPASS) |
	      FIELD_PREP(ARM_SMMU_S2CR_CBNDX, 0xff) |
	      FIELD_PREP(ARM_SMMU_S2CR_PRIVCFG, S2CR_PRIVCFG_DEFAULT);
	arm_smmu_gr0_write(smmu, last_s2cr, reg);
	reg = arm_smmu_gr0_read(smmu, last_s2cr);
	if (FIELD_GET(ARM_SMMU_S2CR_TYPE, reg) != S2CR_TYPE_BYPASS) {
		qsmmu->bypass_quirk = true;
		qsmmu->bypass_cbndx = smmu->num_context_banks - 1;

		set_bit(qsmmu->bypass_cbndx, smmu->context_map);

		reg = FIELD_PREP(ARM_SMMU_CBAR_TYPE, CBAR_TYPE_S1_TRANS_S2_BYPASS);
		arm_smmu_gr1_write(smmu, ARM_SMMU_GR1_CBAR(qsmmu->bypass_cbndx), reg);
	}

	for (i = 0; i < smmu->num_mapping_groups; i++) {
		smr = arm_smmu_gr0_read(smmu, ARM_SMMU_GR0_SMR(i));

		if (FIELD_GET(ARM_SMMU_SMR_VALID, smr)) {
			smmu->smrs[i].id = FIELD_GET(ARM_SMMU_SMR_ID, smr);
			smmu->smrs[i].mask = FIELD_GET(ARM_SMMU_SMR_MASK, smr);
			smmu->smrs[i].valid = true;

			smmu->s2crs[i].type = S2CR_TYPE_BYPASS;
			smmu->s2crs[i].privcfg = S2CR_PRIVCFG_DEFAULT;
			smmu->s2crs[i].cbndx = 0xff;
		}
	}

	return 0;
}

static void qcom_smmu_write_s2cr(struct arm_smmu_device *smmu, int idx)
{
	struct arm_smmu_s2cr *s2cr = smmu->s2crs + idx;
	struct qcom_smmu *qsmmu = to_qcom_smmu(smmu);
	u32 cbndx = s2cr->cbndx;
	u32 type = s2cr->type;
	u32 reg;

	if (qsmmu->bypass_quirk) {
		if (type == S2CR_TYPE_BYPASS) {
			/*
			 * Firmware with quirky S2CR handling will substitute
			 * BYPASS writes with FAULT, so point the stream to the
			 * reserved context bank and ask for translation on the
			 * stream
			 */
			type = S2CR_TYPE_TRANS;
			cbndx = qsmmu->bypass_cbndx;
		} else if (type == S2CR_TYPE_FAULT) {
			/*
			 * Firmware with quirky S2CR handling will ignore FAULT
			 * writes, so trick it to write FAULT by asking for a
			 * BYPASS.
			 */
			type = S2CR_TYPE_BYPASS;
			cbndx = 0xff;
		}
	}

	reg = FIELD_PREP(ARM_SMMU_S2CR_TYPE, type) |
	      FIELD_PREP(ARM_SMMU_S2CR_CBNDX, cbndx) |
	      FIELD_PREP(ARM_SMMU_S2CR_PRIVCFG, s2cr->privcfg);
	arm_smmu_gr0_write(smmu, ARM_SMMU_GR0_S2CR(idx), reg);
}

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
static int qcom_smmu_init_context(struct arm_smmu_domain *smmu_domain,
                struct io_pgtable_cfg *pgtbl_cfg, struct device *dev)
{
	if (of_find_property(dev->of_node, "qcom,use-3lvl-tables", NULL)) {
		/* have doubt to remain this code */
		pgtbl_cfg->ias = min(pgtbl_cfg->ias, 39);
	}
	return 0;
}

static int qcom_generic_smmu_cfg_probe(struct arm_smmu_device *smmu)
{
	u32 i = 0;
	int ret;
	struct device *dev = smmu->dev;
	struct device_node *child = dev->of_node;

	ret = of_property_read_u32(child, "qcom,last-domain", &i);
	if (ret) {
			dev_info(dev, "%pOF invalid 'qcom,last-domain' property", child);
	} else {
		if (i > 0 && i < smmu->num_mapping_groups) {
			dev_info(dev, "set num_mapping_groups from %d to %d\n", smmu->num_mapping_groups, i);
			smmu->num_mapping_groups = i;
		} else {
			dev_info(dev, "set new num_mapping_groups %d", i);
		}
	}

	i = 0;

	ret = of_property_read_u32(child, "qcom,last-context", &i);
	if (ret) {
		dev_info(dev, "%pOF invalid 'qcom,last-context' property", child);
	} else {
		if (i > 0 && i < smmu->num_context_banks) {
			dev_info(dev, "set num_context_banks from %d to %d\n", smmu->num_context_banks, i);
			smmu->num_context_banks = i;
		} else {
			dev_info(dev, "set new num_context_banks  %d", i);
		}
	}

	return 0;
}

static int qcom_adreno_smmu_cfg_probe(struct arm_smmu_device *smmu) {
	qcom_generic_smmu_cfg_probe(smmu);
	qcom_smmu_cfg_probe(smmu);
	return 0;
}

static int arm_adreno_smmu_alloc_context_bank(struct arm_smmu_domain *smmu_domain,
                                  struct arm_smmu_device *smmu,
                                  struct device *dev, int start)
{
        return __arm_smmu_alloc_bitmap(smmu->context_map, 1, smmu->num_context_banks - 1);
}

static const struct arm_smmu_impl qcom_adreno_smmu_impl = {
	.cfg_probe = qcom_adreno_smmu_cfg_probe,
	.def_domain_type = qcom_smmu_def_domain_type,
	.init_context = qcom_smmu_init_context,
	.reset = qcom_smmu500_reset,
	.alloc_context_bank = arm_adreno_smmu_alloc_context_bank,
	.write_s2cr = qcom_smmu_write_s2cr,
};

static const struct arm_smmu_impl qcom_smmu_impl = {
        .cfg_probe = qcom_generic_smmu_cfg_probe,
        .def_domain_type = qcom_smmu_def_domain_type,
        .init_context = qcom_smmu_init_context,
        .reset = qcom_smmu500_reset,
};

struct arm_smmu_device *qcom_smmu_impl_init(struct arm_smmu_device *smmu)
{
	struct qcom_smmu *qsmmu;

	/* Check to make sure qcom_scm has finished probing */
	if (!qcom_scm_is_available())
		return ERR_PTR(-EPROBE_DEFER);

	qsmmu = devm_kzalloc(smmu->dev, sizeof(*qsmmu), GFP_KERNEL);
	if (!qsmmu)
		return ERR_PTR(-ENOMEM);

	qsmmu->smmu = *smmu;

	if (qcom_smmu_def_domain_type(smmu->dev))
		qsmmu->smmu.impl = &qcom_adreno_smmu_impl;
	else
		qsmmu->smmu.impl = &qcom_smmu_impl;

	devm_kfree(smmu->dev, smmu);

	return &qsmmu->smmu;
}
