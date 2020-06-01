// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2020, The Linux Foundation. All rights reserved.

#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_irq.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>

#define REG_PERPH_TYPE                  0x04
#define QCOM_LAB_TYPE			0x24
#define QCOM_IBB_TYPE			0x20

#define REG_LABIBB_STATUS1		0x08
#define REG_LABIBB_ENABLE_CTL		0x46
#define LABIBB_STATUS1_VREG_OK_BIT	BIT(7)
#define LABIBB_CONTROL_ENABLE		BIT(7)
#define LABIBB_STATUS1_SC_DETECT_BIT	BIT(6)

#define LAB_ENABLE_CTL_MASK		BIT(7)
#define IBB_ENABLE_CTL_MASK		(BIT(7) | BIT(6))

#define LABIBB_OFF_ON_DELAY		1000
#define LAB_ENABLE_TIME			(LABIBB_OFF_ON_DELAY * 2)
#define IBB_ENABLE_TIME			(LABIBB_OFF_ON_DELAY * 10)
#define LABIBB_POLL_ENABLED_TIME	1000

#define POLLING_SCP_DONE_INTERVAL_US	5000
#define POLLING_SCP_TIMEOUT		16000

struct labibb_regulator {
	struct regulator_desc		desc;
	struct device			*dev;
	struct regmap			*regmap;
	struct regulator_dev		*rdev;
	u16				base;
	bool				enabled;
	u8				type;
};

struct labibb_regulator_data {
	u16				base;
	const char			*name;
	u8				type;
	unsigned int			enable_time;
	unsigned int			enable_mask;
};

static int qcom_labibb_regulator_is_enabled(struct regulator_dev *rdev)
{
	int ret;
	unsigned int val;
	struct labibb_regulator *reg = rdev_get_drvdata(rdev);
	ret = regmap_read(reg->regmap, reg->base + REG_LABIBB_STATUS1, &val);
	if (ret < 0) {
		dev_err(reg->dev, "Read register failed ret = %d\n", ret);
		return ret;
	}
	return !!(val & LABIBB_STATUS1_VREG_OK_BIT);
}

static int qcom_labibb_regulator_enable(struct regulator_dev *rdev)
{
	int ret;
	struct labibb_regulator *reg = rdev_get_drvdata(rdev);

	ret = regulator_enable_regmap(rdev);
	if (ret >= 0)
		reg->enabled = true;

	return ret;
}

static int qcom_labibb_regulator_disable(struct regulator_dev *rdev)
{
	int ret = 0;
	struct labibb_regulator *reg = rdev_get_drvdata(rdev);

	ret = regulator_disable_regmap(rdev);
	if (ret >= 0)
		reg->enabled = false;

	return ret;
}

static struct regulator_ops qcom_labibb_ops = {
	.enable			= qcom_labibb_regulator_enable,
	.disable		= qcom_labibb_regulator_disable,
	.is_enabled		= qcom_labibb_regulator_is_enabled,
};

static irqreturn_t labibb_sc_err_handler(int irq, void *_reg)
{
	int ret;
	u16 reg;
	unsigned int val;
	struct labibb_regulator *labibb_reg = _reg;
	bool in_sc_err, scp_done = false;

	ret = regmap_read(labibb_reg->regmap,
			  labibb_reg->base + REG_LABIBB_STATUS1, &val);
	if (ret < 0) {
		dev_err(labibb_reg->dev, "sc_err_irq: Read failed, ret=%d\n",
			ret);
		return IRQ_HANDLED;
	}

	dev_dbg(labibb_reg->dev, "%s SC error triggered! STATUS1 = %d\n",
		labibb_reg->desc.name, val);

	in_sc_err = !!(val & LABIBB_STATUS1_SC_DETECT_BIT);

	/*
	 * The SC(short circuit) fault would trigger PBS(Portable Batch
	 * System) to disable regulators for protection. This would
	 * cause the SC_DETECT status being cleared so that it's not
	 * able to get the SC fault status.
	 * Check if the regulator is enabled in the driver but
	 * disabled in hardware, this means a SC fault had happened
	 * and SCP handling is completed by PBS.
	 */
	if (!in_sc_err) {

		reg = labibb_reg->base + REG_LABIBB_ENABLE_CTL;

		ret = regmap_read_poll_timeout(labibb_reg->regmap,
					reg, val,
					!(val & LABIBB_CONTROL_ENABLE),
					POLLING_SCP_DONE_INTERVAL_US,
					POLLING_SCP_TIMEOUT);

		if (!ret && labibb_reg->enabled) {
			dev_dbg(labibb_reg->dev,
				"%s has been disabled by SCP\n",
				labibb_reg->desc.name);
			scp_done = true;
		}
	}

	if (in_sc_err || scp_done) {
		regulator_lock(labibb_reg->rdev);
		regulator_notifier_call_chain(labibb_reg->rdev,
						REGULATOR_EVENT_OVER_CURRENT,
						NULL);
		regulator_unlock(labibb_reg->rdev);
	}
	return IRQ_HANDLED;
}

static struct regulator_dev *register_labibb_regulator(struct labibb_regulator *reg,
				const struct labibb_regulator_data *reg_data,
				struct device_node *of_node)
{
	struct regulator_config cfg = {};
	int ret, sc_irq;

	reg->base = reg_data->base;
	reg->type = reg_data->type;
	reg->desc.enable_mask = reg_data->enable_mask;
	reg->desc.enable_reg = reg->base + REG_LABIBB_ENABLE_CTL;
	reg->desc.enable_val = LABIBB_CONTROL_ENABLE;
	reg->desc.of_match = reg_data->name;
	reg->desc.name = reg_data->name;
	reg->desc.owner = THIS_MODULE;
	reg->desc.type = REGULATOR_VOLTAGE;
	reg->desc.ops = &qcom_labibb_ops;

	reg->desc.enable_time = reg_data->enable_time;
	reg->desc.poll_enabled_time = LABIBB_POLL_ENABLED_TIME;
	reg->desc.off_on_delay = LABIBB_OFF_ON_DELAY;

	sc_irq = of_irq_get_byname(of_node, "sc-err");
	if (sc_irq < 0) {
		dev_err(reg->dev, "Unable to get sc-err, ret = %d\n",
			sc_irq);
		return ERR_PTR(sc_irq);
	} else {
		ret = devm_request_threaded_irq(reg->dev,
						sc_irq,
						NULL, labibb_sc_err_handler,
						IRQF_ONESHOT,
						"sc-err", reg);
		if (ret) {
			dev_err(reg->dev, "Failed to register sc-err irq ret=%d\n",
				ret);
			return ERR_PTR(ret);
		}
	}

	cfg.dev = reg->dev;
	cfg.driver_data = reg;
	cfg.regmap = reg->regmap;
	cfg.of_node = of_node;

	return devm_regulator_register(reg->dev, &reg->desc, &cfg);
}

static const struct labibb_regulator_data pmi8998_labibb_data[] = {
	{0xde00, "lab", QCOM_LAB_TYPE, LAB_ENABLE_TIME, LAB_ENABLE_CTL_MASK},
	{0xdc00, "ibb", QCOM_IBB_TYPE, IBB_ENABLE_TIME, IBB_ENABLE_CTL_MASK},
	{ },
};

static const struct of_device_id qcom_labibb_match[] = {
	{ .compatible = "qcom,pmi8998-lab-ibb", .data = &pmi8998_labibb_data},
	{ },
};
MODULE_DEVICE_TABLE(of, qcom_labibb_match);

static int qcom_labibb_regulator_probe(struct platform_device *pdev)
{
	struct labibb_regulator *labibb_reg;
	struct device *dev = &pdev->dev;
	struct device_node *child;
	const struct of_device_id *match;
	const struct labibb_regulator_data *reg_data;
	struct regmap *reg_regmap;
	unsigned int type;
	int ret;

	reg_regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!reg_regmap) {
		dev_err(&pdev->dev, "Couldn't get parent's regmap\n");
		return -ENODEV;
	}

	match = of_match_device(qcom_labibb_match, &pdev->dev);
	if (!match)
		return -ENODEV;

	for (reg_data = match->data; reg_data->name; reg_data++) {
		child = of_get_child_by_name(pdev->dev.of_node, reg_data->name);

		if (WARN_ON(child == NULL))
			return -EINVAL;

		/* Validate if the type of regulator is indeed
		 * what's mentioned in DT.
		 */
		ret = regmap_read(reg_regmap, reg_data->base + REG_PERPH_TYPE,
				  &type);
		if (ret < 0) {
			dev_err(dev,
				"Peripheral type read failed ret=%d\n",
				ret);
			return -EINVAL;
		}

		if (WARN_ON((type != QCOM_LAB_TYPE) && (type != QCOM_IBB_TYPE)) ||
		    WARN_ON(type != reg_data->type))
			return -EINVAL;

		labibb_reg  = devm_kzalloc(&pdev->dev, sizeof(*labibb_reg),
					   GFP_KERNEL);
		if (!labibb_reg)
			return -ENOMEM;

		labibb_reg->regmap = reg_regmap;
		labibb_reg->dev = dev;

		dev_info(dev, "Registering %s regulator\n", child->full_name);

		labibb_reg->rdev = register_labibb_regulator(labibb_reg, reg_data, child);
		if (IS_ERR(labibb_reg->rdev)) {
			dev_err(dev,
				"qcom_labibb: error registering %s : %d\n",
				child->full_name, ret);
			return PTR_ERR(labibb_reg->rdev);
		}
	}

	return 0;
}

static struct platform_driver qcom_labibb_regulator_driver = {
	.driver	= {
		.name = "qcom-lab-ibb-regulator",
		.of_match_table	= qcom_labibb_match,
	},
	.probe = qcom_labibb_regulator_probe,
};
module_platform_driver(qcom_labibb_regulator_driver);

MODULE_DESCRIPTION("Qualcomm labibb driver");
MODULE_LICENSE("GPL v2");
