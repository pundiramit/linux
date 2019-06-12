// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2019, The Linux Foundation. All rights reserved.

#include <linux/module.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>

#define REG_PERPH_TYPE                  0x04
#define QCOM_LAB_TYPE			0x24
#define QCOM_IBB_TYPE			0x20

#define REG_LAB_STATUS1			0x08
#define REG_LAB_ENABLE_CTL		0x46
#define LAB_STATUS1_VREG_OK_BIT		BIT(7)
#define LAB_ENABLE_CTL_EN		BIT(7)

#define REG_IBB_STATUS1			0x08
#define REG_IBB_ENABLE_CTL		0x46
#define IBB_STATUS1_VREG_OK_BIT		BIT(7)
#define IBB_ENABLE_CTL_MASK		(BIT(7) | BIT(6))
#define IBB_CONTROL_ENABLE		BIT(7)

#define POWER_DELAY			8000

struct lab_regulator {
	struct regulator_dev		*rdev;
	int				vreg_enabled;
};

struct ibb_regulator {
	struct regulator_dev		*rdev;
	int				vreg_enabled;
};

struct qcom_labibb {
	struct device			*dev;
	struct regmap			*regmap;
	u16				lab_base;
	u16				ibb_base;
	struct lab_regulator		lab_vreg;
	struct ibb_regulator		ibb_vreg;
};

static int qcom_labibb_read(struct qcom_labibb *labibb, u16 address,
			    u8 *val, int count)
{
	int ret;

	ret = regmap_bulk_read(labibb->regmap, address, val, count);
	if (ret < 0)
		dev_err(labibb->dev, "spmi read failed ret=%d\n", ret);

	return ret;
}

static int qcom_labibb_write(struct qcom_labibb *labibb, u16 address,
			     u8 *val, int count)
{
	int ret;

	ret = regmap_bulk_write(labibb->regmap, address, val, count);
	if (ret < 0)
		dev_err(labibb->dev, "spmi write failed: ret=%d\n", ret);

	return ret;
}

static int qcom_labibb_masked_write(struct qcom_labibb *labibb, u16 address,
				    u8 mask, u8 val)
{
	int ret;

	ret = regmap_update_bits(labibb->regmap, address, mask, val);
	if (ret < 0)
		dev_err(labibb->dev, "spmi write failed: ret=%d\n", ret);

	return ret;
}

static int qcom_enable_ibb(struct qcom_labibb *labibb, bool enable)
{
	int ret;
	u8 val = enable ? IBB_CONTROL_ENABLE : 0;

	ret = qcom_labibb_masked_write(labibb,
				       labibb->ibb_base + REG_IBB_ENABLE_CTL,
				       IBB_ENABLE_CTL_MASK, val);
	if (ret < 0)
		dev_err(labibb->dev, "Unable to configure IBB_ENABLE_CTL ret=%d\n",
			ret);

	return ret;
}

static int qcom_lab_regulator_enable(struct regulator_dev *rdev)
{
	int ret;
	u8 val;
	struct qcom_labibb *labibb  = rdev_get_drvdata(rdev);

	val = LAB_ENABLE_CTL_EN;
	ret = qcom_labibb_write(labibb,
				labibb->lab_base + REG_LAB_ENABLE_CTL,
				&val, 1);
	if (ret < 0) {
		dev_err(labibb->dev, "Write register failed ret = %d\n", ret);
		return ret;
	}

	/* Wait for a small period before reading REG_LAB_STATUS1 */
	usleep_range(POWER_DELAY, POWER_DELAY + 100);

	ret = qcom_labibb_read(labibb, labibb->lab_base +
			       REG_LAB_STATUS1, &val, 1);
	if (ret < 0) {
		dev_err(labibb->dev, "Read register failed ret = %d\n", ret);
		return ret;
	}

	if (!(val & LAB_STATUS1_VREG_OK_BIT)) {
		dev_err(labibb->dev, "Can't enable LAB\n");
		return -EINVAL;
	}

	labibb->lab_vreg.vreg_enabled = 1;

	return 0;
}

static int qcom_lab_regulator_disable(struct regulator_dev *rdev)
{
	int ret;
	u8 val = 0;
	struct qcom_labibb *labibb  = rdev_get_drvdata(rdev);

	ret = qcom_labibb_write(labibb,
				labibb->lab_base + REG_LAB_ENABLE_CTL,
				&val, 1);
	if (ret < 0) {
		dev_err(labibb->dev, "Write register failed ret = %d\n", ret);
		return ret;
	}
	/* after this delay, lab should get disabled */
	usleep_range(POWER_DELAY, POWER_DELAY + 100);

	ret = qcom_labibb_read(labibb, labibb->lab_base +
			       REG_LAB_STATUS1, &val, 1);
	if (ret < 0) {
		dev_err(labibb->dev, "Read register failed ret = %d\n", ret);
		return ret;
	}

	if (val & LAB_STATUS1_VREG_OK_BIT) {
		dev_err(labibb->dev, "Can't disable LAB\n");
		return -EINVAL;
	}

	labibb->lab_vreg.vreg_enabled = 0;

	return 0;
}

static int qcom_lab_regulator_is_enabled(struct regulator_dev *rdev)
{
	int ret;
	u8 val;
	struct qcom_labibb *labibb  = rdev_get_drvdata(rdev);

	ret = qcom_labibb_read(labibb, labibb->lab_base +
			       REG_LAB_STATUS1, &val, 1);
	if (ret < 0) {
		dev_err(labibb->dev, "Read register failed ret = %d\n", ret);
		return ret;
	}

	return val & LAB_STATUS1_VREG_OK_BIT;
}

static struct regulator_ops qcom_lab_ops = {
	.enable			= qcom_lab_regulator_enable,
	.disable		= qcom_lab_regulator_disable,
	.is_enabled		= qcom_lab_regulator_is_enabled,
};

static const struct regulator_desc lab_desc = {
	.name = "lab_reg",
	.ops = &qcom_lab_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
};

static int qcom_ibb_regulator_enable(struct regulator_dev *rdev)
{
	int ret, retries = 10;
	u8 val;
	struct qcom_labibb *labibb  = rdev_get_drvdata(rdev);

	ret = qcom_enable_ibb(labibb, 1);
	if (ret < 0) {
		dev_err(labibb->dev, "Unable to set IBB mode ret= %d\n", ret);
		return ret;
	}

	while (retries--) {
		/* Wait for a small period before reading IBB_STATUS1 */
		usleep_range(POWER_DELAY, POWER_DELAY + 100);

		ret = qcom_labibb_read(labibb, labibb->ibb_base +
				       REG_IBB_STATUS1, &val, 1);
		if (ret < 0) {
			dev_err(labibb->dev,
				"Read register failed ret = %d\n", ret);
			return ret;
		}

		if (val & IBB_STATUS1_VREG_OK_BIT) {
			labibb->ibb_vreg.vreg_enabled = 1;
			return 0;
		}
	}

	dev_err(labibb->dev, "Can't enable IBB\n");
	return -EINVAL;
}

static int qcom_ibb_regulator_disable(struct regulator_dev *rdev)
{
	int ret, retries = 2;
	u8 val;
	struct qcom_labibb *labibb  = rdev_get_drvdata(rdev);

	ret = qcom_enable_ibb(labibb, 0);
	if (ret < 0) {
		dev_err(labibb->dev,
			"Unable to set IBB_MODULE_EN ret = %d\n", ret);
		return ret;
	}

	/* poll IBB_STATUS to make sure ibb had been disabled */
	while (retries--) {
		usleep_range(POWER_DELAY, POWER_DELAY + 100);
		ret = qcom_labibb_read(labibb, labibb->ibb_base +
				       REG_IBB_STATUS1, &val, 1);
		if (ret < 0) {
			dev_err(labibb->dev, "Read register failed ret = %d\n",
				ret);
			return ret;
		}

		if (!(val & IBB_STATUS1_VREG_OK_BIT)) {
			labibb->ibb_vreg.vreg_enabled = 0;
			return 0;
		}
	}
	dev_err(labibb->dev, "Can't disable IBB\n");
	return -EINVAL;
}

static int qcom_ibb_regulator_is_enabled(struct regulator_dev *rdev)
{
	int ret;
	u8 val;
	struct qcom_labibb *labibb  = rdev_get_drvdata(rdev);

	ret = qcom_labibb_read(labibb, labibb->ibb_base +
			REG_IBB_STATUS1, &val, 1);
	if (ret < 0) {
		dev_err(labibb->dev, "Read register failed ret = %d\n", ret);
		return ret;
	}

	return(val & IBB_STATUS1_VREG_OK_BIT);
}

static struct regulator_ops qcom_ibb_ops = {
	.enable			= qcom_ibb_regulator_enable,
	.disable		= qcom_ibb_regulator_disable,
	.is_enabled		= qcom_ibb_regulator_is_enabled,
};

static const struct regulator_desc ibb_desc = {
	.name = "ibb_reg",
	.ops = &qcom_ibb_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
};

static int register_lab_regulator(struct qcom_labibb *labibb,
				  struct device_node *of_node)
{
	int ret = 0;
	struct regulator_init_data *init_data;
	struct regulator_config cfg = {};

	cfg.dev = labibb->dev;
	cfg.driver_data = labibb;
	cfg.of_node = of_node;
	init_data =
		of_get_regulator_init_data(labibb->dev,
					   of_node, &lab_desc);
	if (!init_data) {
		dev_err(labibb->dev,
			"unable to get init data for LAB\n");
		return -ENOMEM;
	}
	cfg.init_data = init_data;

	labibb->lab_vreg.rdev = devm_regulator_register(labibb->dev, &lab_desc,
							&cfg);
	if (IS_ERR(labibb->lab_vreg.rdev)) {
		ret = PTR_ERR(labibb->lab_vreg.rdev);
		dev_err(labibb->dev,
			"unable to register LAB regulator");
		return ret;
	}
	return 0;
}

static int register_ibb_regulator(struct qcom_labibb *labibb,
				  struct device_node *of_node)
{
	int ret;
	struct regulator_init_data *init_data;
	struct regulator_config cfg = {};

	cfg.dev = labibb->dev;
	cfg.driver_data = labibb;
	cfg.of_node = of_node;
	init_data =
		of_get_regulator_init_data(labibb->dev,
					   of_node, &ibb_desc);
	if (!init_data) {
		dev_err(labibb->dev,
			"unable to get init data for IBB\n");
		return -ENOMEM;
	}
	cfg.init_data = init_data;

	labibb->ibb_vreg.rdev = devm_regulator_register(labibb->dev, &ibb_desc,
							&cfg);
	if (IS_ERR(labibb->ibb_vreg.rdev)) {
		ret = PTR_ERR(labibb->ibb_vreg.rdev);
		dev_err(labibb->dev,
			"unable to register IBB regulator");
		return ret;
	}
	return 0;
}

static int qcom_labibb_regulator_probe(struct platform_device *pdev)
{
	struct qcom_labibb *labibb;
	struct device_node *child;
	unsigned int base;
	u8 type;
	int ret;

	labibb = devm_kzalloc(&pdev->dev, sizeof(*labibb), GFP_KERNEL);
	if (!labibb)
		return -ENOMEM;

	labibb->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!labibb->regmap) {
		dev_err(&pdev->dev, "Couldn't get parent's regmap\n");
		return -ENODEV;
	}

	labibb->dev = &pdev->dev;

	for_each_available_child_of_node(pdev->dev.of_node, child) {
		ret = of_property_read_u32(child, "reg", &base);
		if (ret < 0) {
			dev_err(&pdev->dev,
				"Couldn't find reg in node = %s ret = %d\n",
				child->full_name, ret);
			return ret;
		}

		ret = qcom_labibb_read(labibb, base + REG_PERPH_TYPE,
				       &type, 1);
		if (ret < 0) {
			dev_err(labibb->dev,
				"Peripheral type read failed ret=%d\n",
				ret);
		}

		switch (type) {
		case QCOM_LAB_TYPE:
			labibb->lab_base = base;
			ret = register_lab_regulator(labibb, child);
			if (ret < 0) {
				dev_err(labibb->dev,
					"Failed LAB regulator registration");
				return ret;
			}
			break;

		case QCOM_IBB_TYPE:
			labibb->ibb_base = base;
			ret = register_ibb_regulator(labibb, child);
			if (ret < 0) {
				dev_err(labibb->dev,
					"Failed IBB regulator registration");
				return ret;
			}
			break;

		default:
			dev_err(labibb->dev,
				"qcom_labibb: unknown peripheral type\n");
			return -EINVAL;
		}
	}

	dev_set_drvdata(&pdev->dev, labibb);
	return 0;
}

static const struct of_device_id qcom_labibb_match_table[] = {
	{ .compatible = "qcom,lab-ibb-regulator", },
	{ },
};
MODULE_DEVICE_TABLE(of, qcom_labibb_match_table);

static struct platform_driver qcom_labibb_regulator_driver = {
	.driver		= {
		.name		= "qcom-lab-ibb-regulator",
		.of_match_table	= qcom_labibb_match_table,
	},
	.probe		= qcom_labibb_regulator_probe,
};
module_platform_driver(qcom_labibb_regulator_driver);

MODULE_DESCRIPTION("Qualcomm labibb driver");
MODULE_LICENSE("GPL v2");
