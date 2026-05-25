// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2026 Alexander Warnecke
 * Copyright (c) 2021 Rockchip Electronics Co. Ltd.
 *
 * Based on rk3399_dmc.c
 */

#include <linux/arm-smccc.h>
#include <linux/clk.h>
#include <linux/devfreq.h>
#include <linux/devfreq-event.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/regulator/consumer.h>

#include <soc/rockchip/pm_domains.h>
#include <soc/rockchip/rockchip_sip.h>

#define ROCKCHIP_SIP_SHARE_PAGE_TYPE_INVALID		0
#define ROCKCHIP_SIP_SHARE_PAGE_TYPE_UARTDBG		1
#define ROCKCHIP_SIP_SHARE_PAGE_TYPE_DDR		2
#define ROCKCHIP_SIP_SHARE_PAGE_TYPE_DDRDBG		3
#define ROCKCHIP_SIP_SHARE_PAGE_TYPE_DDRECC		4
#define ROCKCHIP_SIP_SHARE_PAGE_TYPE_DDRFSP		5
#define ROCKCHIP_SIP_SHARE_PAGE_TYPE_DDR_ADDRMAP	6
#define ROCKCHIP_SIP_SHARE_PAGE_TYPE_LAST_LOG		7
#define ROCKCHIP_SIP_SHARE_PAGE_TYPE_HDCP		8
#define ROCKCHIP_SIP_SHARE_PAGE_TYPE_SLEEP		9

#define ROCKCHIP_SIP_RET_SUCCESS		0
#define ROCKCHIP_SIP_RET_SMC_UNKNOWN		-1
#define ROCKCHIP_SIP_RET_NOT_SUPPORTED		-2
#define ROCKCHIP_SIP_RET_INVALID_PARAMS		-3
#define ROCKCHIP_SIP_RET_INVALID_ADDRESS	-4
#define ROCKCHIP_SIP_RET_DENIED			-5
#define ROCKCHIP_SIP_RET_SET_RATE_TIMEOUT	-6

#define ROCKCHIP_SIP_SIZE_PAGE(x) ((x) << 12)

#define MAX_FREQ_COUNT 6

struct rk35_dmcfreq_share_param {
	u32 hz;
	u32 lcdc_type;
	u32 vop;
	u32 vop_dclk_mode;
	u32 sr_idle_en;
	u32 addr_mcu_el3;
	u32 wait_flag1;
	u32 wait_flag0;
	u32 complt_hwirq;
	u32 update_drv_odt_cfg;
	u32 update_deskew_cfg;

	u32 freq_count;
	u32 freq_info_mhz[MAX_FREQ_COUNT];
	u32 wait_mode;
	u32 vop_scan_line_time_ns;
} __packed;

struct rk35_dmcfreq_platdata {
	int upthreshold, downdifferential;
	int min_tfa_version;
	bool has_mem_supply;
	bool suspend_max_voltage;
};

struct rk35_dmcfreq {
	struct device *dev;
	const struct rk35_dmcfreq_platdata *platdata;
	struct devfreq *devfreq;
	struct devfreq_dev_profile profile;
	struct devfreq_simple_ondemand_data ondemand_data;
	struct clk *clk;
	struct devfreq_event_dev *edev;
	struct regulator *center_supply;
	struct regulator *mem_supply;
	struct pm_qos_request qos;
	struct mutex lock;
	int irq;

	unsigned long freq, center_volt, mem_volt;
	unsigned long suspend_freq, suspend_center_volt, suspend_mem_volt;
	unsigned long resume_freq, resume_center_volt, resume_mem_volt;

	struct rk35_dmcfreq_share_param *ddr_psci_param;

	wait_queue_head_t wq;
	bool wait;
	unsigned long err;
};

static irqreturn_t rk35_dmcfreq_complete_irq(int irq, void *d)
{
	struct rk35_dmcfreq *data = d;
	struct arm_smccc_res res;

	/*
	 * After setting DRAM frequency, stop MCU, clear IRQ and check for
	 * errors
	 */
	arm_smccc_smc(ROCKCHIP_SIP_DRAM_CONFIG,
		      ROCKCHIP_SIP_SHARE_PAGE_TYPE_DDR, 0, ROCKCHIP_SIP_CONFIG_DRAM_POST_SET_RATE,
		      0, 0, 0, 0, &res);

	data->wait = false;
	data->err = res.a0;
	wake_up(&data->wq);

	return IRQ_HANDLED;
}

static int rk35_dmcfreq_wait_complete(struct rk35_dmcfreq *data)
{
	struct device *dev = data->dev;
	struct arm_smccc_res res;

	data->wait = true;

	cpu_latency_qos_update_request(&data->qos, 0);
	enable_irq(data->irq);

	/*
	 * Start MCU to begin clock frequency transition
	 */
	arm_smccc_smc(ROCKCHIP_SIP_DRAM_CONFIG,
		      0, 0, ROCKCHIP_SIP_CONFIG_MCU_START,
		      0, 0, 0, 0, &res);
	if (res.a0) {
		disable_irq(data->irq);
		cpu_latency_qos_update_request(&data->qos, PM_QOS_DEFAULT_VALUE);
		dev_err_ratelimited(dev, "Failed to start MCU: %lu", res.a0);
		return -ENOMEM;
	}

	wait_event_timeout(data->wq, (!data->wait), msecs_to_jiffies(85));

	disable_irq(data->irq);
	cpu_latency_qos_update_request(&data->qos, PM_QOS_DEFAULT_VALUE);

	if (data->wait) {
		/*
		 * If waiting for completion times out, stop MCU and get an
		 * error code
		 */
		dev_err_ratelimited(dev, "Set DRAM frequency timeout\n");
		arm_smccc_smc(ROCKCHIP_SIP_DRAM_CONFIG,
			      ROCKCHIP_SIP_SHARE_PAGE_TYPE_DDR, 0, ROCKCHIP_SIP_CONFIG_DRAM_POST_SET_RATE,
			      0, 0, 0, 0, &res);
		data->err = res.a0;
	}

	if (data->err)
		dev_err_ratelimited(dev, "Failed to set DRAM frequency: %lu", data->err);
	return data->err;
}

static int rk35_dmcfreq_set_freq(struct rk35_dmcfreq *data, unsigned long freq)
{
	struct device *dev = data->dev;
	struct arm_smccc_res res;
	int ret;

	ret = rockchip_pmu_block();
	if (ret) {
		dev_err_ratelimited(dev, "Failed to block PMU: %i\n", ret);
		return ret;
	}

	data->ddr_psci_param->hz = freq;
	data->ddr_psci_param->wait_flag1 = 1;
	data->ddr_psci_param->wait_flag0 = 1;

	arm_smccc_smc(ROCKCHIP_SIP_DRAM_CONFIG,
		      ROCKCHIP_SIP_SHARE_PAGE_TYPE_DDR, 0, ROCKCHIP_SIP_CONFIG_DRAM_SET_RATE,
		      0, 0, 0, 0, &res);

	/*
	 * If clock frequency cannot immediately be set, delegate to MCU and
	 * wait for response
	 */
	if ((int)res.a1 == ROCKCHIP_SIP_RET_SET_RATE_TIMEOUT)
		rk35_dmcfreq_wait_complete(data);

	rockchip_pmu_unblock();

	if (res.a0) {
		dev_err_ratelimited(dev, "Failed to set DRAM frequency: %lu", res.a0);
		return res.a0;
	}

	data->freq = clk_get_rate(data->clk);
	if (data->freq != freq) {
		dev_err_ratelimited(dev,
				    "DRAM frequency set incorrectly. Requested rate: %lu, Actual rate: %lu\n",
				    freq, data->freq);
		return -EINVAL;
	}

	return 0;
}

static int rk35_dmcfreq_set_voltage(struct rk35_dmcfreq *data,
				    unsigned long center_volt,
				    unsigned long mem_volt)
{
	struct device *dev = data->dev;
	int ret;

	ret = regulator_set_voltage(data->center_supply, center_volt, center_volt);
	if (ret) {
		dev_err_ratelimited(dev, "Failed to set center voltage: %i\n", ret);
		return ret;
	}
	data->center_volt = center_volt;

	/*if (data->platdata->mem_supply) {
		ret = regulator_set_voltage(data->mem_supply, mem_volt, mem_volt);
		if (ret) {
			dev_err_ratelimited(dev, "Failed to set memory voltage: %i\n", ret);
			return ret;
		}
		data->mem_volt = mem_volt;
	}*/

	return 0;
}

static int rk35_dmcfreq_target(struct device *dev, unsigned long *freq,
			       u32 flags)
{
	struct rk35_dmcfreq *data = dev_get_drvdata(dev);
	struct dev_pm_opp *opp;
	unsigned long target_freq, target_center_volt, target_mem_volt = 0;
	unsigned long old_freq = data->freq;
	unsigned long old_center_volt = data->center_volt;
	unsigned long old_mem_volt = data->mem_volt;
	int ret;

	opp = devfreq_recommended_opp(dev, freq, flags);
	if (IS_ERR(opp))
		return PTR_ERR(opp);

	target_freq = dev_pm_opp_get_freq(opp);
	target_center_volt = dev_pm_opp_get_voltage(opp);
	// TODO: target_mem_volt
	dev_pm_opp_put(opp);

	guard(mutex)(&data->lock);

	if (data->freq == target_freq)
		return 0;

	/*
	 * If transitioning to a higher frequency, set voltage first
	 */
	if (data->freq < target_freq) {
		ret = rk35_dmcfreq_set_voltage(data, target_center_volt,
					       target_mem_volt);
		if (ret)
			return ret;
	}

	ret = rk35_dmcfreq_set_freq(data, *freq);
	if (ret) {
		rk35_dmcfreq_set_voltage(data, old_center_volt, old_mem_volt);
		return ret;
	}

	if (old_freq > data->freq) {
		ret = rk35_dmcfreq_set_voltage(data, target_center_volt,
					       target_mem_volt);
		if (ret)
			rk35_dmcfreq_set_freq(data, old_freq);
	}

	return ret;
}

static int rk35_dmcfreq_get_dev_status(struct device *dev,
				       struct devfreq_dev_status *stat)
{
	struct rk35_dmcfreq *data = dev_get_drvdata(dev);
	struct devfreq_event_data edata;
	int ret;

	ret = devfreq_event_get_event(data->edev, &edata);
	if (ret < 0)
		return ret;

	stat->current_frequency = data->freq;
	stat->busy_time = edata.load_count;
	stat->total_time = edata.total_count;

	return 0;
}

static int rk35_dmcfreq_get_cur_freq(struct device *dev, unsigned long *freq)
{
	struct rk35_dmcfreq *data = dev_get_drvdata(dev);

	guard(mutex)(&data->lock);

	*freq = data->freq;

	return 0;
}

static int rk35_dmcfreq_init(struct rk35_dmcfreq *data)
{
	struct device *dev = data->dev;
	struct arm_smccc_res res;

	arm_smccc_smc(ROCKCHIP_SIP_DRAM_CONFIG,
		      0, 0, ROCKCHIP_SIP_CONFIG_DRAM_GET_VERSION,
		      0, 0, 0, 0, &res);
	dev_info(dev, "Current TF-A version 0x%lx\n", res.a1);
	if (res.a0 || res.a1 < data->platdata->min_tfa_version)
		return dev_err_probe(dev, -ENXIO, "TF-A version too old\n");

	/*
	 * Map shared memory to communicate with TF-A
	 * First 4KiB is interface parameters
	 * Second 4KiB is DTS parameters
	 */
	arm_smccc_smc(ROCKCHIP_SIP_SHARE_MEM,
		      2, ROCKCHIP_SIP_SHARE_PAGE_TYPE_DDR, 0,
		      0, 0, 0, 0, &res);
	if (res.a0)
		return dev_err_probe(dev, -ENOMEM, "No TF-A memory available\n");

	data->ddr_psci_param = (struct rk35_dmcfreq_share_param *)
			       devm_ioremap(dev, res.a1, ROCKCHIP_SIP_SIZE_PAGE(2));
	if (IS_ERR(data->ddr_psci_param))
		return dev_err_probe(dev, PTR_ERR(data->ddr_psci_param),
				     "Failed to map TF-A shared memory\n");
	memset_io(data->ddr_psci_param, 0, ROCKCHIP_SIP_SIZE_PAGE(2));

	arm_smccc_smc(ROCKCHIP_SIP_DRAM_CONFIG,
		      ROCKCHIP_SIP_SHARE_PAGE_TYPE_DDR, 0, ROCKCHIP_SIP_CONFIG_DRAM_INIT,
		      0, 0, 0, 0, &res);
	if (res.a0)
		return dev_err_probe(dev, -ENOMEM,
				     "DRAM configuration initialization error: 0x%lx\n",
				     res.a0);

	arm_smccc_smc(ROCKCHIP_SIP_DRAM_CONFIG,
		      ROCKCHIP_SIP_SHARE_PAGE_TYPE_DDR, 0, ROCKCHIP_SIP_CONFIG_DRAM_GET_FREQ_INFO,
		      0, 0, 0, 0, &res);
	if (res.a0)
		return dev_err_probe(dev, -ENOMEM,
				     "Failed to get DRAM frequency info: 0x%lx\n",
				     res.a0);

	if (!data->ddr_psci_param->freq_count ||
	    data->ddr_psci_param->freq_count > MAX_FREQ_COUNT)
		return dev_err_probe(dev, -EPERM, "No frequencies available\n");

	// TODO: disable unsupported OPPs

	return 0;
}

static int rk35_dmcfreq_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rk35_dmcfreq *data;
	struct dev_pm_opp *opp;
	unsigned long freq;
	int ret;

	data = devm_kzalloc(dev, sizeof(struct rk35_dmcfreq), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->dev = dev;
	platform_set_drvdata(pdev, data);

	data->platdata = of_device_get_match_data(dev);

	mutex_init(&data->lock);
	init_waitqueue_head(&data->wq);

	data->clk = devm_clk_get(dev, "dmc_clk");
	if (IS_ERR(data->clk))
		return dev_err_probe(dev, PTR_ERR(data->clk), "Failed to get clock\n");

	data->edev = devfreq_event_get_edev_by_phandle(dev, "devfreq-events", 0);
	if (IS_ERR(data->edev))
		return dev_err_probe(dev, PTR_ERR(data->edev),
				     "Failed to get devfreq event device\n");

	ret = devfreq_event_enable_edev(data->edev);
	if (ret < 0)
		return dev_err_probe(dev, ret,
				     "Failed to enable devfreq event device\n");

	data->irq = platform_get_irq_byname(pdev, "complete");
	if (data->irq < 0) {
		ret = dev_err_probe(dev, data->irq, "No complete interrupt\n");
		goto err_edev;
	}

	ret = devm_request_irq(dev, data->irq, rk35_dmcfreq_complete_irq, 0,
			       dev_name(dev), data);
	if (ret < 0) {
		dev_err_probe(dev, ret, "Failed to get complete interrupt\n");
		goto err_edev;
	}
	disable_irq(data->irq);

	data->center_supply = devm_regulator_get(dev, "center");
	if (IS_ERR(data->center_supply)) {
		ret = dev_err_probe(dev, PTR_ERR(data->center_supply),
				    "Failed to get center regulator\n");
		goto err_edev;
	}

	if (data->platdata->has_mem_supply) {
		data->mem_supply = devm_regulator_get(dev, "mem");
		if (IS_ERR(data->mem_supply)) {
			ret = dev_err_probe(dev, PTR_ERR(data->mem_supply),
					    "Failed to get memory regulator\n");
			goto err_edev;
		}
	}

	ret = devm_pm_opp_of_add_table(dev);
	if (ret) {
		dev_err_probe(dev, ret, "Invalid operating-points\n");
		goto err_edev;
	}

	ret = rk35_dmcfreq_init(data);
	if (ret)
		goto err_edev;

	data->ondemand_data.upthreshold = data->platdata->upthreshold;
	data->ondemand_data.downdifferential = data->platdata->downdifferential;

	data->freq = clk_get_rate(data->clk);

	opp = devfreq_recommended_opp(dev, &data->freq, 0);
	if (IS_ERR(opp)) {
		ret = dev_err_probe(dev, PTR_ERR(opp),
				    "Failed to get operating point\n");
		goto err_edev;
	}
	data->freq = dev_pm_opp_get_freq(opp);
	data->center_volt = dev_pm_opp_get_voltage(opp);
	// TODO: data->mem_volt
	dev_pm_opp_put(opp);

	freq = 0;
	opp = dev_pm_opp_find_freq_ceil(dev, &freq);
	if (IS_ERR(opp)) {
		ret = dev_err_probe(dev, PTR_ERR(opp),
				    "Failed to get operating point\n");
		goto err_edev;
	}
	data->suspend_freq = dev_pm_opp_get_freq(opp);
	if (!data->platdata->suspend_max_voltage) {
		data->suspend_center_volt = dev_pm_opp_get_voltage(opp);
		// TODO: data->suspend_mem_volt
	}
	dev_pm_opp_put(opp);

	freq = ULONG_MAX;
	opp = dev_pm_opp_find_freq_floor(dev, &freq);
	if (IS_ERR(opp)) {
		ret = dev_err_probe(dev, PTR_ERR(opp),
				    "Failed to get operating point\n");
		goto err_edev;
	}
	data->resume_freq = dev_pm_opp_get_freq(opp);
	data->resume_center_volt = dev_pm_opp_get_voltage(opp);
	// TODO: data->resume_mem_volt
	if (data->platdata->suspend_max_voltage) {
		data->suspend_center_volt = data->resume_center_volt;
		data->suspend_mem_volt = data->resume_mem_volt;
	}
	dev_pm_opp_put(opp);

	data->profile = (struct devfreq_dev_profile) {
		.polling_ms	= 50,
		.target		= rk35_dmcfreq_target,
		.get_dev_status	= rk35_dmcfreq_get_dev_status,
		.get_cur_freq	= rk35_dmcfreq_get_cur_freq,
		.initial_freq	= data->freq,
	};

	cpu_latency_qos_add_request(&data->qos, PM_QOS_DEFAULT_VALUE);

	data->devfreq = devm_devfreq_add_device(dev, &data->profile,
						DEVFREQ_GOV_SIMPLE_ONDEMAND,
						&data->ondemand_data);
	if (IS_ERR(data->devfreq)) {
		ret = dev_err_probe(dev, PTR_ERR(data->devfreq),
				    "Failed to add devfreq device\n");
		goto err_qos;
	}

	ret = devm_devfreq_register_opp_notifier(dev, data->devfreq);
	if (ret) {
		ret = dev_err_probe(dev, ret, "Failed to register opp notifier\n");
		goto err_qos;
	}

	return 0;
err_qos:
	cpu_latency_qos_remove_request(&data->qos);
err_edev:
	devfreq_event_disable_edev(data->edev);

	return ret;
}

static void rk35_dmcfreq_remove(struct platform_device *pdev)
{
	struct rk35_dmcfreq *data = platform_get_drvdata(pdev);

	cpu_latency_qos_remove_request(&data->qos);
	devfreq_event_disable_edev(data->edev);
}

static __maybe_unused int rk35_dmcfreq_suspend(struct device *dev)
{
	struct rk35_dmcfreq *data = dev_get_drvdata(dev);
	struct arm_smccc_res res;
	int ret;

	ret = devfreq_event_disable_edev(data->edev);
	if (ret < 0) {
		dev_err(dev, "Failed to disable devfreq event device: %i\n", ret);
		return ret;
	}

	ret = devfreq_suspend_device(data->devfreq);
	if (ret < 0) {
		dev_err(dev, "Failed to suspend devfreq device: %i\n", ret);
		return ret;
	}

	scoped_guard(mutex, &data->lock) {
		ret = rk35_dmcfreq_set_freq(data, data->suspend_freq);
		if (ret)
			return ret;
		ret = rk35_dmcfreq_set_voltage(data, data->suspend_center_volt,
					       data->suspend_mem_volt);
		if (ret)
			return ret;

		data->ddr_psci_param->sr_idle_en = true;
		arm_smccc_smc(ROCKCHIP_SIP_DRAM_CONFIG,
			      ROCKCHIP_SIP_SHARE_PAGE_TYPE_DDR, 0, ROCKCHIP_SIP_CONFIG_DRAM_SET_AT_SR,
			      0, 0, 0, 0, &res);
		if (res.a0) {
			dev_err(dev, "Failed to enable DRAM self refresh: %lu\n",
				res.a0);
			return res.a0;
		}
	}

	return 0;
}

static __maybe_unused int rk35_dmcfreq_resume(struct device *dev)
{
	struct rk35_dmcfreq *data = dev_get_drvdata(dev);
	struct arm_smccc_res res;
	int ret;

	scoped_guard(mutex, &data->lock) {
		data->ddr_psci_param->sr_idle_en = false;
		arm_smccc_smc(ROCKCHIP_SIP_DRAM_CONFIG,
			      ROCKCHIP_SIP_SHARE_PAGE_TYPE_DDR, 0, ROCKCHIP_SIP_CONFIG_DRAM_SET_AT_SR,
			      0, 0, 0, 0, &res);
		if (res.a0) {
			dev_err(dev, "Failed to disable DRAM self refresh: %lu\n",
				res.a0);
			return res.a0;
		}

		ret = rk35_dmcfreq_set_voltage(data, data->resume_center_volt,
					       data->resume_mem_volt);
		if (ret)
			return ret;
		ret = rk35_dmcfreq_set_freq(data, data->resume_freq);
		if (ret)
			return ret;
	}

	ret = devfreq_resume_device(data->devfreq);
	if (ret < 0) {
		dev_err(dev, "Failed to resume devfreq device: %i\n", ret);
		return ret;
	}

	ret = devfreq_event_enable_edev(data->edev);
	if (ret < 0) {
		dev_err(dev, "Failed to enable devfreq event device: %i\n", ret);
		return ret;
	}

	return 0;
}

static SIMPLE_DEV_PM_OPS(rk35_dmcfreq_pm, rk35_dmcfreq_suspend,
			 rk35_dmcfreq_resume);

static const struct rk35_dmcfreq_platdata rk3568_platdata = {
	.upthreshold = 40,
	.downdifferential = 20,
	.min_tfa_version = 0x101,
};

static const struct rk35_dmcfreq_platdata rk3576_platdata = {
	.upthreshold = 40,
	.downdifferential = 20,
	.has_mem_supply = true,
	.suspend_max_voltage = true,
};

static const struct rk35_dmcfreq_platdata rk3588_platdata = {
	.upthreshold = 25,
	.downdifferential = 20,
	.has_mem_supply = true,
	.suspend_max_voltage = true,
};

static const struct of_device_id rk35_dmcfreq_of_match[] = {
	{ .compatible = "rockchip,rk3568-dmc", .data = &rk3568_platdata },
	{ .compatible = "rockchip,rk3576-dmc", .data = &rk3576_platdata },
	{ .compatible = "rockchip,rk3588-dmc", .data = &rk3588_platdata },
	{ },
};
MODULE_DEVICE_TABLE(of, rk35_dmcfreq_of_match);

static struct platform_driver rk35_dmcfreq_driver = {
	.probe	= rk35_dmcfreq_probe,
	.remove = rk35_dmcfreq_remove,
	.driver = {
		.name	= "rk35xx-dmc-freq",
		.pm	= &rk35_dmcfreq_pm,
		.of_match_table = rk35_dmcfreq_of_match,
	},
};
module_platform_driver(rk35_dmcfreq_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Alexander Warnecke <awarnecke002@hotmail.com>");
MODULE_DESCRIPTION("RK35xx DMC frequency driver with devfreq framework");
