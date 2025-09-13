// SPDX-License-Identifier: GPL-2.0+
/*
 * Pericom PI3DPX1207C Type-C retimer driver
 * Based on OnSemi NB7VPQ904M driver
 *
 * Copyright (C) 2023 Dmitry Baryshkov <dmitry.baryshkov@linaro.org>
 * Copyright (c) 2025 Alexander Warnecke <awarnecke002@hotmail.com>
 */

#include <linux/bitfield.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/regulator/consumer.h>
#include <linux/usb/typec_altmode.h>
#include <linux/usb/typec_dp.h>
#include <linux/usb/typec_mux.h>
#include <linux/usb/typec_retimer.h>

/*
 * PI3DPX1207C has 32 registers, however only some are documented and relevant.
 * It does not use typical i2c addressed reads/writes, instead the host sends a
 * bulk read/write and sends/receives the contents of the number of registers in
 * the command, starting from the first register.
 */
struct pi3dpx1207_regs {
	u8 revision_vendor;
	u8 type_id;
	u8 count;
	u8 mode;
	u8 override;
	u8 setting_con_rx2;
	u8 setting_con_tx2;
	u8 setting_con_tx1;
	u8 setting_con_rx1;
	u8 reserved;
	u8 feature_con_rxtx2;
	u8 feature_con_rxtx1;
	u8 thresh_feature_timing;
} __packed;

#define VENDOR_MASK				GENMASK(3, 0)
#define VENDOR_PERICOM				3
#define REVISION_MASK				GENMASK(7, 4)

#define ID_MASK					GENMASK(3, 0)
#define ID_PI3DPX1207C				1
#define TYPE_MASK				GENMASK(7, 4)
#define TYPE_ACTIVE_MUX				1

#define MODE_PIN_RXDET_EN			BIT(2)
#define MODE_CONF_MASK				GENMASK(7, 4)
#define MODE_CONF_OPEN				0
#define MODE_CONF_FLIP				BIT(0)
#define MODE_CONF_4LANE_DP			BIT(1)
#define MODE_CONF_USB3				BIT(2)
#define MODE_CONF_USB3_AND_2LANE_DP		(MODE_CONF_4LANE_DP | MODE_CONF_USB3)
#define MODE_CONF_USB3_AND_2LANE_DP_FLIP	BIT(3)

#define OVERRIDE_IN_HPD_HIZ			BIT(0)
#define OVERRIDE_IN_HPD				BIT(1)
#define OVERRIDE_HPD_CTL_I2C			BIT(2)
#define OVERRIDE_PD_CON_MASK			GENMASK(7, 4)
#define OVERRIDE_PD_CON_RX2			BIT(0)
#define OVERRIDE_PD_CON_TX2			BIT(1)
#define OVERRIDE_PD_CON_TX1			BIT(2)
#define OVERRIDE_PD_CON_RX1			BIT(3)

#define SETTING_SWING_MASK			GENMASK(1, 0)
#define SETTING_FLATGAIN_MASK			GENMASK(3, 2)
#define SETTING_EQUALIZER_MASK			GENMASK(7, 4)
#define SETTING_EQUALIZER_DEFAULT		10

#define FEATURE_AUX_EN				BIT(2)
#define THRESH_IDET_VTH_MASK			GENMASK(7, 6)

struct pi3dpx1207c {
	struct i2c_client *client;
	struct device *dev;
	struct pi3dpx1207_regs regs;
	struct mutex lock;
	struct typec_switch_dev *sw;
	struct typec_retimer *retimer;

	struct typec_switch *typec_switch;
	struct typec_mux *typec_mux;

	bool swap_data_lanes;

	unsigned long mode;
	unsigned int svid;
	enum typec_orientation orientation;
};

static int pi3dpx1207c_read_write_regs(struct pi3dpx1207c *priv, bool write)
{
	struct i2c_msg msg;
	int ret;

	msg.addr = priv->client->addr;
	msg.flags = write ? 0 : I2C_M_RD;
	msg.len = sizeof(priv->regs);
	msg.buf = (u8 *)&priv->regs;

	ret = i2c_transfer(priv->client->adapter, &msg, 1);

	if (ret < 0) {
		dev_err(priv->dev, "Failed to complete i2c transfer: %i\n", ret);
		return ret;
	}

	dev_notice(priv->dev, "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n", priv->regs.revision_vendor, priv->regs.type_id, priv->regs.count, priv->regs.mode, priv->regs.override, priv->regs.setting_con_rx2, priv->regs.setting_con_tx2, priv->regs.setting_con_tx1, priv->regs.setting_con_rx1, priv->regs.feature_con_rxtx2, priv->regs.feature_con_rxtx1, priv->regs.thresh_feature_timing);

	return 0;
}

static int pi3dpx1207c_set(struct pi3dpx1207c *priv)
{
	bool reverse;
	u8 conf, lanes;
	bool aux;

	reverse = (priv->orientation == TYPEC_ORIENTATION_REVERSE) != priv->swap_data_lanes;

	// TODO: The logic for when to pick each mode needs to be reworked, but either I've misunderstood something or tcpci isn't doing something it should
	if (!priv->orientation) {
		/* No cable attached */
		conf = MODE_CONF_OPEN;
		lanes = 0;
		aux = false;
	} else if (!priv->svid) {
		switch (priv->mode) {
		case TYPEC_STATE_SAFE:
		case TYPEC_STATE_USB:
			/*
			 * Normal Orientation (CC1)
			 * RX2 -> X
			 * TX2 -> X
			 * RX1 -> USB RX
			 * TX1 -> USB TX
			 * Flipped Orientation (CC2)
			 * RX2 -> USB RX
			 * TX2 -> USB TX
			 * RX1 -> X
			 * TX1 -> X
			 *
			 * Reversed if data lanes are swapped
			 */
			if (!reverse) {
				conf = MODE_CONF_USB3;
				/* Enable lanes RX1 and TX1 */
				lanes = OVERRIDE_PD_CON_TX1 | OVERRIDE_PD_CON_RX1;
			} else {
				conf = MODE_CONF_USB3 | MODE_CONF_FLIP;
				/* Enable lanes RX2 and TX2 */
				lanes = OVERRIDE_PD_CON_RX2 | OVERRIDE_PD_CON_TX2;
			}
			aux = false;
			break;
		default:
			return -EINVAL;
		}
	} else if (priv->svid == USB_TYPEC_DP_SID) {
		switch (priv->mode) {
		case TYPEC_DP_STATE_C:
		case TYPEC_DP_STATE_E:
			/*
			 * Normal Orientation
			 * RX2 -> DP1
			 * TX2 -> DP0
			 * RX1 -> DP2
			 * TX1 -> DP3
			 * Flipped Orientation
			 * RX2 -> DP2
			 * TX2 -> DP3
			 * RX1 -> DP1
			 * TX1 -> DP0
			 */
			conf = MODE_CONF_4LANE_DP;
			break;
		case TYPEC_STATE_SAFE:
		case TYPEC_STATE_USB:
		case TYPEC_DP_STATE_D:
		case TYPEC_DP_STATE_F:
			/*
			 * Normal Orientation
			 * RX2 -> DP1
			 * TX2 -> DP0
			 * RX1 -> USB RX
			 * TX1 -> USB TX
			 * Flipped Orientation
			 * RX2 -> USB RX
			 * TX2 -> USB TX
			 * RX1 -> DP1
			 * TX1 -> DP0
			 *
			 * Reversed if data lanes are swapped
			 */
			conf = reverse ?
			       MODE_CONF_USB3_AND_2LANE_DP_FLIP :
			       MODE_CONF_USB3_AND_2LANE_DP;
			break;
		default:
			return -EOPNOTSUPP;
		}
		/* DP modes enable all lanes */
		lanes = OVERRIDE_PD_CON_RX2 | OVERRIDE_PD_CON_TX2 |
			OVERRIDE_PD_CON_TX1 | OVERRIDE_PD_CON_RX1;
		aux = true;
	} else {
		return -EINVAL;
	}

	FIELD_MODIFY(MODE_CONF_MASK, &priv->regs.mode, conf);
	FIELD_MODIFY(OVERRIDE_PD_CON_MASK, &priv->regs.override, ~lanes);
	FIELD_MODIFY(FEATURE_AUX_EN, &priv->regs.thresh_feature_timing, !aux);

	return pi3dpx1207c_read_write_regs(priv, true);
}

static int pi3dpx1207c_initialize(struct pi3dpx1207c *priv)
{
	int vendor, revision, id, type;
	int ret;

	ret = pi3dpx1207c_read_write_regs(priv, false);
	if (ret)
		return ret;

	vendor = FIELD_GET(VENDOR_MASK, priv->regs.revision_vendor);
	revision = FIELD_GET(REVISION_MASK, priv->regs.revision_vendor);
	id = FIELD_GET(ID_MASK, priv->regs.type_id);
	type = FIELD_GET(TYPE_MASK, priv->regs.type_id);

	if (vendor != VENDOR_PERICOM)
		return dev_err_probe(priv->dev, -EINVAL,
				     "Unsupported vendor: %i\n", vendor);

	if (revision != 1)
		return dev_err_probe(priv->dev, -EINVAL,
				     "Unsupported revision: %i\n", revision);

	if (id != ID_PI3DPX1207C)
		return dev_err_probe(priv->dev, -EINVAL,
				     "Unsupported device ID: %i\n", id);

	if (type != TYPE_ACTIVE_MUX)
		return dev_err_probe(priv->dev, -EINVAL,
				     "Unsupported device type: %i\n", type);

	/* Default equalizer setting is not correctly initialized on startup */
	FIELD_MODIFY(SETTING_EQUALIZER_MASK, &priv->regs.setting_con_rx2,
		     SETTING_EQUALIZER_DEFAULT);
	FIELD_MODIFY(SETTING_EQUALIZER_MASK, &priv->regs.setting_con_tx2,
		     SETTING_EQUALIZER_DEFAULT);
	FIELD_MODIFY(SETTING_EQUALIZER_MASK, &priv->regs.setting_con_tx1,
		     SETTING_EQUALIZER_DEFAULT);
	FIELD_MODIFY(SETTING_EQUALIZER_MASK, &priv->regs.setting_con_rx1,
		     SETTING_EQUALIZER_DEFAULT);

	return pi3dpx1207c_set(priv);
}

static int pi3dpx1207c_sw_set(struct typec_switch_dev *sw,
				  enum typec_orientation orientation)
{
	struct pi3dpx1207c *priv = typec_switch_get_drvdata(sw);
	int ret;

	ret = typec_switch_set(priv->typec_switch, orientation);
	if (ret)
		return ret;

	guard(mutex)(&priv->lock);
	if (priv->orientation != orientation) {
		priv->orientation = orientation;
		return pi3dpx1207c_set(priv);
	}

	return 0;
}

static int pi3dpx1207c_retimer_set(struct typec_retimer *retimer,
				   struct typec_retimer_state *state)
{
	struct pi3dpx1207c *priv = typec_retimer_get_drvdata(retimer);
	struct typec_mux_state mux_state;
	int ret = 0;

	scoped_guard(mutex, &priv->lock) {
		if (priv->mode != state->mode) {
			priv->mode = state->mode;

			if (state->alt)
				priv->svid = state->alt->svid;
			else
				priv->svid = 0; // No SVID

			ret = pi3dpx1207c_set(priv);
		}
	}

	if (ret)
		return ret;

	mux_state.alt = state->alt;
	mux_state.data = state->data;
	mux_state.mode = state->mode;

	return typec_mux_set(priv->typec_mux, &mux_state);
}

enum {
	NORMAL_LANE_MAPPING,
	INVERT_LANE_MAPPING,
};

#define DATA_LANES_COUNT	4

static const int supported_data_lane_mapping[][DATA_LANES_COUNT] = {
	[NORMAL_LANE_MAPPING] = { 0, 1, 2, 3 },
	[INVERT_LANE_MAPPING] = { 3, 2, 1, 0 },
};

static int pi3dpx1207c_parse_data_lanes_mapping(struct pi3dpx1207c *priv)
{
	struct device_node *ep;
	u32 data_lanes[4];
	int ret, i, j;

	ep = of_graph_get_endpoint_by_regs(priv->dev->of_node, 1, 0);
	if (!ep)
		return 0;

	ret = of_property_count_u32_elems(ep, "data-lanes");
	if (ret == -EINVAL)
		/* Property isn't here, consider default mapping */
		goto out_done;
	if (ret < 0)
		goto out_error;
	if (ret != DATA_LANES_COUNT) {
		ret = dev_err_probe(priv->dev, -EINVAL, "Expected 4 data lanes\n");
		goto out_error;
	};

	ret = of_property_read_u32_array(ep, "data-lanes", data_lanes, DATA_LANES_COUNT);
	if (ret)
		goto out_error;

	for (i = 0; i < ARRAY_SIZE(supported_data_lane_mapping); i++) {
		for (j = 0; i < DATA_LANES_COUNT; j++)
			if (data_lanes[j] != supported_data_lane_mapping[i][j])
				break;

		if (j == DATA_LANES_COUNT)
			break;
	}

	switch (i) {
	case NORMAL_LANE_MAPPING:
		break;
	case INVERT_LANE_MAPPING:
		priv->swap_data_lanes = true;
		break;
	default:
		ret = dev_err_probe(priv->dev, -EINVAL, "Invalid data lane mapping\n");
		goto out_error;
	}

out_done:
	ret = 0;
out_error:
	of_node_put(ep);
	return ret;
}

static int pi3dpx1207c_probe(struct i2c_client *client)
{
	struct pi3dpx1207c *priv;
	struct typec_switch_desc switch_desc = {};
	struct typec_retimer_desc retimer_desc = {};
	int ret;

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->client = client;
	i2c_set_clientdata(client, priv);

	priv->dev = &client->dev;

	priv->mode = TYPEC_STATE_SAFE;
	priv->orientation = TYPEC_ORIENTATION_NONE;

	mutex_init(&priv->lock);

	ret = devm_regulator_get_enable_optional(priv->dev, "vcc");
	if (ret)
		return ret;

	priv->typec_switch = fwnode_typec_switch_get(priv->dev->fwnode);
	if (IS_ERR(priv->typec_switch))
		return dev_err_probe(priv->dev, PTR_ERR(priv->typec_switch),
				     "Failed to acquire orientation-switch\n");

	priv->typec_mux = fwnode_typec_mux_get(priv->dev->fwnode);
	if (IS_ERR(priv->typec_mux)) {
		ret = dev_err_probe(priv->dev, PTR_ERR(priv->typec_mux),
				    "Failed to acquire mode-switch\n");
		goto err_switch_put;
	}

	ret = pi3dpx1207c_parse_data_lanes_mapping(priv);
	if (ret)
		goto err_mux_put;

	ret = pi3dpx1207c_initialize(priv);
	if (ret)
		goto err_mux_put;

	switch_desc.drvdata = priv;
	switch_desc.fwnode = priv->dev->fwnode;
	switch_desc.set = pi3dpx1207c_sw_set;

	priv->sw = typec_switch_register(priv->dev, &switch_desc);
	if (IS_ERR(priv->sw)) {
		ret = dev_err_probe(priv->dev, PTR_ERR(priv->sw),
				    "Failed to register typec switch\n");
		goto err_mux_put;
	}

	retimer_desc.drvdata = priv;
	retimer_desc.fwnode = priv->dev->fwnode;
	retimer_desc.set = pi3dpx1207c_retimer_set;

	priv->retimer = typec_retimer_register(priv->dev, &retimer_desc);
	if (IS_ERR(priv->retimer)) {
		ret = dev_err_probe(priv->dev, PTR_ERR(priv->retimer),
				    "Failed to register typec retimer\n");
		goto err_switch_unregister;
	}

	return 0;

err_switch_unregister:
	typec_switch_unregister(priv->sw);
err_mux_put:
	typec_mux_put(priv->typec_mux);
err_switch_put:
	typec_switch_put(priv->typec_switch);

	return ret;
}

static void pi3dpx1207c_remove(struct i2c_client *client)
{
	struct pi3dpx1207c *priv = i2c_get_clientdata(client);

	typec_retimer_unregister(priv->retimer);
	typec_switch_unregister(priv->sw);

	typec_mux_put(priv->typec_mux);
	typec_switch_put(priv->typec_switch);
}

static const struct of_device_id pi3dpx1207c_of_table[] = {
	{ .compatible = "pericom,pi3dpx1207c" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, pi3dpx1207c_of_table);

static const struct i2c_device_id pi3dpx1207c_table[] = {
	{ "pi3dpx1207c" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, pi3dpx1207c_table);

static struct i2c_driver pi3dpx1207c_driver = {
	.driver = {
		.name = "pi3dpx1207c",
		.of_match_table = pi3dpx1207c_of_table,
	},
	.probe = pi3dpx1207c_probe,
	.remove = pi3dpx1207c_remove,
	.id_table = pi3dpx1207c_table,
};
module_i2c_driver(pi3dpx1207c_driver);

MODULE_AUTHOR("Alexander Warnecke <awarnecke002@hotmail.com>");
MODULE_DESCRIPTION("Pericom PI3DPX1207C Type-C retimer driver");
MODULE_LICENSE("GPL");
