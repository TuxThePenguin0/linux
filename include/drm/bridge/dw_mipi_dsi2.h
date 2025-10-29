/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2024, Fuzhou Rockchip Electronics Co., Ltd
 *
 * Authors: Guochun Huang <hero.huang@rock-chips.com>
 *          Heiko Stuebner <heiko.stuebner@cherry.de>
 */

#ifndef __DW_MIPI_DSI2__
#define __DW_MIPI_DSI2__

#include <linux/regmap.h>
#include <linux/types.h>

#include <drm/drm_atomic.h>
#include <drm/drm_bridge.h>
#include <drm/drm_connector.h>
#include <drm/drm_crtc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>

#define DSI2_PWR_UP			0x000c
#define RESET				0
#define POWER_UP			BIT(0)
#define CMD_TX_MODE(x)			FIELD_PREP(BIT(24), x)
#define DSI2_SOFT_RESET			0x0010
#define SYS_RSTN			BIT(2)
#define PHY_RSTN			BIT(1)
#define IPI_RSTN			BIT(0)
#define INT_ST_MAIN			0x0014
#define DSI2_MODE_CTRL			0x0018
#define DSI2_MODE_STATUS		0x001c
#define DSI2_CORE_STATUS		0x0020
#define PRI_RD_DATA_AVAIL		BIT(26)
#define PRI_FIFOS_NOT_EMPTY		BIT(25)
#define PRI_BUSY			BIT(24)
#define CRI_RD_DATA_AVAIL		BIT(18)
#define CRT_FIFOS_NOT_EMPTY		BIT(17)
#define CRI_BUSY			BIT(16)
#define IPI_FIFOS_NOT_EMPTY		BIT(9)
#define IPI_BUSY			BIT(8)
#define CORE_FIFOS_NOT_EMPTY		BIT(1)
#define CORE_BUSY			BIT(0)
#define MANUAL_MODE_CFG			0x0024
#define MANUAL_MODE_EN			BIT(0)
#define DSI2_TIMEOUT_HSTX_CFG		0x0048
#define TO_HSTX(x)			FIELD_PREP(GENMASK(15, 0), x)
#define DSI2_TIMEOUT_HSTXRDY_CFG	0x004c
#define TO_HSTXRDY(x)			FIELD_PREP(GENMASK(15, 0), x)
#define DSI2_TIMEOUT_LPRX_CFG		0x0050
#define TO_LPRXRDY(x)			FIELD_PREP(GENMASK(15, 0), x)
#define DSI2_TIMEOUT_LPTXRDY_CFG	0x0054
#define TO_LPTXRDY(x)			FIELD_PREP(GENMASK(15, 0), x)
#define DSI2_TIMEOUT_LPTXTRIG_CFG	0x0058
#define TO_LPTXTRIG(x)			FIELD_PREP(GENMASK(15, 0), x)
#define DSI2_TIMEOUT_LPTXULPS_CFG	0x005c
#define TO_LPTXULPS(x)			FIELD_PREP(GENMASK(15, 0), x)
#define DSI2_TIMEOUT_BTA_CFG		0x60
#define TO_BTA(x)			FIELD_PREP(GENMASK(15, 0), x)

#define DSI2_PHY_MODE_CFG		0x0100
#define PPI_WIDTH(x)			FIELD_PREP(GENMASK(9, 8), x)
#define PHY_LANES(x)			FIELD_PREP(GENMASK(5, 4), (x) - 1)
#define PHY_TYPE(x)			FIELD_PREP(BIT(0), x)
#define DSI2_PHY_CLK_CFG		0X0104
#define PHY_LPTX_CLK_DIV(x)		FIELD_PREP(GENMASK(12, 8), x)
#define CLK_TYPE_MASK			BIT(0)
#define NON_CONTINUOUS_CLK		BIT(0)
#define CONTINUOUS_CLK			0
#define DSI2_PHY_LP2HS_MAN_CFG		0x010c
#define PHY_LP2HS_TIME(x)		FIELD_PREP(GENMASK(28, 0), x)
#define DSI2_PHY_HS2LP_MAN_CFG		0x0114
#define PHY_HS2LP_TIME(x)		FIELD_PREP(GENMASK(28, 0), x)
#define DSI2_PHY_MAX_RD_T_MAN_CFG	0x011c
#define PHY_MAX_RD_TIME(x)		FIELD_PREP(GENMASK(26, 0), x)
#define DSI2_PHY_ESC_CMD_T_MAN_CFG	0x0124
#define PHY_ESC_CMD_TIME(x)		FIELD_PREP(GENMASK(28, 0), x)
#define DSI2_PHY_ESC_BYTE_T_MAN_CFG	0x012c
#define PHY_ESC_BYTE_TIME(x)		FIELD_PREP(GENMASK(28, 0), x)

#define DSI2_PHY_IPI_RATIO_MAN_CFG	0x0134
#define PHY_IPI_RATIO(x)		FIELD_PREP(GENMASK(21, 0), x)
#define DSI2_PHY_SYS_RATIO_MAN_CFG	0x013C
#define PHY_SYS_RATIO(x)		FIELD_PREP(GENMASK(16, 0), x)

#define DSI2_DSI_GENERAL_CFG		0x0200
#define BTA_EN				BIT(1)
#define EOTP_TX_EN			BIT(0)
#define DSI2_DSI_VCID_CFG		0x0204
#define TX_VCID(x)			FIELD_PREP(GENMASK(1, 0), x)
#define DSI2_DSI_SCRAMBLING_CFG		0x0208
#define SCRAMBLING_SEED(x)		FIELD_PREP(GENMASK(31, 16), x)
#define SCRAMBLING_EN			BIT(0)
#define DSI2_DSI_VID_TX_CFG		0x020c
#define LPDT_DISPLAY_CMD_EN		BIT(20)
#define BLK_VFP_HS_EN			BIT(14)
#define BLK_VBP_HS_EN			BIT(13)
#define BLK_VSA_HS_EN			BIT(12)
#define BLK_HFP_HS_EN			BIT(6)
#define BLK_HBP_HS_EN			BIT(5)
#define BLK_HSA_HS_EN			BIT(4)
#define VID_MODE_TYPE(x)		FIELD_PREP(GENMASK(1, 0), x)
#define DSI2_CRI_TX_HDR			0x02c0
#define CMD_TX_MODE(x)			FIELD_PREP(BIT(24), x)
#define DSI2_CRI_TX_PLD			0x02c4
#define DSI2_CRI_RX_HDR			0x02c8
#define DSI2_CRI_RX_PLD			0x02cc

#define DSI2_IPI_COLOR_MAN_CFG		0x0300
#define IPI_DEPTH(x)			FIELD_PREP(GENMASK(7, 4), x)
#define IPI_DEPTH_5_6_5_BITS		0x02
#define IPI_DEPTH_6_BITS		0x03
#define IPI_DEPTH_8_BITS		0x05
#define IPI_DEPTH_10_BITS		0x06
#define IPI_FORMAT(x)			FIELD_PREP(GENMASK(3, 0), x)
#define IPI_FORMAT_RGB			0x0
#define IPI_FORMAT_DSC			0x0b
#define DSI2_IPI_VID_HSA_MAN_CFG	0x0304
#define VID_HSA_TIME(x)			FIELD_PREP(GENMASK(29, 0), x)
#define DSI2_IPI_VID_HBP_MAN_CFG	0x030c
#define VID_HBP_TIME(x)			FIELD_PREP(GENMASK(29, 0), x)
#define DSI2_IPI_VID_HACT_MAN_CFG	0x0314
#define VID_HACT_TIME(x)		FIELD_PREP(GENMASK(29, 0), x)
#define DSI2_IPI_VID_HLINE_MAN_CFG	0x031c
#define VID_HLINE_TIME(x)		FIELD_PREP(GENMASK(29, 0), x)
#define DSI2_IPI_VID_VSA_MAN_CFG	0x0324
#define VID_VSA_LINES(x)		FIELD_PREP(GENMASK(9, 0), x)
#define DSI2_IPI_VID_VBP_MAN_CFG	0X032C
#define VID_VBP_LINES(x)		FIELD_PREP(GENMASK(9, 0), x)
#define DSI2_IPI_VID_VACT_MAN_CFG	0X0334
#define VID_VACT_LINES(x)		FIELD_PREP(GENMASK(13, 0), x)
#define DSI2_IPI_VID_VFP_MAN_CFG	0X033C
#define VID_VFP_LINES(x)		FIELD_PREP(GENMASK(9, 0), x)
#define DSI2_IPI_PIX_PKT_CFG		0x0344
#define MAX_PIX_PKT(x)			FIELD_PREP(GENMASK(15, 0), x)

#define DSI2_INT_ST_PHY			0x0400
#define DSI2_INT_MASK_PHY		0x0404
#define DSI2_INT_ST_TO			0x0410
#define DSI2_INT_MASK_TO		0x0414
#define DSI2_INT_ST_ACK			0x0420
#define DSI2_INT_MASK_ACK		0x0424
#define DSI2_INT_ST_IPI			0x0430
#define DSI2_INT_MASK_IPI		0x0434
#define DSI2_INT_ST_FIFO		0x0440
#define DSI2_INT_MASK_FIFO		0x0444
#define DSI2_INT_ST_PRI			0x0450
#define DSI2_INT_MASK_PRI		0x0454
#define DSI2_INT_ST_CRI			0x0460
#define DSI2_INT_MASK_CRI		0x0464
#define DSI2_INT_FORCE_CRI		0x0468
#define DSI2_MAX_REGISGER		DSI2_INT_FORCE_CRI

struct drm_display_mode;
struct drm_encoder;
struct dw_mipi_dsi2;
struct mipi_dsi_device;
struct platform_device;

enum dw_mipi_dsi2_phy_type {
	DW_MIPI_DSI2_DPHY,
	DW_MIPI_DSI2_CPHY,
};

struct dw_mipi_dsi2_phy_iface {
	int ppi_width;
	enum dw_mipi_dsi2_phy_type phy_type;
};

struct dw_mipi_dsi2_phy_timing {
	u32 data_hs2lp;
	u32 data_lp2hs;
};

struct dw_mipi_dsi2_phy_ops {
	int (*init)(void *priv_data);
	void (*power_on)(void *priv_data);
	void (*power_off)(void *priv_data);
	void (*get_interface)(void *priv_data, struct dw_mipi_dsi2_phy_iface *iface);
	int (*get_lane_mbps)(void *priv_data,
			     const struct drm_display_mode *mode,
			     unsigned long mode_flags, u32 lanes, u32 format,
			     unsigned int *lane_mbps);
	int (*get_timing)(void *priv_data, unsigned int lane_mbps,
			  struct dw_mipi_dsi2_phy_timing *timing);
	int (*get_esc_clk_rate)(void *priv_data, unsigned int *esc_clk_rate);
};

struct dw_mipi_dsi2_host_ops {
	int (*attach)(void *priv_data,
		      struct mipi_dsi_device *dsi);
	int (*detach)(void *priv_data,
		      struct mipi_dsi_device *dsi);
};

struct dw_mipi_dsi2_plat_data {
	struct regmap *regmap;
	unsigned int max_data_lanes;

	enum drm_mode_status (*mode_valid)(void *priv_data,
					   const struct drm_display_mode *mode,
					   unsigned long mode_flags,
					   u32 lanes, u32 format);

	bool (*mode_fixup)(void *priv_data, const struct drm_display_mode *mode,
			   struct drm_display_mode *adjusted_mode);

	u32 *(*get_input_bus_fmts)(void *priv_data,
				   struct drm_bridge *bridge,
				   struct drm_bridge_state *bridge_state,
				   struct drm_crtc_state *crtc_state,
				   struct drm_connector_state *conn_state,
				   u32 output_fmt,
				   unsigned int *num_input_fmts);

	const struct dw_mipi_dsi2_phy_ops *phy_ops;
	const struct dw_mipi_dsi2_host_ops *host_ops;

	void *priv_data;
};

struct dw_mipi_dsi2 {
	struct drm_bridge bridge;
	struct mipi_dsi_host dsi_host;
	struct drm_bridge *panel_bridge;
	struct device *dev;
	struct regmap *regmap;
	struct clk *pclk;
	struct clk *sys_clk;

	unsigned int lane_mbps; /* per lane */
	u32 channel;
	u32 lanes;
	u32 format;
	unsigned long mode_flags;

	struct drm_display_mode mode;
	const struct dw_mipi_dsi2_plat_data *plat_data;
};

struct dw_mipi_dsi2 *dw_mipi_dsi2_probe(struct platform_device *pdev,
					const struct dw_mipi_dsi2_plat_data *plat_data);
void dw_mipi_dsi2_remove(struct dw_mipi_dsi2 *dsi2);
int dw_mipi_dsi2_bind(struct dw_mipi_dsi2 *dsi2, struct drm_encoder *encoder);
void dw_mipi_dsi2_unbind(struct dw_mipi_dsi2 *dsi2);

#endif /* __DW_MIPI_DSI2__ */
