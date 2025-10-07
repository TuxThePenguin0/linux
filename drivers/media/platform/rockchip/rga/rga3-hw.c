// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) Pengutronix e.K.
 * Author: Sven Püschel <s.pueschel@pengutronix.de>
 */

#include <linux/pm_runtime.h>
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/printk.h>

#include <media/v4l2-common.h>

#include "rga3-hw.h"
#include "rga.h"

static unsigned int rga3_get_scaling(unsigned int src, unsigned int dst)
{
	if (dst > src) {
		if (((src - 1) << 16) % (dst - 1) == 0)
			return ((src - 1) << 16) / (dst - 1) - 1;
		else
			return ((src - 1) << 16) / (dst - 1);
	} else {
		return ((dst - 1) << 16) / (src - 1) + 1;
	}
}

static bool rga3_has_alpha(const struct rga3_fmt *fmt)
{
	return fmt->hw_format >= RGA3_COLOR_FMT_FIRST_HAS_ALPHA &&
	       fmt->fourcc != V4L2_PIX_FMT_BGRX32 &&
	       fmt->fourcc != V4L2_PIX_FMT_XBGR32 &&
	       fmt->fourcc != V4L2_PIX_FMT_RGBX32 &&
	       fmt->fourcc != V4L2_PIX_FMT_XRGB32;
}

static bool rga3_can_capture(const struct rga3_fmt *fmt)
{
	return fmt->hw_format <= RGA3_COLOR_FMT_LAST_OUTPUT;
}

static void rga3_cmd_set_trans_info(struct rga_ctx *ctx)
{
	struct rockchip_rga *rga = ctx->rga;
	u32 *cmd = rga->cmdbuf_virt;
	unsigned int src_h, src_w, dst_h, dst_w;
	unsigned int reg;
	u16 hor_scl_fac, ver_scl_fac;

	src_h = ctx->in.crop.height;
	src_w = ctx->in.crop.width;
	dst_h = ctx->out.crop.height;
	dst_w = ctx->out.crop.width;

	reg = RGA3_WIN0_RD_CTRL - RGA3_FIRST_CMD_REG;
	cmd[reg >> 2] |= FIELD_PREP(RGA3_WIN_SCALE_HOR_UP, dst_w > src_w)
		      |  FIELD_PREP(RGA3_WIN_SCALE_HOR_BYPASS, dst_w == src_w)
		      |  FIELD_PREP(RGA3_WIN_SCALE_VER_UP, dst_h > src_h)
		      |  FIELD_PREP(RGA3_WIN_SCALE_VER_BYPASS, dst_h == src_h);

	/* stride needs to be in words */
	reg = RGA3_WIN0_VIR_STRIDE - RGA3_FIRST_CMD_REG;
	cmd[reg >> 2] = ctx->in.pix.plane_fmt[0].bytesperline >> 2;
	reg = RGA3_WIN0_UV_VIR_STRIDE - RGA3_FIRST_CMD_REG;
	if (ctx->in.pix.num_planes >= 2)
		cmd[reg >> 2] = ctx->in.pix.plane_fmt[1].bytesperline >> 2;
	else
		cmd[reg >> 2] = ctx->in.pix.plane_fmt[0].bytesperline >> 2;
	reg = RGA3_WR_VIR_STRIDE - RGA3_FIRST_CMD_REG;
	cmd[reg >> 2] = ctx->out.pix.plane_fmt[0].bytesperline >> 2;
	reg = RGA3_WR_PL_VIR_STRIDE - RGA3_FIRST_CMD_REG;
	if (ctx->out.pix.num_planes >= 2)
		cmd[reg >> 2] = ctx->out.pix.plane_fmt[1].bytesperline >> 2;
	else
		cmd[reg >> 2] = ctx->out.pix.plane_fmt[0].bytesperline >> 2;

	reg = RGA3_WIN0_ACT_SIZE - RGA3_FIRST_CMD_REG;
	cmd[reg >> 2] = FIELD_PREP(RGA3_WIDTH, src_w)
		      | FIELD_PREP(RGA3_HEIGHT, src_h);
	reg = RGA3_WIN0_SRC_SIZE - RGA3_FIRST_CMD_REG;
	cmd[reg >> 2] = FIELD_PREP(RGA3_WIDTH, src_w)
		      | FIELD_PREP(RGA3_HEIGHT, src_h);

	reg = RGA3_WIN0_DST_SIZE - RGA3_FIRST_CMD_REG;
	cmd[reg >> 2] = FIELD_PREP(RGA3_WIDTH, dst_w)
		      | FIELD_PREP(RGA3_HEIGHT, dst_h);

	hor_scl_fac = rga3_get_scaling(src_w, dst_w);
	ver_scl_fac = rga3_get_scaling(src_h, dst_h);
	reg = RGA3_WIN0_SCL_FAC - RGA3_FIRST_CMD_REG;
	cmd[reg >> 2] = FIELD_PREP(RGA3_SCALE_HOR_FAC, hor_scl_fac)
		      | FIELD_PREP(RGA3_SCALE_VER_FAC, ver_scl_fac);

	if (rga3_has_alpha(ctx->in.fmt)) {
		/* copy alpha from input */
		reg = RGA3_OVLP_TOP_ALPHA - RGA3_FIRST_CMD_REG;
		cmd[reg >> 2] = FIELD_PREP(RGA3_ALPHA_SELECT_MODE, 1)
			      | FIELD_PREP(RGA3_ALPHA_BLEND_MODE, 1);
		reg = RGA3_OVLP_BOT_ALPHA - RGA3_FIRST_CMD_REG;
		cmd[reg >> 2] = FIELD_PREP(RGA3_ALPHA_SELECT_MODE, 1)
			      | FIELD_PREP(RGA3_ALPHA_BLEND_MODE, 1);
	} else {
		/* just use a 255 alpha value */
		reg = RGA3_OVLP_TOP_CTRL - RGA3_FIRST_CMD_REG;
		cmd[reg >> 2] = FIELD_PREP(RGA3_OVLP_GLOBAL_ALPHA, 0xff)
			      | FIELD_PREP(RGA3_OVLP_COLOR_MODE, 1);
		reg = RGA3_OVLP_BOT_CTRL - RGA3_FIRST_CMD_REG;
		cmd[reg >> 2] = FIELD_PREP(RGA3_OVLP_GLOBAL_ALPHA, 0xff)
			      | FIELD_PREP(RGA3_OVLP_COLOR_MODE, 1);
	}
}

static void rga3_cmd_set_win0_addr(struct rga_ctx *ctx,
				   const struct rga_addrs *addrs)
{
	u32 *cmd = ctx->rga->cmdbuf_virt;
	unsigned int reg;

	reg = RGA3_WIN0_Y_BASE - RGA3_FIRST_CMD_REG;
	cmd[reg >> 2] = addrs->y_addr;
	reg = RGA3_WIN0_U_BASE - RGA3_FIRST_CMD_REG;
	cmd[reg >> 2] = addrs->u_addr;
}

static void rga3_cmd_set_wr_addr(struct rga_ctx *ctx,
				 const struct rga_addrs *addrs)
{
	u32 *cmd = ctx->rga->cmdbuf_virt;
	unsigned int reg;

	reg = RGA3_WR_Y_BASE - RGA3_FIRST_CMD_REG;
	cmd[reg >> 2] = addrs->y_addr;
	reg = RGA3_WR_U_BASE - RGA3_FIRST_CMD_REG;
	cmd[reg >> 2] = addrs->u_addr;
}

static void rga3_cmd_set_win0_format(struct rga_ctx *ctx)
{
	u32 *cmd = ctx->rga->cmdbuf_virt;
	const struct rga3_fmt *in = ctx->in.fmt;
	const struct rga3_fmt *out = ctx->out.fmt;
	const struct v4l2_format_info *in_fmt, *out_fmt;
	unsigned int src_h, src_w, dst_h, dst_w;
	bool r2y, y2r;
	u8 rd_format;
	unsigned int reg;

	src_h = ctx->in.crop.height;
	src_w = ctx->in.crop.width;
	dst_h = ctx->out.crop.height;
	dst_w = ctx->out.crop.width;

	in_fmt = v4l2_format_info(in->fourcc);
	out_fmt = v4l2_format_info(out->fourcc);
	r2y = v4l2_is_format_rgb(in_fmt) && v4l2_is_format_yuv(out_fmt);
	y2r = v4l2_is_format_yuv(in_fmt) && v4l2_is_format_rgb(out_fmt);

	if (in->semi_planar)
		rd_format = RGA3_RDWR_FORMAT_SEMI_PLANAR;
	else
		rd_format = RGA3_RDWR_FORMAT_INTERLEAVED;

	reg = RGA3_WIN0_RD_CTRL - RGA3_FIRST_CMD_REG;
	cmd[reg >> 2] |= FIELD_PREP(RGA3_WIN_PIC_FORMAT, in->hw_format)
		      |  FIELD_PREP(RGA3_WIN_YC_SWAP, in->yc_swap)
		      |  FIELD_PREP(RGA3_WIN_RBUV_SWAP, in->rbuv_swap)
		      |  FIELD_PREP(RGA3_WIN_RD_FORMAT, rd_format)
		      |  FIELD_PREP(RGA3_WIN_R2Y, r2y)
		      |  FIELD_PREP(RGA3_WIN_Y2R, y2r)
		      |  FIELD_PREP(RGA3_WIN_CSC_MODE, RGA3_WIN_CSC_MODE_BT601_F);
}

static void rga3_cmd_enable_win0(struct rga_ctx *ctx)
{
	u32 *cmd = ctx->rga->cmdbuf_virt;
	unsigned int reg;

	reg = RGA3_WIN0_RD_CTRL - RGA3_FIRST_CMD_REG;
	cmd[reg >> 2] |= FIELD_PREP(RGA3_WIN_ENABLE, 1);
}

static void rga3_cmd_set_wr_format(struct rga_ctx *ctx)
{
	u32 *cmd = ctx->rga->cmdbuf_virt;
	const struct rga3_fmt *out = ctx->out.fmt;
	u8 wr_format;
	unsigned int reg;

	if (out->semi_planar)
		wr_format = RGA3_RDWR_FORMAT_SEMI_PLANAR;
	else
		wr_format = RGA3_RDWR_FORMAT_INTERLEAVED;

	reg = RGA3_WR_CTRL - RGA3_FIRST_CMD_REG;
	cmd[reg >> 2] |= FIELD_PREP(RGA3_WR_PIC_FORMAT, out->hw_format)
		      |  FIELD_PREP(RGA3_WR_YC_SWAP, out->yc_swap)
		      |  FIELD_PREP(RGA3_WR_RBUV_SWAP, out->rbuv_swap)
		      |  FIELD_PREP(RGA3_WR_FORMAT, wr_format);
}

static void rga3_cmd_disable_wr_limitation(struct rga_ctx *ctx)
{
	u32 *cmd = ctx->rga->cmdbuf_virt;
	unsigned int reg;

	/* Use the max value to avoid limiting the write speed */
	reg = RGA3_WR_CTRL - RGA3_FIRST_CMD_REG;
	cmd[reg >> 2] |= FIELD_PREP(RGA3_WR_SW_OUTSTANDING_MAX, 63);
}

static void rga3_cmd_set(struct rga_ctx *ctx,
			 struct rga_vb_buffer *src, struct rga_vb_buffer *dst)
{
	struct rockchip_rga *rga = ctx->rga;

	memset(rga->cmdbuf_virt, 0, RGA3_CMDBUF_SIZE * 4);

	rga3_cmd_set_win0_addr(ctx, &src->dma_addrs);
	rga3_cmd_set_wr_addr(ctx, &dst->dma_addrs);

	rga3_cmd_set_win0_format(ctx);
	rga3_cmd_enable_win0(ctx);
	rga3_cmd_set_trans_info(ctx);
	rga3_cmd_set_wr_format(ctx);
	rga3_cmd_disable_wr_limitation(ctx);

	rga_write(rga, RGA3_CMD_ADDR, rga->cmdbuf_phy);

	/* sync CMD buf for RGA */
	dma_sync_single_for_device(rga->dev, rga->cmdbuf_phy,
				   PAGE_SIZE, DMA_BIDIRECTIONAL);
}

static void rga3_hw_start(struct rockchip_rga *rga,
			  struct rga_vb_buffer *src, struct rga_vb_buffer *dst)
{
	struct rga_ctx *ctx = rga->curr;

	rga3_cmd_set(ctx, src, dst);

	/* set to master mode and start the conversion */
	rga_write(rga, RGA3_SYS_CTRL,
		  FIELD_PREP(RGA3_CMD_MODE, RGA3_CMD_MODE_MASTER));
	rga_write(rga, RGA3_INT_EN,
		  FIELD_PREP(RGA3_INT_FRM_DONE, 1) |
		  FIELD_PREP(RGA3_INT_DMA_READ_BUS_ERR, 1) |
		  FIELD_PREP(RGA3_INT_WIN0_FBC_DEC_ERR, 1) |
		  FIELD_PREP(RGA3_INT_WIN0_HOR_ERR, 1) |
		  FIELD_PREP(RGA3_INT_WIN0_VER_ERR, 1) |
		  FIELD_PREP(RGA3_INT_WR_VER_ERR, 1) |
		  FIELD_PREP(RGA3_INT_WR_HOR_ERR, 1) |
		  FIELD_PREP(RGA3_INT_WR_BUS_ERR, 1) |
		  FIELD_PREP(RGA3_INT_WIN0_IN_FIFO_WR_ERR, 1) |
		  FIELD_PREP(RGA3_INT_WIN0_IN_FIFO_RD_ERR, 1) |
		  FIELD_PREP(RGA3_INT_WIN0_HOR_FIFO_WR_ERR, 1) |
		  FIELD_PREP(RGA3_INT_WIN0_HOR_FIFO_RD_ERR, 1) |
		  FIELD_PREP(RGA3_INT_WIN0_VER_FIFO_WR_ERR, 1) |
		  FIELD_PREP(RGA3_INT_WIN0_VER_FIFO_RD_ERR, 1));
	rga_write(rga, RGA3_CMD_CTRL,
		  FIELD_PREP(RGA3_CMD_LINE_START_PULSE, 1));
}

static void rga3_soft_reset(struct rockchip_rga *rga)
{
	u32 i;

	rga_write(rga, RGA3_SYS_CTRL,
		  FIELD_PREP(RGA3_CCLK_SRESET, 1) |
		  FIELD_PREP(RGA3_ACLK_SRESET, 1));

	for (i = 0; i < RGA3_RESET_TIMEOUT; i++) {
		if (FIELD_GET(RGA3_RO_SRST_DONE, rga_read(rga, RGA3_RO_SRST)))
			break;

		udelay(1);
	}

	if (i == RGA3_RESET_TIMEOUT)
		pr_err("Timeout of %d usec reached while waiting for an rga3 soft reset\n", i);

	rga_write(rga, RGA3_SYS_CTRL, 0);
	rga_iommu_restore(rga);
}

static enum rga_irq_result rga3_handle_irq(struct rockchip_rga *rga)
{
	u32 intr;

	intr = rga_read(rga, RGA3_INT_RAW);
	/* clear all interrupts */
	rga_write(rga, RGA3_INT_CLR, intr);

	if (FIELD_GET(RGA3_INT_FRM_DONE, intr))
		return RGA_IRQ_DONE;
	if (FIELD_GET(RGA3_INT_DMA_READ_BUS_ERR, intr) ||
	    FIELD_GET(RGA3_INT_WIN0_FBC_DEC_ERR, intr) ||
	    FIELD_GET(RGA3_INT_WIN0_HOR_ERR, intr) ||
	    FIELD_GET(RGA3_INT_WIN0_VER_ERR, intr) ||
	    FIELD_GET(RGA3_INT_WR_VER_ERR, intr) ||
	    FIELD_GET(RGA3_INT_WR_HOR_ERR, intr) ||
	    FIELD_GET(RGA3_INT_WR_BUS_ERR, intr) ||
	    FIELD_GET(RGA3_INT_WIN0_IN_FIFO_WR_ERR, intr) ||
	    FIELD_GET(RGA3_INT_WIN0_IN_FIFO_RD_ERR, intr) ||
	    FIELD_GET(RGA3_INT_WIN0_HOR_FIFO_WR_ERR, intr) ||
	    FIELD_GET(RGA3_INT_WIN0_HOR_FIFO_RD_ERR, intr) ||
	    FIELD_GET(RGA3_INT_WIN0_VER_FIFO_WR_ERR, intr) ||
	    FIELD_GET(RGA3_INT_WIN0_VER_FIFO_RD_ERR, intr)) {
		rga3_soft_reset(rga);
		return RGA_IRQ_ERROR;
	}

	return RGA_IRQ_IGNORE;
}

static void rga3_get_version(struct rockchip_rga *rga)
{
	u32 version = rga_read(rga, RGA3_VERSION_NUM);

	rga->version.major = FIELD_GET(RGA3_VERSION_NUM_MAJOR, version);
	rga->version.minor = FIELD_GET(RGA3_VERSION_NUM_MINOR, version);
}

static struct rga3_fmt rga3_formats[] = {
	{
		.fourcc = V4L2_PIX_FMT_RGB24,
		.hw_format = RGA3_COLOR_FMT_BGR888,
		.rbuv_swap = 1,
	},
	{
		.fourcc = V4L2_PIX_FMT_BGR24,
		.hw_format = RGA3_COLOR_FMT_BGR888,
	},
	{
		.fourcc = V4L2_PIX_FMT_ABGR32,
		.hw_format = RGA3_COLOR_FMT_BGRA8888,
	},
	{
		.fourcc = V4L2_PIX_FMT_RGBA32,
		.hw_format = RGA3_COLOR_FMT_BGRA8888,
		.rbuv_swap = 1,
	},
	{
		.fourcc = V4L2_PIX_FMT_XBGR32,
		.hw_format = RGA3_COLOR_FMT_BGRA8888,
	},
	{
		.fourcc = V4L2_PIX_FMT_RGBX32,
		.hw_format = RGA3_COLOR_FMT_BGRA8888,
		.rbuv_swap = 1,
	},
	{
		.fourcc = V4L2_PIX_FMT_RGB565,
		.hw_format = RGA3_COLOR_FMT_BGR565,
		.rbuv_swap = 1,
	},
	{
		.fourcc = V4L2_PIX_FMT_NV12M,
		.hw_format = RGA3_COLOR_FMT_YUV420,
		.semi_planar = 1,
	},
	{
		.fourcc = V4L2_PIX_FMT_NV12,
		.hw_format = RGA3_COLOR_FMT_YUV420,
		.semi_planar = 1,
	},
	{
		.fourcc = V4L2_PIX_FMT_NV21M,
		.hw_format = RGA3_COLOR_FMT_YUV420,
		.rbuv_swap = 1,
		.semi_planar = 1,
	},
	{
		.fourcc = V4L2_PIX_FMT_NV21,
		.hw_format = RGA3_COLOR_FMT_YUV420,
		.rbuv_swap = 1,
		.semi_planar = 1,
	},
	{
		.fourcc = V4L2_PIX_FMT_NV16M,
		.hw_format = RGA3_COLOR_FMT_YUV422,
		.semi_planar = 1,
	},
	{
		.fourcc = V4L2_PIX_FMT_NV16,
		.hw_format = RGA3_COLOR_FMT_YUV422,
		.semi_planar = 1,
	},
	{
		.fourcc = V4L2_PIX_FMT_NV61M,
		.hw_format = RGA3_COLOR_FMT_YUV422,
		.rbuv_swap = 1,
		.semi_planar = 1,
	},
	{
		.fourcc = V4L2_PIX_FMT_NV61,
		.hw_format = RGA3_COLOR_FMT_YUV422,
		.rbuv_swap = 1,
		.semi_planar = 1,
	},
	{
		.fourcc = V4L2_PIX_FMT_YUYV,
		.hw_format = RGA3_COLOR_FMT_YUV422,
		.yc_swap = 1,
	},
	{
		.fourcc = V4L2_PIX_FMT_YVYU,
		.hw_format = RGA3_COLOR_FMT_YUV422,
		.yc_swap = 1,
		.rbuv_swap = 1,
	},
	{
		.fourcc = V4L2_PIX_FMT_UYVY,
		.hw_format = RGA3_COLOR_FMT_YUV422,
	},
	{
		.fourcc = V4L2_PIX_FMT_VYUY,
		.hw_format = RGA3_COLOR_FMT_YUV422,
		.rbuv_swap = 1,
	},
	/* Input only formats last to keep rga3_enum_format simple */
	{
		.fourcc = V4L2_PIX_FMT_ARGB32,
		.hw_format = RGA3_COLOR_FMT_ABGR8888,
		.rbuv_swap = 1,
	},
	{
		.fourcc = V4L2_PIX_FMT_BGRA32,
		.hw_format = RGA3_COLOR_FMT_ABGR8888,
	},
	{
		.fourcc = V4L2_PIX_FMT_XRGB32,
		.hw_format = RGA3_COLOR_FMT_ABGR8888,
		.rbuv_swap = 1,
	},
	{
		.fourcc = V4L2_PIX_FMT_BGRX32,
		.hw_format = RGA3_COLOR_FMT_ABGR8888,
	},
};

static int rga3_enum_format(struct v4l2_fmtdesc *f)
{
	struct rga3_fmt *fmt;

	if (f->index >= ARRAY_SIZE(rga3_formats))
		return -EINVAL;

	fmt = &rga3_formats[f->index];
	if (V4L2_TYPE_IS_CAPTURE(f->type) && !rga3_can_capture(fmt))
		return -EINVAL;

	f->pixelformat = fmt->fourcc;
	return 0;
}

static void *rga3_try_format(u32 *fourcc, bool is_output)
{
	unsigned int i;

	if (!fourcc)
		return &rga3_formats[0];

	for (i = 0; i < ARRAY_SIZE(rga3_formats); i++) {
		if (!is_output && !rga3_can_capture(&rga3_formats[i]))
			continue;

		if (rga3_formats[i].fourcc == *fourcc)
			return &rga3_formats[i];
	}

	*fourcc = rga3_formats[0].fourcc;
	return &rga3_formats[0];
}

const struct rga_hw rga3_hw = {
	.card_type = "rga3",
	.has_internal_iommu = false,
	.cmdbuf_size = RGA3_CMDBUF_SIZE,
	.min_width = RGA3_MIN_WIDTH,
	.min_height = RGA3_MIN_HEIGHT,
	.max_width = RGA3_MAX_INPUT_WIDTH,
	.max_height = RGA3_MAX_INPUT_HEIGHT,

	.start = rga3_hw_start,
	.handle_irq = rga3_handle_irq,
	.get_version = rga3_get_version,
	.enum_format = rga3_enum_format,
	.try_format = rga3_try_format,
};
