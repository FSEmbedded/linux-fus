// SPDX-License-Identifier: GPL-2.0+
/*
 * NEOISP context registers/memory setting helpers
 *
 * Copyright 2023-2025 NXP
 */

#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/media/nxp/nxp_neoisp.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-dma-contig.h>

#include "neoisp.h"
#include "neoisp_ctx.h"

/**
 * This is the initial set of parameters setup by driver upon a streamon ioctl for INPUT0 node.
 * It could be updated later by the driver depending on input/output formats setup by userspace
 * and also if fine tuned parameters are provided by the camera stack.
 */
const struct neoisp_context_s def_context = {
	.hw = {
		.pipe_conf.common = {
			.img_conf =
				NEO_PIPE_CONF_IMG_CONF_CAM0_INALIGN0_SET(0) |
				NEO_PIPE_CONF_IMG_CONF_CAM0_LPALIGN0_SET(1) |
				NEO_PIPE_CONF_IMG_CONF_CAM0_INALIGN1_SET(0) |
				NEO_PIPE_CONF_IMG_CONF_CAM0_LPALIGN1_SET(1),
		},
		.hdr_decompress0 = {
			.ctrl =
				NEO_CTRL_CAM0_ENABLE_SET(1),
			.knee_ratio4 =
				NEO_HDR_DECOMPRESS0_KNEE_RATIO4_CAM0_RATIO4_SET(1 << 5),
		},
		.hdr_decompress1 = {
			.ctrl =
				NEO_CTRL_CAM0_ENABLE_SET(0),
			.knee_ratio4 =
				NEO_HDR_DECOMPRESS1_KNEE_RATIO4_CAM0_RATIO4_SET(1 << 5),
		},
		.obwb0 = {
			.ctrl =
				NEO_OB_WB0_CTRL_CAM0_OBPP_SET(3),
			.r_ctrl =
				NEO_OB_WB0_R_CTRL_CAM0_OFFSET_SET(0) |
				NEO_OB_WB0_R_CTRL_CAM0_GAIN_SET(1 << 8),
			.gr_ctrl =
				NEO_OB_WB0_GR_CTRL_CAM0_OFFSET_SET(0) |
				NEO_OB_WB0_GR_CTRL_CAM0_GAIN_SET(1 << 8),
			.gb_ctrl =
				NEO_OB_WB0_GB_CTRL_CAM0_OFFSET_SET(0) |
				NEO_OB_WB0_GB_CTRL_CAM0_GAIN_SET(1 << 8),
			.b_ctrl =
				NEO_OB_WB0_B_CTRL_CAM0_OFFSET_SET(0) |
				NEO_OB_WB0_B_CTRL_CAM0_GAIN_SET(1 << 8),
		},
		.obwb1 = {
			.ctrl =
				NEO_OB_WB1_CTRL_CAM0_OBPP_SET(2),
			.r_ctrl =
				NEO_OB_WB1_R_CTRL_CAM0_GAIN_SET(1 << 8)	|
				NEO_OB_WB1_R_CTRL_CAM0_OFFSET_SET(0),
			.gr_ctrl =
				NEO_OB_WB1_GR_CTRL_CAM0_GAIN_SET(1 << 8) |
				NEO_OB_WB1_GR_CTRL_CAM0_OFFSET_SET(0),
			.gb_ctrl =
				NEO_OB_WB1_GB_CTRL_CAM0_GAIN_SET(1 << 8) |
				NEO_OB_WB1_GB_CTRL_CAM0_OFFSET_SET(0),
			.b_ctrl =
				NEO_OB_WB1_B_CTRL_CAM0_GAIN_SET(1 << 8)	|
				NEO_OB_WB1_B_CTRL_CAM0_OFFSET_SET(0),
		},
		.obwb2 = {
			.ctrl =
				NEO_OB_WB2_CTRL_CAM0_OBPP_SET(3),
			.r_ctrl =
				NEO_OB_WB2_R_CTRL_CAM0_GAIN_SET(1 << 8) |
				NEO_OB_WB2_R_CTRL_CAM0_OFFSET_SET(0),
			.gr_ctrl =
				NEO_OB_WB2_GR_CTRL_CAM0_GAIN_SET(1 << 8) |
				NEO_OB_WB2_GR_CTRL_CAM0_OFFSET_SET(0),
			.gb_ctrl =
				NEO_OB_WB2_GB_CTRL_CAM0_GAIN_SET(1 << 8) |
				NEO_OB_WB2_GB_CTRL_CAM0_OFFSET_SET(0),
			.b_ctrl =
				NEO_OB_WB2_B_CTRL_CAM0_GAIN_SET(1 << 8) |
				NEO_OB_WB2_B_CTRL_CAM0_OFFSET_SET(0),
		},
		.hdr_merge = {
			.ctrl =
				NEO_HDR_MERGE_CTRL_CAM0_ENABLE_SET(0) |
				NEO_HDR_MERGE_CTRL_CAM0_GAIN1BPP_SET(3) |
				NEO_HDR_MERGE_CTRL_CAM0_GAIN0BPP_SET(3) |
				NEO_HDR_MERGE_CTRL_CAM0_OBPP_SET(3),
			.gain_scale =
				NEO_HDR_MERGE_GAIN_SCALE_CAM0_SCALE1_SET(8) |
				NEO_HDR_MERGE_GAIN_SCALE_CAM0_SCALE0_SET(1 << 12),
			.gain_shift =
				NEO_HDR_MERGE_GAIN_SHIFT_CAM0_SHIFT1_SET(12) |
				NEO_HDR_MERGE_GAIN_SHIFT_CAM0_SHIFT0_SET(4),
			.luma_th =
				NEO_HDR_MERGE_LUMA_TH_CAM0_TH0_SET(4),
			.luma_scale =
				NEO_HDR_MERGE_LUMA_SCALE_CAM0_SCALE_SET(1 << 8) |
				NEO_HDR_MERGE_LUMA_SCALE_CAM0_SHIFT_SET(8) |
				NEO_HDR_MERGE_LUMA_SCALE_CAM0_THSHIFT_SET(8),
			.downscale =
				NEO_HDR_MERGE_DOWNSCALE_CAM0_IMGSCALE0_SET(8),
			.upscale =
				NEO_HDR_MERGE_UPSCALE_CAM0_IMGSCALE1_SET(8),
		},
		.color_temp = {},
		.rgbir = {
			.ctrl =
				NEO_RGBIR_CTRL_CAM0_ENABLE_SET(0),
			.ccm0 =
				NEO_RGBIR_CCM0_CAM0_CCM_SET(1 << 8),
			.ccm1 =
				NEO_RGBIR_CCM1_CAM0_CCM_SET(1 << 8),
			.ccm2 =
				NEO_RGBIR_CCM2_CAM0_CCM_SET(1 << 8),
			.ccm0_th =
				NEO_RGBIR_CCM0_TH_CAM0_THRESHOLD_SET(0xff000),
			.ccm1_th =
				NEO_RGBIR_CCM1_TH_CAM0_THRESHOLD_SET(0xff000),
			.ccm2_th =
				NEO_RGBIR_CCM2_TH_CAM0_THRESHOLD_SET(0xff000),
		},
		.stat = {},
		.ir_compress = {
			.ctrl =
				NEO_IR_COMPRESS_CTRL_CAM0_ENABLE_SET(0) |
				NEO_IR_COMPRESS_CTRL_CAM0_OBPP_SET(0),
			.knee_point1 =
				NEO_IR_COMPRESS_KNEE_POINT1_CAM0_KNEEPOINT_SET((1 << 20) - 1),
			.knee_ratio01 =
				NEO_IR_COMPRESS_KNEE_RATIO01_CAM0_RATIO0_SET(8),
			.knee_ratio4 =
				NEO_IR_COMPRESS_KNEE_RATIO4_CAM0_RATIO4_SET(8),
		},
		.bnr = {
			.ctrl =
				NEO_BNR_CTRL_CAM0_ENABLE_SET(1) |
				NEO_BNR_CTRL_CAM0_NHOOD_SET(0) |
				NEO_BNR_CTRL_CAM0_DEBUG_SET(0) |
				NEO_BNR_CTRL_CAM0_OBPP_SET(3),
			.ypeak =
				NEO_BNR_YPEAK_CAM0_PEAK_OUTSEL_SET(0) |
				NEO_BNR_YPEAK_CAM0_PEAK_HIGH_SET(1 << 8) |
				NEO_BNR_YPEAK_CAM0_PEAK_SEL_SET(0) |
				NEO_BNR_YPEAK_CAM0_PEAK_LOW_SET(1 << 7),
			.yedge_th0 =
				NEO_BNR_YEDGE_TH0_CAM0_EDGE_TH0_SET(20),
			.yedge_scale =
				NEO_BNR_YEDGE_SCALE_CAM0_SHIFT_SET(10) |
				NEO_BNR_YEDGE_SCALE_CAM0_SCALE_SET(1 << 10),
			.yedges_th0 =
				NEO_BNR_YEDGES_TH0_CAM0_EDGE_TH0_SET(20),
			.yedges_scale =
				NEO_BNR_YEDGES_SCALE_CAM0_SHIFT_SET(10) |
				NEO_BNR_YEDGES_SCALE_CAM0_SCALE_SET(1 << 10),
			.yedgea_th0 =
				NEO_BNR_YEDGEA_TH0_CAM0_EDGE_TH0_SET(20),
			.yedgea_scale =
				NEO_BNR_YEDGEA_SCALE_CAM0_SHIFT_SET(10) |
				NEO_BNR_YEDGEA_SCALE_CAM0_SCALE_SET(10),
			.yluma_x_th0 =
				NEO_BNR_YLUMA_X_TH0_CAM0_TH_SET(20),
			.yluma_y_th =
				NEO_BNR_YLUMA_Y_TH_CAM0_LUMA_Y_TH1_SET(1 << 8) |
				NEO_BNR_YLUMA_Y_TH_CAM0_LUMA_Y_TH0_SET(10),
			.yluma_scale =
				NEO_BNR_YLUMA_SCALE_CAM0_SHIFT_SET(10) |
				NEO_BNR_YLUMA_SCALE_CAM0_SCALE_SET(1 << 10),
			.yalpha_gain =
				NEO_BNR_YALPHA_GAIN_CAM0_OFFSET_SET(0) |
				NEO_BNR_YALPHA_GAIN_CAM0_GAIN_SET(1 << 8),
			.cpeak =
				NEO_BNR_CPEAK_CAM0_PEAK_OUTSEL_SET(0) |
				NEO_BNR_CPEAK_CAM0_PEAK_HIGH_SET(1 << 8) |
				NEO_BNR_CPEAK_CAM0_PEAK_SEL_SET(0) |
				NEO_BNR_CPEAK_CAM0_PEAK_LOW_SET(1 << 7),
			.cedge_th0 =
				NEO_BNR_CEDGE_TH0_CAM0_EDGE_TH0_SET(20),
			.cedge_scale =
				NEO_BNR_CEDGE_SCALE_CAM0_SHIFT_SET(10) |
				NEO_BNR_CEDGE_SCALE_CAM0_SCALE_SET(1 << 10),
			.cedges_th0 =
				NEO_BNR_CEDGES_TH0_CAM0_EDGE_TH0_SET(20),
			.cedges_scale =
				NEO_BNR_CEDGES_SCALE_CAM0_SHIFT_SET(10) |
				NEO_BNR_CEDGES_SCALE_CAM0_SCALE_SET(1 << 10),
			.cedgea_th0 =
				NEO_BNR_CEDGEA_TH0_CAM0_EDGE_TH0_SET(20),
			.cedgea_scale =
				NEO_BNR_CEDGEA_SCALE_CAM0_SHIFT_SET(10) |
				NEO_BNR_CEDGEA_SCALE_CAM0_SCALE_SET(1 << 10),
			.cluma_x_th0 =
				NEO_BNR_CLUMA_X_TH0_CAM0_TH_SET(20),
			.cluma_y_th =
				NEO_BNR_CLUMA_Y_TH_CAM0_LUMA_Y_TH1_SET(1 << 8) |
				NEO_BNR_CLUMA_Y_TH_CAM0_LUMA_Y_TH0_SET(10),
			.cluma_scale =
				NEO_BNR_CLUMA_SCALE_CAM0_SHIFT_SET(10) |
				NEO_BNR_CLUMA_SCALE_CAM0_SCALE_SET(1 << 10),
			.calpha_gain =
				NEO_BNR_CALPHA_GAIN_CAM0_OFFSET_SET(0) |
				NEO_BNR_CALPHA_GAIN_CAM0_GAIN_SET(1 << 8),
			.stretch =
				NEO_BNR_STRETCH_CAM0_GAIN_SET(1 << 8),
		},
		.idbg1 = {
			.line_num_t =
				NEO_IDBG1_LINE_NUM_LINE_NUM_MASK,
		},
		.demosaic = {
			.ctrl =
				NEO_DEMOSAIC_CTRL_CAM0_FMT_SET(0),
			.activity_ctl =
				NEO_DEMOSAIC_ACTIVITY_CTL_CAM0_ACT_RATIO_SET(1 << 8) |
				NEO_DEMOSAIC_ACTIVITY_CTL_CAM0_ALPHA_SET(1 << 8),
			.dynamics_ctl0 =
				NEO_DEMOSAIC_DYNAMICS_CTL0_CAM0_STRENGTHC_SET(1 << 8) |
				NEO_DEMOSAIC_DYNAMICS_CTL0_CAM0_STRENGTHG_SET(1 << 8),
			.dynamics_ctl2 =
				NEO_DEMOSAIC_DYNAMICS_CTL2_CAM0_MAX_IMPACT_SET(1 << 7),
		},
		.rgb2yuv = {
			.gain_ctrl =
				NEO_RGB_TO_YUV_GAIN_CTRL_CAM0_BGAIN_SET(1 << 8) |
				NEO_RGB_TO_YUV_GAIN_CTRL_CAM0_RGAIN_SET(1 << 8),
			/* Constants defined by V4L2_YCBCR_ENC_601, full range and
			 * formatted in s8.8. This matrix will define the gcm.imat_rxcy
			 * as its inverse.
			 * https://www.kernel.org/doc/html/latest/userspace-api/media/v4l/colorspaces-details.html
			 *	{77, 150,  29},
			 *	{-43, -85, 128},
			 *	{128, -107, -21},
			 */
			.mat0 =
				NEO_RGB_TO_YUV_MAT0_CAM0_R0C0_SET(77) |
				NEO_RGB_TO_YUV_MAT0_CAM0_R0C1_SET(150),
			.mat1 =
				NEO_RGB_TO_YUV_MAT1_CAM0_R0C2_SET(29),
			.mat2 =
				NEO_RGB_TO_YUV_MAT2_CAM0_R1C0_SET(-43) |
				NEO_RGB_TO_YUV_MAT2_CAM0_R1C1_SET(-85),
			.mat3 =
				NEO_RGB_TO_YUV_MAT3_CAM0_R1C2_SET(128),
			.mat4 =
				NEO_RGB_TO_YUV_MAT4_CAM0_R2C0_SET(128) |
				NEO_RGB_TO_YUV_MAT4_CAM0_R2C1_SET(-107),
			.mat5 =
				NEO_RGB_TO_YUV_MAT5_CAM0_R2C2_SET(-21),
		},
		.drc = {
			.gbl_gain =
				NEO_DRC_GBL_GAIN_CAM0_GAIN_SET(1 << 8),
			.lcl_stretch =
				NEO_DRC_LCL_STRETCH_CAM0_STRETCH_SET(1 << 8),
			.alpha =
				NEO_DRC_ALPHA_CAM0_ALPHA_SET(1 << 8),
		},
		.cas = {
			.gain =
				NEO_CAS_GAIN_CAM0_SCALE_SET(1),
		},
		.packetizer = {
			.ch0_ctrl =
				NEO_PACKETIZER_CH0_CTRL_CAM0_OBPP_SET(6) |
				NEO_PACKETIZER_CH0_CTRL_CAM0_RSA_SET(4) |
				NEO_PACKETIZER_CH0_CTRL_CAM0_LSA_SET(0),
			.ch12_ctrl =
				NEO_PACKETIZER_CH12_CTRL_CAM0_OBPP_SET(6) |
				NEO_PACKETIZER_CH12_CTRL_CAM0_RSA_SET(4) |
				NEO_PACKETIZER_CH12_CTRL_CAM0_LSA_SET(0) |
				NEO_PACKETIZER_CH12_CTRL_CAM0_SUBSAMPLE_SET(0),
			.pack_ctrl =
				NEO_PACKETIZER_PACK_CTRL_CAM0_TYPE_SET(1) |
				NEO_PACKETIZER_PACK_CTRL_CAM0_ORDER0_SET(0) |
				NEO_PACKETIZER_PACK_CTRL_CAM0_ORDER1_SET(1) |
				NEO_PACKETIZER_PACK_CTRL_CAM0_ORDER2_SET(2) |
				NEO_PACKETIZER_PACK_CTRL_CAM0_A0S_SET(0),
		},
		.gcm = {
			.imat0 =
				NEO_GCM_IMAT0_CAM0_R0C0_SET(256) |
				NEO_GCM_IMAT0_CAM0_R0C1_SET(0),
			.imat1 =
				NEO_GCM_IMAT1_CAM0_R0C2_SET(359),
			.imat2 =
				NEO_GCM_IMAT2_CAM0_R1C0_SET(256) |
				NEO_GCM_IMAT2_CAM0_R1C1_SET(-88),
			.imat3 =
				NEO_GCM_IMAT3_CAM0_R1C2_SET(-183),
			.imat4 =
				NEO_GCM_IMAT4_CAM0_R2C0_SET(256) |
				NEO_GCM_IMAT4_CAM0_R2C1_SET(454),
			.imat5 =
				NEO_GCM_IMAT5_CAM0_R2C2_SET(0),
			.omat0 =
				NEO_GCM_OMAT0_CAM0_R0C0_SET(256),
			.omat2 =
				NEO_GCM_OMAT2_CAM0_R1C1_SET(256),
			.omat5 =
				NEO_GCM_OMAT5_CAM0_R2C2_SET(256),
			.mat_confg =
				NEO_GCM_MAT_CONFG_CAM0_SIGN_CONFG_SET(1),
		},
	},
	.gtm = {
		/* Fill default global tonemap lut with 1.0 value (256) */
		.drc_global_tonemap = { [0 ... NEO_DRC_GLOBAL_TONEMAP_SIZE - 1] = (1 << 8) },
	},
};

static const __u32 neoisp_ext_stats_blocks_v1[] = {
	NEOISP_STATS_BLK_RCTEMP,
	NEOISP_STATS_BLK_RDRC,
	NEOISP_STATS_BLK_RAF,
	NEOISP_STATS_BLK_RBNR,
	NEOISP_STATS_BLK_RNR,
	NEOISP_STATS_BLK_REE,
	NEOISP_STATS_BLK_RDF,
	NEOISP_STATS_BLK_MCTEMP,
	NEOISP_STATS_BLK_MRGBIR,
	NEOISP_STATS_BLK_MHIST,
	NEOISP_STATS_BLK_MDRC,
};

struct neoisp_block_handler_s {
	size_t size;
	void (*handler)(struct neoisp_dev_s *neoispd, union neoisp_params_block_u *blk);
};

static dma_addr_t get_addr(struct neoisp_buffer_s *buf, u32 num_plane)
{
	if (buf)
		return vb2_dma_contig_plane_dma_addr(&buf->vb.vb2_buf, num_plane);
	return 0;
}

static u32 *get_vaddr(struct neoisp_buffer_s *buf)
{
	if (buf)
		return vb2_plane_vaddr(&buf->vb.vb2_buf, 0);
	return NULL;
}

static inline void ctx_blk_write(uint32_t field, u32 *ptr, u32 *dest)
{
	u32 woffset, count;

	woffset = ISP_GET_OFF(field) / sizeof(u32);
	count = ISP_GET_SZ(field);
	if (IS_ERR_OR_NULL(ptr) || IS_ERR_OR_NULL(dest)) {
		pr_err("Invalid pointer for memcpy block !");
		return;
	}
	memcpy(dest + woffset, ptr, count);
}

static void neoisp_set_context_img_conf(struct neoisp_dev_s *neoispd,
		union neoisp_params_block_u *blk)
{
	struct neoisp_context_s *ctx = neoispd->queued_job.node_group->context;
	struct neoisp_pipe_conf_s *pc = &ctx->hw.pipe_conf.common;
	u32 tmp = pc->img_conf;

	tmp &= ~(NEO_PIPE_CONF_IMG_CONF_CAM0_INALIGN0 |
		 NEO_PIPE_CONF_IMG_CONF_CAM0_LPALIGN0 |
		 NEO_PIPE_CONF_IMG_CONF_CAM0_INALIGN1 |
		 NEO_PIPE_CONF_IMG_CONF_CAM0_LPALIGN1);
	tmp |=
		NEO_PIPE_CONF_IMG_CONF_CAM0_INALIGN0_SET(blk->pipe_conf.img_conf_inalign0) |
		NEO_PIPE_CONF_IMG_CONF_CAM0_LPALIGN0_SET(blk->pipe_conf.img_conf_lpalign0) |
		NEO_PIPE_CONF_IMG_CONF_CAM0_INALIGN1_SET(blk->pipe_conf.img_conf_inalign1) |
		NEO_PIPE_CONF_IMG_CONF_CAM0_LPALIGN1_SET(blk->pipe_conf.img_conf_lpalign1);
	pc->img_conf = tmp;
}

static void neoisp_set_context_head_color(struct neoisp_dev_s *neoispd,
		union neoisp_params_block_u *blk)
{
	struct neoisp_context_s *ctx = neoispd->queued_job.node_group->context;
	struct neoisp_hc_s *hc = &ctx->hw.hc;

	hc->ctrl =
		NEO_HC_CTRL_CAM0_HOFFSET_SET(blk->head_color.ctrl_hoffset) |
		NEO_HC_CTRL_CAM0_VOFFSET_SET(blk->head_color.ctrl_voffset);
}

static void neoisp_set_context_hdr_decompress0(struct neoisp_dev_s *neoispd,
		union neoisp_params_block_u *blk)
{
	struct neoisp_context_s *ctx = neoispd->queued_job.node_group->context;
	struct neoisp_hdr_decompress0_s *hd0 = &ctx->hw.hdr_decompress0;

	hd0->ctrl =
		NEO_CTRL_CAM0_ENABLE_SET(blk->hdr_decompress0.ctrl_enable);
	hd0->knee_point1 =
		NEO_HDR_DECOMPRESS0_KNEE_POINT1_CAM0_KNEEPOINT_SET(
			blk->hdr_decompress0.knee_point1);
	hd0->knee_point2 =
		NEO_HDR_DECOMPRESS0_KNEE_POINT2_CAM0_KNEEPOINT_SET(
			blk->hdr_decompress0.knee_point2);
	hd0->knee_point3 =
		NEO_HDR_DECOMPRESS0_KNEE_POINT3_CAM0_KNEEPOINT_SET(
			blk->hdr_decompress0.knee_point3);
	hd0->knee_point4 =
		NEO_HDR_DECOMPRESS0_KNEE_POINT4_CAM0_KNEEPOINT_SET(
			blk->hdr_decompress0.knee_point4);
	hd0->knee_offset0 =
		NEO_HDR_DECOMPRESS0_KNEE_OFFSET0_CAM0_OFFSET_SET(blk->hdr_decompress0.knee_offset0);
	hd0->knee_offset1 =
		NEO_HDR_DECOMPRESS0_KNEE_OFFSET1_CAM0_OFFSET_SET(blk->hdr_decompress0.knee_offset1);
	hd0->knee_offset2 =
		NEO_HDR_DECOMPRESS0_KNEE_OFFSET2_CAM0_OFFSET_SET(blk->hdr_decompress0.knee_offset2);
	hd0->knee_offset3 =
		NEO_HDR_DECOMPRESS0_KNEE_OFFSET3_CAM0_OFFSET_SET(blk->hdr_decompress0.knee_offset3);
	hd0->knee_offset4 =
		NEO_HDR_DECOMPRESS0_KNEE_OFFSET4_CAM0_OFFSET_SET(blk->hdr_decompress0.knee_offset4);
	hd0->knee_ratio01 =
		NEO_HDR_DECOMPRESS0_KNEE_RATIO01_CAM0_RATIO0_SET(blk->hdr_decompress0.knee_ratio0) |
		NEO_HDR_DECOMPRESS0_KNEE_RATIO01_CAM0_RATIO1_SET(blk->hdr_decompress0.knee_ratio1);
	hd0->knee_ratio23 =
		NEO_HDR_DECOMPRESS0_KNEE_RATIO23_CAM0_RATIO2_SET(blk->hdr_decompress0.knee_ratio2) |
		NEO_HDR_DECOMPRESS0_KNEE_RATIO23_CAM0_RATIO3_SET(blk->hdr_decompress0.knee_ratio3);
	hd0->knee_ratio4 =
		NEO_HDR_DECOMPRESS0_KNEE_RATIO4_CAM0_RATIO4_SET(blk->hdr_decompress0.knee_ratio4);
	hd0->knee_npoint0 =
		NEO_HDR_DECOMPRESS0_KNEE_NPOINT0_CAM0_KNEEPOINT_SET(
			blk->hdr_decompress0.knee_npoint0);
	hd0->knee_npoint1 =
		NEO_HDR_DECOMPRESS0_KNEE_NPOINT0_CAM0_KNEEPOINT_SET(
			blk->hdr_decompress0.knee_npoint1);
	hd0->knee_npoint2 =
		NEO_HDR_DECOMPRESS0_KNEE_NPOINT0_CAM0_KNEEPOINT_SET(
			blk->hdr_decompress0.knee_npoint2);
	hd0->knee_npoint3 =
		NEO_HDR_DECOMPRESS0_KNEE_NPOINT0_CAM0_KNEEPOINT_SET(
			blk->hdr_decompress0.knee_npoint3);
	hd0->knee_npoint4 =
		NEO_HDR_DECOMPRESS0_KNEE_NPOINT0_CAM0_KNEEPOINT_SET(
			blk->hdr_decompress0.knee_npoint4);
}

static void neoisp_set_context_hdr_decompress1(struct neoisp_dev_s *neoispd,
		union neoisp_params_block_u *blk)
{
	struct neoisp_context_s *ctx = neoispd->queued_job.node_group->context;
	struct neoisp_hdr_decompress1_s *hd1 = &ctx->hw.hdr_decompress1;

	hd1->ctrl = NEO_CTRL_CAM0_ENABLE_SET(
		blk->hdr_decompress1.ctrl_enable);
	hd1->knee_point1 =
		NEO_HDR_DECOMPRESS1_KNEE_POINT1_CAM0_KNEEPOINT_SET(
			blk->hdr_decompress1.knee_point1);
	hd1->knee_point2 =
		NEO_HDR_DECOMPRESS1_KNEE_POINT2_CAM0_KNEEPOINT_SET(
			blk->hdr_decompress1.knee_point2);
	hd1->knee_point3 =
		NEO_HDR_DECOMPRESS1_KNEE_POINT3_CAM0_KNEEPOINT_SET(
			blk->hdr_decompress1.knee_point3);
	hd1->knee_point4 =
		NEO_HDR_DECOMPRESS1_KNEE_POINT4_CAM0_KNEEPOINT_SET(
			blk->hdr_decompress1.knee_point4);
	hd1->knee_offset0 =
		NEO_HDR_DECOMPRESS1_KNEE_OFFSET0_CAM0_OFFSET_SET(blk->hdr_decompress1.knee_offset0);
	hd1->knee_offset1 =
		NEO_HDR_DECOMPRESS1_KNEE_OFFSET1_CAM0_OFFSET_SET(blk->hdr_decompress1.knee_offset1);
	hd1->knee_offset2 =
		NEO_HDR_DECOMPRESS1_KNEE_OFFSET2_CAM0_OFFSET_SET(blk->hdr_decompress1.knee_offset2);
	hd1->knee_offset3 =
		NEO_HDR_DECOMPRESS1_KNEE_OFFSET3_CAM0_OFFSET_SET(blk->hdr_decompress1.knee_offset3);
	hd1->knee_offset4 =
		NEO_HDR_DECOMPRESS1_KNEE_OFFSET4_CAM0_OFFSET_SET(blk->hdr_decompress1.knee_offset4);
	hd1->knee_ratio01 =
		NEO_HDR_DECOMPRESS1_KNEE_RATIO01_CAM0_RATIO0_SET(blk->hdr_decompress1.knee_ratio0) |
		NEO_HDR_DECOMPRESS1_KNEE_RATIO01_CAM0_RATIO1_SET(blk->hdr_decompress1.knee_ratio1);
	hd1->knee_ratio23 =
		NEO_HDR_DECOMPRESS1_KNEE_RATIO23_CAM0_RATIO2_SET(blk->hdr_decompress1.knee_ratio2) |
		NEO_HDR_DECOMPRESS1_KNEE_RATIO23_CAM0_RATIO3_SET(blk->hdr_decompress1.knee_ratio3);
	hd1->knee_ratio4 =
		NEO_HDR_DECOMPRESS1_KNEE_RATIO4_CAM0_RATIO4_SET(blk->hdr_decompress1.knee_ratio4);
	hd1->knee_npoint0 =
		NEO_HDR_DECOMPRESS1_KNEE_NPOINT0_CAM0_KNEEPOINT_SET(
			blk->hdr_decompress1.knee_npoint0);
	hd1->knee_npoint1 =
		NEO_HDR_DECOMPRESS1_KNEE_NPOINT0_CAM0_KNEEPOINT_SET(
			blk->hdr_decompress1.knee_npoint1);
	hd1->knee_npoint2 =
		NEO_HDR_DECOMPRESS1_KNEE_NPOINT0_CAM0_KNEEPOINT_SET(
			blk->hdr_decompress1.knee_npoint2);
	hd1->knee_npoint3 =
		NEO_HDR_DECOMPRESS1_KNEE_NPOINT0_CAM0_KNEEPOINT_SET(
			blk->hdr_decompress1.knee_npoint3);
	hd1->knee_npoint4 =
		NEO_HDR_DECOMPRESS1_KNEE_NPOINT0_CAM0_KNEEPOINT_SET(
			blk->hdr_decompress1.knee_npoint4);
}

static void __neoisp_set_context_obwb(struct neoisp_dev_s *neoispd,
		union neoisp_params_block_u *blk, u8 id)
{
	struct neoisp_context_s *ctx = neoispd->queued_job.node_group->context;
	struct neoisp_obwb_s *obwb;

	switch (id) {
	case 0:
		obwb = &ctx->hw.obwb0;
		break;
	case 1:
		obwb = &ctx->hw.obwb1;
		break;
	case 2:
		obwb = &ctx->hw.obwb2;
		break;
	default:
		pr_err("Unexpected OBWB instance %u\n", id);
		return;
	}

	obwb->ctrl =
		NEO_OB_WB0_CTRL_CAM0_OBPP_SET(blk->obwb.ctrl_obpp);
	obwb->r_ctrl =
		NEO_OB_WB0_R_CTRL_CAM0_OFFSET_SET(blk->obwb.r_ctrl_offset) |
		NEO_OB_WB0_R_CTRL_CAM0_GAIN_SET(blk->obwb.r_ctrl_gain);
	obwb->gr_ctrl =
		NEO_OB_WB0_GR_CTRL_CAM0_OFFSET_SET(blk->obwb.gr_ctrl_offset) |
		NEO_OB_WB0_GR_CTRL_CAM0_GAIN_SET(blk->obwb.gr_ctrl_gain);
	obwb->gb_ctrl =
		NEO_OB_WB0_GB_CTRL_CAM0_OFFSET_SET(blk->obwb.gb_ctrl_offset) |
		NEO_OB_WB0_GB_CTRL_CAM0_GAIN_SET(blk->obwb.gb_ctrl_gain);
	obwb->b_ctrl =
		NEO_OB_WB0_B_CTRL_CAM0_OFFSET_SET(blk->obwb.b_ctrl_offset) |
		NEO_OB_WB0_B_CTRL_CAM0_GAIN_SET(blk->obwb.b_ctrl_gain);
}

static void neoisp_set_context_obwb0(struct neoisp_dev_s *neoispd,
		union neoisp_params_block_u *blk)
{
	__neoisp_set_context_obwb(neoispd, blk, 0);
}

static void neoisp_set_context_obwb1(struct neoisp_dev_s *neoispd,
		union neoisp_params_block_u *blk)
{
	__neoisp_set_context_obwb(neoispd, blk, 1);
}

static void neoisp_set_context_obwb2(struct neoisp_dev_s *neoispd,
		union neoisp_params_block_u *blk)
{
	__neoisp_set_context_obwb(neoispd, blk, 2);
}

static void neoisp_set_context_hdr_merge(struct neoisp_dev_s *neoispd,
		union neoisp_params_block_u *blk)
{
	struct neoisp_context_s *ctx = neoispd->queued_job.node_group->context;
	struct neoisp_hdr_merge_s *hmg = &ctx->hw.hdr_merge;

	hmg->ctrl =
		NEO_HDR_MERGE_CTRL_CAM0_OBPP_SET(blk->hdr_merge.ctrl_obpp) |
		NEO_HDR_MERGE_CTRL_CAM0_MOTION_FIX_EN_SET(blk->hdr_merge.ctrl_motion_fix_en) |
		NEO_HDR_MERGE_CTRL_CAM0_BLEND_3X3_SET(blk->hdr_merge.ctrl_blend_3x3) |
		NEO_HDR_MERGE_CTRL_CAM0_GAIN0BPP_SET(blk->hdr_merge.ctrl_gain0bpp) |
		NEO_HDR_MERGE_CTRL_CAM0_GAIN1BPP_SET(blk->hdr_merge.ctrl_gain1bpp) |
		NEO_HDR_MERGE_CTRL_CAM0_ENABLE_SET(blk->hdr_merge.ctrl_enable);
	hmg->gain_offset =
		NEO_HDR_MERGE_GAIN_OFFSET_CAM0_OFFSET0_SET(blk->hdr_merge.gain_offset_offset0) |
		NEO_HDR_MERGE_GAIN_OFFSET_CAM0_OFFSET1_SET(blk->hdr_merge.gain_offset_offset1);
	hmg->gain_scale =
		NEO_HDR_MERGE_GAIN_SCALE_CAM0_SCALE0_SET(blk->hdr_merge.gain_scale_scale0) |
		NEO_HDR_MERGE_GAIN_SCALE_CAM0_SCALE1_SET(blk->hdr_merge.gain_scale_scale1);
	hmg->gain_shift =
		NEO_HDR_MERGE_GAIN_SHIFT_CAM0_SHIFT0_SET(blk->hdr_merge.gain_shift_shift0) |
		NEO_HDR_MERGE_GAIN_SHIFT_CAM0_SHIFT1_SET(blk->hdr_merge.gain_shift_shift1);
	hmg->luma_th =
		NEO_HDR_MERGE_LUMA_TH_CAM0_TH0_SET(blk->hdr_merge.luma_th_th0);
	hmg->luma_scale =
		NEO_HDR_MERGE_LUMA_SCALE_CAM0_SCALE_SET(blk->hdr_merge.luma_scale_scale) |
		NEO_HDR_MERGE_LUMA_SCALE_CAM0_SHIFT_SET(blk->hdr_merge.luma_scale_shift) |
		NEO_HDR_MERGE_LUMA_SCALE_CAM0_THSHIFT_SET(blk->hdr_merge.luma_scale_thshift);
	hmg->downscale =
		NEO_HDR_MERGE_DOWNSCALE_CAM0_IMGSCALE0_SET(blk->hdr_merge.downscale_imgscale0) |
		NEO_HDR_MERGE_DOWNSCALE_CAM0_IMGSCALE1_SET(blk->hdr_merge.downscale_imgscale1);
	hmg->upscale =
		NEO_HDR_MERGE_UPSCALE_CAM0_IMGSCALE0_SET(blk->hdr_merge.upscale_imgscale0) |
		NEO_HDR_MERGE_UPSCALE_CAM0_IMGSCALE1_SET(blk->hdr_merge.upscale_imgscale1);
	hmg->post_scale =
		NEO_HDR_MERGE_POST_SCALE_CAM0_SCALE_SET(blk->hdr_merge.post_scale_scale);
}

static void neoisp_set_context_rgbir(struct neoisp_dev_s *neoispd,
		union neoisp_params_block_u *blk)
{
	struct neoisp_context_s *ctx = neoispd->queued_job.node_group->context;
	struct neoisp_rgbir_s *rgbir = &ctx->hw.rgbir;
	struct neoisp_stat_hist_cfg_s *hist;

	rgbir->ctrl =
		NEO_RGBIR_CTRL_CAM0_ENABLE_SET(blk->rgbir.ctrl_enable);
	rgbir->ccm0 =
		NEO_RGBIR_CCM0_CAM0_CCM_SET(blk->rgbir.ccm0_ccm);
	rgbir->ccm1 =
		NEO_RGBIR_CCM1_CAM0_CCM_SET(blk->rgbir.ccm1_ccm);
	rgbir->ccm2 =
		NEO_RGBIR_CCM2_CAM0_CCM_SET(blk->rgbir.ccm2_ccm);
	rgbir->ccm0_th =
		NEO_RGBIR_CCM0_TH_CAM0_THRESHOLD_SET(blk->rgbir.ccm0_th_threshold);
	rgbir->ccm1_th =
		NEO_RGBIR_CCM1_TH_CAM0_THRESHOLD_SET(blk->rgbir.ccm1_th_threshold);
	rgbir->ccm2_th =
		NEO_RGBIR_CCM2_TH_CAM0_THRESHOLD_SET(blk->rgbir.ccm2_th_threshold);
	rgbir->roi0_pos =
		NEO_RGBIR_ROI0_POS_CAM0_XPOS_SET(blk->rgbir.roi[0].xpos) |
		NEO_RGBIR_ROI0_POS_CAM0_YPOS_SET(blk->rgbir.roi[0].ypos);
	rgbir->roi0_size =
		NEO_RGBIR_ROI0_SIZE_CAM0_WIDTH_SET(blk->rgbir.roi[0].width) |
		NEO_RGBIR_ROI0_SIZE_CAM0_HEIGHT_SET(blk->rgbir.roi[0].height);
	rgbir->roi1_pos =
		NEO_RGBIR_ROI1_POS_CAM0_XPOS_SET(blk->rgbir.roi[1].xpos) |
		NEO_RGBIR_ROI1_POS_CAM0_YPOS_SET(blk->rgbir.roi[1].ypos);
	rgbir->roi1_size =
		NEO_RGBIR_ROI1_SIZE_CAM0_WIDTH_SET(blk->rgbir.roi[1].width) |
		NEO_RGBIR_ROI1_SIZE_CAM0_HEIGHT_SET(blk->rgbir.roi[1].height);
	hist = &blk->rgbir.hists[0];
	rgbir->hist0_ctrl =
		NEO_RGBIR_HIST0_CTRL_CAM0_LIN_INPUT1_LOG_SET(hist->hist_ctrl_lin_input1_log) |
		NEO_RGBIR_HIST0_CTRL_CAM0_DIR_INPUT1_DIF_SET(hist->hist_ctrl_dir_input1_dif) |
		NEO_RGBIR_HIST0_CTRL_CAM0_PATTERN_SET(hist->hist_ctrl_pattern) |
		NEO_RGBIR_HIST0_CTRL_CAM0_CHANNEL_SET(hist->hist_ctrl_channel) |
		NEO_RGBIR_HIST0_CTRL_CAM0_OFFSET_SET(hist->hist_ctrl_offset);
	rgbir->hist0_scale =
		NEO_RGBIR_HIST0_SCALE_CAM0_SCALE_SET(hist->hist_scale_scale);
	hist = &blk->rgbir.hists[1];
	rgbir->hist1_ctrl =
		NEO_RGBIR_HIST1_CTRL_CAM0_LIN_INPUT1_LOG_SET(hist->hist_ctrl_lin_input1_log) |
		NEO_RGBIR_HIST1_CTRL_CAM0_DIR_INPUT1_DIF_SET(hist->hist_ctrl_dir_input1_dif) |
		NEO_RGBIR_HIST1_CTRL_CAM0_PATTERN_SET(hist->hist_ctrl_pattern) |
		NEO_RGBIR_HIST1_CTRL_CAM0_CHANNEL_SET(hist->hist_ctrl_channel) |
		NEO_RGBIR_HIST1_CTRL_CAM0_OFFSET_SET(hist->hist_ctrl_offset);
	rgbir->hist1_scale =
		NEO_RGBIR_HIST1_SCALE_CAM0_SCALE_SET(hist->hist_scale_scale);
}

static void neoisp_set_context_stat_hists(struct neoisp_dev_s *neoispd,
		union neoisp_params_block_u *blk)
{
	struct neoisp_context_s *ctx = neoispd->queued_job.node_group->context;
	struct neoisp_stat_s *stat = &ctx->hw.stat;
	struct neoisp_stat_hist_cfg_s *hist;

	stat->roi0_pos =
		NEO_STAT_ROI0_POS_CAM0_XPOS_SET(blk->stat.roi0.xpos) |
		NEO_STAT_ROI0_POS_CAM0_YPOS_SET(blk->stat.roi0.ypos);
	stat->roi0_size =
		NEO_STAT_ROI0_SIZE_CAM0_WIDTH_SET(blk->stat.roi0.width) |
		NEO_STAT_ROI0_SIZE_CAM0_HEIGHT_SET(blk->stat.roi0.height);
	stat->roi1_pos =
		NEO_STAT_ROI1_POS_CAM0_XPOS_SET(blk->stat.roi1.xpos) |
		NEO_STAT_ROI1_POS_CAM0_YPOS_SET(blk->stat.roi1.ypos);
	stat->roi1_size =
		NEO_STAT_ROI1_SIZE_CAM0_WIDTH_SET(blk->stat.roi1.width) |
		NEO_STAT_ROI1_SIZE_CAM0_HEIGHT_SET(blk->stat.roi1.height);
	hist = &blk->stat.hists[0];
	stat->hist0_ctrl =
		NEO_STAT_HIST0_CTRL_CAM0_LIN_INPUT1_LOG_SET(hist->hist_ctrl_lin_input1_log) |
		NEO_STAT_HIST0_CTRL_CAM0_DIR_INPUT1_DIF_SET(hist->hist_ctrl_dir_input1_dif) |
		NEO_STAT_HIST0_CTRL_CAM0_PATTERN_SET(hist->hist_ctrl_pattern) |
		NEO_STAT_HIST0_CTRL_CAM0_CHANNEL_SET(hist->hist_ctrl_channel) |
		NEO_STAT_HIST0_CTRL_CAM0_OFFSET_SET(hist->hist_ctrl_offset);
	stat->hist0_scale =
		NEO_STAT_HIST0_SCALE_CAM0_SCALE_SET(hist->hist_scale_scale);
	hist = &blk->stat.hists[1];
	stat->hist1_ctrl =
		NEO_STAT_HIST1_CTRL_CAM0_LIN_INPUT1_LOG_SET(hist->hist_ctrl_lin_input1_log) |
		NEO_STAT_HIST1_CTRL_CAM0_DIR_INPUT1_DIF_SET(hist->hist_ctrl_dir_input1_dif) |
		NEO_STAT_HIST1_CTRL_CAM0_PATTERN_SET(hist->hist_ctrl_pattern) |
		NEO_STAT_HIST1_CTRL_CAM0_CHANNEL_SET(hist->hist_ctrl_channel) |
		NEO_STAT_HIST1_CTRL_CAM0_OFFSET_SET(hist->hist_ctrl_offset);
	stat->hist1_scale =
		NEO_STAT_HIST1_SCALE_CAM0_SCALE_SET(hist->hist_scale_scale);
	hist = &blk->stat.hists[2];
	stat->hist2_ctrl =
		NEO_STAT_HIST2_CTRL_CAM0_LIN_INPUT1_LOG_SET(hist->hist_ctrl_lin_input1_log) |
		NEO_STAT_HIST2_CTRL_CAM0_DIR_INPUT1_DIF_SET(hist->hist_ctrl_dir_input1_dif) |
		NEO_STAT_HIST2_CTRL_CAM0_PATTERN_SET(hist->hist_ctrl_pattern) |
		NEO_STAT_HIST2_CTRL_CAM0_CHANNEL_SET(hist->hist_ctrl_channel) |
		NEO_STAT_HIST2_CTRL_CAM0_OFFSET_SET(hist->hist_ctrl_offset);
	stat->hist2_scale =
		NEO_STAT_HIST2_SCALE_CAM0_SCALE_SET(hist->hist_scale_scale);
	hist = &blk->stat.hists[3];
	stat->hist3_ctrl =
		NEO_STAT_HIST2_CTRL_CAM0_LIN_INPUT1_LOG_SET(hist->hist_ctrl_lin_input1_log) |
		NEO_STAT_HIST2_CTRL_CAM0_DIR_INPUT1_DIF_SET(hist->hist_ctrl_dir_input1_dif) |
		NEO_STAT_HIST2_CTRL_CAM0_PATTERN_SET(hist->hist_ctrl_pattern) |
		NEO_STAT_HIST2_CTRL_CAM0_CHANNEL_SET(hist->hist_ctrl_channel) |
		NEO_STAT_HIST2_CTRL_CAM0_OFFSET_SET(hist->hist_ctrl_offset);
	stat->hist3_scale =
		NEO_STAT_HIST3_SCALE_CAM0_SCALE_SET(hist->hist_scale_scale);
}

static void neoisp_set_context_ir_compress(struct neoisp_dev_s *neoispd,
		union neoisp_params_block_u *blk)
{
	struct neoisp_context_s *ctx = neoispd->queued_job.node_group->context;
	struct neoisp_ir_compress_s *ircomp = &ctx->hw.ir_compress;

	ircomp->ctrl =
		NEO_IR_COMPRESS_CTRL_CAM0_OBPP_SET(blk->ir_compress.ctrl_obpp) |
		NEO_IR_COMPRESS_CTRL_CAM0_ENABLE_SET(blk->ir_compress.ctrl_enable);
	ircomp->knee_point1 =
		NEO_IR_COMPRESS_KNEE_POINT1_CAM0_KNEEPOINT_SET(
			blk->ir_compress.knee_point1_kneepoint);
	ircomp->knee_point2 =
		NEO_IR_COMPRESS_KNEE_POINT2_CAM0_KNEEPOINT_SET(
			blk->ir_compress.knee_point2_kneepoint);
	ircomp->knee_point3 =
		NEO_IR_COMPRESS_KNEE_POINT3_CAM0_KNEEPOINT_SET(
			blk->ir_compress.knee_point3_kneepoint);
	ircomp->knee_point4 =
		NEO_IR_COMPRESS_KNEE_POINT4_CAM0_KNEEPOINT_SET(
			blk->ir_compress.knee_point4_kneepoint);
	ircomp->knee_offset0 =
		NEO_IR_COMPRESS_KNEE_OFFSET0_CAM0_OFFSET_SET(
			blk->ir_compress.knee_offset0_offset);
	ircomp->knee_offset1 =
		NEO_IR_COMPRESS_KNEE_OFFSET1_CAM0_OFFSET_SET(
			blk->ir_compress.knee_offset1_offset);
	ircomp->knee_offset2 =
		NEO_IR_COMPRESS_KNEE_OFFSET2_CAM0_OFFSET_SET(
			blk->ir_compress.knee_offset2_offset);
	ircomp->knee_offset3 =
		NEO_IR_COMPRESS_KNEE_OFFSET3_CAM0_OFFSET_SET(
			blk->ir_compress.knee_offset3_offset);
	ircomp->knee_offset4 =
		NEO_IR_COMPRESS_KNEE_OFFSET4_CAM0_OFFSET_SET(
			blk->ir_compress.knee_offset4_offset);
	ircomp->knee_ratio01 =
		NEO_IR_COMPRESS_KNEE_RATIO01_CAM0_RATIO0_SET(
			blk->ir_compress.knee_ratio01_ratio0) |
		NEO_IR_COMPRESS_KNEE_RATIO01_CAM0_RATIO1_SET(
			blk->ir_compress.knee_ratio01_ratio1);
	ircomp->knee_ratio23 =
		NEO_IR_COMPRESS_KNEE_RATIO23_CAM0_RATIO2_SET(
			blk->ir_compress.knee_ratio23_ratio2) |
		NEO_IR_COMPRESS_KNEE_RATIO23_CAM0_RATIO3_SET(
			blk->ir_compress.knee_ratio23_ratio3);
	ircomp->knee_ratio4 =
		NEO_IR_COMPRESS_KNEE_RATIO4_CAM0_RATIO4_SET(
			blk->ir_compress.knee_ratio4_ratio4);
	ircomp->knee_npoint0 =
		NEO_IR_COMPRESS_KNEE_NPOINT0_CAM0_KNEEPOINT_SET(
			blk->ir_compress.knee_npoint0_kneepoint);
	ircomp->knee_npoint1 =
		NEO_IR_COMPRESS_KNEE_NPOINT1_CAM0_KNEEPOINT_SET(
			blk->ir_compress.knee_npoint1_kneepoint);
	ircomp->knee_npoint2 =
		NEO_IR_COMPRESS_KNEE_NPOINT2_CAM0_KNEEPOINT_SET(
			blk->ir_compress.knee_npoint2_kneepoint);
	ircomp->knee_npoint3 =
		NEO_IR_COMPRESS_KNEE_NPOINT3_CAM0_KNEEPOINT_SET(
			blk->ir_compress.knee_npoint3_kneepoint);
	ircomp->knee_npoint4 =
		NEO_IR_COMPRESS_KNEE_NPOINT4_CAM0_KNEEPOINT_SET(
			blk->ir_compress.knee_npoint4_kneepoint);
}

static void neoisp_set_context_color_temp(struct neoisp_dev_s *neoispd,
		union neoisp_params_block_u *blk)
{
	struct neoisp_context_s *ctx = neoispd->queued_job.node_group->context;
	struct neoisp_color_temp_s *ctemp = &ctx->hw.color_temp;
	struct neoisp_ctemp_roi_desc_s *croi;

	ctemp->ctrl =
		NEO_COLOR_TEMP_CTRL_CAM0_IBPP_SET(blk->ctemp.ctrl_ibpp) |
		NEO_COLOR_TEMP_CTRL_CAM0_CSCON_SET(blk->ctemp.ctrl_cscon) |
		NEO_COLOR_TEMP_CTRL_CAM0_ENABLE_SET(blk->ctemp.ctrl_enable);
	ctemp->roi_pos =
		NEO_COLOR_TEMP_ROI_POS_CAM0_XPOS_SET(blk->ctemp.roi.xpos) |
		NEO_COLOR_TEMP_ROI_POS_CAM0_YPOS_SET(blk->ctemp.roi.ypos);
	ctemp->roi_size =
		NEO_COLOR_TEMP_ROI_SIZE_CAM0_WIDTH_SET(blk->ctemp.roi.width) |
		NEO_COLOR_TEMP_ROI_SIZE_CAM0_HEIGHT_SET(blk->ctemp.roi.height);
	ctemp->redgain =
		NEO_COLOR_TEMP_REDGAIN_CAM0_MIN_SET(blk->ctemp.redgain_min) |
		NEO_COLOR_TEMP_REDGAIN_CAM0_MAX_SET(blk->ctemp.redgain_max);
	ctemp->bluegain =
		NEO_COLOR_TEMP_BLUEGAIN_CAM0_MIN_SET(blk->ctemp.bluegain_min) |
		NEO_COLOR_TEMP_BLUEGAIN_CAM0_MAX_SET(blk->ctemp.bluegain_max);
	ctemp->point1 =
		NEO_COLOR_TEMP_POINT1_CAM0_BLUE_SET(blk->ctemp.point1_blue) |
		NEO_COLOR_TEMP_POINT1_CAM0_RED_SET(blk->ctemp.point1_red);
	ctemp->point2 =
		NEO_COLOR_TEMP_POINT2_CAM0_BLUE_SET(blk->ctemp.point2_blue) |
		NEO_COLOR_TEMP_POINT2_CAM0_RED_SET(blk->ctemp.point2_red);
	ctemp->hoffset =
		NEO_COLOR_TEMP_HOFFSET_CAM0_RIGHT_SET(blk->ctemp.hoffset_right) |
		NEO_COLOR_TEMP_HOFFSET_CAM0_LEFT_SET(blk->ctemp.hoffset_left);
	ctemp->voffset =
		NEO_COLOR_TEMP_VOFFSET_CAM0_UP_SET(blk->ctemp.voffset_up) |
		NEO_COLOR_TEMP_VOFFSET_CAM0_DOWN_SET(blk->ctemp.voffset_down);
	ctemp->point1_slope =
		NEO_COLOR_TEMP_POINT1_SLOPE_CAM0_SLOPE_L_SET(blk->ctemp.point1_slope_slope_l) |
		NEO_COLOR_TEMP_POINT1_SLOPE_CAM0_SLOPE_R_SET(blk->ctemp.point1_slope_slope_r);
	ctemp->point2_slope =
		NEO_COLOR_TEMP_POINT2_SLOPE_CAM0_SLOPE_L_SET(blk->ctemp.point2_slope_slope_l) |
		NEO_COLOR_TEMP_POINT2_SLOPE_CAM0_SLOPE_R_SET(blk->ctemp.point2_slope_slope_r);
	ctemp->luma_th =
		NEO_COLOR_TEMP_LUMA_TH_CAM0_THL_SET(blk->ctemp.luma_th_thl) |
		NEO_COLOR_TEMP_LUMA_TH_CAM0_THH_SET(blk->ctemp.luma_th_thh);
	ctemp->csc_mat0 =
		NEO_COLOR_TEMP_CSC_MAT0_CAM0_R0C0_SET(blk->ctemp.csc_matrix[0][0]) |
		NEO_COLOR_TEMP_CSC_MAT0_CAM0_R0C1_SET(blk->ctemp.csc_matrix[0][1]);
	ctemp->csc_mat1 =
		NEO_COLOR_TEMP_CSC_MAT1_CAM0_R0C2_SET(blk->ctemp.csc_matrix[0][2]) |
		NEO_COLOR_TEMP_CSC_MAT1_CAM0_R1C0_SET(blk->ctemp.csc_matrix[1][0]);
	ctemp->csc_mat2 =
		NEO_COLOR_TEMP_CSC_MAT2_CAM0_R1C1_SET(blk->ctemp.csc_matrix[1][1]) |
		NEO_COLOR_TEMP_CSC_MAT2_CAM0_R1C2_SET(blk->ctemp.csc_matrix[1][2]);
	ctemp->csc_mat3 =
		NEO_COLOR_TEMP_CSC_MAT3_CAM0_R2C0_SET(blk->ctemp.csc_matrix[2][0]) |
		NEO_COLOR_TEMP_CSC_MAT3_CAM0_R2C1_SET(blk->ctemp.csc_matrix[2][1]);
	ctemp->csc_mat4 =
		NEO_COLOR_TEMP_CSC_MAT4_CAM0_R2C2_SET(blk->ctemp.csc_matrix[2][2]);
	ctemp->r_gr_offset =
		NEO_COLOR_TEMP_R_GR_OFFSET_CAM0_OFFSET0_SET(blk->ctemp.offsets[0]) |
		NEO_COLOR_TEMP_R_GR_OFFSET_CAM0_OFFSET1_SET(blk->ctemp.offsets[1]);
	ctemp->gb_b_offset =
		NEO_COLOR_TEMP_GB_B_OFFSET_CAM0_OFFSET0_SET(blk->ctemp.offsets[2]) |
		NEO_COLOR_TEMP_GB_B_OFFSET_CAM0_OFFSET1_SET(blk->ctemp.offsets[3]);
	ctemp->stat_blk_size0 =
		NEO_COLOR_TEMP_STAT_BLK_SIZE0_XSIZE_SET(blk->ctemp.stat_blk_size0_xsize) |
		NEO_COLOR_TEMP_STAT_BLK_SIZE0_YSIZE_SET(blk->ctemp.stat_blk_size0_ysize);
	croi = &blk->ctemp.color_rois[0];
	ctemp->croi0_pos =
		NEO_COLOR_TEMP_CROI0_POS_CAM0_ROVERG_LOW_SET(croi->pos_roverg_low) |
		NEO_COLOR_TEMP_CROI0_POS_CAM0_ROVERG_HIGH_SET(croi->pos_roverg_high) |
		NEO_COLOR_TEMP_CROI0_POS_CAM0_BOVERG_LOW_SET(croi->pos_boverg_low) |
		NEO_COLOR_TEMP_CROI0_POS_CAM0_BOVERG_HIGH_SET(croi->pos_boverg_high);
	croi = &blk->ctemp.color_rois[1];
	ctemp->croi1_pos =
		NEO_COLOR_TEMP_CROI1_POS_CAM0_ROVERG_LOW_SET(croi->pos_roverg_low) |
		NEO_COLOR_TEMP_CROI1_POS_CAM0_ROVERG_HIGH_SET(croi->pos_roverg_high) |
		NEO_COLOR_TEMP_CROI1_POS_CAM0_BOVERG_LOW_SET(croi->pos_boverg_low) |
		NEO_COLOR_TEMP_CROI1_POS_CAM0_BOVERG_HIGH_SET(croi->pos_boverg_high);
	croi = &blk->ctemp.color_rois[2];
	ctemp->croi2_pos =
		NEO_COLOR_TEMP_CROI2_POS_CAM0_ROVERG_LOW_SET(croi->pos_roverg_low) |
		NEO_COLOR_TEMP_CROI2_POS_CAM0_ROVERG_HIGH_SET(croi->pos_roverg_high) |
		NEO_COLOR_TEMP_CROI2_POS_CAM0_BOVERG_LOW_SET(croi->pos_boverg_low) |
		NEO_COLOR_TEMP_CROI2_POS_CAM0_BOVERG_HIGH_SET(croi->pos_boverg_high);
	croi = &blk->ctemp.color_rois[3];
	ctemp->croi3_pos =
		NEO_COLOR_TEMP_CROI3_POS_CAM0_ROVERG_LOW_SET(croi->pos_roverg_low) |
		NEO_COLOR_TEMP_CROI3_POS_CAM0_ROVERG_HIGH_SET(croi->pos_roverg_high) |
		NEO_COLOR_TEMP_CROI3_POS_CAM0_BOVERG_LOW_SET(croi->pos_boverg_low) |
		NEO_COLOR_TEMP_CROI3_POS_CAM0_BOVERG_HIGH_SET(croi->pos_boverg_high);
	croi = &blk->ctemp.color_rois[4];
	ctemp->croi4_pos =
		NEO_COLOR_TEMP_CROI4_POS_CAM0_ROVERG_LOW_SET(croi->pos_roverg_low) |
		NEO_COLOR_TEMP_CROI4_POS_CAM0_ROVERG_HIGH_SET(croi->pos_roverg_high) |
		NEO_COLOR_TEMP_CROI4_POS_CAM0_BOVERG_LOW_SET(croi->pos_boverg_low) |
		NEO_COLOR_TEMP_CROI4_POS_CAM0_BOVERG_HIGH_SET(croi->pos_boverg_high);
	croi = &blk->ctemp.color_rois[5];
	ctemp->croi5_pos =
		NEO_COLOR_TEMP_CROI5_POS_CAM0_ROVERG_LOW_SET(croi->pos_roverg_low) |
		NEO_COLOR_TEMP_CROI5_POS_CAM0_ROVERG_HIGH_SET(croi->pos_roverg_high) |
		NEO_COLOR_TEMP_CROI5_POS_CAM0_BOVERG_LOW_SET(croi->pos_boverg_low) |
		NEO_COLOR_TEMP_CROI5_POS_CAM0_BOVERG_HIGH_SET(croi->pos_boverg_high);
	croi = &blk->ctemp.color_rois[6];
	ctemp->croi6_pos =
		NEO_COLOR_TEMP_CROI6_POS_CAM0_ROVERG_LOW_SET(croi->pos_roverg_low) |
		NEO_COLOR_TEMP_CROI6_POS_CAM0_ROVERG_HIGH_SET(croi->pos_roverg_high) |
		NEO_COLOR_TEMP_CROI6_POS_CAM0_BOVERG_LOW_SET(croi->pos_boverg_low) |
		NEO_COLOR_TEMP_CROI6_POS_CAM0_BOVERG_HIGH_SET(croi->pos_boverg_high);
	croi = &blk->ctemp.color_rois[7];
	ctemp->croi7_pos =
		NEO_COLOR_TEMP_CROI7_POS_CAM0_ROVERG_LOW_SET(croi->pos_roverg_low) |
		NEO_COLOR_TEMP_CROI7_POS_CAM0_ROVERG_HIGH_SET(croi->pos_roverg_high) |
		NEO_COLOR_TEMP_CROI7_POS_CAM0_BOVERG_LOW_SET(croi->pos_boverg_low) |
		NEO_COLOR_TEMP_CROI7_POS_CAM0_BOVERG_HIGH_SET(croi->pos_boverg_high);
	croi = &blk->ctemp.color_rois[8];
	ctemp->croi8_pos =
		NEO_COLOR_TEMP_CROI8_POS_CAM0_ROVERG_LOW_SET(croi->pos_roverg_low) |
		NEO_COLOR_TEMP_CROI8_POS_CAM0_ROVERG_HIGH_SET(croi->pos_roverg_high) |
		NEO_COLOR_TEMP_CROI8_POS_CAM0_BOVERG_LOW_SET(croi->pos_boverg_low) |
		NEO_COLOR_TEMP_CROI8_POS_CAM0_BOVERG_HIGH_SET(croi->pos_boverg_high);
	croi = &blk->ctemp.color_rois[9];
	ctemp->croi9_pos =
		NEO_COLOR_TEMP_CROI9_POS_CAM0_ROVERG_LOW_SET(croi->pos_roverg_low) |
		NEO_COLOR_TEMP_CROI9_POS_CAM0_ROVERG_HIGH_SET(croi->pos_roverg_high) |
		NEO_COLOR_TEMP_CROI9_POS_CAM0_BOVERG_LOW_SET(croi->pos_boverg_low) |
		NEO_COLOR_TEMP_CROI9_POS_CAM0_BOVERG_HIGH_SET(croi->pos_boverg_high);
	ctemp->gr_avg_in =
		NEO_COLOR_TEMP_GR_AVG_IN_CAM0_GR_AGV_SET(blk->ctemp.gr_avg_in_gr_agv);
	ctemp->gb_avg_in =
		NEO_COLOR_TEMP_GB_AVG_IN_CAM0_GB_AGV_SET(blk->ctemp.gb_avg_in_gb_agv);
}

static void neoisp_set_context_bnr(struct neoisp_dev_s *neoispd,
		union neoisp_params_block_u *blk)
{
	struct neoisp_context_s *ctx = neoispd->queued_job.node_group->context;
	struct neoisp_bnr_s *bnr = &ctx->hw.bnr;

	bnr->ctrl =
		NEO_BNR_CTRL_CAM0_OBPP_SET(blk->bnr.ctrl_obpp) |
		NEO_BNR_CTRL_CAM0_DEBUG_SET(blk->bnr.ctrl_debug) |
		NEO_BNR_CTRL_CAM0_NHOOD_SET(blk->bnr.ctrl_nhood) |
		NEO_BNR_CTRL_CAM0_ENABLE_SET(blk->bnr.ctrl_enable);
	bnr->ypeak =
		NEO_BNR_YPEAK_CAM0_PEAK_LOW_SET(blk->bnr.ypeak_peak_low) |
		NEO_BNR_YPEAK_CAM0_PEAK_SEL_SET(blk->bnr.ypeak_peak_sel) |
		NEO_BNR_YPEAK_CAM0_PEAK_HIGH_SET(blk->bnr.ypeak_peak_high) |
		NEO_BNR_YPEAK_CAM0_PEAK_OUTSEL_SET(blk->bnr.ypeak_peak_outsel);
	bnr->yedge_th0 =
		NEO_BNR_YEDGE_TH0_CAM0_EDGE_TH0_SET(blk->bnr.yedge_th0_edge_th0);
	bnr->yedge_scale =
		NEO_BNR_YEDGE_SCALE_CAM0_SCALE_SET(blk->bnr.yedge_scale_scale) |
		NEO_BNR_YEDGE_SCALE_CAM0_SHIFT_SET(blk->bnr.yedge_scale_shift);
	bnr->yedges_th0 =
		NEO_BNR_YEDGES_TH0_CAM0_EDGE_TH0_SET(blk->bnr.yedges_th0_edge_th0);
	bnr->yedges_scale =
		NEO_BNR_YEDGES_SCALE_CAM0_SCALE_SET(blk->bnr.yedges_scale_scale) |
		NEO_BNR_YEDGES_SCALE_CAM0_SHIFT_SET(blk->bnr.yedges_scale_shift);
	bnr->yedgea_th0 =
		NEO_BNR_YEDGEA_TH0_CAM0_EDGE_TH0_SET(blk->bnr.yedgea_th0_edge_th0);
	bnr->yedgea_scale =
		NEO_BNR_YEDGEA_SCALE_CAM0_SCALE_SET(blk->bnr.yedgea_scale_scale) |
		NEO_BNR_YEDGEA_SCALE_CAM0_SHIFT_SET(blk->bnr.yedgea_scale_shift);
	bnr->yluma_x_th0 =
		NEO_BNR_YLUMA_X_TH0_CAM0_TH_SET(blk->bnr.yluma_x_th0_th);
	bnr->yluma_y_th =
		NEO_BNR_YLUMA_Y_TH_CAM0_LUMA_Y_TH0_SET(blk->bnr.yluma_y_th_luma_y_th0) |
		NEO_BNR_YLUMA_Y_TH_CAM0_LUMA_Y_TH1_SET(blk->bnr.yluma_y_th_luma_y_th1);
	bnr->yluma_scale =
		NEO_BNR_YLUMA_SCALE_CAM0_SCALE_SET(blk->bnr.yluma_scale_scale) |
		NEO_BNR_YLUMA_SCALE_CAM0_SHIFT_SET(blk->bnr.yluma_scale_shift);
	bnr->yalpha_gain =
		NEO_BNR_YALPHA_GAIN_CAM0_GAIN_SET(blk->bnr.yalpha_gain_gain) |
		NEO_BNR_YALPHA_GAIN_CAM0_OFFSET_SET(blk->bnr.yalpha_gain_offset);
	bnr->cpeak =
		NEO_BNR_CPEAK_CAM0_PEAK_LOW_SET(blk->bnr.cpeak_peak_low) |
		NEO_BNR_CPEAK_CAM0_PEAK_SEL_SET(blk->bnr.cpeak_peak_sel) |
		NEO_BNR_CPEAK_CAM0_PEAK_HIGH_SET(blk->bnr.cpeak_peak_high) |
		NEO_BNR_CPEAK_CAM0_PEAK_OUTSEL_SET(blk->bnr.cpeak_peak_outsel);
	bnr->cedge_th0 =
		NEO_BNR_CEDGE_TH0_CAM0_EDGE_TH0_SET(blk->bnr.cedge_th0_edge_th0);
	bnr->cedge_scale =
		NEO_BNR_CEDGE_SCALE_CAM0_SCALE_SET(blk->bnr.cedge_scale_scale) |
		NEO_BNR_CEDGE_SCALE_CAM0_SHIFT_SET(blk->bnr.cedge_scale_shift);
	bnr->cedges_th0 =
		NEO_BNR_CEDGES_TH0_CAM0_EDGE_TH0_SET(blk->bnr.cedges_th0_edge_th0);
	bnr->cedges_scale =
		NEO_BNR_CEDGES_SCALE_CAM0_SCALE_SET(blk->bnr.cedges_scale_scale) |
		NEO_BNR_CEDGES_SCALE_CAM0_SHIFT_SET(blk->bnr.cedges_scale_shift);
	bnr->cedgea_th0 =
		NEO_BNR_CEDGEA_TH0_CAM0_EDGE_TH0_SET(blk->bnr.cedgea_th0_edge_th0);
	bnr->cedgea_scale =
		NEO_BNR_CEDGEA_SCALE_CAM0_SCALE_SET(blk->bnr.cedgea_scale_scale) |
		NEO_BNR_CEDGEA_SCALE_CAM0_SHIFT_SET(blk->bnr.cedgea_scale_shift);
	bnr->cluma_x_th0 =
		NEO_BNR_CLUMA_X_TH0_CAM0_TH_SET(blk->bnr.cluma_x_th0_th);
	bnr->cluma_y_th =
		NEO_BNR_CLUMA_Y_TH_CAM0_LUMA_Y_TH0_SET(blk->bnr.cluma_y_th_luma_y_th0) |
		NEO_BNR_CLUMA_Y_TH_CAM0_LUMA_Y_TH1_SET(blk->bnr.cluma_y_th_luma_y_th1);
	bnr->cluma_scale =
		NEO_BNR_CLUMA_SCALE_CAM0_SCALE_SET(blk->bnr.cluma_scale_scale) |
		NEO_BNR_CLUMA_SCALE_CAM0_SHIFT_SET(blk->bnr.cluma_scale_shift);
	bnr->calpha_gain =
		NEO_BNR_CALPHA_GAIN_CAM0_GAIN_SET(blk->bnr.calpha_gain_gain) |
		NEO_BNR_CALPHA_GAIN_CAM0_OFFSET_SET(blk->bnr.calpha_gain_offset);
	bnr->stretch =
		NEO_BNR_STRETCH_CAM0_GAIN_SET(blk->bnr.stretch_gain);
}

static void neoisp_set_context_vignetting(struct neoisp_dev_s *neoispd,
		union neoisp_params_block_u *blk)
{
	struct neoisp_context_s *ctx = neoispd->queued_job.node_group->context;
	struct neoisp_vignetting_s *vignetting_ctl = &ctx->hw.vignetting;

	vignetting_ctl->ctrl =
		NEO_VIGNETTING_CTRL_CAM0_ENABLE_SET(blk->vignetting_ctrl.ctrl_enable);
	vignetting_ctl->blk_conf =
		NEO_VIGNETTING_BLK_CONF_CAM0_COLS_SET(blk->vignetting_ctrl.blk_conf_cols) |
		NEO_VIGNETTING_BLK_CONF_CAM0_ROWS_SET(blk->vignetting_ctrl.blk_conf_rows);
	vignetting_ctl->blk_size =
		NEO_VIGNETTING_BLK_SIZE_CAM0_XSIZE_SET(blk->vignetting_ctrl.blk_size_xsize) |
		NEO_VIGNETTING_BLK_SIZE_CAM0_YSIZE_SET(blk->vignetting_ctrl.blk_size_ysize);
	vignetting_ctl->blk_stepy =
		NEO_VIGNETTING_BLK_STEPY_CAM0_STEP_SET(blk->vignetting_ctrl.blk_stepy_step);
	vignetting_ctl->blk_stepx =
		NEO_VIGNETTING_BLK_STEPX_CAM0_STEP_SET(blk->vignetting_ctrl.blk_stepx_step);
}

static void neoisp_set_context_demosaic(struct neoisp_dev_s *neoispd,
		union neoisp_params_block_u *blk)
{
	struct neoisp_context_s *ctx = neoispd->queued_job.node_group->context;
	struct neoisp_demosaic_s *demosaic = &ctx->hw.demosaic;

	demosaic->ctrl =
		NEO_DEMOSAIC_CTRL_CAM0_FMT_SET(blk->demosaic.ctrl_fmt);
	demosaic->activity_ctl =
		NEO_DEMOSAIC_ACTIVITY_CTL_CAM0_ALPHA_SET(blk->demosaic.activity_ctl_alpha) |
		NEO_DEMOSAIC_ACTIVITY_CTL_CAM0_ACT_RATIO_SET(blk->demosaic.activity_ctl_act_ratio);
	demosaic->dynamics_ctl0 =
		NEO_DEMOSAIC_DYNAMICS_CTL0_CAM0_STRENGTHG_SET(
			blk->demosaic.dynamics_ctl0_strengthg) |
		NEO_DEMOSAIC_DYNAMICS_CTL0_CAM0_STRENGTHC_SET(
			blk->demosaic.dynamics_ctl0_strengthc);
	demosaic->dynamics_ctl2 =
		NEO_DEMOSAIC_DYNAMICS_CTL2_CAM0_MAX_IMPACT_SET(
			blk->demosaic.dynamics_ctl2_max_impact);
}

static void neoisp_set_context_rgb_to_yuv(struct neoisp_dev_s *neoispd,
		union neoisp_params_block_u *blk)
{
	struct neoisp_context_s *ctx = neoispd->queued_job.node_group->context;
	struct neoisp_rgb2yuv_s *rgb2yuv = &ctx->hw.rgb2yuv;

	rgb2yuv->gain_ctrl =
		NEO_RGB_TO_YUV_GAIN_CTRL_CAM0_RGAIN_SET(blk->rgb2yuv.gain_ctrl_rgain) |
		NEO_RGB_TO_YUV_GAIN_CTRL_CAM0_BGAIN_SET(blk->rgb2yuv.gain_ctrl_bgain);
	rgb2yuv->mat0 =
		NEO_RGB_TO_YUV_MAT0_CAM0_R0C0_SET(blk->rgb2yuv.mat_rxcy[0][0]) |
		NEO_RGB_TO_YUV_MAT0_CAM0_R0C1_SET(blk->rgb2yuv.mat_rxcy[0][1]);
	rgb2yuv->mat1 =
		NEO_RGB_TO_YUV_MAT1_CAM0_R0C2_SET(blk->rgb2yuv.mat_rxcy[0][2]);
	rgb2yuv->mat2 =
		NEO_RGB_TO_YUV_MAT2_CAM0_R1C0_SET(blk->rgb2yuv.mat_rxcy[1][0]) |
		NEO_RGB_TO_YUV_MAT2_CAM0_R1C1_SET(blk->rgb2yuv.mat_rxcy[1][1]);
	rgb2yuv->mat3 =
		NEO_RGB_TO_YUV_MAT3_CAM0_R1C2_SET(blk->rgb2yuv.mat_rxcy[1][2]);
	rgb2yuv->mat4 =
		NEO_RGB_TO_YUV_MAT4_CAM0_R2C0_SET(blk->rgb2yuv.mat_rxcy[2][0]) |
		NEO_RGB_TO_YUV_MAT4_CAM0_R2C1_SET(blk->rgb2yuv.mat_rxcy[2][1]);
	rgb2yuv->mat5 =
		NEO_RGB_TO_YUV_MAT5_CAM0_R2C2_SET(blk->rgb2yuv.mat_rxcy[2][2]);
	rgb2yuv->offset0 =
		NEO_RGB_TO_YUV_OFFSET0_CAM0_OFFSET_SET(blk->rgb2yuv.csc_offsets[0]);
	rgb2yuv->offset1 =
		NEO_RGB_TO_YUV_OFFSET1_CAM0_OFFSET_SET(blk->rgb2yuv.csc_offsets[1]);
	rgb2yuv->offset2 =
		NEO_RGB_TO_YUV_OFFSET2_CAM0_OFFSET_SET(blk->rgb2yuv.csc_offsets[2]);
}

static void neoisp_set_context_drc(struct neoisp_dev_s *neoispd,
		union neoisp_params_block_u *blk)
{
	struct neoisp_context_s *ctx = neoispd->queued_job.node_group->context;
	struct neoisp_drc_s *drc = &ctx->hw.drc;

	drc->roi0_pos =
		NEO_DRC_ROI0_POS_CAM0_XPOS_SET(blk->drc.roi0.xpos) |
		NEO_DRC_ROI0_POS_CAM0_YPOS_SET(blk->drc.roi0.ypos);
	drc->roi0_size =
		NEO_DRC_ROI0_SIZE_CAM0_WIDTH_SET(blk->drc.roi0.width) |
		NEO_DRC_ROI0_SIZE_CAM0_HEIGHT_SET(blk->drc.roi0.height);
	drc->roi1_pos =
		NEO_DRC_ROI1_POS_CAM0_XPOS_SET(blk->drc.roi1.xpos) |
		NEO_DRC_ROI1_POS_CAM0_YPOS_SET(blk->drc.roi1.ypos);
	drc->roi1_size =
		NEO_DRC_ROI1_SIZE_CAM0_WIDTH_SET(blk->drc.roi1.width) |
		NEO_DRC_ROI1_SIZE_CAM0_HEIGHT_SET(blk->drc.roi1.height);
	drc->groi_sum_shift =
		NEO_DRC_GROI_SUM_SHIFT_CAM0_SHIFT0_SET(blk->drc.groi_sum_shift_shift0) |
		NEO_DRC_GROI_SUM_SHIFT_CAM0_SHIFT1_SET(blk->drc.groi_sum_shift_shift1);
	drc->gbl_gain =
		NEO_DRC_GBL_GAIN_CAM0_GAIN_SET(blk->drc.gbl_gain_gain);
	drc->lcl_blk_size =
		NEO_DRC_LCL_BLK_SIZE_CAM0_XSIZE_SET(blk->drc.lcl_blk_size_xsize) |
		NEO_DRC_LCL_BLK_SIZE_CAM0_YSIZE_SET(blk->drc.lcl_blk_size_ysize);
	drc->lcl_stretch =
		NEO_DRC_LCL_STRETCH_CAM0_STRETCH_SET(blk->drc.lcl_stretch_stretch) |
		NEO_DRC_LCL_STRETCH_CAM0_OFFSET_SET(blk->drc.lcl_stretch_offset);
	drc->lcl_blk_stepy =
		NEO_DRC_LCL_BLK_STEPY_CAM0_STEP_SET(blk->drc.lcl_blk_stepy_step);
	drc->lcl_blk_stepx =
		NEO_DRC_LCL_BLK_STEPX_CAM0_STEP_SET(blk->drc.lcl_blk_stepx_step);
	drc->lcl_sum_shift =
		NEO_DRC_LCL_SUM_SHIFT_CAM0_SHIFT_SET(blk->drc.lcl_sum_shift_shift);
	drc->alpha =
		NEO_DRC_ALPHA_CAM0_ALPHA_SET(blk->drc.alpha_alpha);
}

static void neoisp_set_context_nr(struct neoisp_dev_s *neoispd,
		union neoisp_params_block_u *blk)
{
	struct neoisp_context_s *ctx = neoispd->queued_job.node_group->context;
	struct neoisp_nr_s *nrc = &ctx->hw.nr;

	nrc->ctrl =
		NEO_NR_CTRL_CAM0_DEBUG_SET(blk->nrc.ctrl_debug) |
		NEO_NR_CTRL_CAM0_ENABLE_SET(blk->nrc.ctrl_enable);
	nrc->blend_scale =
		NEO_NR_BLEND_SCALE_CAM0_SCALE_SET(blk->nrc.blend_scale_scale) |
		NEO_NR_BLEND_SCALE_CAM0_SHIFT_SET(blk->nrc.blend_scale_shift) |
		NEO_NR_BLEND_SCALE_CAM0_GAIN_SET(blk->nrc.blend_scale_gain);
	nrc->blend_th0 =
		NEO_NR_BLEND_TH0_CAM0_TH_SET(blk->nrc.blend_th0_th);
}

static void neoisp_set_context_df(struct neoisp_dev_s *neoispd,
		union neoisp_params_block_u *blk)
{
	struct neoisp_context_s *ctx = neoispd->queued_job.node_group->context;
	struct neoisp_df_s *df = &ctx->hw.df;

	df->ctrl =
		NEO_DF_CTRL_CAM0_DEBUG_SET(blk->dfc.ctrl_debug) |
		NEO_DF_CTRL_CAM0_ENABLE_SET(blk->dfc.ctrl_enable);
	df->th_scale =
		NEO_DF_TH_SCALE_CAM0_SCALE_SET(blk->dfc.th_scale_scale);
	df->blend_shift =
		NEO_DF_BLEND_SHIFT_CAM0_SHIFT_SET(blk->dfc.blend_shift_shift);
	df->blend_th0 =
		NEO_DF_BLEND_TH0_CAM0_TH_SET(blk->dfc.blend_th0_th);
}

static void neoisp_set_context_ee(struct neoisp_dev_s *neoispd,
		union neoisp_params_block_u *blk)
{
	struct neoisp_context_s *ctx = neoispd->queued_job.node_group->context;
	struct neoisp_ee_s *ee = &ctx->hw.ee;

	ee->ctrl =
		NEO_EE_CTRL_CAM0_DEBUG_SET(blk->eec.ctrl_debug) |
		NEO_EE_CTRL_CAM0_ENABLE_SET(blk->eec.ctrl_enable);
	ee->coring =
		NEO_EE_CORING_CAM0_CORING_SET(blk->eec.coring_coring);
	ee->clip =
		NEO_EE_CLIP_CAM0_CLIP_SET(blk->eec.clip_clip);
	ee->maskgain =
		NEO_EE_MASKGAIN_CAM0_GAIN_SET(blk->eec.maskgain_gain);
}

static void neoisp_set_context_convmed(struct neoisp_dev_s *neoispd,
		union neoisp_params_block_u *blk)
{
	struct neoisp_context_s *ctx = neoispd->queued_job.node_group->context;
	struct neoisp_convmed_s *convmed = &ctx->hw.convmed;

	convmed->ctrl =
		NEO_CCONVMED_CTRL_CAM0_FLT_SET(blk->convmed.ctrl_flt);
}

static void neoisp_set_context_cas(struct neoisp_dev_s *neoispd,
		union neoisp_params_block_u *blk)
{
	struct neoisp_context_s *ctx = neoispd->queued_job.node_group->context;
	struct neoisp_cas_s *cas = &ctx->hw.cas;

	cas->gain =
		NEO_CAS_GAIN_CAM0_SCALE_SET(blk->cas.gain_scale) |
		NEO_CAS_GAIN_CAM0_SHIFT_SET(blk->cas.gain_shift);
	cas->corr =
		NEO_CAS_CORR_CAM0_CORR_SET(blk->cas.corr_corr);
	cas->offset =
		NEO_CAS_OFFSET_CAM0_OFFSET_SET(blk->cas.offset_offset);
}

static void neoisp_set_context_gcm(struct neoisp_dev_s *neoispd,
		union neoisp_params_block_u *blk)
{
	struct neoisp_context_s *ctx = neoispd->queued_job.node_group->context;
	struct neoisp_gcm_s *gcm = &ctx->hw.gcm;

	gcm->imat0 =
		NEO_GCM_IMAT0_CAM0_R0C0_SET(blk->gcm.imat_rxcy[0][0]) |
		NEO_GCM_IMAT0_CAM0_R0C1_SET(blk->gcm.imat_rxcy[0][1]);
	gcm->imat1 =
		NEO_GCM_IMAT1_CAM0_R0C2_SET(blk->gcm.imat_rxcy[0][2]);
	gcm->imat2 =
		NEO_GCM_IMAT2_CAM0_R1C0_SET(blk->gcm.imat_rxcy[1][0]) |
		NEO_GCM_IMAT2_CAM0_R1C1_SET(blk->gcm.imat_rxcy[1][1]);
	gcm->imat3 =
		NEO_GCM_IMAT3_CAM0_R1C2_SET(blk->gcm.imat_rxcy[1][2]);
	gcm->imat4 =
		NEO_GCM_IMAT4_CAM0_R2C0_SET(blk->gcm.imat_rxcy[2][0]) |
		NEO_GCM_IMAT4_CAM0_R2C1_SET(blk->gcm.imat_rxcy[2][1]);
	gcm->imat5 =
		NEO_GCM_IMAT5_CAM0_R2C2_SET(blk->gcm.imat_rxcy[2][2]);
	gcm->ioffset0 =
		NEO_GCM_IOFFSET0_CAM0_OFFSET0_SET(blk->gcm.ioffsets[0]);
	gcm->ioffset1 =
		NEO_GCM_IOFFSET1_CAM0_OFFSET1_SET(blk->gcm.ioffsets[1]);
	gcm->ioffset2 =
		NEO_GCM_IOFFSET2_CAM0_OFFSET2_SET(blk->gcm.ioffsets[2]);
	gcm->omat0 =
		NEO_GCM_OMAT0_CAM0_R0C0_SET(blk->gcm.omat_rxcy[0][0]) |
		NEO_GCM_OMAT0_CAM0_R0C1_SET(blk->gcm.omat_rxcy[0][1]);
	gcm->omat1 =
		NEO_GCM_OMAT1_CAM0_R0C2_SET(blk->gcm.omat_rxcy[0][2]);
	gcm->omat2 =
		NEO_GCM_OMAT2_CAM0_R1C0_SET(blk->gcm.omat_rxcy[1][0]) |
		NEO_GCM_OMAT2_CAM0_R1C1_SET(blk->gcm.omat_rxcy[1][1]);
	gcm->omat3 =
		NEO_GCM_OMAT3_CAM0_R1C2_SET(blk->gcm.omat_rxcy[1][2]);
	gcm->omat4 =
		NEO_GCM_OMAT4_CAM0_R2C0_SET(blk->gcm.omat_rxcy[2][0]) |
		NEO_GCM_OMAT4_CAM0_R2C1_SET(blk->gcm.omat_rxcy[2][1]);
	gcm->omat5 =
		NEO_GCM_OMAT5_CAM0_R2C2_SET(blk->gcm.omat_rxcy[2][2]);
	gcm->ooffset0 =
		NEO_GCM_OOFFSET0_CAM0_OFFSET0_SET(blk->gcm.ooffsets[0]);
	gcm->ooffset1 =
		NEO_GCM_OOFFSET1_CAM0_OFFSET1_SET(blk->gcm.ooffsets[1]);
	gcm->ooffset2 =
		NEO_GCM_OOFFSET2_CAM0_OFFSET2_SET(blk->gcm.ooffsets[2]);
	gcm->gamma0 =
		NEO_GCM_GAMMA0_CAM0_GAMMA0_SET(blk->gcm.gamma0_gamma0) |
		NEO_GCM_GAMMA0_CAM0_OFFSET0_SET(blk->gcm.gamma0_offset0);
	gcm->gamma1 =
		NEO_GCM_GAMMA1_CAM0_GAMMA1_SET(blk->gcm.gamma1_gamma1) |
		NEO_GCM_GAMMA1_CAM0_OFFSET1_SET(blk->gcm.gamma1_offset1);
	gcm->gamma2 =
		NEO_GCM_GAMMA2_CAM0_GAMMA2_SET(blk->gcm.gamma2_gamma2) |
		NEO_GCM_GAMMA2_CAM0_OFFSET2_SET(blk->gcm.gamma2_offset2);
	gcm->blklvl0_ctrl =
		NEO_GCM_BLKLVL0_CTRL_CAM0_OFFSET0_SET(blk->gcm.blklvl0_ctrl_offset0) |
		NEO_GCM_BLKLVL0_CTRL_CAM0_GAIN0_SET(blk->gcm.blklvl0_ctrl_gain0);
	gcm->blklvl1_ctrl =
		NEO_GCM_BLKLVL1_CTRL_CAM0_OFFSET1_SET(blk->gcm.blklvl1_ctrl_offset1) |
		NEO_GCM_BLKLVL1_CTRL_CAM0_GAIN1_SET(blk->gcm.blklvl1_ctrl_gain1);
	gcm->blklvl2_ctrl =
		NEO_GCM_BLKLVL2_CTRL_CAM0_OFFSET2_SET(blk->gcm.blklvl2_ctrl_offset2) |
		NEO_GCM_BLKLVL2_CTRL_CAM0_GAIN2_SET(blk->gcm.blklvl2_ctrl_gain2);
	gcm->lowth_ctrl01 =
		NEO_GCM_LOWTH_CTRL01_CAM0_THRESHOLD0_SET(blk->gcm.lowth_ctrl01_threshold0) |
		NEO_GCM_LOWTH_CTRL01_CAM0_THRESHOLD1_SET(blk->gcm.lowth_ctrl01_threshold1);
	gcm->lowth_ctrl2 =
		NEO_GCM_LOWTH_CTRL2_CAM0_THRESHOLD2_SET(blk->gcm.lowth_ctrl2_threshold2);
	gcm->mat_confg =
		NEO_GCM_MAT_CONFG_CAM0_SIGN_CONFG_SET(blk->gcm.mat_confg_sign_confg);
}

static void neoisp_set_context_autofocus(struct neoisp_dev_s *neoispd,
		union neoisp_params_block_u *blk)
{
	struct neoisp_context_s *ctx = neoispd->queued_job.node_group->context;
	struct neoisp_autofocus_s *afc = &ctx->hw.autofocus;

	afc->roi0_pos =
		NEO_AUTOFOCUS_ROI0_POS_CAM0_XPOS_SET(blk->afc.af_roi[0].xpos) |
		NEO_AUTOFOCUS_ROI0_POS_CAM0_YPOS_SET(blk->afc.af_roi[0].ypos);
	afc->roi0_size =
		NEO_AUTOFOCUS_ROI0_SIZE_CAM0_WIDTH_SET(blk->afc.af_roi[0].width) |
		NEO_AUTOFOCUS_ROI0_SIZE_CAM0_HEIGHT_SET(blk->afc.af_roi[0].height);
	afc->roi1_pos =
		NEO_AUTOFOCUS_ROI1_POS_CAM0_XPOS_SET(blk->afc.af_roi[1].xpos) |
		NEO_AUTOFOCUS_ROI1_POS_CAM0_YPOS_SET(blk->afc.af_roi[1].ypos);
	afc->roi1_size =
		NEO_AUTOFOCUS_ROI1_SIZE_CAM0_WIDTH_SET(blk->afc.af_roi[1].width) |
		NEO_AUTOFOCUS_ROI1_SIZE_CAM0_HEIGHT_SET(blk->afc.af_roi[1].height);
	afc->roi2_pos =
		NEO_AUTOFOCUS_ROI2_POS_CAM0_XPOS_SET(blk->afc.af_roi[2].xpos) |
		NEO_AUTOFOCUS_ROI2_POS_CAM0_YPOS_SET(blk->afc.af_roi[2].ypos);
	afc->roi2_size =
		NEO_AUTOFOCUS_ROI2_SIZE_CAM0_WIDTH_SET(blk->afc.af_roi[2].width) |
		NEO_AUTOFOCUS_ROI2_SIZE_CAM0_HEIGHT_SET(blk->afc.af_roi[2].height);
	afc->roi3_pos =
		NEO_AUTOFOCUS_ROI3_POS_CAM0_XPOS_SET(blk->afc.af_roi[3].xpos) |
		NEO_AUTOFOCUS_ROI3_POS_CAM0_YPOS_SET(blk->afc.af_roi[3].ypos);
	afc->roi3_size =
		NEO_AUTOFOCUS_ROI3_SIZE_CAM0_WIDTH_SET(blk->afc.af_roi[3].width) |
		NEO_AUTOFOCUS_ROI3_SIZE_CAM0_HEIGHT_SET(blk->afc.af_roi[3].height);
	afc->roi4_pos =
		NEO_AUTOFOCUS_ROI4_POS_CAM0_XPOS_SET(blk->afc.af_roi[4].xpos) |
		NEO_AUTOFOCUS_ROI4_POS_CAM0_YPOS_SET(blk->afc.af_roi[4].ypos);
	afc->roi4_size =
		NEO_AUTOFOCUS_ROI4_SIZE_CAM0_WIDTH_SET(blk->afc.af_roi[4].width) |
		NEO_AUTOFOCUS_ROI4_SIZE_CAM0_HEIGHT_SET(blk->afc.af_roi[4].height);
	afc->roi5_pos =
		NEO_AUTOFOCUS_ROI5_POS_CAM0_XPOS_SET(blk->afc.af_roi[5].xpos) |
		NEO_AUTOFOCUS_ROI5_POS_CAM0_YPOS_SET(blk->afc.af_roi[5].ypos);
	afc->roi5_size =
		NEO_AUTOFOCUS_ROI5_SIZE_CAM0_WIDTH_SET(blk->afc.af_roi[5].width) |
		NEO_AUTOFOCUS_ROI5_SIZE_CAM0_HEIGHT_SET(blk->afc.af_roi[5].height);
	afc->roi6_pos =
		NEO_AUTOFOCUS_ROI6_POS_CAM0_XPOS_SET(blk->afc.af_roi[6].xpos) |
		NEO_AUTOFOCUS_ROI6_POS_CAM0_YPOS_SET(blk->afc.af_roi[6].ypos);
	afc->roi6_size =
		NEO_AUTOFOCUS_ROI6_SIZE_CAM0_WIDTH_SET(blk->afc.af_roi[6].width) |
		NEO_AUTOFOCUS_ROI6_SIZE_CAM0_HEIGHT_SET(blk->afc.af_roi[6].height);
	afc->roi7_pos =
		NEO_AUTOFOCUS_ROI7_POS_CAM0_XPOS_SET(blk->afc.af_roi[7].xpos) |
		NEO_AUTOFOCUS_ROI7_POS_CAM0_YPOS_SET(blk->afc.af_roi[7].ypos);
	afc->roi7_size =
		NEO_AUTOFOCUS_ROI7_SIZE_CAM0_WIDTH_SET(blk->afc.af_roi[7].width) |
		NEO_AUTOFOCUS_ROI7_SIZE_CAM0_HEIGHT_SET(blk->afc.af_roi[7].height);
	afc->roi8_pos =
		NEO_AUTOFOCUS_ROI8_POS_CAM0_XPOS_SET(blk->afc.af_roi[8].xpos) |
		NEO_AUTOFOCUS_ROI8_POS_CAM0_YPOS_SET(blk->afc.af_roi[8].ypos);
	afc->roi8_size =
		NEO_AUTOFOCUS_ROI8_SIZE_CAM0_WIDTH_SET(blk->afc.af_roi[8].width) |
		NEO_AUTOFOCUS_ROI8_SIZE_CAM0_HEIGHT_SET(blk->afc.af_roi[8].height);
	afc->fil0_coeffs0 =
		NEO_AUTOFOCUS_FIL0_COEFFS0_CAM0_COEFF0_SET(blk->afc.fil0_coeffs[0]) |
		NEO_AUTOFOCUS_FIL0_COEFFS0_CAM0_COEFF1_SET(blk->afc.fil0_coeffs[1]) |
		NEO_AUTOFOCUS_FIL0_COEFFS0_CAM0_COEFF2_SET(blk->afc.fil0_coeffs[2]) |
		NEO_AUTOFOCUS_FIL0_COEFFS0_CAM0_COEFF3_SET(blk->afc.fil0_coeffs[3]);
	afc->fil0_coeffs1 =
		NEO_AUTOFOCUS_FIL0_COEFFS1_CAM0_COEFF4_SET(blk->afc.fil0_coeffs[4]) |
		NEO_AUTOFOCUS_FIL0_COEFFS1_CAM0_COEFF5_SET(blk->afc.fil0_coeffs[5]) |
		NEO_AUTOFOCUS_FIL0_COEFFS1_CAM0_COEFF6_SET(blk->afc.fil0_coeffs[6]) |
		NEO_AUTOFOCUS_FIL0_COEFFS1_CAM0_COEFF7_SET(blk->afc.fil0_coeffs[7]);
	afc->fil0_coeffs2 =
		NEO_AUTOFOCUS_FIL0_COEFFS2_CAM0_COEFF8_SET(blk->afc.fil0_coeffs[8]);
	afc->fil0_shift =
		NEO_AUTOFOCUS_FIL0_SHIFT_CAM0_SHIFT_SET(blk->afc.fil0_shift_shift);
	afc->fil1_coeffs0 =
		NEO_AUTOFOCUS_FIL1_COEFFS0_CAM0_COEFF0_SET(blk->afc.fil1_coeffs[0]) |
		NEO_AUTOFOCUS_FIL1_COEFFS0_CAM0_COEFF1_SET(blk->afc.fil1_coeffs[1]) |
		NEO_AUTOFOCUS_FIL1_COEFFS0_CAM0_COEFF2_SET(blk->afc.fil1_coeffs[2]) |
		NEO_AUTOFOCUS_FIL1_COEFFS0_CAM0_COEFF3_SET(blk->afc.fil1_coeffs[3]);
	afc->fil1_coeffs1 =
		NEO_AUTOFOCUS_FIL1_COEFFS1_CAM0_COEFF4_SET(blk->afc.fil1_coeffs[4]) |
		NEO_AUTOFOCUS_FIL1_COEFFS1_CAM0_COEFF5_SET(blk->afc.fil1_coeffs[5]) |
		NEO_AUTOFOCUS_FIL1_COEFFS1_CAM0_COEFF6_SET(blk->afc.fil1_coeffs[6]) |
		NEO_AUTOFOCUS_FIL1_COEFFS1_CAM0_COEFF7_SET(blk->afc.fil1_coeffs[7]);
	afc->fil1_coeffs2 =
		NEO_AUTOFOCUS_FIL1_COEFFS2_CAM0_COEFF8_SET(blk->afc.fil1_coeffs[8]);
	afc->fil1_shift =
		NEO_AUTOFOCUS_FIL1_SHIFT_CAM0_SHIFT_SET(blk->afc.fil1_shift_shift);
}

static void neoisp_set_context_mem_vignetting_table(struct neoisp_dev_s *neoispd,
		union neoisp_params_block_u *blk)
{
	struct neoisp_context_s *ctx = neoispd->queued_job.node_group->context;

	memcpy((u8 *)(uintptr_t)&ctx->vig,
		(u8 *)(uintptr_t)blk->vignetting_table.vignetting_table,
		sizeof(struct neoisp_vignetting_table_mem_params_s));
}

static void neoisp_set_context_mem_global_tonemap(struct neoisp_dev_s *neoispd,
		union neoisp_params_block_u *blk)
{
	struct neoisp_context_s *ctx = neoispd->queued_job.node_group->context;

	memcpy((u8 *)(uintptr_t)&ctx->gtm,
		(u8 *)(uintptr_t)blk->drc_global_tonemap.drc_global_tonemap,
		sizeof(struct neoisp_drc_global_tonemap_mem_params_s));
}

static void neoisp_set_context_mem_local_tonemap(struct neoisp_dev_s *neoispd,
		union neoisp_params_block_u *blk)
{
	struct neoisp_context_s *ctx = neoispd->queued_job.node_group->context;

	memcpy((u8 *)(uintptr_t)&ctx->ltm,
		(u8 *)(uintptr_t)blk->drc_local_tonemap.drc_local_tonemap,
		sizeof(struct neoisp_drc_local_tonemap_mem_params_s));
}

struct ycbcr_enc {
	/* Matrix stored in s8.8 format */
	s16 matrix[NEO_GAMMA_MATRIX_SIZE][NEO_GAMMA_MATRIX_SIZE];
	/* This range [-128, 127] is remapped to [0, 255] for full-range quantization.
	 * Thus, chrominance channels offset is 0.5 in s0.12 format that is 0.5 * 4096.
	 */
	s16 offsets[NEO_GAMMA_MATRIX_SIZE];
};
struct xfer_func {
	s16 gain; /* s8.8 format*/
	s16 blklvl_gain; /* s8.8 format */
	s16 threshold; /* s0.16 format */
	s16 gamma; /* s1.8 format */
	s16 gamma_offset; /* s0.12 format */
};

static const struct ycbcr_enc enc_lut[] = {
	[V4L2_YCBCR_ENC_601] = {
		/* BT.601 full-range encoding - floating-point matrix:
		 *	[0.299, 0.5870, 0.1140
		 *	 -0.1687, -0.3313, 0.5
		 *	 0.5, -0.4187, -0.0813]
		 */
		.matrix = {
			{77, 150, 29},
			{-43, -85, 128},
			{128, -107, -21},
		},
		.offsets = {0, 2048, 2048},
	}, [V4L2_YCBCR_ENC_709] = {
		/* BT.709 full-range encoding - floating-point matrix:
		 *	[0.2126, 0.7152, 0.0722
		 *	 -0.1146, -0.3854, 0.5
		 *	 0.5, -0.4542, -0.0458]
		 */
		.matrix = {
			{54, 183, 18},
			{-29, -99, 128},
			{128, -116, -12},
		},
		.offsets = {0, 2048, 2048},
	}, [V4L2_YCBCR_ENC_DEFAULT] = {
		/* No encoding - used for RGB output formats */
		.matrix = {
			{256, 0, 0},
			{0, 256, 0},
			{0, 0, 256},
		},
		.offsets = {0, 0, 0},
	},
};

static const struct xfer_func xfer_lut[] = {
	[V4L2_XFER_FUNC_709] = {
		/* L' = 4.5L, for 0 <= L <= 0.018
		 * L' = 1.099L^0.45 - 0.099, for L >= 0.018
		 *    = 1.099 * (L^0.45 - (0.099 / 1.099)), for L >= 0.018
		 */
		.gain = 281,
		.blklvl_gain = 1152,
		.threshold = 1180,
		.gamma = 115,
		.gamma_offset = 369,
	}, [V4L2_XFER_FUNC_SRGB] = {
		/* L' = 12.92L, for 0 <= L <= 0.0031308
		 * L' = 1.055L^(1/2.4) - 0.055, for L >= 0.0031308
		 *    = 1.055 * (L^(1/2.4) - (0.055 / 1.055)), for L >= 0.0031308
		 */
		.gain = 270,
		.blklvl_gain = 3308,
		.threshold = 205,
		.gamma = 107,
		.gamma_offset = 214,
	}, [V4L2_XFER_FUNC_NONE] = {
		.gain = 256,
		.blklvl_gain = 0,
		.threshold = 0,
		.gamma = 256,
		.gamma_offset = 0,
	},
};

void neoisp_update_context_gcm(struct neoisp_dev_s *neoispd,
		struct neoisp_context_s *context,
		enum v4l2_colorspace cspace, enum v4l2_xfer_func xfer,
		enum v4l2_ycbcr_encoding enc, enum v4l2_quantization quant)
{
	struct neoisp_gcm_s *gcm = &context->hw.gcm;
	int i, j;
	s32 value, tmat[NEO_GAMMA_MATRIX_SIZE][NEO_GAMMA_MATRIX_SIZE];
	u32 tmp;

	/* Colorspaces definition are extracted from kernel documentation:
	 * https://www.kernel.org/doc/html/latest/userspace-api/media/v4l/colorspaces-details.html
	 */

	/* Transfer function */
	gcm->lowth_ctrl01 =
		NEO_GCM_LOWTH_CTRL01_CAM0_THRESHOLD0_SET(xfer_lut[xfer].threshold) |
		NEO_GCM_LOWTH_CTRL01_CAM0_THRESHOLD1_SET(xfer_lut[xfer].threshold);
	gcm->lowth_ctrl2 =
		NEO_GCM_LOWTH_CTRL2_CAM0_THRESHOLD2_SET(xfer_lut[xfer].threshold);

	tmp = NEO_GCM_BLKLVL0_CTRL_CAM0_OFFSET0_GET(gcm->blklvl0_ctrl);
	gcm->blklvl0_ctrl |=
		NEO_GCM_BLKLVL0_CTRL_CAM0_OFFSET0_SET(tmp) |
		NEO_GCM_BLKLVL0_CTRL_CAM0_GAIN0_SET(xfer_lut[xfer].blklvl_gain);

	tmp = NEO_GCM_BLKLVL1_CTRL_CAM0_OFFSET1_GET(gcm->blklvl1_ctrl);
	gcm->blklvl1_ctrl |=
		NEO_GCM_BLKLVL1_CTRL_CAM0_OFFSET1_SET(tmp) |
		NEO_GCM_BLKLVL1_CTRL_CAM0_GAIN1_SET(xfer_lut[xfer].blklvl_gain);

	tmp = NEO_GCM_BLKLVL2_CTRL_CAM0_OFFSET2_GET(gcm->blklvl2_ctrl);
	gcm->blklvl2_ctrl |=
		NEO_GCM_BLKLVL2_CTRL_CAM0_OFFSET2_SET(tmp) |
		NEO_GCM_BLKLVL2_CTRL_CAM0_GAIN2_SET(xfer_lut[xfer].blklvl_gain);

	gcm->gamma0 =
		NEO_GCM_GAMMA0_CAM0_GAMMA0_SET(xfer_lut[xfer].gamma) |
		NEO_GCM_GAMMA0_CAM0_OFFSET0_SET(xfer_lut[xfer].gamma_offset);
	gcm->gamma1 =
		NEO_GCM_GAMMA1_CAM0_GAMMA1_SET(xfer_lut[xfer].gamma) |
		NEO_GCM_GAMMA1_CAM0_OFFSET1_SET(xfer_lut[xfer].gamma_offset);
	gcm->gamma2 =
		NEO_GCM_GAMMA2_CAM0_GAMMA2_SET(xfer_lut[xfer].gamma) |
		NEO_GCM_GAMMA2_CAM0_OFFSET2_SET(xfer_lut[xfer].gamma_offset);

	/* Quantization
	 *
	 * The quantization is amended by transfer function gain.
	 * The default quantization is full-range for RGB formats and
	 * V4L2_COLORSPACE_JPEG.
	 *
	 * In limited range the offsets are defined by standard as follow: (16, 128, 128)
	 * for 8-bit range while ISP offsets are defined for 12-bit range.
	 * Hence, the offsets defined by standard should be multiplied by 2^4=16:
	 * (256, 2048, 2048) for 12-bit range
	 * The same quantization factors are applied to Y'CbCr for BT.601 and BT.709:
	 * (219*Y, 224*Pb, 224*Pr)
	 */
	tmp = (quant == V4L2_QUANTIZATION_LIM_RANGE) ?
		256 : enc_lut[enc].offsets[0];
	gcm->ooffset0 = NEO_GCM_OOFFSET0_CAM0_OFFSET0_SET(tmp);

	/* Chrominance has the same offset for full or limited range */
	gcm->ooffset1 = NEO_GCM_OOFFSET1_CAM0_OFFSET1_SET(enc_lut[enc].offsets[1]);
	gcm->ooffset2 = NEO_GCM_OOFFSET2_CAM0_OFFSET2_SET(enc_lut[enc].offsets[2]);
	for (i = 0; i < NEO_GAMMA_MATRIX_SIZE; i++) {
		s32 factor = (quant == V4L2_QUANTIZATION_LIM_RANGE) ?
			(i == 0 ? 219 : 224) : 256;
		for (j = 0; j < NEO_GAMMA_MATRIX_SIZE; j++) {
			value = ((s32) enc_lut[enc].matrix[i][j] * factor) / 256;
			value = ((s32) value * (s32) xfer_lut[xfer].gain) / 256;
			tmat[i][j] = (s16)value;
		}
	}
	gcm->omat0 =
		NEO_GCM_OMAT0_CAM0_R0C0_SET(tmat[0][0]) |
		NEO_GCM_OMAT0_CAM0_R0C1_SET(tmat[0][1]);
	gcm->omat1 =
		NEO_GCM_OMAT1_CAM0_R0C2_SET(tmat[0][2]);
	gcm->omat2 =
		NEO_GCM_OMAT2_CAM0_R1C0_SET(tmat[1][0]) |
		NEO_GCM_OMAT2_CAM0_R1C1_SET(tmat[1][1]);
	gcm->omat3 =
		NEO_GCM_OMAT3_CAM0_R1C2_SET(tmat[1][2]);
	gcm->omat4 =
		NEO_GCM_OMAT4_CAM0_R2C0_SET(tmat[2][0]) |
		NEO_GCM_OMAT4_CAM0_R2C1_SET(tmat[2][1]);
	gcm->omat5 =
		NEO_GCM_OMAT5_CAM0_R2C2_SET(tmat[2][2]);
}

/*
 * extract offset and size in bytes from memory region map
 */
static void neoisp_get_offsize(enum isp_block_map_e map, u32 *offset, u32 *size)
{
	*offset = ISP_GET_OFF(map);
	*size = ISP_GET_SZ(map);
}

static void neoisp_get_stats_blk(struct neoisp_dev_s *neoispd, u32 btype, u8 *src,
		struct neoisp_ext_stats_s *dest, u32 *offset)
{
	union neoisp_stats_block_u *blk = (union neoisp_stats_block_u *)&dest->data[*offset];
	uint32_t size, loff, lsz;

	blk->header.type = btype;
	switch (btype) {
	case NEOISP_STATS_BLK_RCTEMP:
		size = sizeof(struct neoisp_ctemp_reg_stats_s);
		memcpy_fromio(&blk->rctemp.stat, neoispd->mmio + NEO_ALIAS_ALIAS_REG0, size);
		break;
	case NEOISP_STATS_BLK_RDRC:
		size = sizeof(struct neoisp_drc_reg_stats_s);
		memcpy_fromio(&blk->rdrc.stat, neoispd->mmio + NEO_ALIAS_ALIAS_REG59, size);
		break;
	case NEOISP_STATS_BLK_RAF:
		size = sizeof(struct neoisp_af_reg_stats_s);
		memcpy_fromio(&blk->raf.stat, neoispd->mmio + NEO_ALIAS_ALIAS_REG61, size);
		break;
	case NEOISP_STATS_BLK_RBNR:
		size = sizeof(struct neoisp_bnr_reg_stats_s);
		memcpy_fromio(&blk->rbnr.stat, neoispd->mmio + NEO_ALIAS_ALIAS_REG79, size);
		break;
	case NEOISP_STATS_BLK_RNR:
		size = sizeof(struct neoisp_nr_reg_stats_s);
		memcpy_fromio(&blk->rnr.stat, neoispd->mmio + NEO_ALIAS_ALIAS_REG81, size);
		break;
	case NEOISP_STATS_BLK_REE:
		size = sizeof(struct neoisp_ee_reg_stats_s);
		memcpy_fromio(&blk->ree.stat, neoispd->mmio + NEO_ALIAS_ALIAS_REG82, size);
		break;
	case NEOISP_STATS_BLK_RDF:
		size = sizeof(struct neoisp_df_reg_stats_s);
		memcpy_fromio(&blk->rdf.stat, neoispd->mmio + NEO_ALIAS_ALIAS_REG83, size);
		break;
	case NEOISP_STATS_BLK_MCTEMP:
		size = sizeof(struct neoisp_ctemp_mem_stats_s);
		/* Get ctemp stats from memory */
		neoisp_get_offsize(NEO_CTEMP_R_SUM_MAP, &loff, &lsz);
		memcpy(&blk->mctemp.stat.ctemp_r_sum, &src[loff], lsz);

		neoisp_get_offsize(NEO_CTEMP_G_SUM_MAP, &loff, &lsz);
		memcpy(&blk->mctemp.stat.ctemp_g_sum, &src[loff], lsz);

		neoisp_get_offsize(NEO_CTEMP_B_SUM_MAP, &loff, &lsz);
		memcpy(&blk->mctemp.stat.ctemp_b_sum, &src[loff], lsz);

		neoisp_get_offsize(NEO_CTEMP_PIX_CNT_MAP, &loff, &lsz);
		memcpy(&blk->mctemp.stat.ctemp_pix_cnt, &src[loff], lsz);
		break;
	case NEOISP_STATS_BLK_MRGBIR:
		size = sizeof(struct neoisp_rgbir_mem_stats_s);
		/* Get rgbir stats from memory */
		neoisp_get_offsize(NEO_RGBIR_HIST_MAP, &loff, &lsz);
		memcpy(&blk->mrgbir.stat, &src[loff], lsz);
		break;
	case NEOISP_STATS_BLK_MHIST:
		size = sizeof(struct neoisp_hist_mem_stats_s);
		/* Get histograms stats from memory */
		neoisp_get_offsize(NEO_HIST_STAT_MAP, &loff, &lsz);
		memcpy(&blk->mhist.stat, &src[loff], lsz);
		break;
	case NEOISP_STATS_BLK_MDRC:
		size = sizeof(struct neoisp_drc_mem_stats_s);
		/* Get drc local sum stats from memory */
		neoisp_get_offsize(neoispd->info->mems->drc_local_sum, &loff, &lsz);
		memcpy(&blk->mdrc.stat.drc_local_sum, &src[loff], lsz);

		/* Get drc hist roi0 stats from memory */
		neoisp_get_offsize(neoispd->info->mems->drc_global_hist_roi0, &loff, &lsz);
		memcpy(&blk->mdrc.stat.drc_global_hist_roi0, &src[loff], lsz);

		/* Get drc hist roi1 stats from memory */
		neoisp_get_offsize(neoispd->info->mems->drc_global_hist_roi1, &loff, &lsz);
		memcpy(&blk->mdrc.stat.drc_global_hist_roi1, &src[loff], lsz);
		break;
	}
	blk->header.size = ALIGN(size + sizeof(struct neoisp_ext_stats_block_header_s), 8);
	blk->header.flags = NEOISP_EXT_STATS_BLK_FL_NONE;
	*offset += blk->header.size;
}

void neoisp_get_stats(struct neoisp_dev_s *neoispd, struct neoisp_buffer_s *buf)
{
	struct neoisp_node_s *node = &neoispd->queued_job.node_group->node[NEOISP_STATS_NODE];
	u8 *src = (u8 *)(uintptr_t)neoispd->mmio_tcm;
	u32 offset, size, *blk_list, count;

	/* Check if stats node link is enabled */
	if (!neoisp_node_link_is_enabled(node))
		return;

	if (IS_ERR_OR_NULL(buf) || IS_ERR_OR_NULL(src)) {
		dev_err(&neoispd->pdev->dev, "Error: stats pointer\n");
		return;
	}

	switch (neoispd->api_ver) {
	case NEOISP_LEGACY_META_BUFFER: {
		struct neoisp_meta_stats_s *lstats = (struct neoisp_meta_stats_s *)get_vaddr(buf);
		/* Get stats from registers */
		memcpy_fromio((u32 *)(uintptr_t)&lstats->regs,
				neoispd->mmio + NEO_ALIAS_ALIAS_REG0,
				sizeof(struct neoisp_reg_stats_s));

		/* Get ctemp stats from memory */
		neoisp_get_offsize(NEO_CTEMP_R_SUM_MAP, &offset, &size);
		memcpy(&lstats->mems.ctemp.ctemp_r_sum, &src[offset], size);

		neoisp_get_offsize(NEO_CTEMP_G_SUM_MAP, &offset, &size);
		memcpy(&lstats->mems.ctemp.ctemp_g_sum, &src[offset], size);

		neoisp_get_offsize(NEO_CTEMP_B_SUM_MAP, &offset, &size);
		memcpy(&lstats->mems.ctemp.ctemp_b_sum, &src[offset], size);

		neoisp_get_offsize(NEO_CTEMP_PIX_CNT_MAP, &offset, &size);
		memcpy(&lstats->mems.ctemp.ctemp_pix_cnt, &src[offset], size);

		/* Get rgbir stats from memory */
		neoisp_get_offsize(NEO_RGBIR_HIST_MAP, &offset, &size);
		memcpy(&lstats->mems.rgbir.rgbir_hist, &src[offset], size);

		/* Get histograms stats from memory */
		neoisp_get_offsize(NEO_HIST_STAT_MAP, &offset, &size);
		memcpy(&lstats->mems.hist.hist_stat, &src[offset], size);

		/* Get drc local sum stats from memory */
		neoisp_get_offsize(neoispd->info->mems->drc_local_sum, &offset, &size);
		memcpy(&lstats->mems.drc.drc_local_sum, &src[offset], size);

		/* Get drc hist roi0 stats from memory */
		neoisp_get_offsize(neoispd->info->mems->drc_global_hist_roi0, &offset, &size);
		memcpy(&lstats->mems.drc.drc_global_hist_roi0, &src[offset], size);

		/* Get drc hist roi1 stats from memory */
		neoisp_get_offsize(neoispd->info->mems->drc_global_hist_roi1, &offset, &size);
		memcpy(&lstats->mems.drc.drc_global_hist_roi1, &src[offset], size);
		break;
	}

	case NEOISP_EXT_META_BUFFER_V1: {
		struct neoisp_ext_stats_s *dest = (struct neoisp_ext_stats_s *)get_vaddr(buf);

		dest->version = neoispd->api_ver;
		offset = 0;
		blk_list = (u32 *)neoisp_ext_stats_blocks_v1;
		count = ARRAY_SIZE(neoisp_ext_stats_blocks_v1);
		for (int i = 0; i < count; i++)
			neoisp_get_stats_blk(neoispd, blk_list[i], src, dest, &offset);

		dest->data_size = offset;
		break;
	}
	default:
		dev_err(&neoispd->pdev->dev, "Error: unknown api version (%d)\n", neoispd->api_ver);
		break;
	}
}

/*
 * Update relevant IP parameters for monochrome sensors
 */
void neoisp_update_context_monochrome_fmt(struct neoisp_dev_s *neoispd,
		struct neoisp_context_s *context, u32 pixfmt)
{
	struct neoisp_demosaic_s *dmsc;
	struct neoisp_bnr_s *bnr;

	dmsc = &context->hw.demosaic;
	bnr = &context->hw.bnr;

	if (FORMAT_IS_MONOCHROME(pixfmt)) {
		dmsc->ctrl = NEO_DEMOSAIC_CTRL_CAM0_FMT_SET(2); /* Monochrome format */
		bnr->ctrl |= NEO_BNR_CTRL_CAM0_NHOOD; /* 1-pixel Neighbourhood */
	} else {
		dmsc->ctrl = NEO_DEMOSAIC_CTRL_CAM0_FMT_SET(0); /* Bayer format */
		bnr->ctrl &= ~NEO_BNR_CTRL_CAM0_NHOOD; /* 2-pixel Neighbourhood */
	}
}

/*
 * Set Head Color selection
 */
void neoisp_update_context_head_color(struct neoisp_dev_s *neoispd,
		struct neoisp_context_s *context, u32 pixfmt)
{
	struct neoisp_hc_s *hc = &context->hw.hc;
	u8 hoffset, voffset;

	switch (pixfmt) {
	case (V4L2_PIX_FMT_SRGGB8):
	case (V4L2_PIX_FMT_SRGGB10):
	case (V4L2_PIX_FMT_SRGGB12):
	case (V4L2_PIX_FMT_SRGGB14):
	case (V4L2_PIX_FMT_SRGGB16):
		hoffset = 0;
		voffset = 0;
		break;
	case (V4L2_PIX_FMT_SGRBG8):
	case (V4L2_PIX_FMT_SGRBG10):
	case (V4L2_PIX_FMT_SGRBG12):
	case (V4L2_PIX_FMT_SGRBG14):
	case (V4L2_PIX_FMT_SGRBG16):
		hoffset = 1;
		voffset = 0;
		break;
	case (V4L2_PIX_FMT_SGBRG8):
	case (V4L2_PIX_FMT_SGBRG10):
	case (V4L2_PIX_FMT_SGBRG12):
	case (V4L2_PIX_FMT_SGBRG14):
	case (V4L2_PIX_FMT_SGBRG16):
		hoffset = 0;
		voffset = 1;
		break;
	case (V4L2_PIX_FMT_SBGGR8):
	case (V4L2_PIX_FMT_SBGGR10):
	case (V4L2_PIX_FMT_SBGGR12):
	case (V4L2_PIX_FMT_SBGGR14):
	case (V4L2_PIX_FMT_SBGGR16):
		hoffset = 1;
		voffset = 1;
		break;
	}
	hc->ctrl =
		NEO_HC_CTRL_CAM0_HOFFSET_SET(hoffset) |
		NEO_HC_CTRL_CAM0_VOFFSET_SET(voffset);
}

void neoisp_update_context_hdr_mode(struct neoisp_dev_s *neoispd,
		struct neoisp_context_s *context)
{
	struct neoisp_hdr_merge_s *hmg = &context->hw.hdr_merge;
	struct neoisp_hdr_decompress1_s *hd1 = &context->hw.hdr_decompress1;

	hmg->ctrl |= NEO_HDR_MERGE_CTRL_CAM0_ENABLE;
	hd1->ctrl |= NEO_HDR_DECOMPRESS1_CTRL_CAM0_ENABLE;
}

void neoisp_update_context_packetizer(struct neoisp_node_group_s *node_group)
{
	struct neoisp_node_s *nd = &node_group->node[NEOISP_FRAME_NODE];
	struct neoisp_packetizer_s *pck = &node_group->context->hw.packetizer;
	u32 pixfmt, i;
	u8 obpp, lsa, rsa, type, order0, order1, order2, a0s, subsample;

	if (neoisp_node_link_is_enabled(nd)) {
		pixfmt = nd->format.fmt.pix_mp.pixelformat;
		obpp = nd->neoisp_format->bpp_enc;
	} else {
		/* Force dummy buffer configuration to YUYV format */
		pixfmt = V4L2_PIX_FMT_YUYV;
		for (i = 0; i < ARRAY_SIZE(formats_vcap); i++)
			if (formats_vcap[i].fourcc == pixfmt)
				obpp = formats_vcap[i].bpp_enc;
	}

	switch (pixfmt) {
	case V4L2_PIX_FMT_Y10:
		rsa = 2;
		lsa = 0;
		break;
	case V4L2_PIX_FMT_Y12:
		rsa = 0;
		lsa = 0;
		break;
	case V4L2_PIX_FMT_Y16:
		rsa = 0;
		lsa = 4;
		break;
	default:
		rsa = 4;
		lsa = 0;
		break;
	}

	switch (pixfmt) {
	case V4L2_PIX_FMT_GREY:
	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_Y10:
	case V4L2_PIX_FMT_Y12:
	case V4L2_PIX_FMT_Y16:
	case V4L2_PIX_FMT_Y16_BE:
		type = 0;
		subsample = 2;
		/* Set channels orders */
		order0 = 2;
		order1 = 0;
		order2 = 1;
		/* Remove 0-padding */
		a0s = 0;
		break;
	case V4L2_PIX_FMT_NV21:
		type = 0;
		subsample = 2;
		/* Set channels orders */
		order0 = 2;
		order1 = 1;
		order2 = 0;
		/* Remove 0-padding */
		a0s = 0;
		break;
	case V4L2_PIX_FMT_NV16:
		type = 0;
		subsample = 1;
		/* Set channels orders */
		order0 = 2;
		order1 = 0;
		order2 = 1;
		/* Remove 0-padding */
		a0s = 0;
		break;
	case V4L2_PIX_FMT_NV61:
		type = 0;
		subsample = 1;
		/* Set channels orders */
		order0 = 2;
		order1 = 1;
		order2 = 0;
		/* Remove 0-padding */
		a0s = 0;
		break;
	case V4L2_PIX_FMT_YUYV:
		type = 1;
		subsample = 1;
		/* Set channels orders */
		order0 = 0;
		order1 = 1;
		order2 = 3;
		/* Remove 0-padding */
		a0s = 0;
		break;
	case V4L2_PIX_FMT_VYUY:
		type = 1;
		subsample = 1;
		/* Set channels orders */
		order0 = 1;
		order1 = 2;
		order2 = 0;
		/* Remove 0-padding */
		a0s = 0;
		break;
	case V4L2_PIX_FMT_UYVY:
		type = 1;
		subsample = 1;
		/* Set channels orders */
		order0 = 1;
		order1 = 0;
		order2 = 2;
		/* Remove 0-padding */
		a0s = 0;
		break;
	case V4L2_PIX_FMT_YUVX32:
		type = 1;
		subsample = 0;
		/* Set channels orders */
		order0 = 0;
		order1 = 1;
		order2 = 2;
		/* Add 0-padding */
		a0s = 8;
		break;
	case V4L2_PIX_FMT_VUYX32:
		type = 1;
		subsample = 0;
		/* Set channels orders */
		order0 = 2;
		order1 = 1;
		order2 = 0;
		/* Add 0-padding */
		a0s = 8;
		break;
	case V4L2_PIX_FMT_XBGR32:
		type = 1;
		subsample = 0;
		/* Set channels orders */
		order0 = 2;
		order1 = 1;
		order2 = 0;
		/* Add 0-padding */
		a0s = 8;
		break;
	case V4L2_PIX_FMT_RGBX32:
		type = 1;
		subsample = 0;
		/* Set channels orders */
		order0 = 0;
		order1 = 1;
		order2 = 2;
		/* Add 0-padding */
		a0s = 8;
		break;
	case V4L2_PIX_FMT_BGR24:
		type = 1;
		subsample = 0;
		/* Set channels orders */
		order0 = 2;
		order1 = 1;
		order2 = 0;
		/* Remove 0-padding */
		a0s = 0;
		break;
	default: /* All other pixel formats */
		type = 1;
		subsample = 0;
		/* Set channels orders */
		order0 = 0;
		order1 = 1;
		order2 = 2;
		/* Remove 0-padding */
		a0s = 0;
		break;
	}

	pck->ch0_ctrl =
		NEO_PACKETIZER_CH0_CTRL_CAM0_OBPP_SET(obpp) |
		NEO_PACKETIZER_CH0_CTRL_CAM0_RSA_SET(rsa) |
		NEO_PACKETIZER_CH0_CTRL_CAM0_LSA_SET(lsa);

	/* Keep same ch12 lsa/rsa config. */
	lsa = NEO_PACKETIZER_CH12_CTRL_CAM0_LSA_GET(pck->ch12_ctrl);
	rsa = NEO_PACKETIZER_CH12_CTRL_CAM0_RSA_GET(pck->ch12_ctrl);
	pck->ch12_ctrl =
		NEO_PACKETIZER_CH12_CTRL_CAM0_OBPP_SET(obpp) |
		NEO_PACKETIZER_CH12_CTRL_CAM0_RSA_SET(rsa) |
		NEO_PACKETIZER_CH12_CTRL_CAM0_LSA_SET(lsa) |
		NEO_PACKETIZER_CH12_CTRL_CAM0_SUBSAMPLE_SET(subsample);
	pck->pack_ctrl =
		NEO_PACKETIZER_PACK_CTRL_CAM0_TYPE_SET(type) |
		NEO_PACKETIZER_PACK_CTRL_CAM0_ORDER0_SET(order0) |
		NEO_PACKETIZER_PACK_CTRL_CAM0_ORDER1_SET(order1) |
		NEO_PACKETIZER_PACK_CTRL_CAM0_ORDER2_SET(order2) |
		NEO_PACKETIZER_PACK_CTRL_CAM0_A0S_SET(a0s);
}

/*
 * Set pipe conf fixed settings: image size, bpp, line stride, and dummy
 * addresses.
 */
void neoisp_update_context_pipe_conf(struct neoisp_node_group_s *node_group)
{
	struct neoisp_pipe_conf_s *cfg = &node_group->context->hw.pipe_conf.common;
	struct neoisp_node_s *nd;
	u32 tmp, width, height, obpp, irbpp, inp0_stride, inp1_stride;

	/* Input0 specific */
	nd = &node_group->node[NEOISP_INPUT0_NODE];
	width = nd->crop.width;
	height = nd->crop.height;
	inp0_stride = nd->format.fmt.pix_mp.plane_fmt[0].bytesperline;

	tmp = cfg->img_conf & ~NEO_PIPE_CONF_IMG_CONF_CAM0_IBPP0_MASK;
	tmp |= NEO_PIPE_CONF_IMG_CONF_CAM0_IBPP0_SET(nd->neoisp_format->bpp_enc);
	cfg->img_conf = tmp;

	/* Input 1 specific */
	nd = &node_group->node[NEOISP_INPUT1_NODE];
	inp1_stride = nd->format.fmt.pix_mp.plane_fmt[0].bytesperline;

	tmp = cfg->img_conf & ~NEO_PIPE_CONF_IMG_CONF_CAM0_IBPP1_MASK;
	tmp |= NEO_PIPE_CONF_IMG_CONF_CAM0_IBPP1_SET(nd->neoisp_format->bpp_enc);
	cfg->img_conf = tmp;

	/* Configure registers */
	cfg->img_size =
		NEO_PIPE_CONF_IMG_SIZE_CAM0_WIDTH_SET(width) |
		NEO_PIPE_CONF_IMG_SIZE_CAM0_HEIGHT_SET(height);
	cfg->img0_in_ls =
		NEO_PIPE_CONF_IMG0_IN_LS_CAM0_LS_SET(inp0_stride);

	/* Handle hdr inputs */
	nd = &node_group->node[NEOISP_INPUT1_NODE];
	if (neoisp_node_link_is_enabled(nd)) {
		cfg->img1_in_ls =
			NEO_PIPE_CONF_IMG1_IN_LS_CAM0_LS_SET(inp1_stride);
	} else {
		cfg->img1_in_addr =
			NEO_PIPE_CONF_ADDR_SET(0u);
		cfg->img1_in_ls =
			NEO_PIPE_CONF_IMG1_IN_LS_CAM0_LS_SET(0u);
	}

	nd = &node_group->node[NEOISP_FRAME_NODE];
	if (neoisp_node_link_is_enabled(nd)) {
		obpp = (nd->neoisp_format->bit_depth + 7) / 8;

		switch (nd->format.fmt.pix_mp.pixelformat) {
		case V4L2_PIX_FMT_GREY:
		case V4L2_PIX_FMT_Y10:
		case V4L2_PIX_FMT_Y12:
		case V4L2_PIX_FMT_Y16:
		case V4L2_PIX_FMT_Y16_BE:
			/*
			 * Monochrome formats:
			 * - output0 is used for Y component
			 * - output1 on dummy buffer
			 */
			cfg->outch1_addr =
				NEO_PIPE_CONF_ADDR_SET(node_group->dummy_dma);

			cfg->outch0_ls =
				NEO_PIPE_CONF_OUTCH0_LS_CAM0_LS_SET(obpp * width);
			cfg->outch1_ls =
				NEO_PIPE_CONF_OUTCH1_LS_CAM0_LS_SET(0u);
			break;
		case V4L2_PIX_FMT_NV12:
		case V4L2_PIX_FMT_NV21:
		case V4L2_PIX_FMT_NV16:
		case V4L2_PIX_FMT_NV61:
			/*
			 * Semi-Planar formats:
			 * - output0 is used for Y component
			 * - output1 is used for UV components
			 */
			cfg->outch1_ls =
				NEO_PIPE_CONF_OUTCH1_LS_CAM0_LS_SET(obpp * width);
			cfg->outch0_ls =
				NEO_PIPE_CONF_OUTCH0_LS_CAM0_LS_SET(obpp * width);
			break;
		default:
			/*
			 * Interleaved formats:
			 * - output0 is not used at all
			 * - output1 is used for YUV or RGB components
			 */
			cfg->outch0_addr =
				NEO_PIPE_CONF_ADDR_SET(0u);
			cfg->outch0_ls =
				NEO_PIPE_CONF_OUTCH0_LS_CAM0_LS_SET(0u);
			cfg->outch1_ls =
				NEO_PIPE_CONF_OUTCH1_LS_CAM0_LS_SET(obpp * width);
			break;
		}
	} else {
		/* Default dummy pixelformat is set to YUYV */
		cfg->outch0_addr =
			NEO_PIPE_CONF_ADDR_SET(node_group->dummy_dma);
		cfg->outch1_addr =
			NEO_PIPE_CONF_ADDR_SET(node_group->dummy_dma);
		cfg->outch0_ls =
			NEO_PIPE_CONF_OUTCH0_LS_CAM0_LS_SET(0u);
		cfg->outch1_ls =
			NEO_PIPE_CONF_OUTCH1_LS_CAM0_LS_SET(0u);
	}

	nd = &node_group->node[NEOISP_IR_NODE];
	if (neoisp_node_link_is_enabled(nd)) {
		irbpp = (nd->neoisp_format->bit_depth + 7) / 8;

		cfg->outir_ls =
			NEO_PIPE_CONF_OUTIR_LS_CAM0_LS_SET(irbpp * width);
	} else {
		cfg->outir_addr =
			NEO_PIPE_CONF_ADDR_SET(node_group->dummy_dma);
		cfg->outir_ls =
			NEO_PIPE_CONF_OUTIR_LS_CAM0_LS_SET(0u);
	}
}

/*
 * Set pipe conf volatile settings: buffer addresses.
 */
void neoisp_update_context_buf_addr(struct neoisp_dev_s *neoispd)
{
	struct neoisp_job_s *job = &neoispd->queued_job;
	struct neoisp_pipe_conf_s *cfg = &job->node_group->context->hw.pipe_conf.common;
	struct neoisp_buffer_s *buf_inp0 = job->buf[NEOISP_INPUT0_NODE];
	struct neoisp_buffer_s *buf_inp1 = job->buf[NEOISP_INPUT1_NODE];
	struct neoisp_buffer_s *buf_out = job->buf[NEOISP_FRAME_NODE];
	struct neoisp_buffer_s *buf_ir = job->buf[NEOISP_IR_NODE];
	struct neoisp_node_s *nd;
	u32 width, height, ibpp, inp0_stride, inp1_stride;
	dma_addr_t inp0_addr, inp1_addr;

	/* Input0 specific */
	nd = &job->node_group->node[NEOISP_INPUT0_NODE];
	width = nd->crop.width;
	height = nd->crop.height;
	ibpp = (nd->neoisp_format->bit_depth + 7) / 8;
	inp0_stride = nd->format.fmt.pix_mp.plane_fmt[0].bytesperline;

	/* Input0 - Take crop into account if any */
	inp0_addr = get_addr(buf_inp0, 0) + (nd->crop.left * ibpp) + (nd->crop.top * inp0_stride);

	/* Input 1 specific */
	nd = &job->node_group->node[NEOISP_INPUT1_NODE];
	ibpp = (nd->neoisp_format->bit_depth + 7) / 8;
	inp1_stride = nd->format.fmt.pix_mp.plane_fmt[0].bytesperline;

	/* Input1 - Take crop into account if any */
	inp1_addr = get_addr(buf_inp1, 0) + (nd->crop.left * ibpp) + (nd->crop.top * inp1_stride);

	cfg->img0_in_addr =
		NEO_PIPE_CONF_ADDR_SET(inp0_addr);

	/* Handle hdr inputs */
	nd = &job->node_group->node[NEOISP_INPUT1_NODE];
	if (neoisp_node_link_is_enabled(nd)) {
		cfg->img1_in_addr =
			NEO_PIPE_CONF_ADDR_SET(inp1_addr);
	}

	nd = &job->node_group->node[NEOISP_FRAME_NODE];
	if (neoisp_node_link_is_enabled(nd)) {
		/* Planar/multiplanar output image addresses */
		switch (nd->format.fmt.pix_mp.pixelformat) {
		case V4L2_PIX_FMT_GREY:
		case V4L2_PIX_FMT_Y10:
		case V4L2_PIX_FMT_Y12:
		case V4L2_PIX_FMT_Y16:
		case V4L2_PIX_FMT_Y16_BE:
			/* Monochrome formats: only output channel 0 is used */
			cfg->outch0_addr =
				NEO_PIPE_CONF_ADDR_SET(get_addr(buf_out, 0));
			break;
		case V4L2_PIX_FMT_NV12:
		case V4L2_PIX_FMT_NV21:
		case V4L2_PIX_FMT_NV16:
		case V4L2_PIX_FMT_NV61:
			/* Semi-Planar formats: both output channels are used */
			cfg->outch0_addr =
				NEO_PIPE_CONF_ADDR_SET(get_addr(buf_out, 0));
			cfg->outch1_addr =
				NEO_PIPE_CONF_ADDR_SET(get_addr(buf_out, 1));
			break;
		default:
			/* Interleaved formats: only output channel 1 is used */
			cfg->outch1_addr =
				NEO_PIPE_CONF_ADDR_SET(get_addr(buf_out, 0));
			break;
		}
	}

	nd = &job->node_group->node[NEOISP_IR_NODE];
	if (neoisp_node_link_is_enabled(nd))
		cfg->outir_addr =
			NEO_PIPE_CONF_ADDR_SET(get_addr(buf_ir, 0));
}

static const struct neoisp_block_handler_s neoisp_block_handlers[] = {
	[NEOISP_PARAM_BLK_PIPE_CONF] = {
		.size = sizeof(struct neoisp_pipe_conf_cfg_s),
		.handler = &neoisp_set_context_img_conf,
	},
	[NEOISP_PARAM_BLK_HEAD_COLOR] = {
		.size = sizeof(struct neoisp_head_color_cfg_s),
		.handler = &neoisp_set_context_head_color,
	},
	[NEOISP_PARAM_BLK_HDR_DECOMPRESS0] = {
		.size = sizeof(struct neoisp_hdr_decompress0_cfg_s),
		.handler = &neoisp_set_context_hdr_decompress0,
	},
	[NEOISP_PARAM_BLK_HDR_DECOMPRESS1] = {
		.size = sizeof(struct neoisp_hdr_decompress1_cfg_s),
		.handler = &neoisp_set_context_hdr_decompress1,
	},
	[NEOISP_PARAM_BLK_OBWB0] = {
		.size = sizeof(struct neoisp_obwb_cfg_s),
		.handler = &neoisp_set_context_obwb0,
	},
	[NEOISP_PARAM_BLK_OBWB1] = {
		.size = sizeof(struct neoisp_obwb_cfg_s),
		.handler = &neoisp_set_context_obwb1,
	},
	[NEOISP_PARAM_BLK_OBWB2] = {
		.size = sizeof(struct neoisp_obwb_cfg_s),
		.handler = &neoisp_set_context_obwb2,
	},
	[NEOISP_PARAM_BLK_HDR_MERGE] = {
		.size = sizeof(struct neoisp_hdr_merge_cfg_s),
		.handler = &neoisp_set_context_hdr_merge,
	},
	[NEOISP_PARAM_BLK_RGBIR] = {
		.size = sizeof(struct neoisp_rgbir_cfg_s),
		.handler = &neoisp_set_context_rgbir,
	},
	[NEOISP_PARAM_BLK_STAT] = {
		.size = sizeof(struct neoisp_stat_cfg_s),
		.handler = &neoisp_set_context_stat_hists,
	},
	[NEOISP_PARAM_BLK_IR_COMPRESS] = {
		.size = sizeof(struct neoisp_ir_compress_cfg_s),
		.handler = &neoisp_set_context_ir_compress,
	},
	[NEOISP_PARAM_BLK_BNR] = {
		.size = sizeof(struct neoisp_bnr_cfg_s),
		.handler = &neoisp_set_context_bnr,
	},
	[NEOISP_PARAM_BLK_VIGNETTING_CTRL] = {
		.size = sizeof(struct neoisp_vignetting_ctrl_cfg_s),
		.handler = &neoisp_set_context_vignetting,
	},
	[NEOISP_PARAM_BLK_CTEMP] = {
		.size = sizeof(struct neoisp_ctemp_cfg_s),
		.handler = &neoisp_set_context_color_temp,
	},
	[NEOISP_PARAM_BLK_DEMOSAIC] = {
		.size = sizeof(struct neoisp_demosaic_cfg_s),
		.handler = &neoisp_set_context_demosaic,
	},
	[NEOISP_PARAM_BLK_RGB2YUV] = {
		.size = sizeof(struct neoisp_rgb2yuv_cfg_s),
		.handler = &neoisp_set_context_rgb_to_yuv,
	},
	[NEOISP_PARAM_BLK_DR_COMP] = {
		.size = sizeof(struct neoisp_dr_comp_cfg_s),
		.handler = &neoisp_set_context_drc,
	},
	[NEOISP_PARAM_BLK_NR] = {
		.size = sizeof(struct neoisp_nr_cfg_s),
		.handler = &neoisp_set_context_nr,
	},
	[NEOISP_PARAM_BLK_AF] = {
		.size = sizeof(struct neoisp_af_cfg_s),
		.handler = &neoisp_set_context_autofocus,
	},
	[NEOISP_PARAM_BLK_EE] = {
		.size = sizeof(struct neoisp_ee_cfg_s),
		.handler = &neoisp_set_context_ee,
	},
	[NEOISP_PARAM_BLK_DF] = {
		.size = sizeof(struct neoisp_df_cfg_s),
		.handler = &neoisp_set_context_df,
	},
	[NEOISP_PARAM_BLK_CONVMED] = {
		.size = sizeof(struct neoisp_convmed_cfg_s),
		.handler = &neoisp_set_context_convmed,
	},
	[NEOISP_PARAM_BLK_CAS] = {
		.size = sizeof(struct neoisp_cas_cfg_s),
		.handler = &neoisp_set_context_cas,
	},
	[NEOISP_PARAM_BLK_GCM] = {
		.size = sizeof(struct neoisp_gcm_cfg_s),
		.handler = &neoisp_set_context_gcm,
	},
	[NEOISP_PARAM_BLK_VIGNETTING_TABLE] = {
		.size = sizeof(struct neoisp_vignetting_table_mem_params_s),
		.handler = &neoisp_set_context_mem_vignetting_table,
	},
	[NEOISP_PARAM_BLK_DRC_GLOBAL_TONEMAP] = {
		.size = sizeof(struct neoisp_drc_global_tonemap_mem_params_s),
		.handler = &neoisp_set_context_mem_global_tonemap,
	},
	[NEOISP_PARAM_BLK_DRC_LOCAL_TONEMAP] = {
		.size = sizeof(struct neoisp_drc_local_tonemap_mem_params_s),
		.handler = &neoisp_set_context_mem_local_tonemap,
	},
};

/*
 * neoisp_upload_context is used to write all parameters to registers and
 * memory.
 *
 * The register copy starts from PIPE_CONF.IMG_CONF offset, up to the latest
 * writable register in AF unit.
 *
 * The memory copy is performed by block, because base addresses of the LUT
 * depend on the hw version.
 */
void neoisp_upload_context(struct neoisp_dev_s *neoispd)
{
	struct neoisp_node_group_s *node_group = neoispd->queued_job.node_group;
	struct neoisp_context_s *ctx = node_group->context;
	u8 *src = (u8 *)(uintptr_t)&ctx->hw.pipe_conf.common.img_conf;
	u8 *dst = (u8 *)(uintptr_t)(neoispd->mmio + NEO_PIPE_CONF_IMG_CONF_CAM0);
	u32 *imem = (u32 *)(uintptr_t)neoispd->mmio_tcm;

	memcpy(dst, src, NEO_AUTOFOCUS_ROI0_SUM0_CAM0 - NEO_PIPE_CONF_IMG_CONF_CAM0);

	ctx_blk_write(neoispd->info->mems->vignetting_table,
		      (u32 *)ctx->vig.vignetting_table, imem);

	ctx_blk_write(neoispd->info->mems->drc_global_tonemap,
		      (u32 *)ctx->gtm.drc_global_tonemap, imem);

	ctx_blk_write(neoispd->info->mems->drc_local_tonemap,
		      (u32 *)ctx->ltm.drc_local_tonemap, imem);
}

/*
 * neoisp_update_context_w_user_params is used to update the context of the
 * queued node_group with user space values.
 */
void neoisp_update_context_w_user_params(struct neoisp_dev_s *neoispd)
{
	struct neoisp_buffer_s *buf = neoispd->queued_job.buf[NEOISP_PARAMS_NODE];
	const struct neoisp_block_handler_s *blk_handler;
	struct neoisp_ext_params_s *ext_params;
	union neoisp_params_block_u *blk, *cfg;
	size_t max_offset, blk_offset = 0, hdr_size;

	if (IS_ERR_OR_NULL(buf))
		return; /* No params buffer provided */

	/* Check if legacy parameters provided */
	if (neoispd->api_ver == NEOISP_LEGACY_META_BUFFER) {
		struct neoisp_meta_params_s *lparams =
			(struct neoisp_meta_params_s *)get_vaddr(buf);

		/* Update selected blocks wrt feature config flag */
		if (lparams->features_cfg.pipe_conf_cfg) {
			blk_handler = &neoisp_block_handlers[NEOISP_PARAM_BLK_PIPE_CONF];
			blk = (union neoisp_params_block_u *)&lparams->regs.pipe_conf;
			blk_handler->handler(neoispd, blk);
		}
		if (lparams->features_cfg.head_color_cfg) {
			blk_handler = &neoisp_block_handlers[NEOISP_PARAM_BLK_HEAD_COLOR];
			blk = (union neoisp_params_block_u *)&lparams->regs.head_color;
			blk_handler->handler(neoispd, blk);
		}
		if (lparams->features_cfg.hdr_decompress_input0_cfg) {
			blk_handler = &neoisp_block_handlers[NEOISP_PARAM_BLK_HDR_DECOMPRESS0];
			blk = (union neoisp_params_block_u *)&lparams->regs.decompress_input0;
			blk_handler->handler(neoispd, blk);
		}
		if (lparams->features_cfg.hdr_decompress_input1_cfg) {
			blk_handler = &neoisp_block_handlers[NEOISP_PARAM_BLK_HDR_DECOMPRESS1];
			blk = (union neoisp_params_block_u *)&lparams->regs.decompress_input1;
			blk_handler->handler(neoispd, blk);
		}
		if (lparams->features_cfg.obwb0_cfg) {
			blk_handler = &neoisp_block_handlers[NEOISP_PARAM_BLK_OBWB0];
			blk = (union neoisp_params_block_u *)&lparams->regs.obwb[0];
			blk_handler->handler(neoispd, blk);
		}
		if (lparams->features_cfg.obwb1_cfg) {
			blk_handler = &neoisp_block_handlers[NEOISP_PARAM_BLK_OBWB1];
			blk = (union neoisp_params_block_u *)&lparams->regs.obwb[1];
			blk_handler->handler(neoispd, blk);
		}
		if (lparams->features_cfg.obwb2_cfg) {
			blk_handler = &neoisp_block_handlers[NEOISP_PARAM_BLK_OBWB2];
			blk = (union neoisp_params_block_u *)&lparams->regs.obwb[2];
			blk_handler->handler(neoispd, blk);
		}
		if (lparams->features_cfg.hdr_merge_cfg) {
			blk_handler = &neoisp_block_handlers[NEOISP_PARAM_BLK_HDR_MERGE];
			blk = (union neoisp_params_block_u *)&lparams->regs.hdr_merge;
			blk_handler->handler(neoispd, blk);
		}
		if (lparams->features_cfg.rgbir_cfg) {
			blk_handler = &neoisp_block_handlers[NEOISP_PARAM_BLK_RGBIR];
			blk = (union neoisp_params_block_u *)&lparams->regs.rgbir;
			blk_handler->handler(neoispd, blk);
		}
		if (lparams->features_cfg.stat_cfg) {
			blk_handler = &neoisp_block_handlers[NEOISP_PARAM_BLK_STAT];
			blk = (union neoisp_params_block_u *)&lparams->regs.stat;
			blk_handler->handler(neoispd, blk);
		}
		if (lparams->features_cfg.ir_compress_cfg) {
			blk_handler = &neoisp_block_handlers[NEOISP_PARAM_BLK_IR_COMPRESS];
			blk = (union neoisp_params_block_u *)&lparams->regs.ir_compress;
			blk_handler->handler(neoispd, blk);
		}
		if (lparams->features_cfg.bnr_cfg) {
			blk_handler = &neoisp_block_handlers[NEOISP_PARAM_BLK_BNR];
			blk = (union neoisp_params_block_u *)&lparams->regs.bnr;
			blk_handler->handler(neoispd, blk);
		}
		if (lparams->features_cfg.vignetting_ctrl_cfg) {
			blk_handler = &neoisp_block_handlers[NEOISP_PARAM_BLK_VIGNETTING_CTRL];
			blk = (union neoisp_params_block_u *)&lparams->regs.vignetting_ctrl;
			blk_handler->handler(neoispd, blk);
		}
		if (lparams->features_cfg.ctemp_cfg) {
			blk_handler = &neoisp_block_handlers[NEOISP_PARAM_BLK_CTEMP];
			blk = (union neoisp_params_block_u *)&lparams->regs.ctemp;
			blk_handler->handler(neoispd, blk);
		}
		if (lparams->features_cfg.demosaic_cfg) {
			blk_handler = &neoisp_block_handlers[NEOISP_PARAM_BLK_DEMOSAIC];
			blk = (union neoisp_params_block_u *)&lparams->regs.demosaic;
			blk_handler->handler(neoispd, blk);
		}
		if (lparams->features_cfg.rgb2yuv_cfg) {
			blk_handler = &neoisp_block_handlers[NEOISP_PARAM_BLK_RGB2YUV];
			blk = (union neoisp_params_block_u *)&lparams->regs.rgb2yuv;
			blk_handler->handler(neoispd, blk);
		}
		if (lparams->features_cfg.dr_comp_cfg) {
			blk_handler = &neoisp_block_handlers[NEOISP_PARAM_BLK_DR_COMP];
			blk = (union neoisp_params_block_u *)&lparams->regs.drc;
			blk_handler->handler(neoispd, blk);
		}
		if (lparams->features_cfg.nr_cfg) {
			blk_handler = &neoisp_block_handlers[NEOISP_PARAM_BLK_NR];
			blk = (union neoisp_params_block_u *)&lparams->regs.nr;
			blk_handler->handler(neoispd, blk);
		}
		if (lparams->features_cfg.af_cfg) {
			blk_handler = &neoisp_block_handlers[NEOISP_PARAM_BLK_AF];
			blk = (union neoisp_params_block_u *)&lparams->regs.af;
			blk_handler->handler(neoispd, blk);
		}
		if (lparams->features_cfg.ee_cfg) {
			blk_handler = &neoisp_block_handlers[NEOISP_PARAM_BLK_EE];
			blk = (union neoisp_params_block_u *)&lparams->regs.ee;
			blk_handler->handler(neoispd, blk);
		}
		if (lparams->features_cfg.df_cfg) {
			blk_handler = &neoisp_block_handlers[NEOISP_PARAM_BLK_DF];
			blk = (union neoisp_params_block_u *)&lparams->regs.df;
			blk_handler->handler(neoispd, blk);
		}
		if (lparams->features_cfg.convmed_cfg) {
			blk_handler = &neoisp_block_handlers[NEOISP_PARAM_BLK_CONVMED];
			blk = (union neoisp_params_block_u *)&lparams->regs.convmed;
			blk_handler->handler(neoispd, blk);
		}
		if (lparams->features_cfg.cas_cfg) {
			blk_handler = &neoisp_block_handlers[NEOISP_PARAM_BLK_CAS];
			blk = (union neoisp_params_block_u *)&lparams->regs.cas;
			blk_handler->handler(neoispd, blk);
		}
		if (lparams->features_cfg.gcm_cfg) {
			blk_handler = &neoisp_block_handlers[NEOISP_PARAM_BLK_GCM];
			blk = (union neoisp_params_block_u *)&lparams->regs.gcm;
			blk_handler->handler(neoispd, blk);
		}
		if (lparams->features_cfg.vignetting_table_cfg) {
			blk_handler = &neoisp_block_handlers[NEOISP_PARAM_BLK_VIGNETTING_TABLE];
			blk = (union neoisp_params_block_u *)&lparams->mems.vt;
			blk_handler->handler(neoispd, blk);
		}
		if (lparams->features_cfg.drc_global_tonemap_cfg) {
			blk_handler = &neoisp_block_handlers[NEOISP_PARAM_BLK_DRC_GLOBAL_TONEMAP];
			blk = (union neoisp_params_block_u *)&lparams->mems.gtm;
			blk_handler->handler(neoispd, blk);
		}
		if (lparams->features_cfg.drc_local_tonemap_cfg) {
			blk_handler = &neoisp_block_handlers[NEOISP_PARAM_BLK_DRC_LOCAL_TONEMAP];
			blk = (union neoisp_params_block_u *)&lparams->mems.ltm;
			blk_handler->handler(neoispd, blk);
		}

		return;
	}

	/* Extended params API */
	hdr_size = sizeof(struct neoisp_ext_params_block_header_s);
	ext_params = (struct neoisp_ext_params_s *)get_vaddr(buf);
	max_offset = ext_params->data_size - hdr_size;

	/*
	 * Walk the list of parameter blocks and process them. No validation is
	 * done here, as the contents of the config buffer are already checked
	 * when the buffer is queued.
	 */
	while (blk_offset < max_offset) {
		blk = (union neoisp_params_block_u *)&ext_params->data[blk_offset];
		cfg = (union neoisp_params_block_u *)&blk->data[hdr_size];

		/* Avoid infinite loop */
		if (blk->header.size == 0)
			break;

		/* Update selected blocks wrt feature config flag */
		if (blk->header.flags == NEOISP_EXT_PARAMS_BLK_FL_UPDATE) {
			blk_handler = &neoisp_block_handlers[blk->header.type];
			blk_handler->handler(neoispd, cfg);
		}

		blk_offset += blk->header.size;
	}
}
