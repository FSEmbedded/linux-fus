/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * NEOISP context definition
 *
 * Copyright 2023-2025 NXP
 */

#ifndef NEOISP_CTX_H
#define NEOISP_CTX_H

#include <linux/media/nxp/nxp_neoisp.h>

#include "neoisp.h"
#include "neoisp_regs.h"

#define NEOISP_PIPELINE0_BPP    (20) /* Internal bit depth for input 0 path */
#define NEOISP_PIPELINE1_BPP    (16) /* Internal bit depth for input 1 path */
#define NEOISP_HDR_SHIFT_MAX    (7)  /* Hdr decompress block ratio field format is u7.5 */
#define NEOISP_HDR_SHIFT_RADIX  (5)  /* Hdr decompress block ratio field format is u7.5 */
#define NEOISP_OBWB_SHIFT_RADIX (8)  /* Obwb block gain field format is u8.8 */
#define NEOISP_HDR_KNPOINT_MAX  GENMASK(15, 0) /* Knee point field is 16 bits */

/* Block offset */
#define ISP_OFF_POS (0UL)
#define ISP_OFF_MASK (0xFFFFUL << ISP_OFF_POS)
#define ISP_GET_OFF(x) (((x) & ISP_OFF_MASK) >> ISP_OFF_POS)
#define ISP_OFF(x) (((x) << ISP_OFF_POS) & ISP_OFF_MASK)

/* Block size */
#define ISP_SZ_POS (16UL)
#define ISP_SZ_MASK (0xFFFFUL << ISP_SZ_POS)
#define ISP_GET_SZ(x) (((x) & ISP_SZ_MASK) >> ISP_SZ_POS)
#define ISP_SZ(x) (((x) << ISP_SZ_POS) & ISP_SZ_MASK)

#define ISP_MAP_TUPLE(x, y, z) (ISP_OFF((x)) | ISP_SZ(((y) * sizeof(z))))

enum isp_block_map_e {
	NEO_CTEMP_R_SUM_MAP = ISP_MAP_TUPLE(0x0, NEO_CTEMP_R_SUM_CNT, u32),
	NEO_CTEMP_G_SUM_MAP = ISP_MAP_TUPLE(0x100, NEO_CTEMP_G_SUM_CNT, u32),
	NEO_CTEMP_B_SUM_MAP = ISP_MAP_TUPLE(0x200, NEO_CTEMP_B_SUM_CNT, u32),
	NEO_CTEMP_PIX_CNT_MAP = ISP_MAP_TUPLE(0x300, NEO_CTEMP_PIX_CNT_CNT, u16),
	NEO_RGBIR_HIST_MAP = ISP_MAP_TUPLE(0x400, NEO_RGBIR_HIST_CNT, u32),
	NEO_HIST_STAT_MAP = ISP_MAP_TUPLE(0x800, NEO_HIST_STAT_CNT, u32),

	NEO_VIGNETTING_TABLE_MAP_V1 = ISP_MAP_TUPLE(0x1000, NEO_VIGNETTING_TABLE_SIZE, u16),
	NEO_DRC_GLOBAL_TONEMAP_MAP_V1 = ISP_MAP_TUPLE(0x4000, NEO_DRC_GLOBAL_TONEMAP_SIZE, u16),
	NEO_DRC_LOCAL_TONEMAP_MAP_V1 = ISP_MAP_TUPLE(0x4400, NEO_DRC_LOCAL_TONEMAP_SIZE, u8),
	NEO_DRC_LOCAL_SUM_MAP_V1 = ISP_MAP_TUPLE(0x4800, NEO_DRC_LOCAL_SUM_CNT, u32),
	NEO_DRC_GLOBAL_HIST_ROI0_MAP_V1 = ISP_MAP_TUPLE(0x5800, NEO_DRC_GLOBAL_HIST_ROI_CNT, u32),
	NEO_DRC_GLOBAL_HIST_ROI1_MAP_V1 = ISP_MAP_TUPLE(0x5F00, NEO_DRC_GLOBAL_HIST_ROI_CNT, u32),

	NEO_DRC_GLOBAL_HIST_ROI0_MAP_V2 = ISP_MAP_TUPLE(0x1000, NEO_DRC_GLOBAL_HIST_ROI_CNT, u32),
	NEO_DRC_GLOBAL_HIST_ROI1_MAP_V2 = ISP_MAP_TUPLE(0x1700, NEO_DRC_GLOBAL_HIST_ROI_CNT, u32),
	NEO_DRC_LOCAL_SUM_MAP_V2 = ISP_MAP_TUPLE(0x1E00, NEO_DRC_LOCAL_SUM_CNT, u32),
	NEO_VIGNETTING_TABLE_MAP_V2 = ISP_MAP_TUPLE(0x2E00, NEO_VIGNETTING_TABLE_SIZE, u16),
	NEO_DRC_GLOBAL_TONEMAP_MAP_V2 = ISP_MAP_TUPLE(0x4600, NEO_DRC_GLOBAL_TONEMAP_SIZE, u16),
	NEO_DRC_LOCAL_TONEMAP_MAP_V2 = ISP_MAP_TUPLE(0x4A00, NEO_DRC_LOCAL_TONEMAP_SIZE, u8),
};

/**
 * union neoisp_params_block_u - Generalisation of a parameter block
 *
 * This union allows the driver to treat a block as a generic pointer to this
 * union and safely access the header and block-specific struct without having
 * to resort to casting. The header member is accessed first, and the type field
 * checked which allows the driver to determine which of the other members
 * should be used. The data member at the end allows a pointer to an address
 * within the data member of :c:type:`neoisp_ext_params_s` to initialise a
 * union variable.
 *
 * @header:		Pointer to the shared header struct embedded as the
 *			first member of all the possible other members (except
 *			@data). This member would be accessed first and the type
 *			field checked to determine which of the other members
 *			should be accessed.
 * @pipe_conf:		For header->type == NEOISP_PARAM_BLK_PIPE_CONF,
 * @head_color:		For header->type == NEOISP_PARAM_BLK_HEAD_COLOR,
 * @hdr_decompress0:	For header->type == NEOISP_PARAM_BLK_HDR_DECOMPRESS0,
 * @hdr_decompress1:	For header->type == NEOISP_PARAM_BLK_HDR_DECOMPRESS1,
 * @obwb:		For header->type == NEOISP_PARAM_BLK_OBWB0 or
 *			header->type == NEOISP_PARAM_BLK_OBWB1 or
 *			header->type == NEOISP_PARAM_BLK_OBWB2,
 * @hdr_merge:		For header->type == NEOISP_PARAM_BLK_HDR_MERGE,
 * @rgbir:		For header->type == NEOISP_PARAM_BLK_RGBIR,
 * @stat:		For header->type == NEOISP_PARAM_BLK_STAT,
 * @ir_compress:	For header->type == NEOISP_PARAM_BLK_IR_COMPRESS,
 * @bnr:		For header->type == NEOISP_PARAM_BLK_BNR,
 * @vignetting_ctrl:	For header->type == NEOISP_PARAM_BLK_VIG_CTRL,
 * @ctemp:		For header->type == NEOISP_PARAM_BLK_CTEMP,
 * @demosaic:		For header->type == NEOISP_PARAM_BLK_DEMOSAIC,
 * @rgb2yuv:		For header->type == NEOISP_PARAM_BLK_RGB2YUV,
 * @drc:		For header->type == NEOISP_PARAM_BLK_DR_COMP,
 * @nrc:		For header->type == NEOISP_PARAM_BLK_NR,
 * @afc:		For header->type == NEOISP_PARAM_BLK_AF,
 * @eec:		For header->type == NEOISP_PARAM_BLK_EE,
 * @dfc:		For header->type == NEOISP_PARAM_BLK_DF,
 * @convmed:		For header->type == NEOISP_PARAM_BLK_CONVMED,
 * @cas:		For header->type == NEOISP_PARAM_BLK_CAS,
 * @gcm:		For header->type == NEOISP_PARAM_BLK_GCM,
 * @vignetting_table:	For header->type == NEOISP_PARAM_BLK_VIG_TABLE,
 * @drc_global_tonemap:	For header->type == NEOISP_PARAM_BLK_DRC_GLO_TMAP,
 * @drc_local_tonemap:	For header->type == NEOISP_PARAM_BLK_DRC_LOC_TMAP,
 * @data:		Allows easy initialization of a union variable with a
 *			pointer into a u8 array.
 */
union neoisp_params_block_u {
	struct neoisp_ext_params_block_header_s header;
	struct neoisp_pipe_conf_cfg_s pipe_conf;
	struct neoisp_head_color_cfg_s head_color;
	struct neoisp_hdr_decompress0_cfg_s hdr_decompress0;
	struct neoisp_hdr_decompress1_cfg_s hdr_decompress1;
	struct neoisp_obwb_cfg_s obwb;
	struct neoisp_hdr_merge_cfg_s hdr_merge;
	struct neoisp_rgbir_cfg_s rgbir;
	struct neoisp_stat_cfg_s stat;
	struct neoisp_ir_compress_cfg_s ir_compress;
	struct neoisp_bnr_cfg_s bnr;
	struct neoisp_vignetting_ctrl_cfg_s vignetting_ctrl;
	struct neoisp_ctemp_cfg_s ctemp;
	struct neoisp_demosaic_cfg_s demosaic;
	struct neoisp_rgb2yuv_cfg_s rgb2yuv;
	struct neoisp_dr_comp_cfg_s drc;
	struct neoisp_nr_cfg_s nrc;
	struct neoisp_af_cfg_s afc;
	struct neoisp_ee_cfg_s eec;
	struct neoisp_df_cfg_s dfc;
	struct neoisp_convmed_cfg_s convmed;
	struct neoisp_cas_cfg_s cas;
	struct neoisp_gcm_cfg_s gcm;
	struct neoisp_vignetting_table_mem_params_s vignetting_table;
	struct neoisp_drc_global_tonemap_mem_params_s drc_global_tonemap;
	struct neoisp_drc_local_tonemap_mem_params_s drc_local_tonemap;
	u8 data[sizeof(struct neoisp_pipe_conf_cfg_es)];
};

union neoisp_stats_block_u {
	struct neoisp_ext_stats_block_header_s header;
	struct neoisp_ctemp_reg_stats_es rctemp;
	struct neoisp_drc_reg_stats_es rdrc;
	struct neoisp_af_reg_stats_es raf;
	struct neoisp_bnr_reg_stats_es rbnr;
	struct neoisp_nr_reg_stats_es rnr;
	struct neoisp_ee_reg_stats_es ree;
	struct neoisp_df_reg_stats_es rdf;
	struct neoisp_ctemp_mem_stats_es mctemp;
	struct neoisp_rgbir_mem_stats_es mrgbir;
	struct neoisp_hist_mem_stats_es mhist;
	struct neoisp_drc_mem_stats_es mdrc;
};

/*
 * Functions
 */
void neoisp_update_context_packetizer(struct neoisp_node_group_s *node_group);
void neoisp_update_context_pipe_conf(struct neoisp_node_group_s *node_group);

void neoisp_upload_context(struct neoisp_dev_s *neoispd);
void neoisp_update_context_w_user_params(struct neoisp_dev_s *neoispd);
void neoisp_update_context_buf_addr(struct neoisp_dev_s *neoispd);
void neoisp_update_context_monochrome_fmt(struct neoisp_dev_s *neoispd,
					  struct neoisp_context_s *context,
					  u32 pixfmt);
void neoisp_update_context_head_color(struct neoisp_dev_s *neoispd,
				      struct neoisp_context_s *context,
				      u32 pixfmt);
void neoisp_update_context_gcm(struct neoisp_dev_s *neoispd,
			       struct neoisp_context_s *context,
			       enum v4l2_colorspace cspace,
			       enum v4l2_xfer_func xfer,
			       enum v4l2_ycbcr_encoding enc,
			       enum v4l2_quantization quant);
void neoisp_update_context_hdr_mode(struct neoisp_dev_s *neoispd,
				    struct neoisp_context_s *context);
void neoisp_get_stats(struct neoisp_dev_s *neoispd,
		      struct neoisp_buffer_s *buf);

#endif /* NEOISP_CTX_H */
