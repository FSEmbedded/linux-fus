/* SPDX-License-Identifier: GPL-2.0 OR Linux-OpenIB */
/* Copyright (c) 2019 Mellanox Technologies. */

#ifndef __MLX5E_KTLS_H__
#define __MLX5E_KTLS_H__

#include "en.h"

#ifdef CONFIG_MLX5_EN_TLS

#define MLX5E_KTLS_STATIC_UMR_WQE_SZ \
	(offsetof(struct mlx5e_umr_wqe, tls_static_params_ctx) + \
	 MLX5_ST_SZ_BYTES(tls_static_params))
#define MLX5E_KTLS_STATIC_WQEBBS \
	(DIV_ROUND_UP(MLX5E_KTLS_STATIC_UMR_WQE_SZ, MLX5_SEND_WQE_BB))

#define MLX5E_KTLS_PROGRESS_WQE_SZ \
	(offsetof(struct mlx5e_tx_wqe, tls_progress_params_ctx) + \
	 MLX5_ST_SZ_BYTES(tls_progress_params))
#define MLX5E_KTLS_PROGRESS_WQEBBS \
	(DIV_ROUND_UP(MLX5E_KTLS_PROGRESS_WQE_SZ, MLX5_SEND_WQE_BB))

struct mlx5e_dump_wqe {
	struct mlx5_wqe_ctrl_seg ctrl;
	struct mlx5_wqe_data_seg data;
};

#define MLX5E_KTLS_DUMP_WQEBBS \
	(DIV_ROUND_UP(sizeof(struct mlx5e_dump_wqe), MLX5_SEND_WQE_BB))

enum {
	MLX5E_TLS_PROGRESS_PARAMS_AUTH_STATE_NO_OFFLOAD     = 0,
	MLX5E_TLS_PROGRESS_PARAMS_AUTH_STATE_OFFLOAD        = 1,
	MLX5E_TLS_PROGRESS_PARAMS_AUTH_STATE_AUTHENTICATION = 2,
};

enum {
	MLX5E_TLS_PROGRESS_PARAMS_RECORD_TRACKER_STATE_START     = 0,
	MLX5E_TLS_PROGRESS_PARAMS_RECORD_TRACKER_STATE_TRACKING  = 1,
	MLX5E_TLS_PROGRESS_PARAMS_RECORD_TRACKER_STATE_SEARCHING = 2,
};

struct mlx5e_ktls_offload_context_tx {
	struct tls_offload_context_tx *tx_ctx;
	struct tls12_crypto_info_aes_gcm_128 crypto_info;
	u32 expected_seq;
	u32 tisn;
	u32 key_id;
	bool ctx_post_pending;
};

struct mlx5e_ktls_offload_context_tx_shadow {
	struct tls_offload_context_tx         tx_ctx;
	struct mlx5e_ktls_offload_context_tx *priv_tx;
};

static inline void mlx5e_ktls_build_netdev(struct mlx5e_priv *priv)
{
}

static inline int mlx5e_ktls_init_rx(struct mlx5e_priv *priv)
{
	return 0;
}

static inline void mlx5e_ktls_cleanup_rx(struct mlx5e_priv *priv)
{
}

static inline int mlx5e_ktls_set_feature_rx(struct net_device *netdev, bool enable)
{
	netdev_warn(netdev, "kTLS is not supported\n");
	return -EOPNOTSUPP;
}

#endif

#endif /* __MLX5E_TLS_H__ */
