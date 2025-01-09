// Copyright 2023-2025, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Header defining visibility mask helper struct.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @author Simon Zeni <simon.zeni@collabora.com>
 * @ingroup xrt_iface
 */

#pragma once

#include "xrt/xrt_defines.h"

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * Visibility mask helper, the indices and vertices are tightly packed after
 * this struct.
 *
 * @ingroup xrt_iface
 */
struct xrt_visibility_mask
{
	enum xrt_visibility_mask_type type;
	uint32_t vertex_count;
	struct xrt_vec2 *vertices;
	uint32_t index_count;
	uint32_t *indices;
};

#ifdef __cplusplus
}
#endif
