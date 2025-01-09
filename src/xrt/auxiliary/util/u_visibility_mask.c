// Copyright 2023, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief Visibility mask utilitary
 * @author Simon Zeni <simon.zeni@collabora.com>
 * @ingroup aux_util
 */

#include "math/m_mathinclude.h"

#include "u_misc.h"
#include "u_visibility_mask.h"
#include "u_logging.h"


#include <string.h>

static const struct xrt_vec2 vertices_hidden[] = {
    {1.0, 0.75},   {1.0, 1.0},   {0.75, 1.0},   {-1.0, 1.0},  {-1.0, 0.75}, {-0.75, 1.0},
    {-1.0, -0.75}, {-1.0, -1.0}, {-0.75, -1.0}, {0.75, -1.0}, {1.0, -1.0},  {1.0, -0.75},
};

static const uint32_t indices_hidden[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};

static const struct xrt_vec2 vertices_visible[] = {
    {1.f, -.75f}, {.75f, -1.f}, {-.75f, -1.f}, {-1.f, -.75f}, {-1.f, .75f},
    {-.75f, 1.f}, {.75f, 1.f},  {1.f, .75f},   {0.f, 0.f},
};

static const uint32_t indices_visible[] = {
    8, 2, 1, 3, 2, 8, 8, 1, 0, 6, 8, 7, 4, 8, 5, 8, 0, 7, 5, 8, 6, 4, 3, 8,
};

static const struct xrt_vec2 vertices_line[] = {
    {-.75f, -1.f}, {.75f, -1.f}, {1.f, -.75f}, {1.f, .75f}, {.75f, 1.f}, {-.75f, 1.f}, {-1.f, .75f}, {-1.f, -.75f},
};

static const uint32_t indices_line[] = {0, 1, 2, 3, 4, 5, 6, 7};

void
u_visibility_mask_get_default(enum xrt_visibility_mask_type type,
                              const struct xrt_fov *fov,
                              struct xrt_visibility_mask *out_mask)
{
	const struct xrt_vec2 *vertices = NULL;
	const uint32_t *indices = NULL;

	switch (type) {
	case XRT_VISIBILITY_MASK_TYPE_HIDDEN_TRIANGLE_MESH:
		out_mask->vertex_count = ARRAY_SIZE(vertices_hidden);
		out_mask->index_count = ARRAY_SIZE(indices_hidden);
		vertices = vertices_hidden;
		indices = indices_hidden;
		break;
	case XRT_VISIBILITY_MASK_TYPE_VISIBLE_TRIANGLE_MESH:
		out_mask->vertex_count = ARRAY_SIZE(vertices_visible);
		out_mask->index_count = ARRAY_SIZE(indices_visible);
		vertices = vertices_visible;
		indices = indices_visible;
		break;
	case XRT_VISIBILITY_MASK_TYPE_LINE_LOOP:
		out_mask->vertex_count = ARRAY_SIZE(vertices_line);
		out_mask->index_count = ARRAY_SIZE(indices_line);
		vertices = vertices_line;
		indices = indices_line;
		break;
	}

	if (out_mask->vertices == NULL || out_mask->indices == NULL) {
		return;
	}

	const struct xrt_fov copy = *fov;

	const double tan_left = tan(copy.angle_left);
	const double tan_right = tan(copy.angle_right);

	const double tan_down = tan(copy.angle_down);
	const double tan_up = tan(copy.angle_up);

	const double tan_half_width = (tan_right - tan_left);
	const double tan_half_height = (tan_up - tan_down);

	const double tan_offset_x = ((tan_right + tan_left) - tan_half_width) / 2;
	const double tan_offset_y = (-(tan_up + tan_down) - tan_half_height) / 2;

	for (uint32_t i = 0; i < out_mask->vertex_count; i++) {
		const struct xrt_vec2 *v = &vertices[i];
		struct xrt_vec2 *dst = &out_mask->vertices[i];

		// Yes this is really the simplest form, WolframAlpha agrees.
		dst->x = (v->x * 0.5 + 0.5) * tan_half_width + tan_offset_x;
		dst->y = (v->y * 0.5 + 0.5) * tan_half_height + tan_offset_y;
	}

	memcpy(out_mask->indices, indices, sizeof(uint32_t) * out_mask->index_count);
	return;
}
