// Copyright 2019-2023, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  The thing that binds all of the OpenXR driver together.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 */

#include "xrt/xrt_config_build.h"

#include "util/u_trace_marker.h"


#ifdef XRT_FEATURE_IPC_CLIENT

// Insert the on load constructor to setup trace marker.
U_TRACE_TARGET_SETUP(U_TRACE_WHICH_OPENXR)

#include "xrt/xrt_instance.h"
#include "client/ipc_client_interface.h"


xrt_result_t
xrt_instance_create(struct xrt_instance_info *ii, struct xrt_instance **out_xinst)
{
	u_trace_marker_init();

	XRT_TRACE_MARKER();

	return ipc_instance_create(ii, out_xinst);
}

#else

// Insert the on load constructor to setup trace marker.
U_TRACE_TARGET_SETUP(U_TRACE_WHICH_SERVICE)

/*
 * For a non-service runtime, xrt_instance_create is defined in target_instance
 * helper lib, so we just have a placeholder symbol below to silence warnings about
 * empty translation units.
 */
#include <xrt/xrt_compiler.h>
XRT_MAYBE_UNUSED static const int PLACEHOLDER = 42;

#endif
