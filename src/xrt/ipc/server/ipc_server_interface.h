// Copyright 2020-2023, Collabora, Ltd.
// Copyright 2024-2025, NVIDIA CORPORATION.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Interface for IPC server code.
 * @author Pete Black <pblack@collabora.com>
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @author Rylie Pavlik <rylie.pavlik@collabora.com>
 * @ingroup ipc_server
 */

#include "xrt/xrt_config_os.h"
#include "xrt/xrt_results.h"

#include "util/u_debug_gui.h"


#ifdef __cplusplus
extern "C" {
#endif


struct ipc_server;

/*!
 * Information passed into the IPC server main function, used for customization
 * of the IPC server.
 *
 * @ingroup ipc_server
 */
struct ipc_server_main_info
{
	//! Information passed onto the debug gui.
	struct u_debug_gui_create_info udgci;
};

/*!
 *
 * @ingroup ipc_server
 */
struct ipc_server_callbacks
{
	/*!
	 * The IPC server failed to init.
	 *
	 * param xret The error code generated during init.
	 * param data User data given passed into the main function.
	 */
	void (*init_failed)(xrt_result_t xret, void *data);

	/*!
	 * The service has completed init and is entering its mainloop.
	 *
	 * param s     The IPC server.
	 * param xinst Instance that was created by the IPC server.
	 * param data  User data given passed into the main function.
	 */
	void (*mainloop_entering)(struct ipc_server *s, struct xrt_instance *xinst, void *data);

	/*!
	 * The service has entered the mainloop,
	 * which means it has successfully started.
	 *
	 * param s     The IPC server.
	 * param xinst Instance that was created by the IPC server.
	 * param data  User data given passed into the main function.
	 */
	void (*mainloop_left)(struct ipc_server *s, struct xrt_instance *xinst, void *data);
};

/*!
 * Common main function for starting the IPC service.
 *
 * @ingroup ipc_server
 */
int
ipc_server_main_common(const struct ipc_server_main_info *ismi, const struct ipc_server_callbacks *iscb, void *data);


#ifndef XRT_OS_ANDROID

/*!
 * Main entrypoint to the compositor process.
 *
 * @ingroup ipc_server
 */
int
ipc_server_main(int argc, char **argv, const struct ipc_server_main_info *ismi);

#endif


#ifdef __cplusplus
}
#endif
