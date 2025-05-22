// Copyright 2025, rcelyte
// SPDX-License-Identifier: BSL-1.0

#include "ipc_socket.h"
#include "ipc_message.h"

#include "os/os_time.h"
#include "shared/ipc_message_channel.h"
#include "util/u_file.h"

#include <endian.h>
#include <errno.h>
#include <linux/un.h>
#include <netinet/in.h>
#include <poll.h>
#include <sched.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

static bool
ipc_socket_ensure_capacity(struct ipc_socket *const state, const uint32_t capacity)
{
	if (state->buffer_cap >= capacity) {
		return true;
	}
	if (capacity > 0x100000u) {
		U_LOG_IFL_E(state->log_level, "packet too large");
		return false;
	}
	uint8_t *const new_buffer = realloc(state->buffer, capacity);
	if (new_buffer == NULL) {
		U_LOG_IFL_E(state->log_level, "realloc failed");
		return false;
	}
	state->buffer = new_buffer;
	state->buffer_cap = capacity;
	return true;
}

void
ipc_socket_init(struct ipc_socket *const state, const enum u_logging_level log_level)
{
	*state = (struct ipc_socket){
	    .ipc_handle = XRT_IPC_HANDLE_INVALID,
	    .log_level = log_level,
	    .timestamp = (int64_t)os_monotonic_get_ns(),
	};
	ipc_socket_ensure_capacity(state, 0x1000); // optimistic allocation to avoid reallocs during receive
}

static void
ipc_socket_close(struct ipc_socket *const state)
{
	struct ipc_message_channel channel = {
	    .ipc_handle = atomic_exchange(&state->ipc_handle, XRT_IPC_HANDLE_INVALID),
	    .log_level = state->log_level,
	};
	if (channel.ipc_handle == XRT_IPC_HANDLE_INVALID) {
		return;
	}
	shutdown(channel.ipc_handle, SHUT_RDWR); // unblock `ipc_socket_wait()`
	_Static_assert(sizeof(state->reference.count) == sizeof(volatile _Atomic(int)), "");
	while (atomic_load((volatile _Atomic(int) *)&state->reference.count) != 0) {
		sched_yield();
	}
	ipc_message_channel_close(&channel);
}

void
ipc_socket_destroy(struct ipc_socket *const state)
{
	ipc_socket_close(state);
	free(state->buffer);
	state->buffer = NULL;
	state->buffer_cap = 0;
}

static bool
path_is_socket(const char path[const])
{
	struct stat result = {0};
	return stat(path, &result) == 0 && S_ISSOCK(result.st_mode);
}

bool
ipc_socket_connect(struct ipc_socket *const state,
                   const char runtime_path[const],
                   const char fallback_path[const],
                   char path_out[const],
                   const size_t path_cap)
{
	ipc_socket_close(state);
	const xrt_ipc_handle_t ipc_handle = socket(AF_UNIX, SOCK_STREAM, 0);
	if (ipc_handle == XRT_IPC_HANDLE_INVALID) {
		U_LOG_IFL_E(state->log_level, "socket() failed");
		return false;
	}
	struct sockaddr_un addr = {
	    .sun_family = AF_UNIX,
	};
	ssize_t path_len = u_file_get_path_in_runtime_dir(runtime_path, addr.sun_path, sizeof(addr.sun_path));
	if (path_len <= 0 || (size_t)path_len >= sizeof(addr.sun_path)) {
		U_LOG_IFL_E(state->log_level, "u_file_get_path_in_runtime_dir() failed");
		goto fail;
	}
	if (!path_is_socket(addr.sun_path)) {
		U_LOG_IFL_W(state->log_level, "path not found: %s", addr.sun_path);
		const char *env;
		if ((env = getenv("XDG_DATA_HOME")) != NULL) {
			path_len = snprintf(addr.sun_path, sizeof(addr.sun_path), "%s/%s", env, fallback_path);
		} else if ((env = getenv("HOME")) != NULL) {
			path_len =
			    snprintf(addr.sun_path, sizeof(addr.sun_path), "%s/.local/share/%s", env, fallback_path);
		} else {
			path_len = 0;
		}
		if (path_len <= 0 || (size_t)path_len >= sizeof(addr.sun_path)) {
			U_LOG_IFL_E(state->log_level, "failed to resolve SlimeVR socket path");
			goto fail;
		}
		if (!path_is_socket(addr.sun_path)) {
			U_LOG_IFL_E(state->log_level, "path not found: %s", addr.sun_path);
			goto fail;
		}
	}
	if (connect(ipc_handle, (const struct sockaddr *)&addr, sizeof(addr)) != 0) {
		U_LOG_IFL_E(state->log_level, "connect() failed: %s", strerror(errno));
		goto fail;
	}
	if (path_cap >= 1) {
		if ((size_t)path_len >= path_cap) {
			path_len = path_cap - 1;
		}
		memcpy(path_out, addr.sun_path, path_len + 1);
	}
	atomic_store(&state->ipc_handle, ipc_handle);
	return true;
fail:
	close(ipc_handle);
	return false;
}

// release reference with `xrt_reference_dec(&state->reference);`
static bool
ipc_socket_reference_channel(struct ipc_socket *const state, struct ipc_message_channel *const channel_out)
{
	xrt_reference_inc(&state->reference);
	const xrt_ipc_handle_t ipc_handle = atomic_load(&state->ipc_handle);
	if (ipc_handle == XRT_IPC_HANDLE_INVALID) {
		xrt_reference_dec(&state->reference);
		return false;
	}
	*channel_out = (struct ipc_message_channel){
	    .ipc_handle = ipc_handle,
	    .log_level = state->log_level,
	};
	return true;
}

bool
ipc_socket_wait(struct ipc_socket *const state)
{
	struct ipc_message_channel channel;
	if (!ipc_socket_reference_channel(state, &channel)) {
		return false;
	}
	struct pollfd fd = {channel.ipc_handle, POLLIN, 0};
	const bool result = poll(&fd, 1, -1) != -1 || errno == EINTR;
	xrt_reference_dec(&state->reference);
	return result;
}

bool
ipc_socket_send_raw(struct ipc_socket *const state, const uint8_t packet[const], const uint32_t packet_len)
{
	struct ipc_message_channel channel;
	if (!ipc_socket_reference_channel(state, &channel)) {
		return false;
	}
	const xrt_result_t result = ipc_send(&channel, packet, packet_len);
	xrt_reference_dec(&state->reference);
	return result == XRT_SUCCESS;
}

static bool
recv_nonblock(const struct ipc_message_channel channel,
              uint8_t buffer[const],
              uint32_t *const head,
              const uint32_t buffer_cap)
{
	// TODO: use a platform agnostic function like `ipc_receive()`, but with support for partial reads
	const ssize_t length = recv(channel.ipc_handle, &buffer[*head], buffer_cap - *head, MSG_DONTWAIT);
	if (length < 0) {
		if (errno == EAGAIN) {
			return true;
		}
		U_LOG_IFL_E(channel.log_level, "recv() failed: %s", strerror(errno));
		return false;
	}
	if (length > buffer_cap - *head) {
		U_LOG_IFL_E(channel.log_level, "recv() returned invalid length");
		return false;
	}
	*head += (size_t)length;
	return true;
}

uint32_t
ipc_socket_receive(struct ipc_socket *const state)
{
	struct ipc_message_channel channel;
	if (!ipc_socket_reference_channel(state, &channel)) {
		return 0;
	}
	if (state->head == state->buffer_len) {
		state->buffer_len = 0;
		state->head = 0;
	}
	if (state->buffer_len == 0) {
		struct ipc_message header;
		if (!ipc_socket_ensure_capacity(state, sizeof(header)) ||
		    !recv_nonblock(channel, state->buffer, &state->head, sizeof(header))) {
			goto fail;
		}
		if (state->head < sizeof(header)) {
			goto unref;
		}
		memcpy(&header, state->buffer, sizeof(header));
		const uint32_t packet_length = le32toh(header.length) - sizeof(header);
		if (!ipc_socket_ensure_capacity(state, packet_length)) {
			goto fail;
		}
		state->buffer_len = packet_length;
		state->head = 0;
		state->timestamp = (int64_t)os_monotonic_get_ns();
	}
	if (!recv_nonblock(channel, state->buffer, &state->head, state->buffer_len)) {
		goto fail;
	}
unref:
	xrt_reference_dec(&state->reference);
	return (state->head < state->buffer_len) ? 0 : state->buffer_len;
fail:
	xrt_reference_dec(&state->reference);
	ipc_socket_destroy(state);
	return 0;
}
