// Copyright 2020-2024, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Small cli application to control IPC service.
 * @author Pete Black <pblack@collabora.com>
 * @ingroup ipc
 */

#include "util/u_file.h"

#include "client/ipc_client.h"
#include "client/ipc_client_connection.h"

#include "ipc_client_generated.h"

#include <getopt.h>
#include <ctype.h>


#define P(...) fprintf(stdout, __VA_ARGS__)
#define PE(...) fprintf(stderr, __VA_ARGS__)

typedef enum op_mode
{
	MODE_GET,
	MODE_SET_PRIMARY,
	MODE_SET_FOCUSED,
	MODE_TOGGLE_IO,
	MODE_RECENTER,
	MODE_SET_BRIGHTNESS,
} op_mode_t;


int
get_mode(struct ipc_connection *ipc_c)
{
	struct ipc_client_list clients;

	xrt_result_t r;

	r = ipc_call_system_get_clients(ipc_c, &clients);
	if (r != XRT_SUCCESS) {
		PE("Failed to get client list.\n");
		exit(1);
	}

	P("Clients:\n");
	for (uint32_t i = 0; i < clients.id_count; i++) {
		uint32_t id = clients.ids[i];

		struct ipc_app_state cs;
		r = ipc_call_system_get_client_info(ipc_c, id, &cs);
		if (r != XRT_SUCCESS) {
			PE("Failed to get client info for client %d.\n", id);
			return 1;
		}

		P("\tid: %d"
		  "\tact: %d"
		  "\tdisp: %d"
		  "\tfoc: %d"
		  "\tio: %d"
		  "\tovly: %d"
		  "\tz: %d"
		  "\tpid: %d"
		  "\t%s\n",
		  clients.ids[i],     //
		  cs.session_active,  //
		  cs.session_visible, //
		  cs.session_focused, //
		  cs.io_active,       //
		  cs.session_overlay, //
		  cs.z_order,         //
		  cs.pid,             //
		  cs.info.application_name);
	}

	P("\nDevices:\n");
	for (uint32_t i = 0; i < ipc_c->ism->isdev_count; i++) {
		struct ipc_shared_device *isdev = &ipc_c->ism->isdevs[i];
		P("\tid: %d"
		  "\tname: %d"
		  "\t\"%s\"\n",
		  i,           //
		  isdev->name, //
		  isdev->str); //
	}

	return 0;
}

int
set_primary(struct ipc_connection *ipc_c, int client_id)
{
	xrt_result_t r;

	r = ipc_call_system_set_primary_client(ipc_c, client_id);
	if (r != XRT_SUCCESS) {
		PE("Failed to set active client to %d.\n", client_id);
		return 1;
	}

	return 0;
}

int
set_focused(struct ipc_connection *ipc_c, int client_id)
{
	xrt_result_t r;

	r = ipc_call_system_set_focused_client(ipc_c, client_id);
	if (r != XRT_SUCCESS) {
		PE("Failed to set focused client to %d.\n", client_id);
		return 1;
	}

	return 0;
}

int
toggle_io(struct ipc_connection *ipc_c, int client_id)
{
	xrt_result_t r;

	r = ipc_call_system_toggle_io_client(ipc_c, client_id);
	if (r != XRT_SUCCESS) {
		PE("Failed to toggle io for client %d.\n", client_id);
		return 1;
	}

	return 0;
}

int
recenter_local_spaces(struct ipc_connection *ipc_c)
{
	xrt_result_t r;

	r = ipc_call_space_recenter_local_spaces(ipc_c);
	if (r != XRT_SUCCESS) {
		PE("Failed to recenter local spaces.\n");
		return 1;
	}

	return 0;
}

int
set_brightness(struct ipc_connection *ipc_c, int client_id, const char *value)
{
	const int length = strlen(value);
	if (length == 0) {
		return 1;
	}

	bool relative = (value[0] == '-' || value[0] == '+');

	char *end = NULL;
	float target_brightness = strtof(value, &end);

	if ((length > (end - value)) && *end == '%') {
		target_brightness /= 100.f;
	}

	xrt_result_t r =
	    ipc_call_device_set_brightness(ipc_c, client_id, target_brightness, relative);

	IPC_CHK_AND_RET(ipc_c, r, "ipc_call_device_set_brightness");

	float out_brightness;
	r = ipc_call_device_get_brightness(ipc_c, client_id, &out_brightness);

	IPC_CHK_AND_RET(ipc_c, r, "ipc_call_device_get_brightness");

	if (relative || out_brightness != target_brightness) {
		P("Set brightness to %d%%\n", (int)(out_brightness * 100));
	}

	return 0;
}

enum LongOptions
{
	OPTION_CLIENT = 100,
	OPTION_SET_BRIGHTNESS,
};

int
main(int argc, char *argv[])
{
	op_mode_t op_mode = MODE_GET;

	// parse arguments
	int c;
	int s_val = 0;
	char *brightness;

	static struct option long_options[] = {
	    {"client", required_argument, NULL, OPTION_CLIENT},
	    {"set-brightness", required_argument, NULL, OPTION_SET_BRIGHTNESS},
	    {NULL, 0, NULL, 0},
	};

	int option_index = 0;
	opterr = 0;
	while ((c = getopt_long(argc, argv, "p:f:i:c", long_options, &option_index)) != -1) {
		switch (c) {
		case 'p':
			s_val = atoi(optarg);
			op_mode = MODE_SET_PRIMARY;
			break;
		case 'f':
			s_val = atoi(optarg);
			op_mode = MODE_SET_FOCUSED;
			break;
		case 'i':
			s_val = atoi(optarg);
			op_mode = MODE_TOGGLE_IO;
			break;
		case 'c': op_mode = MODE_RECENTER; break;
		case OPTION_CLIENT: {
			s_val = atoi(optarg);
			break;
		}
		case OPTION_SET_BRIGHTNESS: {
			brightness = optarg;
			op_mode = MODE_SET_BRIGHTNESS;
			break;
		}
		case '?':
			if (optopt == 's') {
				PE("Option -s requires an id to set.\n");
			} else if (isprint(optopt)) {
				PE("Option `-%c' unknown. Usage:\n", optopt);
				PE("    -c: Recenter local spaces\n");
				PE("    -f <id>: Set focused client\n");
				PE("    -p <id>: Set primary client\n");
				PE("    -i <id>: Toggle whether client receives input\n");
				PE("    --client <id>: Set client for subsequent command\n");
				PE("    --set-brightness <[+-]brightness[%%]>: Set display brightness\n");
			} else {
				PE("Option `\\x%x' unknown.\n", optopt);
			}
			exit(1);
		default: exit(0);
		}
	}

	// Connection struct on the stack, super simple.
	struct ipc_connection ipc_c = {0};

	struct xrt_instance_info info = {
	    .app_info.application_name = "monado-ctl",
	};

	xrt_result_t xret = ipc_client_connection_init(&ipc_c, U_LOGGING_INFO, &info);
	if (xret != XRT_SUCCESS) {
		U_LOG_E("ipc_client_connection_init: %u", xret);
		return -1;
	}

	switch (op_mode) {
	case MODE_GET: exit(get_mode(&ipc_c)); break;
	case MODE_SET_PRIMARY: exit(set_primary(&ipc_c, s_val)); break;
	case MODE_SET_FOCUSED: exit(set_focused(&ipc_c, s_val)); break;
	case MODE_TOGGLE_IO: exit(toggle_io(&ipc_c, s_val)); break;
	case MODE_RECENTER: exit(recenter_local_spaces(&ipc_c)); break;
	case MODE_SET_BRIGHTNESS: exit(set_brightness(&ipc_c, s_val, brightness)); break;
	default: P("Unrecognised operation mode.\n"); exit(1);
	}

	return 0;
}
