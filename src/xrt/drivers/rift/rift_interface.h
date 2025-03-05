// Copyright 2025, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Interface to Oculus Rift driver code.
 * @author Beyley Cardellion <ep1cm1n10n123@gmail.com>
 * @ingroup drv_rift
 */

#pragma once

#include "xrt/xrt_device.h"
#include "xrt/xrt_defines.h"
#include "xrt/xrt_prober.h"

#include "os/os_hid.h"

#include "util/u_device.h"
#include "util/u_logging.h"

#include <stdlib.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

#define REPORT_MAX_SIZE 69 // max size of a feature report (FEATURE_REPORT_CALIBRATE)

enum rift_feature_reports
{
	// DK1
	FEATURE_REPORT_CONFIG = 2,         // get + set
	FEATURE_REPORT_CALIBRATE = 3,      // get + set
	FEATURE_REPORT_RANGE = 4,          // get + set
	FEATURE_REPORT_REGISTER = 5,       // get + set
	FEATURE_REPORT_DFU = 6,            // get + set
	FEATURE_REPORT_DK1_KEEP_ALIVE = 8, // get + set
	FEATURE_REPORT_DISPLAY_INFO = 9,   // get + set
	FEATURE_REPORT_SERIAL = 10,        // get + set
	                            // DK2
	FEATURE_REPORT_TRACKING = 12,        // get + set
	FEATURE_REPORT_DISPLAY = 13,         // get + set
	FEATURE_REPORT_MAG_CALIBRATION = 14, // get + set
	FEATURE_REPORT_POS_CALIBRATION = 15, // get + set
	FEATURE_REPORT_CUSTOM_PATTERN = 16,  // get + set
	FEATURE_REPORT_KEEPALIVE_MUX = 17,   // get + set
	FEATURE_REPORT_MANUFACTURING = 18,   // get + set
	FEATURE_REPORT_UUID = 19,            // get + set
	FEATURE_REPORT_TEMPERATURE = 20,     // get + set
	FEATURE_REPORT_GYROOFFSET = 21,      // get only
	FEATURE_REPORT_LENS_DISTORTION = 22, // get + set
};

enum rift_config_report_flags
{
	// output the sample data raw from the sensors without converting them to known units
	RIFT_CONFIG_REPORT_USE_RAW = 1,
	// internal test mode for calibrating zero rate drift on gyro
	RIFT_CONFIG_REPORT_INTERNAL_CALIBRATION = 1 << 1,
	// use the calibration parameters stored on the device
	RIFT_CONFIG_REPORT_USE_CALIBRATION = 1 << 2,
	// recalibrate the gyro zero rate offset when the device is stationary
	RIFT_CONFIG_REPORT_AUTO_CALIBRATION = 1 << 3,
	// stop sending IN reports when the device has stopped moving for Interval milliseconds
	RIFT_CONFIG_REPORT_MOTION_KEEP_ALIVE = 1 << 4,
	// stop sending IN reports when the device has stopped recieving feature reports for Interval milliseconds
	RIFT_CONFIG_REPORT_COMMAND_KEEP_ALIVE = 1 << 5,
	// output the IN report data in the coordinate system used by LibOVR relative to the tracker, otherwise, report
	// in the coordinate system of the device
	RIFT_CONFIG_REPORT_USE_SENSOR_COORDINATES = 1 << 6,
	// override the power state of the USB hub, forcing it to act as if the external power source is connected (DK2
	// only, does nothing on DK1)
	RIFT_CONFIG_REPORT_OVERRIDE_POWER = 1 << 7,
};

enum rift_distortion_type
{
	RIFT_DISTORTION_TYPE_DIMS = 1,
	RIFT_DISTORTION_TYPE_K = 2,
};

enum rift_lens_type
{
	// firmware indirectly states lens type A is 0
	RIFT_LENS_TYPE_A = 0,
	// firmware does not state what lens type B is, 1 is an educated guess
	RIFT_LENS_TYPE_B = 1,
};

#define CATMULL_COEFFICIENTS 11
#define CHROMATIC_ABBERATION_COEFFEICENT_COUNT 4

enum rift_lens_distortion_version
{
	// no distortion data is stored
	RIFT_LENS_DISTORTION_NONE = 0,
	// standard distortion matrix
	RIFT_LENS_DISTORTION_LCSV_CATMULL_ROM_10_VERSION_1 = 1,
};

/*
 *
 * Packed structs for USB communication (borrowed from Rokid driver)
 *
 */

#if defined(__GNUC__)
#define RIFT_PACKED __attribute__((packed))
#else
#define RIFT_PACKED
#endif /* __GNUC__ */

#if defined(_MSC_VER)
#pragma pack(push, 1)
#endif

struct rift_config_report
{
	uint16_t command_id;
	uint8_t config_flags;
	uint8_t interval;
	// sample rate of the IMU, always 1000hz on DK1/DK2, read-only
	uint16_t sample_rate;
} RIFT_PACKED;

struct rift_display_info_report
{
	uint16_t command_id;
	uint8_t distortion_type;
	// the horizontal resolution of the display, in pixels
	uint16_t resolution_x;
	// the vertical resolution of the display, in pixels
	uint16_t resolution_y;
	// width in micrometers
	uint32_t display_width;
	// height in micrometers
	uint32_t display_height;
	// the vertical center of the display, in micrometers
	uint32_t center_v;
	// the separation between the two lenses, in micrometers
	uint32_t lens_separation;
	uint32_t lens_distance_l;
	uint32_t lens_distance_r;
	float distortion[6];
} RIFT_PACKED;

struct rift_catmull_rom_distortion_report_data
{
	// eye relief setting, in micrometers from front surface of lens
	uint16_t eye_relief;
	// the k coeffecients of the distortion
	uint16_t k[CATMULL_COEFFICIENTS];
	uint16_t max_r;
	uint16_t meters_per_tan_angle_at_center;
	uint16_t chromatic_abberation[CHROMATIC_ABBERATION_COEFFEICENT_COUNT];
	uint8_t unused[14];
} RIFT_PACKED;

struct rift_lens_distortion_report
{
	uint16_t command_id;
	// the amount of distortions on this device
	uint8_t num_distortions;
	// the index of this distortion in the devices array
	uint8_t distortion_idx;
	// unused bitmask field
	uint8_t bitmask;
	// the type of the lenses
	uint16_t lens_type;
	// the version of the lens distortion data
	uint16_t distortion_version;

	union {
		struct rift_catmull_rom_distortion_report_data lcsv_catmull_rom_10;
	} RIFT_PACKED data;
} RIFT_PACKED;

#define IN_REPORT_DK2 11
struct dk2_report_keepalive_mux
{
	uint16_t command;
	uint8_t in_report;
	uint16_t interval;
} RIFT_PACKED;

#if defined(_MSC_VER)
#pragma pack(pop)
#endif

struct rift_catmull_rom_distortion_data
{
	// the k coeffecients of the distortion
	float k[CATMULL_COEFFICIENTS];
	float max_r;
	float meters_per_tan_angle_at_center;
	float chromatic_abberation[CHROMATIC_ABBERATION_COEFFEICENT_COUNT];
};

struct rift_lens_distortion
{
	// the version of the lens distortion data
	uint16_t distortion_version;

	union {
		struct rift_catmull_rom_distortion_data lcsv_catmull_rom_10;
	} RIFT_PACKED data;
};

enum rift_variant
{
	RIFT_VARIANT_DK1,
	RIFT_VARIANT_DK2,
};

#define OCULUS_VR_VID 0x2833

#define OCULUS_DK2_PID 0x0021

/*!
 * Probing function for Oculus Rift devices.
 *
 * @ingroup drv_rift
 * @see xrt_prober_found_func_t
 */
int
rift_found(struct xrt_prober *xp,
           struct xrt_prober_device **devices,
           size_t device_count,
           size_t index,
           cJSON *attached_data,
           struct xrt_device **out_xdev);

/*!
 * A rift HMD device.
 *
 * @implements xrt_device
 */
struct rift_hmd
{
	struct xrt_device base;

	struct xrt_pose pose;

	enum u_logging_level log_level;

	// has built-in mutex so thread safe
	struct m_relation_history *relation_hist;

	struct os_hid_device *hid_dev;

	int64_t last_keepalive_time;
	enum rift_variant variant;
	struct rift_config_report config;
	struct rift_display_info_report display_info;

	struct rift_lens_distortion *lens_distortions;
	uint16_t num_lens_distortions;
};

struct rift_hmd *
rift_hmd_create(struct os_hid_device *dev, enum rift_variant variant, char *device_name, char *serial_number);

/*!
 * @dir drivers/rift
 *
 * @brief @ref drv_rift files.
 */

#ifdef __cplusplus
}
#endif
