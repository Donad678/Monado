// Copyright 2023, Shawn Wallace
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief SteamVR driver device implementation.
 * @author Shawn Wallace <yungwallace@live.com>
 * @ingroup drv_steamvr_lh
 */

#include <functional>
#include <cstring>
#include <numbers>
#include <openvr_driver.h>
#include <thread>
#include <algorithm>

#include "math/m_api.h"
#include "math/m_relation_history.h"
#include "math/m_space.h"
#include "device.hpp"
#include "interfaces/context.hpp"
#include "util/u_debug.h"
#include "util/u_device.h"
#include "util/u_hand_simulation.h"
#include "util/u_hand_tracking.h"
#include "util/u_logging.h"
#include "util/u_json.hpp"
#include "xrt/xrt_defines.h"
#include "xrt/xrt_device.h"

#include "vive/vive_poses.h"

#define DEV_ERR(...) U_LOG_IFL_E(ctx->log_level, __VA_ARGS__)
#define DEV_WARN(...) U_LOG_IFL_W(ctx->log_level, __VA_ARGS__)
#define DEV_INFO(...) U_LOG_IFL_I(ctx->log_level, __VA_ARGS__)
#define DEV_DEBUG(...) U_LOG_IFL_D(ctx->log_level, __VA_ARGS__)

DEBUG_GET_ONCE_BOOL_OPTION(lh_emulate_hand, "LH_EMULATE_HAND", true)

// Each device will have its own input class.
struct InputClass
{
	xrt_device_name name;
	const std::vector<xrt_input_name> poses;
	const std::unordered_map<std::string_view, xrt_input_name> non_poses;
};

namespace {
using namespace std::string_view_literals;
// From https://github.com/ValveSoftware/openvr/blob/master/docs/Driver_API_Documentation.md#bone-structure
enum HandSkeletonBone : int32_t
{
	eBone_Root = 0,
	eBone_Wrist,
	eBone_Thumb0,
	eBone_Thumb1,
	eBone_Thumb2,
	eBone_Thumb3,
	eBone_IndexFinger0,
	eBone_IndexFinger1,
	eBone_IndexFinger2,
	eBone_IndexFinger3,
	eBone_IndexFinger4,
	eBone_MiddleFinger0,
	eBone_MiddleFinger1,
	eBone_MiddleFinger2,
	eBone_MiddleFinger3,
	eBone_MiddleFinger4,
	eBone_RingFinger0,
	eBone_RingFinger1,
	eBone_RingFinger2,
	eBone_RingFinger3,
	eBone_RingFinger4,
	eBone_PinkyFinger0,
	eBone_PinkyFinger1,
	eBone_PinkyFinger2,
	eBone_PinkyFinger3,
	eBone_PinkyFinger4,
	eBone_Aux_Thumb,
	eBone_Aux_IndexFinger,
	eBone_Aux_MiddleFinger,
	eBone_Aux_RingFinger,
	eBone_Aux_PinkyFinger,
	eBone_Count
};

// Adding support for a new controller is a simple as adding it here.
// The key for the map needs to be the name of input profile as indicated by the lighthouse driver.
const std::unordered_map<std::string_view, InputClass> controller_classes{
    {
        "vive_controller",
        InputClass{
            XRT_DEVICE_VIVE_WAND,
            {
                XRT_INPUT_VIVE_GRIP_POSE,
                XRT_INPUT_VIVE_AIM_POSE,
                XRT_INPUT_GENERIC_PALM_POSE,
            },
            {
                {"/input/application_menu/click", XRT_INPUT_VIVE_MENU_CLICK},
                {"/input/trackpad/click", XRT_INPUT_VIVE_TRACKPAD_CLICK},
                {"/input/trackpad/touch", XRT_INPUT_VIVE_TRACKPAD_TOUCH},
                {"/input/system/click", XRT_INPUT_VIVE_SYSTEM_CLICK},
                {"/input/trigger/click", XRT_INPUT_VIVE_TRIGGER_CLICK},
                {"/input/trigger/value", XRT_INPUT_VIVE_TRIGGER_VALUE},
                {"/input/grip/click", XRT_INPUT_VIVE_SQUEEZE_CLICK},
                {"/input/trackpad", XRT_INPUT_VIVE_TRACKPAD},
            },
        },
    },
    {
        "index_controller",
        InputClass{
            XRT_DEVICE_INDEX_CONTROLLER,
            {
                XRT_INPUT_INDEX_GRIP_POSE,
                XRT_INPUT_INDEX_AIM_POSE,
                XRT_INPUT_GENERIC_PALM_POSE,
            },
            {
                {"/input/system/click", XRT_INPUT_INDEX_SYSTEM_CLICK},
                {"/input/system/touch", XRT_INPUT_INDEX_SYSTEM_TOUCH},
                {"/input/a/click", XRT_INPUT_INDEX_A_CLICK},
                {"/input/a/touch", XRT_INPUT_INDEX_A_TOUCH},
                {"/input/b/click", XRT_INPUT_INDEX_B_CLICK},
                {"/input/b/touch", XRT_INPUT_INDEX_B_TOUCH},
                {"/input/trigger/click", XRT_INPUT_INDEX_TRIGGER_CLICK},
                {"/input/trigger/touch", XRT_INPUT_INDEX_TRIGGER_TOUCH},
                {"/input/trigger/value", XRT_INPUT_INDEX_TRIGGER_VALUE},
                {"/input/grip/force", XRT_INPUT_INDEX_SQUEEZE_FORCE},
                {"/input/grip/value", XRT_INPUT_INDEX_SQUEEZE_VALUE},
                {"/input/thumbstick/click", XRT_INPUT_INDEX_THUMBSTICK_CLICK},
                {"/input/thumbstick/touch", XRT_INPUT_INDEX_THUMBSTICK_TOUCH},
                {"/input/thumbstick", XRT_INPUT_INDEX_THUMBSTICK},
                {"/input/trackpad/force", XRT_INPUT_INDEX_TRACKPAD_FORCE},
                {"/input/trackpad/touch", XRT_INPUT_INDEX_TRACKPAD_TOUCH},
                {"/input/trackpad", XRT_INPUT_INDEX_TRACKPAD},
            },
        },
    },
    {
        "vive_tracker",
        InputClass{
            XRT_DEVICE_VIVE_TRACKER,
            {
                XRT_INPUT_GENERIC_TRACKER_POSE,
            },
            {
                {"/input/power/click", XRT_INPUT_VIVE_TRACKER_SYSTEM_CLICK},
                {"/input/grip/click", XRT_INPUT_VIVE_TRACKER_SQUEEZE_CLICK},
                {"/input/application_menu/click", XRT_INPUT_VIVE_TRACKER_MENU_CLICK},
                {"/input/trigger/click", XRT_INPUT_VIVE_TRACKER_TRIGGER_CLICK},
                {"/input/thumb/click", XRT_INPUT_VIVE_TRACKER_TRACKPAD_CLICK},
            },
        },
    },
    {
        "tundra_tracker",
        InputClass{
            XRT_DEVICE_VIVE_TRACKER,
            {
                XRT_INPUT_GENERIC_TRACKER_POSE,
            },
            {
                {"/input/power/click", XRT_INPUT_VIVE_TRACKER_SYSTEM_CLICK},
                {"/input/grip/click", XRT_INPUT_VIVE_TRACKER_SQUEEZE_CLICK},
                {"/input/application_menu/click", XRT_INPUT_VIVE_TRACKER_MENU_CLICK},
                {"/input/trigger/click", XRT_INPUT_VIVE_TRACKER_TRIGGER_CLICK},
                {"/input/thumb/click", XRT_INPUT_VIVE_TRACKER_TRACKPAD_CLICK},
            },
        },
    },
};

int64_t
chrono_timestamp_ns()
{
	auto now = std::chrono::steady_clock::now().time_since_epoch();
	int64_t ts = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
	return ts;
}

// Template for calling a member function of Device from a free function
template <typename DeviceType, auto Func, typename Ret, typename... Args>
inline Ret
device_bouncer(struct xrt_device *xdev, Args... args)
{
	auto *dev = static_cast<DeviceType *>(xdev);
	return std::invoke(Func, dev, args...);
}

xrt_pose
bone_to_pose(const vr::VRBoneTransform_t &bone)
{
	return xrt_pose{xrt_quat{bone.orientation.x, bone.orientation.y, bone.orientation.z, bone.orientation.w},
	                xrt_vec3{bone.position.v[0], bone.position.v[1], bone.position.v[2]}};
}

} // namespace

HmdDevice::HmdDevice(const DeviceBuilder &builder) : Device(builder)
{
	this->name = XRT_DEVICE_GENERIC_HMD;
	this->device_type = XRT_DEVICE_TYPE_HMD;
	this->container_handle = 0;

	inputs_vec = {xrt_input{true, 0, XRT_INPUT_GENERIC_HEAD_POSE, {}}};
	this->inputs = inputs_vec.data();
	this->input_count = inputs_vec.size();

#define SETUP_MEMBER_FUNC(name) this->xrt_device::name = &device_bouncer<HmdDevice, &HmdDevice::name>
	SETUP_MEMBER_FUNC(get_view_poses);
	SETUP_MEMBER_FUNC(compute_distortion);
#undef SETUP_MEMBER_FUNC
}

ControllerDevice::ControllerDevice(vr::PropertyContainerHandle_t handle, const DeviceBuilder &builder) : Device(builder)
{
	this->device_type = XRT_DEVICE_TYPE_UNKNOWN;
	this->container_handle = handle;

#define SETUP_MEMBER_FUNC(name) this->xrt_device::name = &device_bouncer<ControllerDevice, &ControllerDevice::name>
	SETUP_MEMBER_FUNC(set_output);
	SETUP_MEMBER_FUNC(get_hand_tracking);
#undef SETUP_MEMBER_FUNC

	this->inputs_map["/skeleton/hand/left"] = &hand_tracking_inputs[XRT_HAND_LEFT];
	this->inputs_map["/skeleton/hand/right"] = &hand_tracking_inputs[XRT_HAND_RIGHT];
}

Device::~Device()
{
	m_relation_history_destroy(&relation_hist);
}

Device::Device(const DeviceBuilder &builder) : xrt_device({}), ctx(builder.ctx), driver(builder.driver)
{
	m_relation_history_create(&relation_hist);
	std::strncpy(this->serial, builder.serial, XRT_DEVICE_NAME_LEN - 1);
	this->serial[XRT_DEVICE_NAME_LEN - 1] = 0;
	this->tracking_origin = ctx.get();
	this->orientation_tracking_supported = true;
	this->position_tracking_supported = true;
	this->hand_tracking_supported = true;
	this->force_feedback_supported = false;
	this->form_factor_check_supported = false;
	this->battery_status_supported = true;

	this->xrt_device::update_inputs = &device_bouncer<Device, &Device::update_inputs, xrt_result_t>;
#define SETUP_MEMBER_FUNC(name) this->xrt_device::name = &device_bouncer<Device, &Device::name>
	SETUP_MEMBER_FUNC(get_tracked_pose);
	SETUP_MEMBER_FUNC(get_battery_status);
#undef SETUP_MEMBER_FUNC

	this->xrt_device::destroy = [](xrt_device *xdev) {
		auto *dev = static_cast<Device *>(xdev);
		dev->driver->Deactivate();
		delete dev;
	};

	init_chaperone(builder.steam_install);
}

// NOTE: No operations that would force inputs_vec or finger_inputs_vec to reallocate (such as insertion)
// should be done after this function is called, otherwise the pointers in inputs_map/finger_inputs_map
// would be invalidated.
void
ControllerDevice::set_input_class(const InputClass *input_class)
{
	// this should only be called once
	assert(inputs_vec.empty());
	this->input_class = input_class;

	// reserve to ensure our pointers don't get invalidated
	inputs_vec.reserve(input_class->poses.size() + input_class->non_poses.size() + 1);
	for (xrt_input_name input : input_class->poses) {
		inputs_vec.push_back({true, 0, input, {}});
	}
	for (const auto &[path, input] : input_class->non_poses) {
		assert(inputs_vec.capacity() >= inputs_vec.size() + 1);
		inputs_vec.push_back({true, 0, input, {}});
		inputs_map.insert({path, &inputs_vec.back()});
	}

	this->inputs = inputs_vec.data();
	this->input_count = inputs_vec.size();
}

xrt_hand
ControllerDevice::get_xrt_hand()
{
	switch (this->device_type) {
	case XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER: {
		return xrt_hand::XRT_HAND_LEFT;
	}
	case XRT_DEVICE_TYPE_RIGHT_HAND_CONTROLLER: {
		return xrt_hand::XRT_HAND_RIGHT;
	}
	default: DEV_ERR("Device %s cannot be tracked as a hand!", serial); return xrt_hand::XRT_HAND_LEFT;
	}
}

void
ControllerDevice::set_active_hand(xrt_hand hand)
{
	this->skeleton_hand = hand;
}

namespace {
xrt_quat
from_euler_angles(float x, float y, float z)
{
	const xrt_vec3 v{x, y, z};
	xrt_quat out;
	math_quat_from_euler_angles(&v, &out);
	return out;
}

constexpr float pi = std::numbers::pi_v<float>;
constexpr float frac_pi_2 = pi / 2.0f;

// OpenVR skeletal poses are defined with the palms facing each other, but OpenXR
// hand tracking is defined with the palms facing down. These per hand rotations
// are necessary to translate to what OpenXR expects.
const xrt_quat right_hand_rotate = from_euler_angles(0.0f, frac_pi_2, 0.0f);
const xrt_quat left_hand_rotate = from_euler_angles(0.0f, frac_pi_2, pi);

xrt_quat
right_wrist_rotate_init()
{
	const xrt_quat rot1 = from_euler_angles(0.0f, 0.0f, frac_pi_2);
	const xrt_quat rot2 = from_euler_angles(0.0f, pi, 0.0f);
	xrt_quat ret;
	math_quat_rotate(&rot1, &rot2, &ret);
	return ret;
}
xrt_quat
left_wrist_rotate_init()
{
	const xrt_quat rot1 = from_euler_angles(pi, 0.0f, 0.0f);
	const xrt_quat rot2 = from_euler_angles(0.0f, 0.0f, -frac_pi_2);
	xrt_quat ret;
	math_quat_rotate(&rot1, &rot2, &ret);
	return ret;
}

const xrt_quat right_wrist_rotate = right_wrist_rotate_init();
const xrt_quat left_wrist_rotate = left_wrist_rotate_init();

xrt_pose
generate_palm_pose(const xrt_pose &metacarpal_pose, const xrt_pose &proximal_pose)
{
	// OpenVR doesn't provide a palm joint, but the OpenXR palm is in the middle of
	// the metacarpal and proximal bones of the middle finger,
	// so we'll interpolate between them to generate it.
	xrt_pose pose;
	math_pose_interpolate(&metacarpal_pose, &proximal_pose, 0.5, &pose);
	// Use metacarpal orientation, because the palm shouldn't really rotate
	pose.orientation = metacarpal_pose.orientation;
	return pose;
}

} // namespace

void
ControllerDevice::set_skeleton(std::span<const vr::VRBoneTransform_t> bones,
                               xrt_hand hand,
                               bool is_simulated,
                               const char *path)
{
	assert(bones.size() == eBone_Count);
	generate_palm_pose_offset(bones, hand);
	if (!is_simulated && debug_get_bool_option_lh_emulate_hand()) {
		assert(inputs_vec.capacity() >= inputs_vec.size() + 1);
		const xrt_input_name tracker_name = (hand == XRT_HAND_RIGHT) ? XRT_INPUT_GENERIC_HAND_TRACKING_RIGHT
		                                                             : XRT_INPUT_GENERIC_HAND_TRACKING_LEFT;
		inputs_vec.push_back({true, 0, tracker_name, {}});
		inputs_map.insert({path, &inputs_vec.back()});
		this->input_count = inputs_vec.size();
		has_hand_tracking = true;
	}
}

void
ControllerDevice::generate_palm_pose_offset(std::span<const vr::VRBoneTransform_t> bones, xrt_hand hand)
{
	// The palm pose offset is generated from the OpenVR provided skeleton.
	// https://github.com/ValveSoftware/openvr/blob/master/docs/Driver_API_Documentation.md#notes-on-the-skeleton

	xrt_pose root = bone_to_pose(bones[eBone_Root]);
	xrt_pose wrist = bone_to_pose(bones[eBone_Wrist]);
	xrt_pose metacarpal = bone_to_pose(bones[eBone_MiddleFinger0]);
	xrt_pose proximal = bone_to_pose(bones[eBone_MiddleFinger1]);

	// The skeleton pose is given with the Root bone as origin.
	// To convert from this, according to OpenVR docs we transform the wrist
	// and then counter-transform the metacarpals
	xrt_pose root_inv;
	math_pose_invert(&root, &root_inv);
	math_pose_transform(&root_inv, &wrist, &wrist);
	math_pose_transform(&root, &metacarpal, &metacarpal);
	math_pose_transform(&wrist, &metacarpal, &metacarpal);
	math_pose_transform(&metacarpal, &proximal, &proximal);

	xrt_pose palm_offset = generate_palm_pose(metacarpal, proximal);
	xrt_quat palm_rotate = from_euler_angles(0.0f, 0.0f, frac_pi_2);

	switch (hand) {
	case XRT_HAND_LEFT: {
		math_quat_rotate(&palm_offset.orientation, &left_hand_rotate, &palm_offset.orientation);
		math_quat_invert(&palm_rotate, &palm_rotate);
		break;
	}
	case XRT_HAND_RIGHT: {
		math_quat_rotate(&palm_offset.orientation, &right_hand_rotate, &palm_offset.orientation);
		break;
	}
	}
	math_quat_rotate(&palm_offset.orientation, &palm_rotate, &palm_offset.orientation);

	// For controllers like the Vive Wands which can be in any hand, it will store both the left hand
	// and the right hand skeletons, so we need to store both.
	palm_offsets[hand] = palm_offset;
}

void
ControllerDevice::update_skeleton_transforms(std::span<const vr::VRBoneTransform_t> bones)
{
	if (!has_hand_tracking) {
		return;
	}

	assert(bones.size() == eBone_Count);

	xrt_hand_joint_set joint_set;
	int64_t ts;
	if (!m_relation_history_get_latest(relation_hist, &ts, &joint_set.hand_pose)) {
		return;
	}
	joint_set.is_active = true;
	auto &joints = joint_set.values.hand_joint_set_default;

	xrt_pose root = bone_to_pose(bones[eBone_Root]);
	xrt_pose wrist = bone_to_pose(bones[eBone_Wrist]);

	// Here we're doing the same transformation as seen in set_skeleton.
	xrt_pose root_inv;
	math_pose_invert(&root, &root_inv);
	math_pose_transform(&root_inv, &wrist, &wrist);

	constexpr auto valid_flags = (enum xrt_space_relation_flags)(
	    XRT_SPACE_RELATION_POSITION_VALID_BIT | XRT_SPACE_RELATION_ORIENTATION_VALID_BIT |
	    XRT_SPACE_RELATION_POSITION_TRACKED_BIT | XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT);

	xrt_pose wrist_xr = wrist;

	switch (skeleton_hand) {
	case XRT_HAND_LEFT: {
		math_quat_rotate(&wrist_xr.orientation, &left_wrist_rotate, &wrist_xr.orientation);
		break;
	}
	case XRT_HAND_RIGHT: {
		math_quat_rotate(&wrist_xr.orientation, &right_wrist_rotate, &wrist_xr.orientation);
		break;
	}
	}

	joints[XRT_HAND_JOINT_WRIST].relation.pose = wrist_xr;
	joints[XRT_HAND_JOINT_WRIST].relation.relation_flags = valid_flags;

	xrt_pose parent_pose;
	for (int joint = XRT_HAND_JOINT_THUMB_METACARPAL; joint <= XRT_HAND_JOINT_LITTLE_TIP; ++joint) {
		// Luckily openvr and openxr joint values match
		xrt_pose pose = bone_to_pose(bones[joint]);
		joints[joint].relation.relation_flags = valid_flags;

		if (u_hand_joint_is_metacarpal((xrt_hand_joint)joint)) {
			// Counter transform metacarpals
			math_pose_transform(&root, &pose, &pose);
			math_pose_transform(&wrist, &pose, &pose);
		} else {
			math_pose_transform(&parent_pose, &pose, &pose);
		}

		parent_pose = pose;

		// Rotate joint to OpenXR orientation
		switch (skeleton_hand) {
		case XRT_HAND_LEFT: math_quat_rotate(&pose.orientation, &left_hand_rotate, &pose.orientation); break;
		case XRT_HAND_RIGHT: math_quat_rotate(&pose.orientation, &right_hand_rotate, &pose.orientation); break;
		}
		joints[joint].relation.pose = pose;
	}

	joints[XRT_HAND_JOINT_PALM].relation.relation_flags = valid_flags;
	joints[XRT_HAND_JOINT_PALM].relation.pose =
	    generate_palm_pose(joints[XRT_HAND_JOINT_MIDDLE_METACARPAL].relation.pose,
	                       joints[XRT_HAND_JOINT_MIDDLE_PROXIMAL].relation.pose);

	u_hand_joints_apply_joint_width(&joint_set);
	this->joint_set = joint_set;
}

xrt_input *
Device::get_input_from_name(const std::string_view name)
{
	static const std::array ignore_inputs = {"/input/finger/index"sv, "/input/finger/middle"sv,
	                                         "/input/finger/ring"sv, "/input/finger/pinky"sv,
	                                         "/input/grip/touch"sv};

	// Return nullptr without any other output to suppress a pile of useless warnings found below.
	if (std::ranges::find(ignore_inputs, name) != std::ranges::end(ignore_inputs)) {
		return nullptr;
	}
	auto input = inputs_map.find(name);
	if (input == inputs_map.end()) {
		DEV_WARN("requested unknown input name %s for device %s", std::string(name).c_str(), serial);
		return nullptr;
	}
	return input->second;
}

void
ControllerDevice::set_haptic_handle(vr::VRInputComponentHandle_t handle)
{
	// this should only be set once
	assert(output == nullptr);
	DEV_DEBUG("setting haptic handle for %" PRIu64, handle);
	haptic_handle = handle;
	xrt_output_name name;
	switch (this->name) {
	case XRT_DEVICE_VIVE_WAND: {
		name = XRT_OUTPUT_NAME_VIVE_HAPTIC;
		break;
	}
	case XRT_DEVICE_INDEX_CONTROLLER: {
		name = XRT_OUTPUT_NAME_INDEX_HAPTIC;
		break;
	}
	case XRT_DEVICE_VIVE_TRACKER: {
		name = XRT_OUTPUT_NAME_VIVE_TRACKER_HAPTIC;
		break;
	}
	default: {
		DEV_WARN("Unknown device name (%u), haptics will not work", this->name);
		return;
	}
	}
	output = std::make_unique<xrt_output>(xrt_output{name});
	this->output_count = 1;
	this->outputs = output.get();
}

xrt_result_t
Device::update_inputs()
{
	std::lock_guard<std::mutex> lock(frame_mutex);
	ctx->maybe_run_frame(++current_frame);
	return XRT_SUCCESS;
}

void
ControllerDevice::get_hand_tracking(enum xrt_input_name name,
                                    int64_t desired_timestamp_ns,
                                    struct xrt_hand_joint_set *out_value,
                                    int64_t *out_timestamp_ns)
{
	if (!has_hand_tracking) {
		return;
	}

	*out_value = joint_set;
	*out_timestamp_ns = desired_timestamp_ns;
}

void
Device::get_pose(uint64_t at_timestamp_ns, xrt_space_relation *out_relation)
{
	m_relation_history_get(this->relation_hist, at_timestamp_ns, out_relation);
}

xrt_result_t
Device::get_battery_status(bool *out_present, bool *out_charging, float *out_charge)
{
	*out_present = this->provides_battery_status;
	*out_charging = this->charging;
	*out_charge = this->charge;
	return XRT_SUCCESS;
}

xrt_result_t
HmdDevice::get_tracked_pose(xrt_input_name name, uint64_t at_timestamp_ns, xrt_space_relation *out_relation)
{
	switch (name) {
	case XRT_INPUT_GENERIC_HEAD_POSE: Device::get_pose(at_timestamp_ns, out_relation); break;
	default: U_LOG_XDEV_UNSUPPORTED_INPUT(this, ctx->log_level, name); return XRT_ERROR_INPUT_UNSUPPORTED;
	}

	return XRT_SUCCESS;
}

xrt_result_t
ControllerDevice::get_tracked_pose(xrt_input_name name, uint64_t at_timestamp_ns, xrt_space_relation *out_relation)
{
	xrt_space_relation rel = {};
	Device::get_pose(at_timestamp_ns, &rel);

	xrt_pose pose_offset = XRT_POSE_IDENTITY;

	if (name == XRT_INPUT_GENERIC_PALM_POSE) {
		if (!palm_offsets[skeleton_hand].has_value()) {
			DEV_ERR("%s hand skeleton has not been initialized",
			        skeleton_hand == XRT_HAND_LEFT ? "left" : "right");
			*out_relation = XRT_SPACE_RELATION_ZERO;
			return XRT_SUCCESS;
		}
		pose_offset = *palm_offsets[skeleton_hand];
	} else {
		vive_poses_get_pose_offset(input_class->name, device_type, name, &pose_offset);
	}
	xrt_relation_chain relchain = {};

	m_relation_chain_push_pose(&relchain, &pose_offset);
	m_relation_chain_push_relation(&relchain, &rel);
	m_relation_chain_resolve(&relchain, out_relation);

	struct xrt_pose *p = &out_relation->pose;
	DEV_DEBUG("controller %u: GET_POSITION (%f %f %f) GET_ORIENTATION (%f, %f, %f, %f)", name, p->position.x,
	          p->position.y, p->position.z, p->orientation.x, p->orientation.y, p->orientation.z, p->orientation.w);

	return XRT_SUCCESS;
}

void
ControllerDevice::set_output(xrt_output_name name, const xrt_output_value *value)

{
	const auto &vib = value->vibration;
	if (vib.amplitude == 0.0)
		return;
	vr::VREvent_HapticVibration_t event;
	event.containerHandle = container_handle;
	event.componentHandle = haptic_handle;
	event.fDurationSeconds = (float)vib.duration_ns / 1e9f;
	// 0.0f in OpenXR means let the driver determine a frequency, but
	// in OpenVR means no haptic.
	event.fFrequency = std::max(vib.frequency, 1.0f);
	event.fAmplitude = vib.amplitude;

	ctx->add_haptic_event(event);
}

void
HmdDevice::SetDisplayEyeToHead(uint32_t unWhichDevice,
                               const vr::HmdMatrix34_t &eyeToHeadLeft,
                               const vr::HmdMatrix34_t &eyeToHeadRight)
{
	xrt_matrix_3x3 leftEye_prequat;
	xrt_matrix_3x3 rightEye_prequat;

	xrt_pose leftEye_postquat;
	xrt_pose rightEye_postquat;

	// This is a HmdMatrix34 to xrt_matrix_3x3 copy.
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			leftEye_prequat.v[i * 3 + j] = eyeToHeadLeft.m[i][j];
			rightEye_prequat.v[i * 3 + j] = eyeToHeadRight.m[i][j];
		}
	}

	math_quat_from_matrix_3x3(&leftEye_prequat, &leftEye_postquat.orientation);
	math_quat_from_matrix_3x3(&rightEye_prequat, &rightEye_postquat.orientation);
	leftEye_postquat.position.x = eyeToHeadLeft.m[0][3];
	leftEye_postquat.position.y = eyeToHeadLeft.m[1][3];
	leftEye_postquat.position.z = eyeToHeadLeft.m[2][3];

	rightEye_postquat.position.x = eyeToHeadRight.m[0][3];
	rightEye_postquat.position.y = eyeToHeadRight.m[1][3];
	rightEye_postquat.position.z = eyeToHeadRight.m[2][3];

	this->eye[0].orientation = leftEye_postquat.orientation;
	this->eye[0].position = leftEye_postquat.position;
	this->eye[1].orientation = rightEye_postquat.orientation;
	this->eye[1].position = rightEye_postquat.position;
}

void
HmdDevice::get_view_poses(const xrt_vec3 *default_eye_relation,
                          uint64_t at_timestamp_ns,
                          uint32_t view_count,
                          xrt_space_relation *out_head_relation,
                          xrt_fov *out_fovs,
                          xrt_pose *out_poses)
{
	struct xrt_vec3 eye_relation = *default_eye_relation;
	eye_relation.x = ipd;

	u_device_get_view_poses( //
	    this,                //
	    &eye_relation,       //
	    at_timestamp_ns,     //
	    view_count,          //
	    out_head_relation,   //
	    out_fovs,            //
	    out_poses);          //

	out_poses[0].orientation = this->eye[0].orientation;
	out_poses[0].position.z = this->eye[0].position.z;
	out_poses[0].position.y = this->eye[0].position.y;
	out_poses[1].orientation = this->eye[1].orientation;
	out_poses[1].position.z = this->eye[1].position.z;
	out_poses[1].position.y = this->eye[1].position.y;
}

bool
HmdDevice::compute_distortion(uint32_t view, float u, float v, xrt_uv_triplet *out_result)
{
	vr::EVREye eye = (view == 0) ? vr::Eye_Left : vr::Eye_Right;
	vr::DistortionCoordinates_t coords = this->hmd_parts->display->ComputeDistortion(eye, u, v);
	out_result->r = {coords.rfRed[0], coords.rfRed[1]};
	out_result->g = {coords.rfGreen[0], coords.rfGreen[1]};
	out_result->b = {coords.rfBlue[0], coords.rfBlue[1]};
	return true;
}

void
HmdDevice::set_hmd_parts(std::unique_ptr<Parts> parts)
{
	{
		std::lock_guard lk(hmd_parts_mut);
		hmd_parts = std::move(parts);
		this->hmd = &hmd_parts->base;
	}
	hmd_parts_cv.notify_all();
}

namespace {
xrt_quat
copy_quat(const vr::HmdQuaternion_t &quat)
{
	return xrt_quat{(float)quat.x, (float)quat.y, (float)quat.z, (float)quat.w};
}

xrt_vec3
copy_vec3(const double (&vec)[3])
{
	return xrt_vec3{(float)vec[0], (float)vec[1], (float)vec[2]};
}

xrt_pose
copy_pose(const vr::HmdQuaternion_t &orientation, const double (&position)[3])
{
	return xrt_pose{copy_quat(orientation), copy_vec3(position)};
}
} // namespace

void
Device::init_chaperone(const std::string &steam_install)
{
	static bool initialized = false;
	if (initialized)
		return;

	initialized = true;

	// Lighthouse driver seems to create a lighthousedb.json and a chaperone_info.vrchap (which is json)
	// We will use the known_universes from the lighthousedb.json to match to a universe from chaperone_info.vrchap

	using xrt::auxiliary::util::json::JSONNode;
	auto lighthousedb = JSONNode::loadFromFile(steam_install + "/config/lighthouse/lighthousedb.json");
	if (lighthousedb.isInvalid()) {
		DEV_ERR("Couldn't load lighthousedb file, playspace center will be off - was Room Setup run?");
		return;
	}
	auto chap_info = JSONNode::loadFromFile(steam_install + "/config/chaperone_info.vrchap");
	if (chap_info.isInvalid()) {
		DEV_ERR("Couldn't load chaperone info, playspace center will be off - was Room Setup run?");
		return;
	}

	JSONNode info = {};
	bool universe_found = false;

	// XXX: This may be broken if there are multiple known universes - how do we determine which to use then?
	auto known_universes = lighthousedb["known_universes"].asArray();
	for (auto &universe : known_universes) {
		const std::string id = universe["id"].asString();
		for (const JSONNode &u : chap_info["universes"].asArray()) {
			if (u["universeID"].asString() == id) {
				DEV_INFO("Found info for universe %s", id.c_str());
				info = u;
				universe_found = true;
				break;
			}
		}
		if (universe_found) {
			break;
		}
	}

	if (info.isInvalid()) {
		DEV_ERR("Couldn't find chaperone info for any known universe, playspace center will be off");
		return;
	}

	std::vector<JSONNode> translation_arr = info["standing"]["translation"].asArray();

	// If the array is missing elements, add zero.
	for (size_t i = translation_arr.size(); i < 3; i++) {
		translation_arr.push_back(JSONNode("0.0"));
	}

	const double yaw = info["standing"]["yaw"].asDouble();
	const xrt_vec3 yaw_axis{0.0, -1.0, 0.0};
	math_quat_from_angle_vector(static_cast<float>(yaw), &yaw_axis, &chaperone.orientation);
	chaperone.position = copy_vec3({
	    translation_arr[0].asDouble(),
	    translation_arr[1].asDouble(),
	    translation_arr[2].asDouble(),
	});
	math_quat_rotate_vec3(&chaperone.orientation, &chaperone.position, &chaperone.position);
	DEV_INFO("Initialized chaperone data.");
}

inline xrt_space_relation_flags
operator|(xrt_space_relation_flags a, xrt_space_relation_flags b)
{
	return static_cast<xrt_space_relation_flags>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}

inline xrt_space_relation_flags &
operator|=(xrt_space_relation_flags &a, xrt_space_relation_flags b)
{
	a = a | b;
	return a;
}

void
Device::update_pose(const vr::DriverPose_t &newPose) const
{
	xrt_space_relation relation = {};
	// These relation hookups are a bit seat of the pants however they produce good full body track results
	// especially when occluded from basestations linear drift off into space is minimized.
	if (newPose.deviceIsConnected) {
		relation.relation_flags |= xrt_space_relation_flags::XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT |
		                           xrt_space_relation_flags::XRT_SPACE_RELATION_POSITION_TRACKED_BIT;
	}
	if (newPose.poseIsValid) {
		relation.relation_flags |= xrt_space_relation_flags::XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT |
		                           xrt_space_relation_flags::XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT;
	}
	if (newPose.result == vr::TrackingResult_Running_OK) {
		relation.relation_flags |= xrt_space_relation_flags::XRT_SPACE_RELATION_POSITION_VALID_BIT |
		                           xrt_space_relation_flags::XRT_SPACE_RELATION_ORIENTATION_VALID_BIT;
	}

	// The driver still outputs good pose data regardless of the pose results above
	relation.pose = copy_pose(newPose.qRotation, newPose.vecPosition);
	relation.linear_velocity = copy_vec3(newPose.vecVelocity);
	relation.angular_velocity = copy_vec3(newPose.vecAngularVelocity);

	math_quat_rotate_vec3(&relation.pose.orientation, &relation.angular_velocity, &relation.angular_velocity);

	// apply over local transform
	const xrt_pose local = copy_pose(newPose.qDriverFromHeadRotation, newPose.vecDriverFromHeadTranslation);
	math_pose_transform(&relation.pose, &local, &relation.pose);

	// apply world transform
	const xrt_pose world = copy_pose(newPose.qWorldFromDriverRotation, newPose.vecWorldFromDriverTranslation);
	math_pose_transform(&world, &relation.pose, &relation.pose);
	math_quat_rotate_vec3(&world.orientation, &relation.linear_velocity, &relation.linear_velocity);
	math_quat_rotate_vec3(&world.orientation, &relation.angular_velocity, &relation.angular_velocity);

	// apply chaperone transform
	math_pose_transform(&chaperone, &relation.pose, &relation.pose);
	math_quat_rotate_vec3(&chaperone.orientation, &relation.linear_velocity, &relation.linear_velocity);
	math_quat_rotate_vec3(&chaperone.orientation, &relation.angular_velocity, &relation.angular_velocity);

	const uint64_t ts = chrono_timestamp_ns() + static_cast<uint64_t>(newPose.poseTimeOffset * 1000000.0);

	m_relation_history_push(relation_hist, &relation, ts);
}

void
Device::handle_properties(const vr::PropertyWrite_t *batch, uint32_t count)
{
	for (uint32_t i = 0; i < count; ++i) {
		handle_property_write(batch[i]);
	}
}

void
HmdDevice::set_nominal_frame_interval(uint64_t interval_ns)
{
	auto set = [this, interval_ns] { hmd_parts->base.screens[0].nominal_frame_interval_ns = interval_ns; };

	if (hmd_parts) {
		set();
	} else {
		std::thread t([this, set] {
			std::unique_lock lk(hmd_parts_mut);
			hmd_parts_cv.wait(lk, [this] { return hmd_parts != nullptr; });
			set();
		});
		t.detach();
	}
}

namespace {
// From openvr driver documentation
// (https://github.com/ValveSoftware/openvr/blob/master/docs/Driver_API_Documentation.md#Input-Profiles):
// "Input profiles are expected to be a valid JSON file,
// and should be located: <driver_name>/resources/input/<device_name>_profile.json"
// So we will just parse the file name to get the device name.
std::string_view
parse_profile(std::string_view path)
{
	size_t name_start_idx = path.find_last_of('/') + 1;
	size_t name_end_idx = path.find_last_of('_');
	return path.substr(name_start_idx, name_end_idx - name_start_idx);
}
} // namespace

void
Device::handle_property_write(const vr::PropertyWrite_t &prop)
{
	switch (prop.prop) {
	case vr::Prop_ManufacturerName_String: {
		this->manufacturer = std::string(static_cast<char *>(prop.pvBuffer), prop.unBufferSize);
		if (!this->model.empty()) {
			std::snprintf(this->str, std::size(this->str), "%s %s", this->manufacturer.c_str(),
			              this->model.c_str());
		}
		break;
	}
	case vr::Prop_ModelNumber_String: {
		this->model = std::string(static_cast<char *>(prop.pvBuffer), prop.unBufferSize);
		if (!this->manufacturer.empty()) {
			std::snprintf(this->str, std::size(this->str), "%s %s", this->manufacturer.c_str(),
			              this->model.c_str());
		}
		break;
	}
	default: {
		DEV_DEBUG("Unhandled property: %i", prop.prop);
		break;
	}
	}
}

void
HmdDevice::handle_property_write(const vr::PropertyWrite_t &prop)
{
	switch (prop.prop) {
	case vr::Prop_DisplayFrequency_Float: {
		assert(prop.unBufferSize == sizeof(float));
		float freq = *static_cast<float *>(prop.pvBuffer);
		set_nominal_frame_interval((1.f / freq) * 1e9f);
		break;
	}
	case vr::Prop_UserIpdMeters_Float: {
		if (*static_cast<float *>(prop.pvBuffer) != 0) {
			ipd = *static_cast<float *>(prop.pvBuffer);
		}
		break;
	}
	case vr::Prop_SecondsFromVsyncToPhotons_Float: {
		vsync_to_photon_ns = *static_cast<float *>(prop.pvBuffer) * 1e9f;
		break;
	}
	case vr::Prop_DeviceProvidesBatteryStatus_Bool: {
		float supported = *static_cast<bool *>(prop.pvBuffer);
		this->provides_battery_status = supported;
		DEV_DEBUG("Has battery status: HMD: %s", supported ? "true" : "false");
		break;
	}
	case vr::Prop_DeviceIsCharging_Bool: {
		float charging = *static_cast<bool *>(prop.pvBuffer);
		this->charging = charging;
		DEV_DEBUG("Charging: HMD: %s", charging ? "true" : "false");
		break;
	}
	case vr::Prop_DeviceBatteryPercentage_Float: {
		float bat = *static_cast<float *>(prop.pvBuffer);
		this->charge = bat;
		DEV_DEBUG("Battery: HMD: %f", bat);
		break;
	}
	default: {
		Device::handle_property_write(prop);
		break;
	}
	}
}

void
ControllerDevice::handle_property_write(const vr::PropertyWrite_t &prop)
{
	switch (prop.prop) {
	case vr::Prop_InputProfilePath_String: {
		std::string_view profile =
		    parse_profile(std::string_view(static_cast<char *>(prop.pvBuffer), prop.unBufferSize));
		auto input_class = controller_classes.find(profile);
		if (input_class == controller_classes.end()) {
			DEV_ERR("Could not find input class for controller profile %s", std::string(profile).c_str());
		} else {
			this->name = input_class->second.name;
			set_input_class(&input_class->second);
		}
		break;
	}
	case vr::Prop_ModelNumber_String: {
		using namespace std::literals::string_view_literals;
		vr::PropertyWrite_t fixedProp = prop;
		const std::string_view name = {static_cast<char *>(prop.pvBuffer), prop.unBufferSize};
		if (name == "SlimeVR Virtual Tracker\0"sv) {
			static const InputClass input_class = {
			    XRT_DEVICE_VIVE_TRACKER, {XRT_INPUT_GENERIC_TRACKER_POSE}, {}};
			this->name = input_class.name;
			set_input_class(&input_class);
			this->manufacturer = name.substr(0, name.find_first_of(' '));
			fixedProp.pvBuffer = (char *)fixedProp.pvBuffer + this->manufacturer.size() +
			                     (this->manufacturer.size() != name.size());
			fixedProp.unBufferSize = name.end() - (char *)fixedProp.pvBuffer;
		}
		Device::handle_property_write(fixedProp);
		break;
	}
	case vr::Prop_ControllerRoleHint_Int32: {
		vr::ETrackedControllerRole role = *static_cast<vr::ETrackedControllerRole *>(prop.pvBuffer);
		switch (role) {
		case vr::TrackedControllerRole_Invalid: {
			this->device_type = XRT_DEVICE_TYPE_ANY_HAND_CONTROLLER;
			break;
		}
		case vr::TrackedControllerRole_RightHand: {
			this->device_type = XRT_DEVICE_TYPE_RIGHT_HAND_CONTROLLER;
			set_active_hand(XRT_HAND_RIGHT);
			break;
		}
		case vr::TrackedControllerRole_LeftHand: {
			this->device_type = XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER;
			set_active_hand(XRT_HAND_LEFT);
			break;
		}
		case vr::TrackedControllerRole_OptOut: {
			this->device_type = XRT_DEVICE_TYPE_GENERIC_TRACKER;
			break;
		}
		default: {
			this->device_type = XRT_DEVICE_TYPE_UNKNOWN;
			DEV_WARN("requested unimplemented role hint %i", this->device_type);
			break;
		}
		}
		break;
	}
	case vr::Prop_DeviceProvidesBatteryStatus_Bool: {
		float supported = *static_cast<bool *>(prop.pvBuffer);
		const char *name;
		switch (this->device_type) {
		case XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER: {
			name = "Left";
			break;
		}
		case XRT_DEVICE_TYPE_RIGHT_HAND_CONTROLLER: {
			name = "Right";
			break;
		}
		default: {
			name = "Unknown";
			break;
		}
		}
		this->provides_battery_status = supported;
		DEV_DEBUG("Has battery status: %s: %s", name, supported ? "true" : "false");
		break;
	}
	case vr::Prop_DeviceIsCharging_Bool: {
		float charging = *static_cast<bool *>(prop.pvBuffer);
		const char *name;
		switch (this->device_type) {
		case XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER: {
			name = "Left";
			break;
		}
		case XRT_DEVICE_TYPE_RIGHT_HAND_CONTROLLER: {
			name = "Right";
			break;
		}
		default: {
			name = "Unknown";
		}
		}
		this->charging = charging;
		DEV_DEBUG("Charging: %s: %s", name, charging ? "true" : "false");
		break;
	}
	case vr::Prop_DeviceBatteryPercentage_Float: {
		float bat = *static_cast<float *>(prop.pvBuffer);
		const char *name;
		switch (this->device_type) {
		case XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER: {
			name = "Left";
			break;
		}
		case XRT_DEVICE_TYPE_RIGHT_HAND_CONTROLLER: {
			name = "Right";
			break;
		}
		default: {
			name = "Unknown";
		}
		}
		this->charge = bat;
		DEV_DEBUG("Battery: %s: %f", name, bat);
		break;
	}
	default: {
		Device::handle_property_write(prop);
		break;
	}
	}
}
