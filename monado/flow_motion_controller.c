// SPDX-License-Identifier: BSL-1.0

#include "flow_motion_controller.h"

#include "os/os_time.h"
#include "util/u_device.h"
#include "util/u_logging.h"
#include "vive/vive_bindings.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

enum flow_motion_input
{
	FMC_TRIGGER_CLICK,
	FMC_TRIGGER_VALUE,
	FMC_A_CLICK,
	FMC_B_CLICK,
	FMC_GRIP_VALUE,
	FMC_GRIP_FORCE,
	FMC_SYSTEM_CLICK,
	FMC_THUMBSTICK,
	FMC_THUMBSTICK_CLICK,
	FMC_GRIP_POSE,
	FMC_AIM_POSE,
	FMC_INPUT_COUNT,
};

struct flow_motion_device
{
	struct xrt_device base;
	struct xrt_device *tracker;
	void *rust_state;
	bool is_left;
};

static xrt_result_t
flow_motion_update_inputs(struct xrt_device *xdev)
{
	struct flow_motion_device *device = (struct flow_motion_device *)xdev;
	struct FlowMotionInput input = {0};
	if (!flow_motion_rust_poll(device->rust_state, device->is_left, &input)) {
		return XRT_SUCCESS;
	}

	int64_t now_ns = os_monotonic_get_ns();
	xdev->inputs[FMC_TRIGGER_CLICK].value.boolean = input.trigger != 0;
	xdev->inputs[FMC_TRIGGER_VALUE].value.vec1.x = input.trigger ? 1.0f : 0.0f;
	xdev->inputs[FMC_A_CLICK].value.boolean = input.a != 0;
	xdev->inputs[FMC_B_CLICK].value.boolean = input.b != 0;
	xdev->inputs[FMC_GRIP_VALUE].value.vec1.x = input.grip ? 1.0f : 0.0f;
	xdev->inputs[FMC_GRIP_FORCE].value.vec1.x = input.grip ? 1.0f : 0.0f;
	xdev->inputs[FMC_SYSTEM_CLICK].value.boolean = input.system != 0;
	xdev->inputs[FMC_THUMBSTICK].value.vec2.x = input.joystick_x;
	xdev->inputs[FMC_THUMBSTICK].value.vec2.y = input.joystick_y;
	xdev->inputs[FMC_THUMBSTICK_CLICK].value.boolean = input.joystick_button != 0;
	for (size_t i = 0; i < FMC_INPUT_COUNT; i++) {
		xdev->inputs[i].timestamp = now_ns;
	}

	return XRT_SUCCESS;
}

static xrt_result_t
flow_motion_get_tracked_pose(struct xrt_device *xdev,
                             enum xrt_input_name name,
                             int64_t at_timestamp_ns,
                             struct xrt_space_relation *out_relation)
{
	if (name != XRT_INPUT_INDEX_GRIP_POSE && name != XRT_INPUT_INDEX_AIM_POSE) {
		return XRT_ERROR_INPUT_UNSUPPORTED;
	}

	struct flow_motion_device *device = (struct flow_motion_device *)xdev;
	xrt_result_t result = xrt_device_get_tracked_pose(
	    device->tracker, XRT_INPUT_GENERIC_TRACKER_POSE, at_timestamp_ns, out_relation);
	if (result == XRT_SUCCESS) {
		struct FlowMotionRelation relation = {
		    .relation_flags = (int32_t)out_relation->relation_flags,
		    .orientation = {
		        .x = out_relation->pose.orientation.x,
		        .y = out_relation->pose.orientation.y,
		        .z = out_relation->pose.orientation.z,
		        .w = out_relation->pose.orientation.w,
		    },
		    .position = {
		        .x = out_relation->pose.position.x,
		        .y = out_relation->pose.position.y,
		        .z = out_relation->pose.position.z,
		    },
		    .linear_velocity = {
		        .x = out_relation->linear_velocity.x,
		        .y = out_relation->linear_velocity.y,
		        .z = out_relation->linear_velocity.z,
		    },
		    .angular_velocity = {
		        .x = out_relation->angular_velocity.x,
		        .y = out_relation->angular_velocity.y,
		        .z = out_relation->angular_velocity.z,
		    },
		};
		flow_motion_rust_apply_offset(device->rust_state,
		                              device->is_left,
		                              name == XRT_INPUT_INDEX_GRIP_POSE,
		                              &relation);
		out_relation->pose.orientation.x = relation.orientation.x;
		out_relation->pose.orientation.y = relation.orientation.y;
		out_relation->pose.orientation.z = relation.orientation.z;
		out_relation->pose.orientation.w = relation.orientation.w;
		out_relation->pose.position.x = relation.position.x;
		out_relation->pose.position.y = relation.position.y;
		out_relation->pose.position.z = relation.position.z;
		out_relation->linear_velocity.x = relation.linear_velocity.x;
		out_relation->linear_velocity.y = relation.linear_velocity.y;
		out_relation->linear_velocity.z = relation.linear_velocity.z;
		out_relation->angular_velocity.x = relation.angular_velocity.x;
		out_relation->angular_velocity.y = relation.angular_velocity.y;
		out_relation->angular_velocity.z = relation.angular_velocity.z;
	}
	return result;
}

static void
flow_motion_destroy(struct xrt_device *xdev)
{
	struct flow_motion_device *device = (struct flow_motion_device *)xdev;
	flow_motion_rust_destroy(device->rust_state);
	u_device_free(xdev);
}

bool
flow_motion_controller_calibrate(struct xrt_device *tracker,
                                 struct xrt_device *head,
                                 void *rust_state,
                                 bool is_left)
{
	if (tracker == NULL || head == NULL || rust_state == NULL) {
		return false;
	}

	uint32_t countdown_ms = flow_motion_rust_calibration_countdown_ms();
	uint32_t duration_ms = flow_motion_rust_calibration_duration_ms();
	size_t capacity = duration_ms / 10 + 1;
	struct FlowMotionQuat *tracker_samples = calloc(capacity, sizeof(*tracker_samples));
	struct FlowMotionQuat *head_samples = calloc(capacity, sizeof(*head_samples));
	if (tracker_samples == NULL || head_samples == NULL) {
		U_LOG_E("Failed to allocate Flow Motion calibration samples");
		free(tracker_samples);
		free(head_samples);
		return false;
	}

	U_LOG_I("Calibrating the %s hand in %.1f seconds: hold the hand flat with the palm down and keep "
	        "the headset and fingers facing the same horizontal direction; looking down is fine",
	        is_left ? "left" : "right",
	        countdown_ms / 1000.0);
	os_nanosleep((int64_t)countdown_ms * 1000 * 1000);
	U_LOG_I("Sampling %s tracker and headset orientations for %.1f seconds",
	        is_left ? "left" : "right",
	        duration_ms / 1000.0);

	size_t sample_count = 0;
	int64_t end_ns = os_monotonic_get_ns() + (int64_t)duration_ms * 1000 * 1000;
	while (os_monotonic_get_ns() < end_ns && sample_count < capacity) {
		int64_t now_ns = os_monotonic_get_ns();
		struct xrt_space_relation tracker_relation = {0};
		struct xrt_space_relation head_relation = {0};
		xrt_result_t tracker_result = xrt_device_get_tracked_pose(
		    tracker, XRT_INPUT_GENERIC_TRACKER_POSE, now_ns, &tracker_relation);
		xrt_result_t head_result =
		    xrt_device_get_tracked_pose(head, XRT_INPUT_GENERIC_HEAD_POSE, now_ns, &head_relation);
		if (tracker_result == XRT_SUCCESS && head_result == XRT_SUCCESS &&
		    (tracker_relation.relation_flags & XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT) != 0 &&
		    (head_relation.relation_flags & XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT) != 0) {
			tracker_samples[sample_count] = (struct FlowMotionQuat){
			    .x = tracker_relation.pose.orientation.x,
			    .y = tracker_relation.pose.orientation.y,
			    .z = tracker_relation.pose.orientation.z,
			    .w = tracker_relation.pose.orientation.w,
			};
			head_samples[sample_count] = (struct FlowMotionQuat){
			    .x = head_relation.pose.orientation.x,
			    .y = head_relation.pose.orientation.y,
			    .z = head_relation.pose.orientation.z,
			    .w = head_relation.pose.orientation.w,
			};
			sample_count++;
		}
		os_nanosleep(10 * 1000 * 1000);
	}

	size_t minimum_samples = capacity * 3 / 4;
	bool success = sample_count >= minimum_samples &&
	               flow_motion_rust_save_calibration(rust_state,
	                                                 is_left,
	                                                 tracker->serial,
	                                                 tracker_samples,
	                                                 head_samples,
	                                                 sample_count);
	free(tracker_samples);
	free(head_samples);
	if (!success) {
		U_LOG_E("Flow Motion calibration failed: collected %zu of at least %zu required tracked samples",
		        sample_count,
		        minimum_samples);
	}
	return success;
}

struct xrt_device *
flow_motion_controller_create(struct xrt_device *tracker, void *rust_state, bool is_left)
{
	if (tracker == NULL || rust_state == NULL) {
		return NULL;
	}

	struct flow_motion_device *device = U_DEVICE_ALLOCATE(
	    struct flow_motion_device, U_DEVICE_ALLOC_NO_FLAGS, FMC_INPUT_COUNT, 0);
	if (device == NULL) {
		flow_motion_rust_destroy(rust_state);
		return NULL;
	}

	device->tracker = tracker;
	device->rust_state = rust_state;
	device->is_left = is_left;
	device->base.name = XRT_DEVICE_INDEX_CONTROLLER;
	device->base.device_type = is_left ? XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER
	                                   : XRT_DEVICE_TYPE_RIGHT_HAND_CONTROLLER;
	device->base.tracking_origin = tracker->tracking_origin;
	device->base.binding_profiles = vive_binding_profiles_index;
	device->base.binding_profile_count = vive_binding_profiles_index_count;
	device->base.supported.orientation_tracking = true;
	device->base.supported.position_tracking = true;
	device->base.update_inputs = flow_motion_update_inputs;
	device->base.get_tracked_pose = flow_motion_get_tracked_pose;
	device->base.destroy = flow_motion_destroy;

	snprintf(device->base.str,
	         XRT_DEVICE_NAME_LEN,
	         "Flow Motion Controller (%s)",
	         is_left ? "left" : "right");
	snprintf(device->base.serial,
	         XRT_DEVICE_NAME_LEN,
	         "flow-motion-%s",
	         is_left ? "left" : "right");

	device->base.inputs[FMC_TRIGGER_CLICK].name = XRT_INPUT_INDEX_TRIGGER_CLICK;
	device->base.inputs[FMC_TRIGGER_VALUE].name = XRT_INPUT_INDEX_TRIGGER_VALUE;
	device->base.inputs[FMC_A_CLICK].name = XRT_INPUT_INDEX_A_CLICK;
	device->base.inputs[FMC_B_CLICK].name = XRT_INPUT_INDEX_B_CLICK;
	device->base.inputs[FMC_GRIP_VALUE].name = XRT_INPUT_INDEX_SQUEEZE_VALUE;
	device->base.inputs[FMC_GRIP_FORCE].name = XRT_INPUT_INDEX_SQUEEZE_FORCE;
	device->base.inputs[FMC_SYSTEM_CLICK].name = XRT_INPUT_INDEX_SYSTEM_CLICK;
	device->base.inputs[FMC_THUMBSTICK].name = XRT_INPUT_INDEX_THUMBSTICK;
	device->base.inputs[FMC_THUMBSTICK_CLICK].name = XRT_INPUT_INDEX_THUMBSTICK_CLICK;
	device->base.inputs[FMC_GRIP_POSE].name = XRT_INPUT_INDEX_GRIP_POSE;
	device->base.inputs[FMC_AIM_POSE].name = XRT_INPUT_INDEX_AIM_POSE;

	U_LOG_I("Created %s from tracker %s", device->base.str, tracker->serial);
	return &device->base;
}
