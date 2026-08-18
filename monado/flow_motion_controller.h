// SPDX-License-Identifier: BSL-1.0
#pragma once

#include "flow_motion_controller_bindings.h"
#include "xrt/xrt_device.h"

#ifdef __cplusplus
extern "C" {
#endif
struct xrt_device *
flow_motion_controller_create(struct xrt_device *tracker, void *rust_state, bool is_left);

bool
flow_motion_controller_calibrate(struct xrt_device *tracker,
                                 struct xrt_device *head,
                                 void *rust_state,
                                 bool is_left);

#ifdef __cplusplus
}
#endif
