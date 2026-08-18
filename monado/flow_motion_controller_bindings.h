#ifndef FLOW_MOTION_CONTROLLER_H
#define FLOW_MOTION_CONTROLLER_H

#pragma once

#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

#define FRAME_SIZE 16

#define BUTTON_TRIGGER (1 << 0)

#define BUTTON_SWITCH_HAND (1 << 1)

#define BUTTON_A (1 << 2)

#define BUTTON_B (1 << 3)

#define BUTTON_GRIP (1 << 4)

#define BUTTON_SYSTEM (1 << 5)

#define BUTTON_JOYSTICK (1 << 7)

typedef struct FlowMotionInput {
  uint8_t trigger;
  uint8_t a;
  uint8_t b;
  uint8_t grip;
  uint8_t system;
  uint8_t joystick_button;
  float joystick_x;
  float joystick_y;
} FlowMotionInput;

typedef struct FlowMotionQuat {
  float x;
  float y;
  float z;
  float w;
} FlowMotionQuat;

typedef struct FlowMotionVec3 {
  float x;
  float y;
  float z;
} FlowMotionVec3;

typedef struct FlowMotionRelation {
  int32_t relation_flags;
  struct FlowMotionQuat orientation;
  struct FlowMotionVec3 position;
  struct FlowMotionVec3 linear_velocity;
  struct FlowMotionVec3 angular_velocity;
} FlowMotionRelation;

void *flow_motion_rust_create(void);

void *flow_motion_rust_clone(const void *state);

void flow_motion_rust_destroy(void *state);

bool flow_motion_rust_poll(const void *state, bool is_left, struct FlowMotionInput *out_input);

bool flow_motion_rust_should_calibrate(const void *state, bool is_left);

uint32_t flow_motion_rust_calibration_duration_ms(void);

uint32_t flow_motion_rust_calibration_countdown_ms(void);

bool flow_motion_rust_save_calibration(const void *state,
                                       bool is_left,
                                       const char *serial,
                                       const struct FlowMotionQuat *tracker_samples,
                                       const struct FlowMotionQuat *head_samples,
                                       size_t sample_count);

bool flow_motion_rust_write_calibration_status(const void *state, bool success);

void flow_motion_rust_apply_offset(const void *state,
                                   bool is_left,
                                   bool is_grip,
                                   struct FlowMotionRelation *relation);

#endif  /* FLOW_MOTION_CONTROLLER_H */
