#pragma once

#include "protocol.h"

#define DT_ALIAS_EXISTS(alias) DT_NODE_EXISTS(DT_ALIAS(alias))

enum screen_frames {
    FRAME_INFO,
    FRAME_TELEMETRY,
    FRAME_TRACKING,
    FRAME_GET_READY,
    FRAME_ENTER_IDLE,
    FRAME_ENTER_SUDO,
    FRAME_PYRO1,
    FRAME_PYRO2,
    FRAME_PYRO3,
    FRAME_MAX
};

typedef struct {
    volatile enum screen_frames current_frame;
    volatile struct telemetry_packet telemetry;
    volatile struct pyros_enabled pyros_enabled;
    volatile int32_t local_rssi;
    volatile float latitude;
    volatile float longitude;
} tracker_t;

extern tracker_t tracker_god;