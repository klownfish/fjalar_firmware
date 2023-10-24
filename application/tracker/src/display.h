#pragma once

#include "tracker.h"

void init_display(tracker_t *tracker);
void draw_frame(tracker_t *tracker, enum screen_frames frame);
void next_frame(tracker_t *tracker);