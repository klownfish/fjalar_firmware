#pragma once

#include "tracker.h"

float haversine_distance(float lat1, float lon1, float lat2, float lon2);
void init_sensors(tracker_t *tracker);