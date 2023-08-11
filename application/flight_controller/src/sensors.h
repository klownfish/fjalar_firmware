#pragma once

#include "fjalar.h"

struct pressure_queue_entry {
    uint32_t t;
    float pressure;
} __attribute__((aligned(4)));

struct imu_queue_entry {
    uint32_t t;
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
} __attribute__((aligned(4)));

extern struct k_msgq pressure_msgq;
extern struct k_msgq imu_msgq;

void init_sensors(fjalar_t *fjalar);