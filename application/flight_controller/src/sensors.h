#pragma once

#include <zephyr/zbus/zbus.h>

#include "fjalar.h"

struct pressure_zbus_msg {
    uint32_t t;
    float pressure;
};

struct imu_zbus_msg {
    uint32_t t;
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
};

ZBUS_CHAN_DECLARE(
    pressure_zchan,
    imu_zchan
);

void init_sensors(fjalar_t *fjalar);