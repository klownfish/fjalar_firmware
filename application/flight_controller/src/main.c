/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include <math.h>

#include "fjalar.h"
#include "sensors.h"
#include "flight_state.h"
#include "communication.h"

LOG_MODULE_REGISTER(main, CONFIG_APP_MAIN_LOG_LEVEL);

fjalar_t fjalar_god;

int main(void) {
	init_sensors(&fjalar_god);
	init_flight_state(&fjalar_god);
	init_communication(&fjalar_god);
	return 0;
}