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
	// int ret = 0;

	// struct sensor_value odr_attr;

	/* set accel/gyro sampling frequency to 12.5 Hz */
	// odr_attr.val1 = 1;
	// odr_attr.val2 = 0;
	// ret = sensor_attr_set(imu_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr);
	// if (ret) {
	// 	LOG_ERR("Could not set imu accel XYZ odr");
	// }
	// ret = sensor_attr_set(imu_dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr);
	// if (ret) {
	// 	LOG_ERR("Could not set imu gyro XYZ odr");
	// }

	// while (1) {
		// struct sensor_value press;
		// struct sensor_value temp;
		// struct sensor_value ax;
		// struct sensor_value ay;
		// struct sensor_value az;
		// struct sensor_value gx;
		// struct sensor_value gy;
		// struct sensor_value gz;

		// sensor_sample_fetch_chan(imu_dev, SENSOR_CHAN_ACCEL_XYZ);
		// sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_X, &ax);
		// sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_Y, &ay);
		// sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_Z, &az);

		// sensor_sample_fetch_chan(imu_dev, SENSOR_CHAN_GYRO_XYZ);
		// sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_X, &gx);
		// sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_Y, &gy);
		// sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_Z, &gz);

		// sensor_sample_fetch_chan(baro_dev, SENSOR_CHAN_ALL);
		// sensor_channel_get(baro_dev, SENSOR_CHAN_PRESS, &press);
		// sensor_channel_get(baro_dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);

		// k_msleep(5000);
		// gpio_pin_set_dt(&pyro0_dt, 1);
		// gpio_pin_set_dt(&pyro1_dt, 1);
		// gpio_pin_set_dt(&pyro2_dt, 1);
		// LOG_ERR("pyros enabled");
		// k_msleep(5000);
		// gpio_pin_set_dt(&pyro0_dt, 0);
		// gpio_pin_set_dt(&pyro1_dt, 0);
		// gpio_pin_set_dt(&pyro2_dt, 0);
		// LOG_ERR("pyros disabled");
		// k_msleep(5000);
		// bool a = gpio_pin_get_dt(&pyro0_sense_dt);
		// bool b = gpio_pin_get_dt(&pyro1_sense_dt);
		// bool c = gpio_pin_get_dt(&pyro2_sense_dt);
		// LOG_ERR("pyros connected? %d %d %d", a, b, c);

		// sensor_channel_get(baro_dev, SENSOR_CHAN_PRESS, &press);
		// sensor_channel_get(baro_dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
		// sensor_channel_get(baro_dev, SENSOR)

		// printf("accel: %f %f %f\n", sensor_to_float(ax), sensor_to_float(ay), sensor_to_float(az));
		// printf("gyro: %f %f %f\n", sensor_to_float(gx), sensor_to_float(gy), sensor_to_float(gz));
		// printf("temp: %f\n", sensor_to_float(temp));
		// printf("pressure: %f\n", sensor_to_float(press));
	// }
	return 0;
}