#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include <minmea.h>

#include "fjalar.h"
#include "sensors.h"

LOG_MODULE_REGISTER(sensors, CONFIG_APP_SENSORS_LOG_LEVEL);

#define IMU_THREAD_PRIORITY 7
#define IMU_THREAD_STACK_SIZE 2048

#define BAROMETER_THREAD_PRIORITY 7
#define BAROMETER_THREAD_STACK_SIZE 2048

#define VBAT_THREAD_PRIORITY 7
#define VBAT_THREAD_STACK_SIZE 2048

#define GPS_THREAD_PRIORITY 7
#define GPS_THREAD_STACK_SIZE 2048

void barometer_thread(fjalar_t *fjalar, void *p2, void *p3);
void imu_thread(fjalar_t *fjalar, void *p2, void *p3);
void gps_thread(fjalar_t *fjalar, void *p2, void *p3);
void vbat_thread(fjalar_t *fjalar, void *p2, void *p3);

K_MSGQ_DEFINE(pressure_msgq, sizeof(struct pressure_queue_entry), 3, 4);
K_MSGQ_DEFINE(imu_msgq, sizeof(struct imu_queue_entry), 3, 4);

K_THREAD_STACK_DEFINE(barometer_thread_stack, BAROMETER_THREAD_STACK_SIZE);
struct k_thread barometer_thread_data;
k_tid_t barometer_thread_id;

K_THREAD_STACK_DEFINE(imu_thread_stack, IMU_THREAD_STACK_SIZE);
struct k_thread imu_thread_data;
k_tid_t imu_thread_id;

K_THREAD_STACK_DEFINE(vbat_thread_stack, VBAT_THREAD_STACK_SIZE);
struct k_thread vbat_thread_data;
k_tid_t vbat_thread_id;

#if DT_ALIAS_EXISTS(gps_uart)
K_THREAD_STACK_DEFINE(gps_thread_stack, GPS_THREAD_STACK_SIZE);
struct k_thread gps_thread_data;
k_tid_t gps_thread_id;
#endif

void init_sensors(fjalar_t *fjalar) {
	barometer_thread_id = k_thread_create(
		&barometer_thread_data,
		barometer_thread_stack,
		K_THREAD_STACK_SIZEOF(barometer_thread_stack),
		(k_thread_entry_t) barometer_thread,
		fjalar, NULL, NULL,
		BAROMETER_THREAD_PRIORITY, 0, K_NO_WAIT
	);
	k_thread_name_set(barometer_thread_id, "barometer");

	imu_thread_id = k_thread_create(
		&imu_thread_data,
		imu_thread_stack,
		K_THREAD_STACK_SIZEOF(imu_thread_stack),
		(k_thread_entry_t) imu_thread,
		fjalar, NULL, NULL,
		IMU_THREAD_PRIORITY, 0, K_NO_WAIT
	);
	k_thread_name_set(imu_thread_id, "imu");

	#if DT_ALIAS_EXISTS(bat_adc)
	vbat_thread_id = k_thread_create(
		&vbat_thread_data,
		vbat_thread_stack,
		K_THREAD_STACK_SIZEOF(vbat_thread_stack),
		(k_thread_entry_t) vbat_thread,
		fjalar, NULL, NULL,
		VBAT_THREAD_PRIORITY, 0, K_NO_WAIT
	);
	k_thread_name_set(vbat_thread_id, "bat adc");
	#endif

	#if DT_ALIAS_EXISTS(gps_uart)
	gps_thread_id = k_thread_create(
		&gps_thread_data,
		gps_thread_stack,
		K_THREAD_STACK_SIZEOF(gps_thread_stack),
		(k_thread_entry_t) gps_thread,
		fjalar, NULL, NULL,
		GPS_THREAD_PRIORITY, 0, K_NO_WAIT
	);
	k_thread_name_set(gps_thread_id, "gps");
	#endif
}

void imu_thread(fjalar_t *fjalar, void *p2, void *p3) {
    const struct device *const imu_dev = DEVICE_DT_GET(DT_ALIAS(imu));
    if (!device_is_ready(imu_dev)) {
		LOG_ERR("imu is not ready");
		return;
	}
	while (true) {
		int ret = 0;
		k_msleep(10);
		ret = sensor_sample_fetch(imu_dev);
		if (ret != 0) {
			LOG_ERR("Could not fetch imu sample");
		}

		struct sensor_value ax;
		struct sensor_value ay;
		struct sensor_value az;

		struct sensor_value gx;
		struct sensor_value gy;
		struct sensor_value gz;

		ret = sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_X, &ax);
		ret = sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_Y, &ay);
		ret = sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_Z, &az);

		ret = sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_X, &gx);
		ret = sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_Y, &gy);
		ret = sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_Z, &gz);

		if (ret != 0) {
			LOG_ERR("Could get imu values");
		}

		LOG_DBG("read imu: %f %f %f %f %f %f",
			sensor_value_to_float(&ax), sensor_value_to_float(&ay), sensor_value_to_float(&az),
			sensor_value_to_float(&gx), sensor_value_to_float(&gy), sensor_value_to_float(&gz)
		);

		struct imu_queue_entry msg = {
			.t = k_uptime_get_32(),
			.ax = sensor_value_to_float(&ax),
			.ay = sensor_value_to_float(&ay),
			.az = sensor_value_to_float(&az),
			.gx = sensor_value_to_float(&gx),
			.gy = sensor_value_to_float(&gy),
			.gz = sensor_value_to_float(&gz)
		};
		ret = k_msgq_put(&imu_msgq, &msg, K_NO_WAIT);
		if (ret != 0) {
			LOG_ERR("Could not write to imu msgq");
		}
	}
}

void barometer_thread(fjalar_t *fjalar, void *p2, void *p3) {
    const struct device *const baro_dev = DEVICE_DT_GET(DT_ALIAS(barometer));
    if (!device_is_ready(baro_dev)) {
		LOG_ERR("barometer is not ready");
		return;
	}
	int ret;
	struct sensor_value osr;
	osr.val1 = 2048;
	osr.val2 = 0;
	ret = sensor_attr_set(baro_dev, SENSOR_CHAN_ALL, SENSOR_ATTR_OVERSAMPLING, &osr);
	if (ret != 0) {
		LOG_ERR("Could not set barometer oversample");
	}
	while (true) {
		int ret = 0;
		k_msleep(10);
		ret = sensor_sample_fetch(baro_dev);
		if (ret != 0) {
			LOG_ERR("Could not fetch barometer sample");
		}
		struct sensor_value pressure;
		ret = sensor_channel_get(baro_dev, SENSOR_CHAN_PRESS, &pressure);
		if (ret != 0) {
			LOG_ERR("Could not get barometer pressure");
		}
		LOG_DBG("read pressure: %f", sensor_value_to_float(&pressure));
		struct pressure_queue_entry msg;
		msg.t = k_uptime_get_32();
		msg.pressure = sensor_value_to_float(&pressure);
		ret = k_msgq_put(&pressure_msgq, &msg, K_NO_WAIT);
		if (ret != 0) {
			LOG_ERR("Could not write to pressure msgq");
		}
	}
}

#if DT_NODE_EXISTS(DT_ALIAS(gps_uart))
void handle_nmea(char *buf, int index) {

}

void gps_uart_cb(const struct device *dev, void *user_data) {
	uint8_t buf[256];
	int len = 0;

	if (!uart_irq_update(dev)) {
		return;
	}

	if (!uart_irq_rx_ready(dev)) {
		return;
	}

	/* read until FIFO empty */
	len = uart_fifo_read(dev, buf, sizeof(buf));
	// printk("%.*s", len, buf);
}

void gps_thread(fjalar_t *fjalar, void *p2, void *p3) {
    const struct device *const gps_dev = DEVICE_DT_GET(DT_ALIAS(gps_uart));
    if (!device_is_ready(gps_dev)) {
		LOG_ERR("gps is not ready");
		return;
	}
    int ret;
    struct uart_config uart_config = {
		.baudrate = 9600,
		.data_bits = UART_CFG_DATA_BITS_8,
		.flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
		.parity = UART_CFG_PARITY_NONE,
		.stop_bits = UART_CFG_STOP_BITS_1
	};
	ret = uart_configure(gps_dev, &uart_config);
	// uart_irq_callback_set(gps_dev, gps_uart_cb);
	// uart_irq_rx_enable(gps_dev);

	const char pulse_msg[] = "$PMTK285,2,100*3E\r\n";
	for (int i = 0; i < strlen(pulse_msg); i++) {
		uart_poll_out(gps_dev, pulse_msg[i]);
	}

	const char mode_msg[] = "$PMTK886,3*2B\r\n"; //balloon mode
	for (int i = 0; i < strlen(mode_msg); i++) {
		uart_poll_out(gps_dev, mode_msg[i]);
	}

	char buf[MINMEA_MAX_SENTENCE_LENGTH];
	int buf_index = 0;
	while (true) {
		char byte;
		int ret;
		ret = uart_poll_in(gps_dev, &byte);
		if (ret == 0) {
			if (byte == '$') {
				buf_index = 0;
			}
			if (buf_index < sizeof(buf)) {
				buf[buf_index] = byte;
				buf_index++;
			}
			if (byte == '\n') {
				handle_nmea(buf, buf_index);
			}
			continue;
		}
		k_msleep(1);
	}
}
#endif

#if DT_ALIAS_EXISTS(bat_adc)
void vbat_thread(fjalar_t *fjalar, void *p2, void *p3) {

}
#endif