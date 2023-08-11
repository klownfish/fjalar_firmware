#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/console/tty.h>
#include <zephyr/drivers/lora.h>

#include <protocol.h>

#include "fjalar.h"
#include "commands.h"

LOG_MODULE_REGISTER(communication, CONFIG_APP_TELEMETRY_LOG_LEVEL);


#define LORA_THREAD_PRIORITY 7
#define LORA_THREAD_STACK_SIZE 1024

#define FLASH_THREAD_PRIORITY 7
#define FLASH_THREAD_STACK_SIZE 1024

#define UART_THREAD_PRIORITY 7
#define UART_THREAD_STACK_SIZE 1024

#define USB_THREAD_PRIORITY 7
#define USB_THREAD_STACK_SIZE 1024

#define SAMPLER_THREAD_PRIORITY 7
#define SAMPLER_THREAD_STACK_SIZE 1024

K_THREAD_STACK_DEFINE(sampler_thread_stack, SAMPLER_THREAD_STACK_SIZE);
struct k_thread sampler_thread_data;
k_tid_t sampler_thread_id;
void sampler_thread(fjalar_t *fjalar, void *p2, void *p3);

#if DT_ALIAS_EXISTS(lora)
K_THREAD_STACK_DEFINE(lora_thread_stack, LORA_THREAD_STACK_SIZE);
struct k_thread lora_thread_data;
k_tid_t lora_thread_id;
void lora_thread(fjalar_t *fjalar, void *p2, void *p3);
#endif

#if DT_ALIAS_EXISTS(external_uart)
K_THREAD_STACK_DEFINE(uart_thread_stack, UART_THREAD_STACK_SIZE);
struct k_thread uart_thread_data;
k_tid_t uart_thread_id;
void uart_thread(fjalar_t *fjalar, void *p2, void *p3);
#endif

#if DT_ALIAS_EXISTS(data_usb)
K_THREAD_STACK_DEFINE(usb_thread_stack, USB_THREAD_STACK_SIZE);
struct k_thread usb_thread_data;
k_tid_t usb_thread_id;
void usb_thread(fjalar_t *fjalar, void *p2, void *p3);
#endif

#if DT_ALIAS_EXISTS(data_flash)
K_THREAD_STACK_DEFINE(flash_thread_stack, FLASH_THREAD_STACK_SIZE);
struct k_thread flash_thread_data;
k_tid_t flash_thread_id;
void flash_thread(fjalar_t *fjalar, void *p2, void *p3);
#endif

struct padded_buf {
	uint8_t buf[PROTOCOL_BUFFER_LENGTH];
} __attribute__((aligned(4)));

_Static_assert(sizeof(struct padded_buf) % 4 == 0, "protocol buffer length is not aligned");

K_MSGQ_DEFINE(uart_msgq, sizeof(struct padded_buf), 32, 4);
K_MSGQ_DEFINE(flash_msgq, sizeof(struct padded_buf), 32, 4);
K_MSGQ_DEFINE(usb_msgq, sizeof(struct padded_buf), 32, 4);
K_MSGQ_DEFINE(lora_msgq, sizeof(struct padded_buf), 5, 4);


enum message_priority {
	MSG_PRIO_LOW,
	MSG_PRIO_HIGH,
};

void init_communication(fjalar_t *fjalar) {
	#if DT_ALIAS_EXISTS(lora)
	lora_thread_id = k_thread_create(
		&lora_thread_data,
		lora_thread_stack,
		K_THREAD_STACK_SIZEOF(lora_thread_stack),
		(k_thread_entry_t) lora_thread,
		fjalar, NULL, NULL,
		LORA_THREAD_PRIORITY, 0, K_NO_WAIT
	);
	#endif

	#if DT_ALIAS_EXISTS(data_usb)
	usb_thread_id = k_thread_create(
		&usb_thread_data,
		usb_thread_stack,
		K_THREAD_STACK_SIZEOF(usb_thread_stack),
		(k_thread_entry_t) usb_thread,
		fjalar, NULL, NULL,
		USB_THREAD_PRIORITY, 0, K_NO_WAIT
	);
	#endif

	#if DT_ALIAS_EXISTS(data_flash)
	flash_thread_id = k_thread_create(
		&flash_thread_data,
		flash_thread_stack,
		K_THREAD_STACK_SIZEOF(flash_thread_stack),
		(k_thread_entry_t) flash_thread,
		fjalar, NULL, NULL,
		FLASH_THREAD_PRIORITY, 0, K_NO_WAIT
	);
	#endif

	#if DT_ALIAS_EXISTS(external_uart)
	uart_thread_id = k_thread_create(
		&uart_thread_data,
		uart_thread_stack,
		K_THREAD_STACK_SIZEOF(uart_thread_stack),
		(k_thread_entry_t) uart_thread,
		fjalar, NULL, NULL,
		UART_THREAD_PRIORITY, 0, K_NO_WAIT
	);
	#endif

	sampler_thread_id = k_thread_create(
		&sampler_thread_data,
		sampler_thread_stack,
		K_THREAD_STACK_SIZEOF(sampler_thread_stack),
		(k_thread_entry_t) sampler_thread,
		fjalar, NULL, NULL,
		SAMPLER_THREAD_PRIORITY, 0, K_NO_WAIT
	);
}

void send_message(fjalar_t *fjalar, fjalar_message_t *msg, enum message_priority prio) {
	struct padded_buf pbuf;
	int size = encode_fjalar_message(msg, pbuf.buf);
	if (size < 0) {
		LOG_ERR("encode_fjalar_message failed");
		return;
	}
	switch(prio) {
		case MSG_PRIO_LOW:
			if (fjalar->flight_state == STATE_IDLE) {
				break;
			}
			#if DT_ALIAS_EXISTS(data_flash)
			if (k_msgq_put(&flash_msgq, &pbuf, K_NO_WAIT)) {
				LOG_WRN("could not insert into flash msgq");
			}
			#endif
			#if DT_ALIAS_EXISTS(external_uart)
			if (k_msgq_put(&uart_msgq, &pbuf, K_NO_WAIT)) {
				LOG_WRN("could not insert into uart msgq");
			}
			#endif
			#if DT_ALIAS_EXISTS(data_usb)
			if (k_msgq_put(&usb_msgq, &pbuf, K_NO_WAIT)) {
				LOG_INF("could not insert data usb msgq");
			}
			#endif
			break;

		case MSG_PRIO_HIGH:
			#if DT_ALIAS_EXISTS(data_flash)
			if (k_msgq_put(&flash_msgq, &pbuf, K_NO_WAIT)) {
				LOG_WRN("could not insert into flash msgq");
			}
			#endif
			#if DT_ALIAS_EXISTS(lora)
			if (k_msgq_put(&lora_msgq, &pbuf, K_NO_WAIT)) {
				LOG_ERR("could not insert into lora msgq");
			}
			#endif
			#if DT_ALIAS_EXISTS(external_uart)
			if (k_msgq_put(&uart_msgq, &pbuf, K_NO_WAIT)) {
				LOG_WRN("could not insert into uart msgq");
			}
			#endif
			#if DT_ALIAS_EXISTS(data_usb)
			if (k_msgq_put(&usb_msgq, &pbuf, K_NO_WAIT)) {
				LOG_INF("could not insert data usb msgq");
			}
			#endif
			break;
	}
}

void send_response(fjalar_t *fjalar, fjalar_message_t *msg, enum com_channels channel) {

}

void sampler_thread(fjalar_t *fjalar, void *p2, void *p3) {
	while (true) {
		k_msleep(1000);
		fjalar_message_t msg;
		msg.time = k_uptime_get_32();
		msg.data.which_data = FJALAR_DATA_TELEMETRY_PACKET_TAG;
		msg.data.data.telemetry_packet.altitude = fjalar->altitude - fjalar->ground_level;
		msg.data.data.telemetry_packet.az = fjalar->az;
		msg.data.data.telemetry_packet.flight_state = fjalar->flight_state;
		msg.data.data.telemetry_packet.velocity = fjalar->velocity;
		msg.data.data.telemetry_packet.battery = 10.6;
		msg.data.data.telemetry_packet.latitude = 39.342443;
		msg.data.data.telemetry_packet.longitude = 39.342434;
		msg.data.data.telemetry_packet.flash_address = 0;
		send_message(fjalar, &msg, MSG_PRIO_HIGH);
	}
}

#if DT_ALIAS_EXISTS(data_flash)
void flash_thread(fjalar_t *fjalar, void *p2, void *p3) {
	const struct device *flash_dev = DEVICE_DT_GET(DT_ALIAS(data_flash));
    if (!device_is_ready(flash_dev)) {
		LOG_ERR("Flash not ready");
		return;
	}
}
#endif

#if DT_ALIAS_EXISTS(lora)
void lora_thread(fjalar_t *fjalar, void *p2, void *p3) {
	const struct device *lora_dev = DEVICE_DT_GET(DT_ALIAS(lora));

    if (!device_is_ready(lora_dev)) {
		LOG_ERR("LoRa not ready");
		return;
	}

}
#endif

#if DT_ALIAS_EXISTS(external_uart)
void uart_thread(fjalar_t *fjalar, void *p2, void *p3) {
	const struct device *uart_dev = DEVICE_DT_GET(DT_ALIAS(external_uart));
    if (!device_is_ready(uart_dev)) {
		LOG_ERR("External uart not ready");
		return;
	}
	struct protocol_state ps;
	reset_protocol_state(&ps);
	while(1) {
		// rx
		int ret;
		uint8_t byte;
		ret = uart_poll_in(uart_dev, &byte);
		if (ret == 0) {
			handle_fjalar_buf(&ps, fjalar, &byte, 1, COM_CHAN_EXT_UART);
			continue;
		} else
		if (ret != -1) {
			LOG_ERR("UART read error");
		}

		// tx
		uint8_t buf[PROTOCOL_BUFFER_LENGTH];
		ret = k_msgq_get(&uart_msgq, buf, K_NO_WAIT);
		if (ret == 0) {
			int size = get_encoded_message_length(buf);
			for (int i = 0; i < size; i++) {
				uart_poll_out(uart_dev, buf[i]);
			}
		}
		k_msleep(1);
	}
}
#endif

#if DT_ALIAS_EXISTS(data_usb)
void usb_thread(fjalar_t *fjalar, void *p2, void *p3) {
	const struct device *usb_dev = DEVICE_DT_GET(DT_ALIAS(data_usb));
    if (!device_is_ready(usb_dev)) {
		LOG_ERR("data usb not ready");
		return;
	}
	struct protocol_state ps;
	reset_protocol_state(&ps);
	while(1) {
		// rx
		int ret;
		uint8_t byte;
		ret = uart_poll_in(usb_dev, &byte);
		if (ret == 0) {
			handle_fjalar_buf(&ps, fjalar, &byte, 1, COM_CHAN_USB);
			continue;
		} else
		if (ret != -1) {
			LOG_ERR("USB read error");
		}

		// tx
		uint8_t buf[PROTOCOL_BUFFER_LENGTH];
		ret = k_msgq_get(&usb_msgq, buf, K_NO_WAIT);
		if (ret == 0) {
			int size = get_encoded_message_length(buf);
			for (int i = 0; i < size; i++) {
				uart_poll_out(usb_dev, buf[i]);
			}
		}
		k_msleep(1);
	}
}
#endif