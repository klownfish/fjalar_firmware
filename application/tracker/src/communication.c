#include <zephyr/kernel.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <protocol.h>

#include "communication.h"
#include "tracker.h"

LOG_MODULE_REGISTER(com, CONFIG_APP_COM_LOG_LEVEL);

#define LORA_TRANSMIT 1
#define LORA_RECEIVE 0

#define LORA_THREAD_PRIORITY 7
#define LORA_THREAD_STACK_SIZE 4096

#define USB_THREAD_PRIORITY 7
#define USB_THREAD_STACK_SIZE 4096

#define UART_THREAD_PRIORITY 7
#define UART_THREAD_STACK_SIZE 4096

#if DT_ALIAS_EXISTS(telemetry_lora)
K_THREAD_STACK_DEFINE(lora_thread_stack, LORA_THREAD_STACK_SIZE);
struct k_thread lora_thread_data;
k_tid_t lora_thread_id;
void lora_thread(tracker_t *tracker, void *p2, void *p3);
#endif

#if DT_ALIAS_EXISTS(telemetry_uart)
K_THREAD_STACK_DEFINE(uart_thread_stack, UART_THREAD_STACK_SIZE);
struct k_thread uart_thread_data;
k_tid_t uart_thread_id;
void uart_thread(tracker_t *tracker, void *p2, void *p3);
#endif

K_MSGQ_DEFINE(lora_msgq, sizeof(struct padded_buf), 10, 4);
K_MSGQ_DEFINE(usb_msgq, sizeof(struct padded_buf), 10, 4);
K_MSGQ_DEFINE(uart_msgq, sizeof(struct padded_buf), 10, 4);

void init_communication(tracker_t *tracker) {
    #if DT_ALIAS_EXISTS(telemetry_lora)
	lora_thread_id = k_thread_create(
		&lora_thread_data,
		lora_thread_stack,
		K_THREAD_STACK_SIZEOF(lora_thread_stack),
		(k_thread_entry_t) lora_thread,
		tracker, NULL, NULL,
		LORA_THREAD_PRIORITY, 0, K_NO_WAIT
	);
    #endif

    #if DT_ALIAS_EXISTS(telemetry_uart)
	uart_thread_id = k_thread_create(
		&uart_thread_data,
		uart_thread_stack,
		K_THREAD_STACK_SIZEOF(uart_thread_stack),
		(k_thread_entry_t) uart_thread,
		tracker, NULL, NULL,
		UART_THREAD_PRIORITY, 0, K_NO_WAIT
	);
    #endif
}

void handle_fjalar_message(tracker_t *tracker, struct fjalar_message *msg) {
    switch (msg->data.which_data) {
        case FJALAR_DATA_TELEMETRY_PACKET_TAG:
            tracker->telemetry = msg->data.data.telemetry_packet;
            break;
        case FJALAR_DATA_PYROS_ENABLED_TAG:
            tracker->pyros_enabled = msg->data.data.pyros_enabled;
            break;
        default:
    }
}

void send_to_gs(tracker_t *tracker, uint8_t *buf, int len) {
}

void send_to_fc(tracker_t *tracker, uint8_t *buf, int len) {

}

#if DT_ALIAS_EXISTS(telemetry_lora)
int lora_configure(const struct device *dev, bool transmit) {
    // struct lora_modem_config config;
    struct lora_modem_config config = PROTOCOL_ZEPHYR_LORA_CONFIG;
    config.tx = transmit;
	config.tx_power = 20;
    int ret = lora_config(dev, &config);
	if (ret < 0) {
		LOG_ERR("Could not configure lora %d", ret);
	}
	return ret;
}

struct lora_rx {
	uint8_t buf[PROTOCOL_BUFFER_LENGTH];
	uint16_t size;
	int16_t rssi;
	int8_t snr;
} __attribute__((aligned(4)));

K_MSGQ_DEFINE(lora_rx_msgq, sizeof(struct lora_rx), 5, 4);

void lora_cb(const struct device *dev, uint8_t *buf, uint16_t size, int16_t rssi, int8_t snr) {
    struct lora_rx rx;
	rx.size = MIN(size, PROTOCOL_BUFFER_LENGTH);
	memcpy(rx.buf, buf, rx.size);
	rx.rssi = rssi;
	rx.snr = snr;
	k_msgq_put(&lora_rx_msgq, &rx, K_NO_WAIT);
}

void lora_thread(tracker_t *tracker, void* p2, void* p3) {
    const struct device *lora_dev = DEVICE_DT_GET(DT_ALIAS(telemetry_lora));
    if (!device_is_ready(lora_dev)) {
        LOG_ERR("LoRa is not ready");
        return;
    }
    struct lora_rx rx;
    int ret;
    struct padded_buf pbuf;
    fjalar_message_t msg;
    while (true) {
		struct k_poll_event events[2] = {
			K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_MSGQ_DATA_AVAILABLE,
											K_POLL_MODE_NOTIFY_ONLY,
											&lora_msgq),
			K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_MSGQ_DATA_AVAILABLE,
											K_POLL_MODE_NOTIFY_ONLY,
											&lora_rx_msgq),
		};

        ret = lora_configure(lora_dev, LORA_RECEIVE);
		if (ret) {
			LOG_ERR("LoRa rx configure failed");
		} else {
			LOG_DBG("LoRa rxing");
		}
		lora_recv_async(lora_dev, lora_cb);
        LOG_DBG("HAHA YEESSSS");
		k_poll(&events[1], 2, K_MSEC(10000)); //poll only rx first to not interrupt messages
		k_poll(events, 2, K_FOREVER);

		ret = k_msgq_get(&lora_rx_msgq, &rx, K_NO_WAIT);
        if (ret == 0) {
			events[1].state = K_POLL_STATE_NOT_READY;
			LOG_INF("received LoRa");
			struct protocol_state ps;
			reset_protocol_state(&ps);
            for (int i = 0; i < rx.size; i++) {
                int got_msg = parse_fjalar_message(&ps, rx.buf[i], &msg);
                if (got_msg == 1 && i == rx.size) {
                    LOG_INF("parsed message from lora");
                    handle_fjalar_message(tracker, &msg);
                    send_to_gs(tracker, rx.buf, rx.size);
                } else
                if (got_msg == -1) {
                    LOG_ERR("Invalid message from lora");
                }
            }
        }

        ret = k_msgq_get(&lora_msgq, &pbuf, K_NO_WAIT);
        if (ret == 0) {
			events[0].state = K_POLL_STATE_NOT_READY;
			lora_recv_async(lora_dev, NULL);
            ret = lora_configure(lora_dev, LORA_TRANSMIT);
			if (ret) {
				LOG_ERR("LORA tx configure failed");
			}else {
				LOG_DBG("LoRa txing");
			}
            int size = get_encoded_message_length(pbuf.buf);
            ret = lora_send(lora_dev, pbuf.buf, size);
			if (ret) {
				LOG_ERR("Could not send LoRa message");
			}
        }
    }
}
#endif

#if DT_ALIAS_EXISTS(telemetry_uart)
void uart_thread(tracker_t *tracker, void* p2, void* p3) {
	const struct device *uart_dev = DEVICE_DT_GET(DT_ALIAS(telemetry_uart));
    if (!device_is_ready(uart_dev)) {
		LOG_ERR("telemetry uart not ready");
		return;
	}
	struct protocol_state ps;
	reset_protocol_state(&ps);
    uint8_t rx_buf[PROTOCOL_BUFFER_LENGTH]; //TODO: please mangage lengths properly
    uint8_t rx_index = 0;
	while(1) {
		// rx
		int ret;
		uint8_t byte;
		ret = uart_poll_in(uart_dev, &byte);
		if (ret == 0) {
            LOG_DBG("received byte on telemetry_uart");
            if (rx_index < sizeof(rx_buf)) {
                rx_buf[rx_index] = byte;
                rx_index++;
            }
            fjalar_message_t msg;
            int got_msg = parse_fjalar_message(&ps, byte, &msg);
            if (got_msg == 1) {
                LOG_INF("received message from telemetry_uart");
                handle_fjalar_message(tracker, &msg);
                send_to_gs(tracker, rx_buf, rx_index);
                rx_index = 0;
            } else
            if (got_msg == -1) {
                rx_index = 0;
            }
			continue;
		} else
		if (ret != -1) {
			LOG_ERR("UART read error");
		}

		// tx
		struct padded_buf pbuf;
		ret = k_msgq_get(&uart_msgq, &pbuf, K_NO_WAIT);
		if (ret == 0) {
			int size = get_encoded_message_length(pbuf.buf);
			for (int i = 0; i < size; i++) {
				uart_poll_out(uart_dev, pbuf.buf[i]);
			}
		}
		k_msleep(1);
	}
}
#endif

#if DT_ALIAS_EXISTS(commands_usb)
void usb_thread(tracker_t *tracker, void* p2, void* p3) {

}
#endif