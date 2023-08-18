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
#define LORA_THREAD_STACK_SIZE 1024

#define USB_THREAD_PRIORITY 7
#define USB_THREAD_STACK_SIZE 1024

#define UART_THREAD_PRIORITY 7
#define UART_THREAD_STACK_SIZE 1024

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
void lora_configure(const struct device *dev, bool transmit) {
    struct lora_modem_config config;
    config.bandwidth = BW_500_KHZ;
    config.coding_rate = CR_4_8;
    config.datarate = 0; // what is this
    config.frequency = 437000000;
    config.iq_inverted = false;
    config.preamble_len = 8; //LoRa WAN uses this so I guess it's good
    config.public_network = false;
    config.tx = transmit;
    config.tx_power = 20;
    lora_config(dev, &config);
}
tracker_t *tracker_lora;
void lora_cb(const struct device *dev, uint8_t *buf, uint16_t size, int16_t rssi, int8_t snr) {
    struct protocol_state ps;
    struct fjalar_message msg;
    reset_protocol_state(&ps);
    for (int i = 0; i < size; i++) {
        bool had_msg = parse_fjalar_message(&ps, buf[i], &msg);
        if (had_msg && i == size) {
            send_to_gs(tracker_lora, buf, size);
            handle_fjalar_message(tracker_lora, &msg);
        }
    }
}

void lora_thread(tracker_t *tracker, void* p2, void* p3) {
    const struct device *lora_dev = DEVICE_DT_GET(DT_ALIAS(telemetry_lora));
    struct lora_modem_config config;
    config.bandwidth = BW_500_KHZ;
    config.coding_rate = CR_4_8;
    config.datarate = 0; // what is this
    config.frequency = 437000000;
    config.iq_inverted = false;
    config.preamble_len = 8; //LoRa WAN uses this so I guess it's good
    config.public_network = false;
    config.tx = true;
    config.tx_power = 20;
    tracker_lora = tracker;
    if (!device_is_ready(lora_dev)) {
        LOG_ERR("LoRa is not ready");
    }
    while (true) {
        int ret;
        struct padded_buf buf;
        lora_configure(lora_dev, LORA_RECEIVE);
        lora_recv_async(lora_dev, (lora_recv_cb) lora_cb);
        ret = k_msgq_get(&lora_msgq, &buf, K_FOREVER);
        if (ret == 0) {
            lora_configure(lora_dev, LORA_TRANSMIT);
            int size = get_encoded_message_length(buf.buf);
            lora_send(lora_dev, buf.buf, size);
        } else {
            LOG_ERR("lora msgq error");
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
    uint8_t rx_buf[300]; //TODO: please mangage lengths properly
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
            bool got_msg = parse_fjalar_message(&ps, byte, &msg);
            if (got_msg) {
                LOG_INF("received message from telemetry_uart");
                handle_fjalar_message(tracker, &msg);
                send_to_gs(tracker, rx_buf, rx_index);
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