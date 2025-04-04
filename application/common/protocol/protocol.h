#pragma once

#include "pb.h"
#include "schema.pb.h"

#define PROTOCOL_HEADER_SIZE 2
#define PROTOCOL_FOOTER_SIZE 2
#define PROTOCOL_BUFFER_LENGTH (PROTOCOL_HEADER_SIZE + PROTOCOL_FOOTER_SIZE + FJALAR_MESSAGE_SIZE)

enum com_channels {
    COM_CHAN_USB,
    COM_CHAN_EXT_UART,
    COM_CHAN_LORA,
    COM_CHAN_FLASH,
    COM_CHAN_MAX,
};

enum protocol_states {
    PROT_STATE_ALIGNMENT,
    PROT_STATE_LENGTH,
    PROT_STATE_DATA,
    PROT_STATE_CHECKSUM0,
    PROT_STATE_CHECKSUM1
};

struct protocol_state {
    enum protocol_states state;
    uint8_t data[FJALAR_MESSAGE_SIZE];
    int data_index;
    uint16_t crc;
    uint8_t length;
};

int encode_fjalar_message(fjalar_message_t *data, uint8_t *buf);
void reset_protocol_state(struct protocol_state *ps);
int parse_fjalar_message(struct protocol_state *ps, uint8_t byte, fjalar_message_t *message);

int get_encoded_message_length(uint8_t *buf);

#define PROTOCOL_ZEPHYR_LORA_CONFIG { \
    .bandwidth = BW_500_KHZ, \
    .coding_rate = CR_4_6, \
    .datarate = 	SF_8, \
    .frequency = 437000000, \
    .iq_inverted = false, \
    .preamble_len = 8, \
    .public_network = false, \
    .tx = 0, \
    .tx_power = 10 \
};
