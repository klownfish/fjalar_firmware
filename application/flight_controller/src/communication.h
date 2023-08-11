#pragma once

#include "fjalar.h"
#include "protocol.h"

enum message_priority {
	MSG_PRIO_LOW,
	MSG_PRIO_HIGH,
};

void init_communication(fjalar_t *fjalar);
void send_message(fjalar_t *fjalar, fjalar_message_t *msg, enum message_priority prio);
void send_response(fjalar_t *fjalar, fjalar_message_t *msg, enum com_channels channel);