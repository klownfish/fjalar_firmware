#pragma once

#include "fjalar.h"
#include <protocol.h>

void handle_fjalar_message(fjalar_message_t *msg, fjalar_t *fjalar, enum com_channels channel);
void handle_fjalar_buf(struct protocol_state *ps, fjalar_t *fjalar, uint8_t buf[], int len, enum com_channels channel);
