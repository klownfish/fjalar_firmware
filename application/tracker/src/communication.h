#include <protocol.h>

#include "tracker.h"

struct padded_buf {
	uint8_t buf[PROTOCOL_BUFFER_LENGTH];
} __attribute__((aligned(4)));


void init_communication(tracker_t *tracker);