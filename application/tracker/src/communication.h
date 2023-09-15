#include <protocol.h>

#include "tracker.h"

struct padded_buf {
	uint8_t buf[PROTOCOL_BUFFER_LENGTH];
} __attribute__((aligned(4)));



void send_message(tracker_t *tracker, fjalar_message_t *msg);
void write_to_gs(tracker_t *tracker, uint8_t *buf, int len);
void write_to_fc(tracker_t *tracker, uint8_t *buf, int len);
void send_screen_command(tracker_t *tracker);

void init_communication(tracker_t *tracker);