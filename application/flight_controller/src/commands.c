#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <protocol.h>


#include "commands.h"
#include "fjalar.h"

LOG_MODULE_REGISTER(commands, CONFIG_APP_COMMANDS_LOG_LEVEL);

void handle_set_sudo(set_sudo_t *msg, fjalar_t *fjalar, enum com_channels channel);
void handle_ready_up(ready_up_t *msg, fjalar_t *fjalar, enum com_channels channel);
void handle_enter_idle(enter_idle_t *msg, fjalar_t *fjalar, enum com_channels channel);

void handle_fjalar_buf(struct protocol_state *ps, fjalar_t *fjalar, uint8_t *buf, int len, enum com_channels channel) {
    fjalar_message_t msg;
    for (int i = 0; i < len; i++) {
        int ret;
        ret = parse_fjalar_message(ps, buf[i], &msg);
        if (ret == 1) {
            handle_fjalar_message(&msg, fjalar, channel);
        } else if (ret == -1) {
            reset_protocol_state(ps);
        }
    }
}

void handle_fjalar_message(fjalar_message_t *msg, fjalar_t *fjalar, enum com_channels channel) {
    LOG_INF("handling msg with id %d", msg->data.which_data);
    switch (msg->data.which_data) {
        case FJALAR_DATA_SET_SUDO_TAG:
            handle_set_sudo(&msg->data.data.set_sudo, fjalar, channel);
            break;

        case FJALAR_DATA_READY_UP_TAG:
            handle_ready_up(&msg->data.data.ready_up, fjalar, channel);
            break;

        case FJALAR_DATA_ENTER_IDLE_TAG:
            handle_enter_idle(&msg->data.data.enter_idle, fjalar, channel);
            break;

        default:
            LOG_ERR("Unsupported message: %d", msg->data.which_data);
    }
}

void handle_set_sudo(set_sudo_t *msg, fjalar_t *fjalar, enum com_channels channel) {
    fjalar->sudo = msg->enabled;
}


void handle_ready_up(ready_up_t *msg, fjalar_t *fjalar, enum com_channels channel) {
    bool succesful;
    if (fjalar->flight_state == STATE_IDLE) {
        fjalar->flight_state = STATE_LAUNCHPAD;
        succesful = true;
    } else {
        succesful = false;
    }
}

void handle_enter_idle(enter_idle_t *msg, fjalar_t *fjalar, enum com_channels channel) {
    bool succesful;
    if (fjalar->sudo == true) {
        fjalar->flight_state = STATE_IDLE;
        succesful = true;
    } else {
        succesful = false;
    }
}