#include <zephyr/kernel.h>
#include <zephyr/drivers/display.h>
#include <zephyr/display/cfb.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>

#include "tracker.h"
#include "display.h"

LOG_MODULE_REGISTER(app_display, CONFIG_APP_DISPLAY_LOG_LEVEL);

#define DRAWING_THREAD_PRIORITY 7
#define DRAWING_THREAD_STACK_SIZE 1024

K_THREAD_STACK_DEFINE(drawing_thread_stack, DRAWING_THREAD_STACK_SIZE);
struct k_thread drawing_thread_data;
k_tid_t drawing_thread_id;
void drawing_thread(tracker_t *tracker, void *p2, void *p3);

uint8_t font_width;
uint8_t font_height;

const struct device *display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));

void init_display(tracker_t *tracker) {
    drawing_thread_id = k_thread_create(
		&drawing_thread_data,
		drawing_thread_stack,
		K_THREAD_STACK_SIZEOF(drawing_thread_stack),
		(k_thread_entry_t) drawing_thread,
		tracker, NULL, NULL,
		DRAWING_THREAD_PRIORITY, 0, K_NO_WAIT
	);
}


void drawing_thread(tracker_t *tracker, void *p2, void *p3) {

    // init screen
    if (!device_is_ready(display_dev)) {
        LOG_ERR("Display not ready");
    }
    if (display_set_pixel_format(display_dev, PIXEL_FORMAT_MONO10) != 0) {
        LOG_ERR("Failed to set required pixel format\n");
    }
    if (cfb_framebuffer_init(display_dev)) {
        LOG_ERR("Framebuffer initialization failed!\n");
    }
    cfb_framebuffer_clear(display_dev, true);
    display_blanking_off(display_dev);
    for (int idx = 0; idx < 42; idx++) {
        if (cfb_get_font_size(display_dev, idx, &font_width, &font_height)) {
			break;
		}
		cfb_framebuffer_set_font(display_dev, idx);
	}

    //init gpio
    const struct gpio_dt_spec touch_sw = GPIO_DT_SPEC_GET(DT_ALIAS(sw1), gpios);
    gpio_pin_configure_dt(&touch_sw, GPIO_INPUT);

    enum screen_frames last_frame = tracker->current_frame;
    draw_frame(tracker, last_frame);
    int ret;
    while (true) {
        ret = gpio_pin_get_dt(&touch_sw);
        if (last_frame != tracker->current_frame || ret) {
            k_msleep(200); // for the double click lol
            if (last_frame != tracker->current_frame || ret) {
                draw_frame(tracker, tracker->current_frame);
                k_msleep(800);
                last_frame = tracker->current_frame;
            }
        }
        k_msleep(100);
    }
}

const static char* state_to_string(enum flight_state state) {
    switch(state) {
        case FLIGHT_STATE_IDLE:
            return "Idle";
        case FLIGHT_STATE_LAUNCHPAD:
            return "Launchpad";
        case FLIGHT_STATE_BOOST:
            return "Boost";
        case FLIGHT_STATE_COAST:
            return "Coast";
        case FLIGHT_STATE_FREE_FALL:
            return "Free Fall";
        case FLIGHT_STATE_DROGUE_DESCENT:
            return "Drogue";
        case FLIGHT_STATE_MAIN_DESCENT:
            return "Main";
        case FLIGHT_STATE_LANDED:
            return "Landed";
        default:
            return "error";
            LOG_ERR("Invalid flight state");
    }
}

void draw_frame(tracker_t *tracker, enum screen_frames frame) {
    LOG_DBG("Draw frame");
    cfb_framebuffer_clear(display_dev, false);

    uint8_t buf[128];
    int len;
    int i = 0;
    int x_offset = 10;
    switch (frame) {
        case FRAME_INFO:
            i = 3;
            len = snprintk(buf, sizeof(buf), "info");
            cfb_draw_text(display_dev, buf, x_offset, i++ * font_height);
            break;
        case FRAME_TRACKING:
            len = snprintk(buf, sizeof(buf), "tracking");
            cfb_draw_text(display_dev, buf, x_offset, i++ * font_height);
            break;
        case FRAME_TELEMETRY:
            len = snprintk(buf, sizeof(buf), "alt: %fm", tracker->telemetry.altitude);
            cfb_draw_text(display_dev, buf, x_offset, i++ * font_height);
            len = snprintk(buf, sizeof(buf), "vel: %fm/s", tracker->telemetry.velocity);
            cfb_draw_text(display_dev, buf, x_offset, i++ * font_height);
            len = snprintk(buf, sizeof(buf), "az: %fm/s2", tracker->telemetry.az);
            cfb_draw_text(display_dev, buf, x_offset, i++ * font_height);
            len = snprintk(buf, sizeof(buf), "p1:%d p2:%d p3:%d", tracker->telemetry.pyro0_connected, tracker->telemetry.pyro1_connected, tracker->telemetry.pyro2_connected);
            cfb_draw_text(display_dev, buf, x_offset, i++ * font_height);
            len = snprintk(buf, sizeof(buf), "rssi: %fdBm", tracker->local_rssi);
            cfb_draw_text(display_dev, buf, x_offset, i++ * font_height);
            len = snprintk(buf, sizeof(buf), "state: %s", state_to_string(tracker->telemetry.flight_state));
            cfb_draw_text(display_dev, buf, x_offset, i++ * font_height);
            len = snprintk(buf, sizeof(buf), "flash: %f%", tracker->telemetry.flash_address);
            cfb_draw_text(display_dev, buf, x_offset, i++ * font_height);
            len = snprintk(buf, sizeof(buf), "volt: %fV", tracker->telemetry.battery);
            cfb_draw_text(display_dev, buf, x_offset, i++ * font_height);
            break;
        case FRAME_PYRO1:
            i = 3;
            len = snprintk(buf, sizeof(buf), "PYRO 1");
            cfb_draw_text(display_dev, buf, x_offset, i++ * font_height);
            len = snprintk(buf, sizeof(buf), "state: %d", tracker->pyros_enabled.pyro0);
            cfb_draw_text(display_dev, buf, x_offset, i++ * font_height);
            break;
        case FRAME_PYRO2:
            i = 3;
            len = snprintk(buf, sizeof(buf), "PYRO 2");
            cfb_draw_text(display_dev, buf, x_offset, i++ * font_height);
            len = snprintk(buf, sizeof(buf), "state: %d", tracker->pyros_enabled.pyro1);
            cfb_draw_text(display_dev, buf, x_offset, i++ * font_height);
            break;
        case FRAME_PYRO3:
            i = 3;
            len = snprintk(buf, sizeof(buf), "PYRO 3");
            cfb_draw_text(display_dev, buf, x_offset, i++ * font_height);
            len = snprintk(buf, sizeof(buf), "state: %d", tracker->pyros_enabled.pyro2);
            cfb_draw_text(display_dev, buf, x_offset, i++ * font_height);
            break;
        case FRAME_GET_READY:
            i = 3;
            len = snprintk(buf, sizeof(buf), "BECOME READY?");
            cfb_draw_text(display_dev, buf, x_offset, i++ * font_height);
            len = snprintk(buf, sizeof(buf), "state: %s", state_to_string(tracker->telemetry.flight_state));
            cfb_draw_text(display_dev, buf, x_offset, i++ * font_height);
            break;
        case FRAME_ENTER_IDLE:
            i = 3;
            len = snprintk(buf, sizeof(buf), "BECOME IDLE?");
            cfb_draw_text(display_dev, buf, x_offset, i++ * font_height);
            len = snprintk(buf, sizeof(buf), "state: %s", state_to_string(tracker->telemetry.flight_state));
            cfb_draw_text(display_dev, buf, x_offset, i++ * font_height);
            break;
        case FRAME_ENTER_SUDO:
            i = 3;
            len = snprintk(buf, sizeof(buf), "SUDO MODE?");
            cfb_draw_text(display_dev, buf, x_offset, i++ * font_height);
            len = snprintk(buf, sizeof(buf), "Sudo: %d", 1);
            cfb_draw_text(display_dev, buf, x_offset, i++ * font_height);
            break;
        case FRAME_MAX:
            break;
    }
    cfb_framebuffer_finalize(display_dev);
}