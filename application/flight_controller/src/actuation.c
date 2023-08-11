#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>

#include "fjalar.h"

LOG_MODULE_REGISTER(actuation, CONFIG_APP_ACTUATION_LOG_LEVEL);

void led_thread(fjalar_t *fjalar, void *p2, void *p3) {
    const struct gpio_dt_spec io_led_dt = GPIO_DT_SPEC_GET(DT_ALIAS(io_led), gpios);
    int ret = 0;
    ret |= gpio_pin_configure_dt(&io_led_dt, GPIO_OUTPUT_ACTIVE);
    if (ret) {
        LOG_ERR("Could not configure the IO-led");
    }

    while (true) {
        switch (fjalar->flight_state) {
            case STATE_IDLE:

            default:
                //beep
                //beep
        }
    }
}

void buzzer_thread(fjalar_t *fjalar, void *p2, void *p3) {

}

void pyro_thread(fjalar_t *fjalar, void *p2, void *p3) {
    const struct gpio_dt_spec pyro0_dt = GPIO_DT_SPEC_GET(DT_ALIAS(pyro0), gpios);
    const struct gpio_dt_spec pyro1_dt = GPIO_DT_SPEC_GET(DT_ALIAS(pyro1), gpios);
    const struct gpio_dt_spec pyro2_dt = GPIO_DT_SPEC_GET(DT_ALIAS(pyro2), gpios);
    const struct gpio_dt_spec pyro0_sense_dt = GPIO_DT_SPEC_GET(DT_ALIAS(pyro0_sense), gpios);
    const struct gpio_dt_spec pyro1_sense_dt = GPIO_DT_SPEC_GET(DT_ALIAS(pyro1_sense), gpios);
    const struct gpio_dt_spec pyro2_sense_dt = GPIO_DT_SPEC_GET(DT_ALIAS(pyro2_sense), gpios);

    int ret = 0;
    ret |= gpio_pin_set_dt(&pyro0_dt, 0);
    ret |= gpio_pin_set_dt(&pyro1_dt, 0);
    ret |= gpio_pin_set_dt(&pyro2_dt, 0);
    ret |= gpio_pin_configure_dt(&pyro0_dt, GPIO_OUTPUT_ACTIVE);
    ret |= gpio_pin_configure_dt(&pyro1_dt, GPIO_OUTPUT_ACTIVE);
    ret |= gpio_pin_configure_dt(&pyro2_dt, GPIO_OUTPUT_ACTIVE);
    ret |= gpio_pin_set_dt(&pyro0_dt, 0);
    ret |= gpio_pin_set_dt(&pyro1_dt, 0);
    ret |= gpio_pin_set_dt(&pyro2_dt, 0);

    ret |= gpio_pin_configure_dt(&pyro0_sense_dt, GPIO_INPUT);
    ret |= gpio_pin_configure_dt(&pyro1_sense_dt, GPIO_INPUT);
    ret |= gpio_pin_configure_dt(&pyro2_sense_dt, GPIO_INPUT);
    if (ret) {
        LOG_ERR("Could not configure pyro pins");
    }
}