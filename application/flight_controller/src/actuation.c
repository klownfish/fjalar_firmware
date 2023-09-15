#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>

#include "fjalar.h"

LOG_MODULE_REGISTER(actuation, CONFIG_APP_ACTUATION_LOG_LEVEL);

#define LED_THREAD_PRIORITY 7
#define LED_THREAD_STACK_SIZE 2048

#define BUZZER_THREAD_PRIORITY 7
#define BUZZER_THREAD_STACK_SIZE 2048

K_THREAD_STACK_DEFINE(led_thread_stack, LED_THREAD_STACK_SIZE);
struct k_thread led_thread_data;
k_tid_t led_thread_id;
void led_thread(fjalar_t *fjalar, void *, void *);

K_THREAD_STACK_DEFINE(buzzer_thread_stack, BUZZER_THREAD_STACK_SIZE);
struct k_thread buzzer_thread_data;
k_tid_t buzzer_thread_id;
void buzzer_thread(fjalar_t *fjalar, void *, void *);

void init_actuation(fjalar_t *fjalar) {
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

    led_thread_id = k_thread_create(
		&led_thread_data,
		led_thread_stack,
		K_THREAD_STACK_SIZEOF(led_thread_stack),
		(k_thread_entry_t) led_thread,
		fjalar, NULL, NULL,
		LED_THREAD_PRIORITY, 0, K_NO_WAIT
	);
	k_thread_name_set(led_thread_id, "led");

	#if DT_ALIAS_EXISTS(buzzer)
    buzzer_thread_id = k_thread_create(
		&buzzer_thread_data,
		buzzer_thread_stack,
		K_THREAD_STACK_SIZEOF(buzzer_thread_stack),
		(k_thread_entry_t) buzzer_thread,
		fjalar, NULL, NULL,
		BUZZER_THREAD_PRIORITY, 0, K_NO_WAIT
	);
	k_thread_name_set(buzzer_thread_id, "buzzer");
	#endif
}

void led_thread(fjalar_t *fjalar, void *p2, void *p3) {
    const struct gpio_dt_spec io_led_dt = GPIO_DT_SPEC_GET(DT_ALIAS(io_led), gpios);
    int ret = 0;
    ret |= gpio_pin_configure_dt(&io_led_dt, GPIO_OUTPUT_ACTIVE);
    if (ret) {
        LOG_ERR("Could not configure the IO-led");
    }

    while (true) {
        gpio_pin_toggle_dt(&io_led_dt);
        k_msleep(1000);
    }
}

#if DT_ALIAS_EXISTS(buzzer)
void buzzer_thread(fjalar_t *fjalar, void *p2, void *p3) {
    #if !CONFIG_BUZZER_ENABLED
    LOG_INF("Buzzer disabled");
    return;
    #endif
    int err;
    const struct pwm_dt_spec buzzer_dt = PWM_DT_SPEC_GET(DT_ALIAS(buzzer));
    if (!device_is_ready(buzzer_dt.dev)) {
        LOG_ERR("buzzer is not ready");
        return;
    }

    uint32_t period = 1e9 / 2000;
    err = pwm_set_dt(&buzzer_dt, period, period / 2);
    if (err) {
        LOG_ERR("Could not set buzzer");
    }

    while (true) {
        err = pwm_set_dt(&buzzer_dt, period, period / 2);
        k_msleep(100);
        err = pwm_set_dt(&buzzer_dt, period, 0);
        k_msleep(100);
        err = pwm_set_dt(&buzzer_dt, period, period / 2);
        k_msleep(100);
        err = pwm_set_dt(&buzzer_dt, period, 0);
        k_msleep(1500);
    }
}
#endif