#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/display.h>
#include <lvgl.h>

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

void power_thread(void *p1, void *p2, void *p3);

K_THREAD_DEFINE(power_thread_id, 512, power_thread, NULL, NULL, NULL, 7, 0, 0);

// void handle_power() {
// 	gpio_pin_configure_dt(&gpio_pwr_en, GPIO_OUTPUT_ACTIVE);
// 	volatile uint32_t* reset_reason_reg = (0x40005000 + 0x400); //non-secure partition
// 	if ((*reset_reason_reg & 1) == 1) { //first bit is reset
// 		gpio_pin_set_dt(&gpio_pwr_en, 0);
// 	} else {
// 		gpio_pin_set_dt(&gpio_pwr_en, 0);
// 	}
// }

int main(void)
{

	int ret;

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	int count = 0;
	while (1) {
		ret = gpio_pin_toggle_dt(&led);
		if (ret < 0) {
			return 0;
		}
		k_msleep(5000);
		count++;
	}
	return 0;
}

void power_thread(void *p1, void *p2, void *p3) {
	const struct gpio_dt_spec gpio_button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
	const struct gpio_dt_spec gpio_pwr_en = GPIO_DT_SPEC_GET(DT_ALIAS(pwr_en), gpios);

	gpio_pin_configure_dt(&gpio_button, GPIO_INPUT);

	gpio_pin_set_dt(&gpio_pwr_en, 1);

	int pressed_for = 0;
	while (true) {
		if (gpio_pin_get_dt(&gpio_button)) {
			pressed_for++;
			if (pressed_for > 50) {
				gpio_pin_set_dt(&gpio_pwr_en, 0);
			}
		} else {
			pressed_for = 0;
		}
		k_msleep(100);
	}
}