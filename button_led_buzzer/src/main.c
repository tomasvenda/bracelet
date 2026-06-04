/*
 * button_led_buzzer
 * Press the emergency button to activate the red LED and buzzer.
 */

#include <assert.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/sys/printk.h>

#define BUTTON_NODE DT_ALIAS(sw0)
#define PWM_CTLR_NODE DT_NODELABEL(pwm0)

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(BUTTON_NODE, gpios);
static const struct device *const pwm_dev = DEVICE_DT_GET(PWM_CTLR_NODE);
static const uint32_t led_channel = 0U;
static const uint32_t buzzer_channel = 3U;

static struct gpio_callback button_cb;
static struct k_work button_work;

static void button_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    const uint32_t led_period = PWM_MSEC(20);
    const uint32_t led_pulse = led_period / 2U;
    const uint32_t buzzer_period = PWM_HZ(4000);
    const uint32_t buzzer_pulse = buzzer_period / 2U;

    pwm_set(pwm_dev, led_channel, led_period, led_pulse, PWM_POLARITY_NORMAL);
    pwm_set(pwm_dev, buzzer_channel, buzzer_period, buzzer_pulse, PWM_POLARITY_NORMAL);

    k_msleep(200);

    pwm_set(pwm_dev, led_channel, led_period, 0U, PWM_POLARITY_NORMAL);
    pwm_set(pwm_dev, buzzer_channel, buzzer_period, 0U, PWM_POLARITY_NORMAL);
}

static void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);

    printk("button_led_buzzer: button pressed\n");
    k_work_submit(&button_work);
}

int main(void)
{
    int ret;

    static_assert(DT_NODE_HAS_STATUS(BUTTON_NODE, okay), "Button alias sw0 is missing in devicetree");

    if (!device_is_ready(button.port)) {
        printk("Button GPIO device not ready\n");
        return 0;
    }

    if (!device_is_ready(pwm_dev)) {
        printk("PWM device not ready\n");
        return 0;
    }

    ret = gpio_pin_configure_dt(&button, GPIO_INPUT | GPIO_PULL_UP);
    if (ret < 0) {
        printk("Failed to configure button pin: %d\n", ret);
        return 0;
    }

    gpio_init_callback(&button_cb, button_pressed, BIT(button.pin));
    gpio_add_callback(button.port, &button_cb);

    ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret < 0) {
        printk("Failed to configure button interrupt: %d\n", ret);
        return 0;
    }

    k_work_init(&button_work, button_work_handler);

    printk("button_led_buzzer ready: press the button to flash LED and beep\n");

    while (1) {
        k_msleep(1000);
    }
}
