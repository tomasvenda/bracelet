/*
 * button_led_buzzer
 * Press the emergency button to activate the red LED and buzzer.
 */

#include <assert.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/regulator.h> /* REQUIRED for PMIC control */
#include <zephyr/sys/printk.h>

#define BUTTON_NODE DT_ALIAS(sw0)
#define PWM_CTLR_NODE DT_NODELABEL(pwm0)

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(BUTTON_NODE, gpios);
static const struct device *const pwm_dev = DEVICE_DT_GET(PWM_CTLR_NODE);
static const uint32_t led_channel = 3U;
static const uint32_t buzzer_channel = 0U;

static struct gpio_callback button_cb;
static struct k_work button_work;

static void button_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    /* Unify the frequency! Both MUST share the same period on pwm0 */
    const uint32_t unified_period = PWM_HZ(4000); 
    const uint32_t unified_pulse = unified_period / 2U; /* 50% duty cycle */

    /* Both channels will successfully activate now */
    //pwm_set(pwm_dev, led_channel, unified_period, unified_pulse, PWM_POLARITY_NORMAL);
    pwm_set(pwm_dev, buzzer_channel, unified_period, unified_pulse, PWM_POLARITY_NORMAL);

    k_msleep(200);

    //pwm_set(pwm_dev, led_channel, unified_period, 0U, PWM_POLARITY_NORMAL);
    pwm_set(pwm_dev, buzzer_channel, unified_period, 0U, PWM_POLARITY_NORMAL);
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
    const struct device *const ldo1_dev = DEVICE_DT_GET(DT_NODELABEL(npm1300_ldo1));

    if (!device_is_ready(ldo1_dev)) {
        printk("Error: PMIC LDO1 not ready.\n");
        return 0;
    }
    
    /* Force 3.3V so the Buck Converter EN pin is driven HIGH */
    ret = regulator_set_voltage(ldo1_dev, 3300000, 3300000);
    if (ret == 0) {
        regulator_enable(ldo1_dev);
        printk("LDO1 Powered ON at 3.3V: Buck Converter is active.\n");
    } else {
        printk("Failed to set LDO1 voltage. Err: %d\n", ret);
    }

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