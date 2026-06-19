/*
 * button_led_buzzer
 * Press the emergency button to activate the red LED and buzzer.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/sys/printk.h>

#define BUTTON_NODE DT_ALIAS(sw0)
#define PWM_CTLR_NODE DT_NODELABEL(pwm0)

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(BUTTON_NODE, gpios);

/* Direct reference to the PWM controller */
static const struct device *const pwm_dev = DEVICE_DT_GET(PWM_CTLR_NODE);

/* 
 * Based on app.overlay:
 * led_red is mapped to PWM channel 0
 * pwm_buzzer is mapped to PWM channel 3
 */
static const uint32_t red_led_channel = 0U;
static const uint32_t green_led_channel = 1U;
static const uint32_t blue_led_channel = 2U;
static const uint32_t buzzer_channel = 3U;

static struct gpio_callback button_cb;
static struct k_work button_work;

static void button_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    /* 
     * The nRF PWM hardware shares ONE period counter across all channels.
     * We MUST use the same period for both the LED and the Buzzer.
     * We use 4000 Hz because it is the resonant frequency required for the buzzer's max volume.
     */
    uint32_t unified_period = PWM_HZ(4000);
    
    uint32_t buzzer_pulse = unified_period / 2U; /* 50% duty cycle for MAX volume */
   /* Use a 50% duty cycle for the LEDs too. 
    * At 4000 Hz, it will appear as a solid bright light without triggering 
    * the 100% duty-cycle hardware wrap-around bug. */
    uint32_t led_pulse = unified_period / 2U; 
    
/* Turn ON Buzzer */
    pwm_set(pwm_dev, buzzer_channel, unified_period, buzzer_pulse, PWM_POLARITY_NORMAL);

    pwm_set(pwm_dev, red_led_channel, unified_period, led_pulse, PWM_POLARITY_NORMAL);  // Turn on red LED
    k_msleep(700);
    pwm_set(pwm_dev, red_led_channel, unified_period, 0U, PWM_POLARITY_NORMAL);         // Turn off red LED

    pwm_set(pwm_dev, green_led_channel, unified_period, led_pulse, PWM_POLARITY_NORMAL);
    k_msleep(700);
    pwm_set(pwm_dev, green_led_channel, unified_period, 0U, PWM_POLARITY_NORMAL);

    pwm_set(pwm_dev, blue_led_channel, unified_period, led_pulse, PWM_POLARITY_NORMAL);
    k_msleep(700);
    pwm_set(pwm_dev, blue_led_channel, unified_period, 0U, PWM_POLARITY_NORMAL);

    /* Turn OFF buzzer */
    pwm_set(pwm_dev, buzzer_channel, unified_period, 0U, PWM_POLARITY_NORMAL);
}

static void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);

    printk("Emergency button pressed! Activating LED and Buzzer for 2 seconds...\n");
    k_work_submit(&button_work);
}

int main(void)
{
    int ret;
    
    /* --- PMIC Initialization --- */
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

    /* --- Peripheral Initialization --- */
    if (!device_is_ready(button.port)) {
        printk("Error: Button GPIO device not ready.\n");
        return 0;
    }

    if (!device_is_ready(pwm_dev)) {
        printk("Error: PWM device not ready.\n");
        return 0;
    }

    /* --- Button Interrupt Configuration --- */
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

    printk("System Ready: Press the emergency button to flash LED and beep.\n");

    while (1) {
        k_msleep(1000);
    }
    
    return 0;
}