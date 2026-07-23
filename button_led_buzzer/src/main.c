/*
 * BUZZER, LED and BUTTON TESTING CODE:
 *
 * Desired behavior:
 *   - Short press (release before 1.5s) -> cycle LED color, beep buzzer
 *   - Long press (held for 1.5s, LED turns off immediately, DURING the hold)
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(button_led_test, LOG_LEVEL_INF);

#define BUTTON_NODE   DT_ALIAS(sw0)
#define PWM_CTLR_NODE DT_NODELABEL(pwm0)
#define LDO1_NODE     DT_NODELABEL(npm1300_ldo1)

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(BUTTON_NODE, gpios);
static const struct device *const pwm_dev = DEVICE_DT_GET(PWM_CTLR_NODE);
static const struct device *const ldo1_dev = DEVICE_DT_GET(LDO1_NODE);

#define PWM_CH_RED    0U
#define PWM_CH_GREEN  1U
#define PWM_CH_BLUE   2U
#define PWM_CH_BUZZER  3U         
#define PWM_UNIFIED_PERIOD PWM_HZ(4000)
#define PWM_LED_PULSE (PWM_UNIFIED_PERIOD / 2U)
#define PWM_BUZZ_PULSE (PWM_UNIFIED_PERIOD / 2U)  

#define LONG_PRESS_MS 1500
#define DEBOUNCE_MS   40
#define BEEP_MS 250 

static struct gpio_callback button_cb_data;
static struct k_work_delayable button_debounce_work;
static bool button_was_pressed;
static bool long_press_fired;   /* prevents double action on release */

enum led_color { LED_OFF, LED_RED, LED_GREEN, LED_BLUE, LED_COLOR_COUNT };
static enum led_color current_color = LED_OFF;

static void short_beep(void)
{
    pwm_set(pwm_dev, PWM_CH_BUZZER, PWM_UNIFIED_PERIOD, PWM_BUZZ_PULSE, PWM_POLARITY_NORMAL);
    k_sleep(K_MSEC(BEEP_MS));
    pwm_set(pwm_dev, PWM_CH_BUZZER, PWM_UNIFIED_PERIOD, 0U, PWM_POLARITY_NORMAL);
}

static void set_led(enum led_color color)
{
    pwm_set(pwm_dev, PWM_CH_RED,   PWM_UNIFIED_PERIOD, 0U, PWM_POLARITY_NORMAL);
    pwm_set(pwm_dev, PWM_CH_GREEN, PWM_UNIFIED_PERIOD, 0U, PWM_POLARITY_NORMAL);
    pwm_set(pwm_dev, PWM_CH_BLUE,  PWM_UNIFIED_PERIOD, 0U, PWM_POLARITY_NORMAL);

    switch (color) {
    case LED_RED:
        pwm_set(pwm_dev, PWM_CH_RED, PWM_UNIFIED_PERIOD, PWM_LED_PULSE, PWM_POLARITY_NORMAL);
        break;
    case LED_GREEN:
        pwm_set(pwm_dev, PWM_CH_GREEN, PWM_UNIFIED_PERIOD, PWM_LED_PULSE, PWM_POLARITY_NORMAL);
        break;
    case LED_BLUE:
        pwm_set(pwm_dev, PWM_CH_BLUE, PWM_UNIFIED_PERIOD, PWM_LED_PULSE, PWM_POLARITY_NORMAL);
        break;
    case LED_OFF:
    default:
        break;
    }

    current_color = color;
    LOG_INF("[LED] color -> %d", (int)color);

    if (color = LED_OFF) {
        short_beep();
    }
}

/* Fires exactly once, LONG_PRESS_MS after a confirmed press begins,
 * as long as the button is still held down at that moment. */
static void long_press_timer_fn(struct k_timer *timer_id)
{
    if (button_was_pressed) {
        LOG_INF("[BUTTON] Long-press threshold reached WHILE HELD -> LED OFF");
        set_led(LED_OFF);
        long_press_fired = true;
    }
}
K_TIMER_DEFINE(long_press_timer, long_press_timer_fn, NULL);

static void button_debounce_fn(struct k_work *work)
{
    bool pressed_now = gpio_pin_get_dt(&button) > 0;

    if (pressed_now == button_was_pressed) {
        gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_BOTH);
        return;
    }

    if (pressed_now) {
        /* Confirmed press start: arm the long-press timer */
        button_was_pressed = true;
        long_press_fired = false;
        k_timer_start(&long_press_timer, K_MSEC(LONG_PRESS_MS), K_NO_WAIT);
    } else {
        /* Confirmed release */
        k_timer_stop(&long_press_timer);

        if (!long_press_fired) {
            /* Released before the long-press threshold -> short press */
            enum led_color next = (current_color + 1) % LED_COLOR_COUNT;
            if (next == LED_OFF) {
                next = LED_RED;
            }
            LOG_INF("[BUTTON] Short press -> next color");
            set_led(next);
        }
        /* If long_press_fired is true, LED was already turned off during
         * the hold -- do nothing else on release. */

        button_was_pressed = false;
    }

    gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_BOTH);
}

static void button_pressed_isr(const struct device *dev,
                                struct gpio_callback *cb, uint32_t pins)
{
    gpio_pin_interrupt_configure_dt(&button, GPIO_INT_DISABLE);
    k_work_reschedule(&button_debounce_work, K_MSEC(DEBOUNCE_MS));
}

int main(void)
{
    LOG_INF("=== BUTTON/LED TEST HARNESS ===");

    if (!device_is_ready(ldo1_dev)) {
        LOG_ERR("LDO1 not ready!");
        return -1;
    }
    regulator_enable(ldo1_dev);
    k_msleep(5);

    if (!device_is_ready(pwm_dev)) {
        LOG_ERR("PWM device not ready!");
        return -1;
    }

    if (!gpio_is_ready_dt(&button)) {
        LOG_ERR("Button GPIO not ready!");
        return -1;
    }

    k_work_init_delayable(&button_debounce_work, button_debounce_fn);

    gpio_pin_configure_dt(&button, GPIO_INPUT | GPIO_PULL_UP);
    gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_BOTH);
    gpio_init_callback(&button_cb_data, button_pressed_isr, BIT(button.pin));
    gpio_add_callback(button.port, &button_cb_data);

    LOG_INF("Ready. Short press = cycle color, long press = OFF (fires while held).");

    set_led(LED_OFF);

    while (1) {
        k_sleep(K_SECONDS(10));
    }

    return 0;
}