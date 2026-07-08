#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/regulator.h>
#include <stdio.h>

#include "bmi2_defs.h"
#include "bmi270_legacy.h"

BMI2_INTF_RETURN_TYPE bmi2_i2c_read(uint8_t reg_addr, uint8_t *data,
                                    uint32_t len, void *intf_ptr);
BMI2_INTF_RETURN_TYPE bmi2_i2c_write(uint8_t reg_addr, const uint8_t *data,
                                     uint32_t len, void *intf_ptr);
void bmi2_delay_us(uint32_t period, void *intf_ptr);

#define BMI270_NODE DT_NODELABEL(bmi270)
#define LDO2_NODE   DT_NODELABEL(npm1300_ldsw2)

const struct i2c_dt_spec bmi_i2c = I2C_DT_SPEC_GET(BMI270_NODE);
static const struct gpio_dt_spec bmi_int = GPIO_DT_SPEC_GET(BMI270_NODE, irq_gpios);
static struct gpio_callback bmi_int_cb;
K_SEM_DEFINE(bmi_irq_sem, 0, 1);

static volatile uint32_t isr_count = 0;   /* counted from ISR */

static int power_up_imu_during_boot(void)
{
    const struct device *const ldo2_dev = DEVICE_DT_GET(LDO2_NODE);
    regulator_enable(ldo2_dev);
    k_sleep(K_MSEC(100));
    return 0;
}
SYS_INIT(power_up_imu_during_boot, POST_KERNEL, 85);

void bmi_isr_handler(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins)
{
    isr_count++;
    k_sem_give(&bmi_irq_sem);
}

static void bmi2_error_codes_print_result(int8_t rslt)
{
    if (rslt != BMI2_OK) {
        printf("BMI2 API error code: %d\n", rslt);
    }
}

/* Poll int_status + read INT1 pin level, so we can see BOTH sides */
static void diagnostic_wait(const char *label, uint16_t status_mask)
{
    printf("\n[DIAG] Waiting for %s. Will report every 1s.\n", label);
    printf("[DIAG] Move/drop the board now.\n");

    int elapsed = 0;
    uint32_t last_isr_seen = isr_count;

    while (1) {
        /* Non-blocking check on the semaphore */
        if (k_sem_take(&bmi_irq_sem, K_MSEC(1000)) == 0) {
            printf("[DIAG] *** ISR fired! Semaphore taken. isr_count=%u ***\n", isr_count);
            return;
        }

        elapsed++;

        /* Read what the chip thinks */
        uint16_t int_status = 0;
        int8_t r = bmi2_get_int_status(&int_status, NULL);   /* dummy — see note */
        (void)r;

        /* Read the physical GPIO level */
        int pin_lvl = gpio_pin_get_dt(&bmi_int);

        /* Report every second */
        printf("[DIAG] t=%ds  isr_count=%u  int1_pin_level=%d  int_status=0x%04x  match=%s\n",
               elapsed, isr_count, pin_lvl, int_status,
               (int_status & status_mask) ? "YES" : "no");

        if (isr_count != last_isr_seen) {
            printf("[DIAG] ISR count changed since last report (%u -> %u)\n",
                   last_isr_seen, isr_count);
            last_isr_seen = isr_count;
        }

        if (elapsed >= 30) {
            printf("[DIAG] 30s timeout — moving on without event.\n");
            return;
        }
    }
}


int main(void)
{
    int8_t rslt;
    struct bmi2_dev dev = { 0 };
    struct bmi2_sens_config config[2] = { 0 };
    uint8_t high_g_sens_list[2] = { BMI2_ACCEL, BMI2_HIGH_G };
    uint8_t low_g_sens_list[2]  = { BMI2_ACCEL, BMI2_LOW_G };
    struct bmi2_feat_sensor_data sensor_data = { 0 };
    uint16_t int_status = 0;
    uint8_t high_g_out = 0;

    struct bmi2_sens_int_config high_g_int = { .type = BMI2_HIGH_G, .hw_int_pin = BMI2_INT1 };
    struct bmi2_sens_int_config low_g_int  = { .type = BMI2_LOW_G,  .hw_int_pin = BMI2_INT1 };

    printf("\n*** BMI270 High-G / Low-G Zephyr App ***\n");

    printf("[STEP 1] Checking I2C + GPIO readiness...\n");
    if (!i2c_is_ready_dt(&bmi_i2c)) {
        printf("  ERROR: I2C bus not ready\n");
        return 0;
    }
    if (!gpio_is_ready_dt(&bmi_int)) {
        printf("  ERROR: INT GPIO not ready\n");
        return 0;
    }
    printf("  OK. INT pin: port=%s pin=%d\n", bmi_int.port->name, bmi_int.pin);

    printf("[STEP 2] Wiring Bosch API function pointers...\n");
    dev.intf = BMI2_I2C_INTF;
    dev.read = bmi2_i2c_read;
    dev.write = bmi2_i2c_write;
    dev.delay_us = bmi2_delay_us;
    dev.intf_ptr = NULL;
    dev.read_write_len = 32;
    printf("  OK.\n");

    printf("[STEP 3] Calling bmi270_legacy_init()... (uploads ~8KB firmware, takes ~200ms)\n");
    rslt = bmi270_legacy_init(&dev);
    printf("  bmi270_legacy_init returned %d (%s)\n", rslt, rslt == BMI2_OK ? "OK" : "FAIL");
    if (rslt != BMI2_OK) {
        printf("  Halting.\n");
        return 0;
    }
    printf("  Chip ID: 0x%02x (expected 0x24 for legacy)\n", dev.chip_id);

    printf("[STEP 4] Configuring INT1 pin (push-pull, active-high, output enable)...\n");
    struct bmi2_int_pin_config int_pin_cfg = { 0 };
    int_pin_cfg.pin_type = BMI2_INT1;
    int_pin_cfg.int_latch = BMI2_INT_LATCH;   /* was NON_LATCH */
    int_pin_cfg.pin_cfg[0].lvl = BMI2_INT_ACTIVE_HIGH;
    int_pin_cfg.pin_cfg[0].od  = BMI2_INT_PUSH_PULL;
    int_pin_cfg.pin_cfg[0].output_en = BMI2_INT_OUTPUT_ENABLE;
    int_pin_cfg.pin_cfg[0].input_en  = BMI2_INT_INPUT_DISABLE;
    rslt = bmi2_set_int_pin_config(&int_pin_cfg, &dev);
    printf("  bmi2_set_int_pin_config returned %d\n", rslt);

    printf("[STEP 5] Configuring Zephyr GPIO callback...\n");
    gpio_pin_configure_dt(&bmi_int, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(&bmi_int, GPIO_INT_LEVEL_ACTIVE);
    gpio_init_callback(&bmi_int_cb, bmi_isr_handler, BIT(bmi_int.pin));
    gpio_add_callback(bmi_int.port, &bmi_int_cb);
    int initial_lvl = gpio_pin_get_dt(&bmi_int);
    printf("  OK. Initial INT1 pin level = %d (expected 0 when idle)\n", initial_lvl);

    printf("[STEP 6] Enabling accel + high-g feature...\n");
    rslt = bmi270_legacy_sensor_enable(high_g_sens_list, 2, &dev);
    printf("  sensor_enable returned %d\n", rslt);

    config[0].type = BMI2_HIGH_G;
    config[1].type = BMI2_LOW_G;
    rslt = bmi270_legacy_get_sensor_config(config, 2, &dev);
    printf("HIGH-G defaults: threshold=0x%04x  hyst=0x%04x  dur=0x%04x\n",
        config[0].cfg.high_g.threshold,
        config[0].cfg.high_g.hysteresis,
        config[0].cfg.high_g.duration);
    printf("LOW-G  defaults: threshold=0x%04x  hyst=0x%04x  dur=0x%04x\n",
        config[1].cfg.low_g.threshold,
        config[1].cfg.low_g.hysteresis,
        config[1].cfg.low_g.duration);

    /* HIGH-G: fire on ~2g impact, sustained for 40ms */
    config[0].cfg.high_g.threshold  = 0x1000;   /* ~2g   (0x800 = 1g) */
    config[0].cfg.high_g.hysteresis = 0x0200;
    config[0].cfg.high_g.duration   = 0x0002;   /* 2 samples @ 50Hz = 40ms */
    config[0].cfg.high_g.select_x   = BMI2_ENABLE;
    config[0].cfg.high_g.select_y   = BMI2_ENABLE;
    config[0].cfg.high_g.select_z   = BMI2_ENABLE;

    /* LOW-G: fire when |a| < ~0.3g for at least 80ms (real free fall) */
    config[1].cfg.low_g.threshold  = 0x0260;   /* ~0.3g */
    config[1].cfg.low_g.hysteresis = 0x0100;
    config[1].cfg.low_g.duration   = 0x0004;   /* 4 samples @ 50Hz = 80ms */

    rslt = bmi270_legacy_set_sensor_config(config, 2, &dev);
    bmi2_error_codes_print_result(rslt);
    

    printf("[STEP 7] Mapping high-g feature to INT1...\n");
    rslt = bmi270_legacy_map_feat_int(&high_g_int, 1, &dev);
    printf("  map_feat_int returned %d\n", rslt);

    sensor_data.type = BMI2_HIGH_G;

    /* ---------- Diagnostic wait for high-g ---------- */
    printf("\n>>> READY. Shake or tap the board sharply for HIGH-G <<<\n");
    printf(">>> Reports every 1s. Will time out after 30s. <<<\n\n");

    int elapsed = 0;
    bool got_event = false;
    while (elapsed < 30) {
        if (k_sem_take(&bmi_irq_sem, K_MSEC(1000)) == 0) {
            printf("[t=%ds] *** ISR FIRED — semaphore taken (isr_count=%u) ***\n",
                   elapsed, isr_count);
            got_event = true;
            break;
        }
        elapsed++;

        /* Poll the chip's status register directly */
        int_status = 0;
        bmi2_get_int_status(&int_status, &dev);
        int pin_lvl = gpio_pin_get_dt(&bmi_int);

        printf("[t=%2ds] isr_count=%u  int1_gpio=%d  int_status=0x%04x  high_g_bit=%s\n",
               elapsed, isr_count, pin_lvl, int_status,
               (int_status & BMI270_LEGACY_HIGH_G_STATUS_MASK) ? "SET" : "clear");
    }

    if (!got_event) {
        printf("\n[TIMEOUT] No high-g interrupt in 30s. Skipping to low-g section anyway.\n");
    } else {
        rslt = bmi2_get_int_status(&int_status, &dev);
        printf("Post-ISR int_status = 0x%04x\n", int_status);

        if (int_status & BMI270_LEGACY_HIGH_G_STATUS_MASK) {
            printf("High-g interrupt confirmed by status register\n");
            rslt = bmi270_legacy_get_feature_data(&sensor_data, 1, &dev);
            high_g_out = sensor_data.sens_data.high_g_output;
            printf("high_g_out = 0x%02x\n", high_g_out);
            if (high_g_out & BMI270_LEGACY_HIGH_G_DETECT_X) printf("  X-axis\n");
            if (high_g_out & BMI270_LEGACY_HIGH_G_DETECT_Y) printf("  Y-axis\n");
            if (high_g_out & BMI270_LEGACY_HIGH_G_DETECT_Z) printf("  Z-axis\n");
            printf("  %s axis\n",
                   (high_g_out & BMI270_LEGACY_HIGH_G_DETECT_SIGN) ? "negative" : "positive");
        } else {
            printf("WARNING: ISR fired but HIGH_G_STATUS_MASK is not set. "
                   "The interrupt came from something else.\n");
        }
    }

    k_sem_reset(&bmi_irq_sem);

    /* ---------- LOW-G ---------- */
    printf("\n[STEP 8] Switching to low-g...\n");
    rslt = bmi270_legacy_sensor_disable(high_g_sens_list, 2, &dev);
    printf("  disable high-g returned %d\n", rslt);
    rslt = bmi270_legacy_sensor_enable(low_g_sens_list, 2, &dev);
    printf("  enable low-g returned %d\n", rslt);
    rslt = bmi270_legacy_map_feat_int(&low_g_int, 1, &dev);
    printf("  map low-g to INT1 returned %d\n", rslt);

    printf("\n>>> READY. Drop the board (or hold still and toss gently) for LOW-G <<<\n\n");

    elapsed = 0;
    got_event = false;
    while (elapsed < 30) {
        if (k_sem_take(&bmi_irq_sem, K_MSEC(1000)) == 0) {
            printf("[t=%ds] *** ISR FIRED (isr_count=%u) ***\n", elapsed, isr_count);
            got_event = true;
            break;
        }
        elapsed++;
        int_status = 0;
        bmi2_get_int_status(&int_status, &dev);
        int pin_lvl = gpio_pin_get_dt(&bmi_int);
        printf("[t=%2ds] isr_count=%u  int1_gpio=%d  int_status=0x%04x  low_g_bit=%s\n",
               elapsed, isr_count, pin_lvl, int_status,
               (int_status & BMI270_LEGACY_LOW_G_STATUS_MASK) ? "SET" : "clear");
    }

    if (got_event) {
        bmi2_get_int_status(&int_status, &dev);
        printf("Post-ISR int_status = 0x%04x\n", int_status);
        if (int_status & BMI270_LEGACY_LOW_G_STATUS_MASK) {
            printf("Low-g (free fall) confirmed by status register\n");
        }
    } else {
        printf("[TIMEOUT] No low-g event in 30s.\n");
    }

    printf("\nAll done. Idling.\n");
    while (1) { k_sleep(K_SECONDS(1)); }
    return 0;
}