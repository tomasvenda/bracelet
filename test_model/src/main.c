#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/regulator.h>
#include <stdio.h>
#include <zephyr/drivers/gpio.h>

#define BMI270_NODE DT_NODELABEL(bmi270)
#define LDO2_NODE DT_NODELABEL(npm1300_ldsw2)

/* Get the I2C specification directly from the device tree */
static const struct i2c_dt_spec bmi_i2c = I2C_DT_SPEC_GET(BMI270_NODE);

/* BMI270 Hardware Registers */
#define BMI270_REG_CMD 0x7E
#define BMI270_CMD_FIFO_FLUSH 0xB0
#define BMI270_REG_FIFO_CONFIG_1 0x49
#define BMI270_REG_FIFO_LENGTH_0 0x24
#define BMI270_REG_FIFO_DATA 0x26

/* 1.5 seconds at 50Hz = 75 samples. 
   In headerless mode, 1 Accel sample = 6 bytes (X_LSB, X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB).
   Total payload = 450 bytes. */
#define TARGET_FIFO_BYTES 450
uint8_t fifo_buffer[TARGET_FIFO_BYTES];

static const struct gpio_dt_spec bmi_int = GPIO_DT_SPEC_GET(BMI270_NODE, irq_gpios);
static struct gpio_callback bmi_int_cb;
static struct k_work fall_work;

/* --- Boot PMIC --- */
static int power_up_imu_during_boot(void) {
    const struct device *const ldo2_dev = DEVICE_DT_GET(LDO2_NODE);
    regulator_enable(ldo2_dev);
    k_sleep(K_MSEC(100));
    return 0;
}

static void read_fifo_history(void)
{
    uint8_t len_buf[2];
    i2c_burst_read_dt(&bmi_i2c, BMI270_REG_FIFO_LENGTH_0, len_buf, 2);
    uint16_t fifo_len = len_buf[0] | ((len_buf[1] & 0x1F) << 8);
    uint16_t bytes_to_read = (fifo_len / 6) * 6;

    if (bytes_to_read > 0 && bytes_to_read <= TARGET_FIFO_BYTES) {
        i2c_burst_read_dt(&bmi_i2c, BMI270_REG_FIFO_DATA, fifo_buffer, bytes_to_read);
        for (int i = 0; i < bytes_to_read; i += 6) {
            int16_t raw_x = (int16_t)((fifo_buffer[i+1] << 8) | fifo_buffer[i]);
            int16_t raw_y = (int16_t)((fifo_buffer[i+3] << 8) | fifo_buffer[i+2]);
            int16_t raw_z = (int16_t)((fifo_buffer[i+5] << 8) | fifo_buffer[i+4]);
            float x = (raw_x / 16384.0f) * 9.80665f;
            float y = (raw_y / 16384.0f) * 9.80665f;
            float z = (raw_z / 16384.0f) * 9.80665f;
            printf("Sample -> X: %7.3f | Y: %7.3f | Z: %7.3f\n", x, y, z);
        }
    }
}

/* Runs on the system work queue, outside ISR context */
static void fall_work_handler(struct k_work *work)
{
    printf("\n*** FREE FALL / LOW-G EVENT DETECTED ***\n");

    /* Clear the latched interrupt by reading INT_STATUS1 (0x1D) */
    uint8_t int_status;
    i2c_reg_read_byte_dt(&bmi_i2c, 0x1D, &int_status);

    /* Flush + capture the 1.5s window you already implemented */
    i2c_reg_write_byte_dt(&bmi_i2c, BMI270_REG_CMD, BMI270_CMD_FIFO_FLUSH);
    k_sleep(K_MSEC(1500));
    read_fifo_history();
}

void bmi_int_handler(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins)
{
    k_work_submit(&fall_work);
}

static int configure_bmi270_interrupt(void)
{
    int ret;

    if (!gpio_is_ready_dt(&bmi_int)) {
        printf("ERROR: INT GPIO not ready\n");
        return -ENODEV;
    }

    ret = gpio_pin_configure_dt(&bmi_int, GPIO_INPUT);
    if (ret) return ret;

    ret = gpio_pin_interrupt_configure_dt(&bmi_int, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret) return ret;

    gpio_init_callback(&bmi_int_cb, bmi_int_handler, BIT(bmi_int.pin));
    gpio_add_callback(bmi_int.port, &bmi_int_cb);

    k_work_init(&fall_work, fall_work_handler);

    /* --- BMI270 side: route INT1 pin as push-pull, active-high, latched --- */
    i2c_reg_write_byte_dt(&bmi_i2c, 0x53, 0x0A); /* INT1_IO_CTRL: output enable, active high, push-pull */
    i2c_reg_write_byte_dt(&bmi_i2c, 0x55, 0x01); /* INT_LATCH: latched mode until INT_STATUS is read */

    return 0;
}


SYS_INIT(power_up_imu_during_boot, POST_KERNEL, 85);

int main(void)
{
    const struct device *const bmi_dev = DEVICE_DT_GET(BMI270_NODE);
    printf("\n*** Booting Hardware FIFO Test ***\n");

    if (!device_is_ready(bmi_dev) || !i2c_is_ready_dt(&bmi_i2c)) {
        printf("ERROR: Hardware not ready.\n");
        return 0;
    }

    /* 1. Use Zephyr to set standard configs */
    struct sensor_value full_scale = { .val1 = 2, .val2 = 0 };
    struct sensor_value sampling_freq = { .val1 = 50, .val2 = 0 };
    sensor_attr_set(bmi_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_FULL_SCALE, &full_scale);
    sensor_attr_set(bmi_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &sampling_freq);

    /* TRICK ZEPHYR: Force the driver to power on */
    printf("Waking up the accelerometer from deep sleep...\n");
    sensor_sample_fetch(bmi_dev);

    /* --- THE WHIPLASH FIX --- */
    /* 1. Disable Advanced Power Save (Register 0x7C -> 0x00).
          This stops the chip from auto-sleeping when the I2C bus is silent! */
    i2c_reg_write_byte_dt(&bmi_i2c, 0x7C, 0x00);

    /* 2. Lock the hardware ON (Register 0x7D -> 0x04). */
    i2c_reg_write_byte_dt(&bmi_i2c, 0x7D, 0x04);
    
    printf("Waiting for MEMS to stabilize...\n");
    k_sleep(K_SECONDS(1));
    /* ------------------------ */

    /* 2. Bypass Zephyr and configure the FIFO directly via I2C */
    uint8_t fifo_cfg;
    i2c_reg_read_byte_dt(&bmi_i2c, BMI270_REG_FIFO_CONFIG_1, &fifo_cfg);
    fifo_cfg |= (1 << 6);  /* Enable Accel in FIFO */
    fifo_cfg &= ~(1 << 4); /* Disable Header (forces contiguous 6-byte raw frames) */
    i2c_reg_write_byte_dt(&bmi_i2c, BMI270_REG_FIFO_CONFIG_1, fifo_cfg);

    printf("FIFO Configured. Simulating 10 seconds of system sleep...\n");
    k_sleep(K_SECONDS(10));
    
    /* 3. Flush the FIFO to clear the old data, then wait exactly 1.5 seconds */
    printf("Flushing FIFO and collecting exactly 1.5 seconds of historical data...\n");
    i2c_reg_write_byte_dt(&bmi_i2c, BMI270_REG_CMD, 0xB0); 
    k_sleep(K_MSEC(1500));

    /* 4. Check how many bytes the hardware recorded */
    uint8_t len_buf[2];
    i2c_burst_read_dt(&bmi_i2c, BMI270_REG_FIFO_LENGTH_0, len_buf, 2);
    uint16_t fifo_len = len_buf[0] | ((len_buf[1] & 0x1F) << 8);
    printf("Hardware FIFO contains: %d bytes\n", fifo_len);

    /* Calculate how many complete 6-byte samples we have */
    uint16_t bytes_to_read = (fifo_len / 6) * 6;

    if (bytes_to_read > 0 && bytes_to_read <= TARGET_FIFO_BYTES) {
        printf("Pulling %d bytes (%d samples) from historical buffer...\n", bytes_to_read, bytes_to_read / 6);
        
        /* 5. Pull the historical data dynamically based on what is available */
        i2c_burst_read_dt(&bmi_i2c, BMI270_REG_FIFO_DATA, fifo_buffer, bytes_to_read);
        
        printf("\n--- PLAYING BACK HISTORY ---\n");
        /* 6. Parse the raw bytes back into physical G-forces */
        for (int i = 0; i < bytes_to_read; i += 6) {
            int16_t raw_x = (int16_t)((fifo_buffer[i+1] << 8) | fifo_buffer[i]);
            int16_t raw_y = (int16_t)((fifo_buffer[i+3] << 8) | fifo_buffer[i+2]);
            int16_t raw_z = (int16_t)((fifo_buffer[i+5] << 8) | fifo_buffer[i+4]);
            
            float x = (raw_x / 16384.0f) * 9.80665f;
            float y = (raw_y / 16384.0f) * 9.80665f;
            float z = (raw_z / 16384.0f) * 9.80665f;

            int sample_num = (i / 6) + 1;
            printf("Past Sample %02d -> X: %7.3f | Y: %7.3f | Z: %7.3f\n", sample_num, x, y, z);
        }
    } else {
        printf("Not enough valid data in FIFO, or FIFO overflowed.\n");
    }

    while (1) { k_sleep(K_SECONDS(1)); }
    return 0;
}