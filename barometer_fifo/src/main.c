#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(icp_test, LOG_LEVEL_INF);

#define ICP20100_NODE DT_NODELABEL(icp20100)
const struct i2c_dt_spec icp_i2c = I2C_DT_SPEC_GET(ICP20100_NODE);

/* Hardware Registers */
#define ICP20100_REG_MODE_SELECT 0xC0
#define ICP20100_REG_FIFO_FILL   0xC4  
#define ICP20100_REG_FIFO_BASE   0xFA  
#define ICP20100_REG_DUMMY       0x00  

#define ICP20100_FIFO_LEVEL_MASK 0x1F
#define ICP20100_CMD_FIFO_FLUSH  0x80

static uint8_t icp_fifo_buffer[16 * 6];

/* Helper to flush the FIFO cleanly */
static void flush_pressure_fifo(void)
{
    uint8_t current_fill = 0;
    i2c_reg_read_byte_dt(&icp_i2c, ICP20100_REG_FIFO_FILL, &current_fill);
    current_fill |= ICP20100_CMD_FIFO_FLUSH;
    i2c_reg_write_byte_dt(&icp_i2c, ICP20100_REG_FIFO_FILL, current_fill);
}

/* Reads the FIFO and outputs pressure in kPa. */
static uint16_t read_pressure_fifo(float *out_pressure, uint16_t max_samples)
{
    uint8_t fifo_fill_reg = 0;
    int ret;

    ret = i2c_reg_read_byte_dt(&icp_i2c, ICP20100_REG_FIFO_FILL, &fifo_fill_reg);
    if (ret) return 0;

    uint8_t fifo_count = fifo_fill_reg & ICP20100_FIFO_LEVEL_MASK;
    
    if (fifo_count == 0) return 0;
    if (fifo_count > max_samples) fifo_count = max_samples;

    uint16_t bytes_to_read = fifo_count * 6;
    ret = i2c_burst_read_dt(&icp_i2c, ICP20100_REG_FIFO_BASE, icp_fifo_buffer, bytes_to_read);
    if (ret) return 0;

    uint8_t dummy;
    i2c_reg_read_byte_dt(&icp_i2c, ICP20100_REG_DUMMY, &dummy);

    for (int i = 0; i < fifo_count; i++) {
        uint8_t *packet = &icp_fifo_buffer[i * 6];
        
        int32_t data_press = ((int32_t)(packet[2] & 0x0f) << 16) | 
                             ((int32_t)packet[1] << 8) | 
                             packet[0];
        
        if (data_press & 0x080000) {
            data_press |= 0xFFF00000; 
        }
        
        out_pressure[i] = ((float)(data_press) * 40.0f / 131072.0f) + 70.0f;
    }

    return fifo_count;
}

int main(void)
{
    LOG_INF("ICP-20100 FIFO Sleep Test Starting...");

    if (!i2c_is_ready_dt(&icp_i2c)) {
        LOG_ERR("I2C bus not ready");
        return 0;
    }

    /* 1. Force the sensor into Continuous Mode @ 25Hz (Mode 0) 
     * Mode Select Bit 3 = Continuous Mode (1)
     * All other bits 0 = Normal power, Mode 0, Press+Temp FIFO 
     */
    LOG_INF("Configuring sensor for continuous measurement (25Hz)...");
    i2c_reg_write_byte_dt(&icp_i2c, ICP20100_REG_MODE_SELECT, 0x08);
    
    k_sleep(K_MSEC(100)); /* Let it stabilize */

    float pressure_data[16];
    uint16_t count = 0;

    /* 2. Flush FIFO so we start with a clean slate */
    LOG_INF("Flushing FIFO...");
    flush_pressure_fifo();
    k_sleep(K_MSEC(200)); /* Wait 200ms (should generate about 5 samples) */

    /* 3. The "BEFORE" Read */
    count = read_pressure_fifo(pressure_data, 16);
    LOG_INF("--- BEFORE 10s SLEEP ---");
    LOG_INF("Read %d samples from FIFO", count);
    for (int i = 0; i < count; i++) {
        LOG_INF("  Sample %d: %.3f kPa", i, (double)pressure_data[i]);
    }

    /* Clear it out again before the long sleep */
    flush_pressure_fifo();

    /* 4. The Deep Sleep */
    LOG_INF("CPU sleeping for 10 seconds... (Hardware FIFO should fill and overflow)");
    k_sleep(K_SECONDS(10));

    /* 5. The "AFTER" Read */
    count = read_pressure_fifo(pressure_data, 16);
    LOG_INF("--- AFTER 10s SLEEP ---");
    LOG_INF("Read %d samples from FIFO", count);
    for (int i = 0; i < count; i++) {
        LOG_INF("  Sample %d: %.3f kPa", i, (double)pressure_data[i]);
    }

    LOG_INF("Test Complete!");
    return 0;
}