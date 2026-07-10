#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include "bmi2_defs.h"

/* Defined in main.c */
extern const struct i2c_dt_spec bmi_i2c;

BMI2_INTF_RETURN_TYPE bmi2_i2c_read(uint8_t reg_addr, uint8_t *data,
                                    uint32_t len, void *intf_ptr)
{
    ARG_UNUSED(intf_ptr);
    return i2c_burst_read_dt(&bmi_i2c, reg_addr, data, len);
}

BMI2_INTF_RETURN_TYPE bmi2_i2c_write(uint8_t reg_addr, const uint8_t *data,
                                     uint32_t len, void *intf_ptr)
{
    ARG_UNUSED(intf_ptr);
    return i2c_burst_write_dt(&bmi_i2c, reg_addr, (uint8_t *)data, len);
}

void bmi2_delay_us(uint32_t period, void *intf_ptr)
{
    ARG_UNUSED(intf_ptr);
    k_busy_wait(period);
}