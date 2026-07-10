#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/display/cfb.h>
#include <zephyr/sys/printk.h>
#include <stdio.h>

#define OLED_NODE  DT_NODELABEL(ssd1306)
#define OLED_ADDR  DT_REG_ADDR(OLED_NODE)

#define PWR_PIN 26  /* P0.26, active high  */
#define RST_PIN 23  /* P0.23, active low   */

static const struct device *gpio0 = DEVICE_DT_GET(DT_NODELABEL(gpio0));

static bool i2c_probe(const struct device *bus, uint8_t addr)
{
	struct i2c_msg msg;
	uint8_t dummy;

	msg.buf   = &dummy;
	msg.len   = 0U;
	msg.flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	return i2c_transfer(bus, &msg, 1, addr) == 0;
}

static void i2c_bus_scan(const struct device *bus)
{
	int found = 0;

	printk("[SCAN] scanning %s...\n", bus->name);
	for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
		if (i2c_probe(bus, addr)) {
			printk("[SCAN]   ACK at 0x%02X\n", addr);
			found++;
		}
	}
	printk("[SCAN] done, %d device(s)\n", found);
}

int main(void)
{
	const struct device *display = DEVICE_DT_GET(OLED_NODE);
	const struct device *bus     = DEVICE_DT_GET(DT_BUS(OLED_NODE));
	struct display_capabilities caps;
	char line[32];
	int rc, t = 0;

	printk("\n=== SSD1315 manual bring-up ===\n");

	if (!device_is_ready(gpio0) || !device_is_ready(bus)) {
		printk("[0] !! gpio0 or i2c bus not ready, stopping\n");
		return 0;
	}

	/* --- Step 1: reset line low BEFORE power (avoid latch-up) --- */
	rc = gpio_pin_configure(gpio0, RST_PIN, GPIO_OUTPUT_LOW);
	printk("[1] P0.23 (RST) configured LOW (in reset): rc=%d\n", rc);

	/* --- Step 2: power on --- */
	rc = gpio_pin_configure(gpio0, PWR_PIN, GPIO_OUTPUT_HIGH);
	printk("[2] P0.26 (PWR) configured HIGH: rc=%d\n", rc);
	printk("[2] >> measure P0.26 and display VDD with multimeter NOW <<\n");
	k_msleep(100);

	/* --- Step 3: release reset --- */
	rc = gpio_pin_set_raw(gpio0, RST_PIN, 1);
	printk("[3] P0.23 (RST) released HIGH: rc=%d\n", rc);
	k_msleep(120);

	/* --- Step 4: probe OLED specifically, then full scan --- */
	printk("[4] probe 0x3C: %s\n", i2c_probe(bus, 0x3C) ? "ACK" : "no ACK");
	printk("[4] probe 0x3D: %s\n", i2c_probe(bus, 0x3D) ? "ACK" : "no ACK");
	i2c_bus_scan(bus);

	/* --- Step 5: init display driver (deferred in DT) --- */
	rc = device_init(display);
	printk("[5] device_init(%s): %d %s\n", display->name, rc,
	       rc == 0 ? "(OK)" : "(FAIL)");
	if (rc != 0 || !device_is_ready(display)) {
		printk("[5] !! stopping. If no ACK in step 4 -> hardware:\n");
		printk("       - VDD present at panel?\n");
		printk("       - BS1/BS2 strapping = I2C mode? (BS1=1, BS2=0)\n");
		printk("       - D1+D2 tied together for I2C SDA?\n");
		return 0;
	}

	display_get_capabilities(display, &caps);
	printk("[6] caps: %ux%u, formats 0x%02x, current 0x%02x\n",
	       caps.x_resolution, caps.y_resolution,
	       caps.supported_pixel_formats, caps.current_pixel_format);

	rc = cfb_framebuffer_init(display);
	printk("[7] cfb_framebuffer_init: %d\n", rc);
	if (rc != 0) {
		return 0;
	}

	rc = cfb_framebuffer_clear(display, true);
	printk("[8] cfb_framebuffer_clear: %d\n", rc);
	rc = display_blanking_off(display);
	printk("[8] display_blanking_off: %d %s\n", rc,
	       rc == 0 ? "(panel ON)" : "(FAIL)");

	printk("[9] draw loop\n");
	while (1) {
		cfb_framebuffer_clear(display, false);
		cfb_print(display, "SSD1315 OK", 0, 0);
		snprintf(line, sizeof(line), "uptime %d s", t);
		cfb_print(display, line, 0, 16);
		rc = cfb_framebuffer_finalize(display);
		if (rc) {
			printk("[9] finalize failed: %d\n", rc);
		} else if (t % 10 == 0) {
			printk("[9] alive, t=%d\n", t);
		}
		t++;
		k_sleep(K_SECONDS(1));
	}
	return 0;
}