#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/sys/printk.h>

#define FLASH_NODE DT_NODELABEL(extflash)

#if !DT_NODE_HAS_STATUS(FLASH_NODE, okay)
#error "extflash devicetree node is not okay"
#endif

int main(void)
{
	const struct device *flash_dev;
	uint8_t buf[16];
	int ret;

	k_sleep(K_MSEC(100));

	flash_dev = DEVICE_DT_GET(FLASH_NODE);
	if (!device_is_ready(flash_dev)) {
		printk("extflash device not ready\n");
		return 0;
	}

	printk("extflash read test: first 16 bytes...\n");

	ret = flash_read(flash_dev, 0, buf, sizeof(buf));
	if (ret) {
		printk("flash_read failed: %d\n", ret);
		return 0;
	}

	printk("Data @0x000000: ");
	for (int i = 0; i < sizeof(buf); i++) {
		printk("%02X ", buf[i]);
	}
	printk("\n");

	while (1) {
		k_sleep(K_SECONDS(1));
	}
}