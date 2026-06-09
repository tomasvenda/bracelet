#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/sys/printk.h>
#include <zephyr/types.h>
#include <string.h>

#define FLASH_NODE DT_NODELABEL(mx25u32)
#if !DT_NODE_EXISTS(FLASH_NODE)
#error "Flash node mx25u32 not found in devicetree"
#endif

#define FLASH_TEST_OFFSET 0x00000
#define FLASH_TEST_SIZE 256

static void dump_hex(const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        if ((i % 16) == 0) {
            if (i != 0) {
                printk("\n");
            }
            printk("%04zx: ", i);
        }
        printk("%02x ", data[i]);
    }
    printk("\n");
}

int main(void)
{
    const struct device *flash = DEVICE_DT_GET(FLASH_NODE);
    uint8_t write_data[FLASH_TEST_SIZE];
    uint8_t read_data[FLASH_TEST_SIZE];
    const char test_string[] =
        "Zephyr SPI NOR flash test.\n"
        "Save this to external flash and read it back over UART.\n";
    int rc;
    size_t test_string_len = sizeof(test_string) - 1;

    printk("Flash test starting...\n");

    if (!device_is_ready(flash)) {
        printk("ERROR: Flash device is not ready\n");
        return -1;
    }

    memset(write_data, 0x00, sizeof(write_data));
    memcpy(write_data, test_string, test_string_len);
    for (size_t i = test_string_len; i < sizeof(write_data); i++) {
        write_data[i] = (uint8_t)(i & 0xFF);
    }

    struct flash_pages_info info;
    
    printk("Fetching flash page info...\n");
    rc = flash_get_page_info_by_offs(flash, FLASH_TEST_OFFSET, &info);
    if (rc != 0) {
        printk("ERROR: Unable to get flash page info for offset 0x%08x\n", FLASH_TEST_OFFSET);
        return -1;
    }
    
    size_t erase_size = info.size;

    printk("Flash device ready, erasing %zu bytes at offset 0x%08x...\n", erase_size, FLASH_TEST_OFFSET);
    rc = flash_erase(flash, FLASH_TEST_OFFSET, erase_size);
    if (rc != 0) {
        printk("ERROR: flash_erase failed: %d\n", rc);
        return -1;
    }

    printk("Writing %u bytes to flash at offset 0x%08x...\n", FLASH_TEST_SIZE, FLASH_TEST_OFFSET);
    rc = flash_write(flash, FLASH_TEST_OFFSET, write_data, FLASH_TEST_SIZE);
    if (rc != 0) {
        printk("ERROR: flash_write failed: %d\n", rc);
        return -1;
    }

    printk("Reading back %u bytes from flash...\n", FLASH_TEST_SIZE);
    rc = flash_read(flash, FLASH_TEST_OFFSET, read_data, FLASH_TEST_SIZE);
    if (rc != 0) {
        printk("ERROR: flash_read failed: %d\n", rc);
        return -1;
    }

    if (memcmp(write_data, read_data, FLASH_TEST_SIZE) != 0) {
        printk("ERROR: read data does not match written data\n");
        return -1;
    } else {
        printk("SUCCESS: read data matches written data\n");
    }

    printk("--- Saved flash content ---\n");
    dump_hex(read_data, FLASH_TEST_SIZE);
    printk("--- End of flash content ---\n");

    while (1) {
        k_msleep(5000);
    }
    return 0;
}
