#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(flash_spi_test, LOG_LEVEL_INF);

/* Devices */
const struct device *spi_dev    = DEVICE_DT_GET(DT_NODELABEL(spi3));
const struct device *gpio0_dev  = DEVICE_DT_GET(DT_NODELABEL(gpio0));
const struct device *wifi_ldo1  = DEVICE_DT_GET(DT_NODELABEL(npm1300_ldsw1));

/* Pin definitions */
#define FLASH_CS_PIN  10   /* P0.10 */
#define WIFI_CS_PIN   12   /* P0.12 */
#define WIFI_IRQ_PIN  16   /* P0.16 */
#define WIFI_EN_PIN   17   /* P0.17 */

static void disable_wifi_ldo1(void)
{
    int ret;
    int attempts = 0;

    LOG_INF("Draining nPM1300 LDO1 reference counter...");

    /* Keep disabling until the hardware is actually off */
    while (regulator_is_enabled(wifi_ldo1)) {
        ret = regulator_disable(wifi_ldo1);
        attempts++;
        if (ret != 0) {
            LOG_ERR("  disable() failed with %d on attempt %d", ret, attempts);
            break;
        }
        LOG_WRN("  Ref count drained by 1 (attempt %d)", attempts);

        if (attempts > 50) {
            LOG_ERR("  Giving up after 50 attempts — driver may be broken.");
            break;
        }
    }

    if (!regulator_is_enabled(wifi_ldo1)) {
        LOG_INF("  LDO1 confirmed OFF after %d attempt(s).", attempts);
    }
}

int main(void)
{
    int err;
    uint32_t loop_count = 0;

    /* Allow PMIC to fully initialise over I2C before we touch it */
    k_msleep(1000);

    LOG_INF("=== SPI Flash Diagnostic (Wi-Fi LDO1 kill) ===");

    /* --- Readiness checks --- */
    if (!device_is_ready(wifi_ldo1)) {
        LOG_ERR("nPM1300 LDO1 not ready! Check I2C wiring and overlay.");
        return -1;
    }
    if (!device_is_ready(spi_dev) || !device_is_ready(gpio0_dev)) {
        LOG_ERR("SPI or GPIO driver not ready! Halting.");
        return -1;
    }

    /* =========================================
     * STEP 1: KILL WI-FI POWER VIA nPM1300 LDO1
     * ========================================= */
    disable_wifi_ldo1();
    LOG_INF("Wi-Fi LDO1 (PMIC) disabled — no power to Wi-Fi chip.");

    /* =========================================
     * STEP 2: QUARANTINE WI-FI GPIO LINES
     * Belt-and-suspenders: also lock out the
     * Wi-Fi chip's SPI and control pins.
     * ========================================= */
    gpio_pin_configure(gpio0_dev, WIFI_CS_PIN,  GPIO_OUTPUT_HIGH);
    gpio_pin_configure(gpio0_dev, WIFI_EN_PIN,  GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure(gpio0_dev, WIFI_IRQ_PIN, GPIO_INPUT | GPIO_PULL_DOWN);
    LOG_INF("Wi-Fi GPIO lines quarantined: CS=HIGH, EN=LOW, IRQ=pull-down.");

    /* =========================================
     * STEP 3: CONFIGURE FLASH CS PIN
     * ========================================= */
    gpio_pin_configure(gpio0_dev, FLASH_CS_PIN, GPIO_OUTPUT_HIGH);

    /* SPI Mode 3 (CPOL=1, CPHA=1) to match board pull-ups */
    struct spi_config spi_cfg = {
        .frequency = 1000000,
        .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB |
                     SPI_MODE_CPOL | SPI_MODE_CPHA,
        .slave     = 0,
        .cs        = NULL,  /* CS driven manually below */
    };

    /* =========================================
     * STEP 4: READ LOOP — JEDEC ID (0x9F)
     * ========================================= */
    while (1) {
        loop_count++;
        LOG_INF("--- Flash Read Attempt #%u ---", loop_count);

        static uint8_t tx_buf[] = { 0x9F, 0x00, 0x00, 0x00 };
        static uint8_t rx_buf[4];
        memset(rx_buf, 0, sizeof(rx_buf));

        struct spi_buf     tx_spi_bufs[] = { { .buf = tx_buf, .len = sizeof(tx_buf) } };
        struct spi_buf_set spi_tx        = { .buffers = tx_spi_bufs, .count = 1 };

        struct spi_buf     rx_spi_bufs[] = { { .buf = rx_buf, .len = sizeof(rx_buf) } };
        struct spi_buf_set spi_rx        = { .buffers = rx_spi_bufs, .count = 1 };

        gpio_pin_set(gpio0_dev, FLASH_CS_PIN, 0);          /* CS LOW — assert */
        err = spi_transceive(spi_dev, &spi_cfg, &spi_tx, &spi_rx);
        gpio_pin_set(gpio0_dev, FLASH_CS_PIN, 1);          /* CS HIGH — release */
        
        if (err == -116) {
            LOG_ERR("TIMEOUT (-116): DMA engine hung. Check MISO line.");
        } else if (err) {
            LOG_ERR("SPI error: %d", err);
        } else {
            LOG_INF("Raw JEDEC ID: [0x%02X, 0x%02X, 0x%02X, 0x%02X]",
                    rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3]);

            if (rx_buf[1] == 0xEF) {
                LOG_INF("SUCCESS: Winbond Flash detected (Manufacturer ID = 0xEF)!");
            } else if (rx_buf[1] == 0x9D) {
                LOG_INF("SUCCESS: ISSI Flash detected (Manufacturer ID = 0x9D)!");
            } else if (rx_buf[1] == 0x00 || rx_buf[1] == 0xFF) {
                LOG_WRN("SPI OK but MISO is dead (0x%02X) — check wiring.", rx_buf[1]);
            } else {
                LOG_INF("Unknown manufacturer ID: 0x%02X", rx_buf[1]);
            }
        }

        k_msleep(2000);
    }

    return 0;
}