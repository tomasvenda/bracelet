#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/sys/printk.h>
#include <zephyr/devicetree.h>
#include <stdio.h>  /* Added for snprintf */
#include <string.h>

#define DISPLAY_NODE DT_NODELABEL(ssd1306)

static const uint8_t font_5x7[][5] = {
    {0x7F, 0x09, 0x09, 0x09, 0x7F}, /* H (0) */
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, /* O (1) */
    {0x7F, 0x08, 0x08, 0x08, 0x70}, /* U (2) */
    {0x7F, 0x08, 0x08, 0x08, 0x08}, /* R (3) */
    {0x00, 0x00, 0x00, 0x00, 0x00}, /* space (4) */
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, /* 0 (5) */
    {0x00, 0x42, 0x7F, 0x40, 0x00}, /* 1 (6) */
    {0x62, 0x51, 0x49, 0x49, 0x46}, /* 2 (7) */
    {0x22, 0x49, 0x49, 0x49, 0x36}, /* 3 (8) */
    {0x18, 0x14, 0x12, 0x7F, 0x10}, /* 4 (9) */
    {0x2F, 0x49, 0x49, 0x49, 0x31}, /* 5 (10) */
    {0x3E, 0x49, 0x49, 0x49, 0x30}, /* 6 (11) */
    {0x01, 0x71, 0x09, 0x05, 0x03}, /* 7 (12) */
    {0x36, 0x49, 0x49, 0x49, 0x36}, /* 8 (13) */
    {0x26, 0x49, 0x49, 0x49, 0x3E}, /* 9 (14) */
    {0x00, 0x14, 0x00, 0x00, 0x00}, /* : (15) */
    {0x01, 0x01, 0x7F, 0x01, 0x01}, /* T (16) */
    {0x00, 0x41, 0x7F, 0x41, 0x00}, /* I (17) */
    {0x7F, 0x02, 0x0C, 0x02, 0x7F}, /* M (18) */
    {0x7F, 0x49, 0x49, 0x49, 0x41}  /* E (19) */
};

static inline void set_pixel(uint8_t *fb, int x, int y, int width, int pitch_bytes, bool vtiled)
{
    if (x < 0 || y < 0 || x >= width) {
        return;
    }

    if (vtiled) {
        int byte_index = (y / 8) * width + x;
        fb[byte_index] |= 1 << (y % 8);
    } else {
        int byte_index = y * pitch_bytes + x / 8;
        fb[byte_index] |= 1 << (7 - (x % 8));
    }
}

static void draw_char(uint8_t *fb, int x0, int y0, char ch, int width, int pitch_bytes, bool vtiled)
{
    const uint8_t *glyph;

    /* Map the incoming character to our font array index */
    if (ch == 'H') glyph = font_5x7[0];
    else if (ch == 'O') glyph = font_5x7[1];
    else if (ch == 'U') glyph = font_5x7[2];
    else if (ch == 'R') glyph = font_5x7[3];
    else if (ch == ' ') glyph = font_5x7[4];
    else if (ch >= '0' && ch <= '9') glyph = font_5x7[5 + (ch - '0')];
    else if (ch == ':') glyph = font_5x7[15];
    else if (ch == 'T') glyph = font_5x7[16];
    else if (ch == 'I') glyph = font_5x7[17];
    else if (ch == 'M') glyph = font_5x7[18];
    else if (ch == 'E') glyph = font_5x7[19];
    else glyph = font_5x7[4]; /* Default to space */

    for (int col = 0; col < 5; col++) {
        uint8_t data = glyph[col];
        for (int row = 0; row < 7; row++) {
            if (data & (1 << row)) {
                set_pixel(fb, x0 + col, y0 + row, width, pitch_bytes, vtiled);
            }
        }
    }
}

static void draw_text(uint8_t *fb, int x, int y, const char *text, int width, int pitch_bytes, bool vtiled)
{
    while (*text) {
        draw_char(fb, x, y, *text, width, pitch_bytes, vtiled);
        x += 6;
        text++;
    }
}

void main(void)
{
    const struct device *display = DEVICE_DT_GET(DISPLAY_NODE);
    struct display_capabilities caps;
    int ret;

    if (!device_is_ready(display)) {
        printk("Display device not ready\n");
        return;
    }

    display_get_capabilities(display, &caps);
    printk("Display resolution: %ux%u\n", caps.x_resolution, caps.y_resolution);

    ret = display_blanking_off(display);
    if (ret < 0) {
        printk("display_blanking_off failed: %d\n", ret);
        return;
    }

    const size_t buf_size = (caps.x_resolution * caps.y_resolution) / 8;
    static uint8_t fb[1024];
    bool vtiled = caps.screen_info & SCREEN_INFO_MONO_VTILED;
    int pitch_bytes = caps.x_resolution / 8;

    if (buf_size > sizeof(fb)) {
        printk("Framebuffer buffer too small (%u > %u)\n", (unsigned)buf_size, (unsigned)sizeof(fb));
        return;
    }

    struct display_buffer_descriptor desc = {
        .buf_size = buf_size,
        .width = caps.x_resolution,
        .height = caps.y_resolution,
        .pitch = caps.x_resolution,
        .frame_incomplete = false,
    };

    /* Continuous Watch Loop */
    while (1) {
        /* 1. Clear the framebuffer for the new frame */
        memset(fb, 0x00, buf_size);

        /* 2. Calculate current time based on system uptime */
        uint32_t uptime_s = k_uptime_get() / 1000;
        uint32_t hours = (uptime_s / 3600) % 24;
        uint32_t minutes = (uptime_s / 60) % 60;
        uint32_t seconds = uptime_s % 60;

        /* 3. Format the string to HH:MM:SS */
        char time_str[9];
        snprintf(time_str, sizeof(time_str), "%02u:%02u:%02u", hours, minutes, seconds);

        /* 4. Draw the UI to the buffer */
        draw_text(fb, 0, 0, "TIME", caps.x_resolution, pitch_bytes, vtiled);
        draw_text(fb, 0, 16, time_str, caps.x_resolution, pitch_bytes, vtiled);

        /* 5. Send the buffer to the OLED driver */
        ret = display_write(display, 0, 0, &desc, fb);
        if (ret < 0) {
            printk("display_write failed: %d\n", ret);
        }

        /* 6. Wait exactly 1 second before ticking again */
        k_sleep(K_SECONDS(1));
    }
}