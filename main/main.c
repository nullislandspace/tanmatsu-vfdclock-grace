// SPDX-License-Identifier: MIT
// Tanmatsu VFD Clock
//
// Displays the current time on a NE-HCS12SS59T-R1 I2C VFD (12-character ASCII).
// The VFD is jumpered to address 0x13 (the default 0x10 collides with an on-board device on the internal
// bus). It is looked for on the CATT port I2C bus first, then on the internal I2C bus.
// Press Escape to exit.
//
// VFD Register Map:
//   Register 0:     System control (bit 0 = enable, bit 1 = test, bit 2 = LED)
//   Register 1:     Display offset
//   Registers 4-5:  Scroll speed
//   Register 6:     Brightness (0-255, default 110)
//   Registers 10+:  ASCII text data buffer

#include <stdio.h>
#include <string.h>
#include <time.h>
#include "bsp/catt.h"
#include "bsp/device.h"
#include "bsp/display.h"
#include "bsp/i2c.h"
#include "bsp/input.h"
#include "bsp/rtc.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "gl_input.h"
#include "nvs_flash.h"
#include "pax_fonts.h"
#include "pax_gfx.h"
#include "pax_text.h"

// Constants
static char const TAG[] = "vfdclock";

// VFD configuration
#define VFD_ADDRESS  0x13    // 7-bit I2C address (default 0x10 + jumper 1 + jumper 2)
#define VFD_SPEED_HZ 100000  // 100kHz
#define VFD_CHARS    12      // Display character count
#define VFD_TIMEOUT  50      // I2C transfer timeout in ms
#define SCAN_TIMEOUT 20      // Per-address probe timeout in ms

// VFD register addresses
#define VFD_REG_CONTROL    0
#define VFD_REG_BRIGHTNESS 6
#define VFD_REG_TEXT       10

// VFD control bits
#define VFD_CTRL_ENABLE (1 << 0)

#define BLACK 0xFF000000
#define WHITE 0xFFFFFFFF

typedef struct {
    char const*             name;
    i2c_master_bus_handle_t handle;
    bool                    shared;          // Internal bus: shared with the coprocessor, needs claim/release
    char                    found[128];     // Addresses that answered the scan, as text
    bool                    has_vfd;
} i2c_bus_t;

// Global variables
static size_t                     display_h_res        = 0;
static size_t                     display_v_res        = 0;
static bsp_display_color_format_t display_color_format = BSP_DISPLAY_COLOR_FORMAT_16_565RGB;
static bsp_display_endianness_t   display_data_endian  = BSP_DISPLAY_ENDIAN_LITTLE;
static pax_buf_t                  fb                   = {0};
static QueueHandle_t              input_event_queue    = NULL;
static i2c_bus_t                  buses[2]             = {
    {.name = "CATT I2C", .shared = false},
    {.name = "Internal I2C", .shared = true},
};
static i2c_bus_t*              vfd_bus = NULL;
static i2c_master_dev_handle_t vfd_dev = NULL;

static void blit(void) {
    bsp_display_blit(0, 0, display_h_res, display_v_res, pax_buf_get_pixels(&fb));
}

static void show_lines(char const* lines[], size_t count) {
    pax_background(&fb, WHITE);
    pax_draw_text(&fb, BLACK, pax_font_sky_mono, 16, 0, 0, "VFD Clock - press Escape to exit");
    for (size_t i = 0; i < count; i++) {
        pax_draw_text(&fb, BLACK, pax_font_sky_mono, 16, 0, 36 + i * 18, lines[i]);
    }
    blit();
}

static void bus_lock(i2c_bus_t* bus) {
    if (bus->shared) bsp_i2c_primary_bus_claim();
}

static void bus_unlock(i2c_bus_t* bus) {
    if (bus->shared) bsp_i2c_primary_bus_release();
}

static void scan_bus(i2c_bus_t* bus) {
    if (bus->handle == NULL) {
        snprintf(bus->found, sizeof(bus->found), "bus unavailable");
        ESP_LOGW(TAG, "%s: bus unavailable", bus->name);
        return;
    }
    size_t pos = 0;
    for (uint16_t addr = 0x08; addr < 0x78; addr++) {
        bus_lock(bus);
        esp_err_t res = i2c_master_probe(bus->handle, addr, SCAN_TIMEOUT);
        bus_unlock(bus);
        if (res != ESP_OK) continue;
        if (addr == VFD_ADDRESS) bus->has_vfd = true;
        if (pos < sizeof(bus->found)) {
            pos += snprintf(&bus->found[pos], sizeof(bus->found) - pos, "%s0x%02X", pos ? " " : "", addr);
        }
    }
    if (pos == 0) snprintf(bus->found, sizeof(bus->found), "no devices");
    ESP_LOGI(TAG, "%s: %s", bus->name, bus->found);
}

static esp_err_t vfd_write_reg(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    bus_lock(vfd_bus);
    esp_err_t res = i2c_master_transmit(vfd_dev, data, sizeof(data), VFD_TIMEOUT);
    bus_unlock(vfd_bus);
    return res;
}

static esp_err_t vfd_write_text(char const* text) {
    size_t len = strlen(text);
    if (len > VFD_CHARS) len = VFD_CHARS;

    // Register address followed by text data, padded with spaces
    uint8_t buf[1 + VFD_CHARS];
    buf[0] = VFD_REG_TEXT;
    memcpy(&buf[1], text, len);
    memset(&buf[1 + len], ' ', VFD_CHARS - len);
    bus_lock(vfd_bus);
    esp_err_t res = i2c_master_transmit(vfd_dev, buf, sizeof(buf), VFD_TIMEOUT);
    bus_unlock(vfd_bus);
    return res;
}

static esp_err_t vfd_init(void) {
    // CATT bus: bsp_device_initialize() leaves it down if an add-on held the lines, so retry once
    if (bsp_catt_i2c_bus_get_handle(&buses[0].handle) != ESP_OK) {
        buses[0].handle = NULL;
        if (bsp_catt_set_i2c_enabled(true) != ESP_OK || bsp_catt_i2c_bus_get_handle(&buses[0].handle) != ESP_OK) {
            buses[0].handle = NULL;
        }
    }
    if (bsp_i2c_primary_bus_get_handle(&buses[1].handle) != ESP_OK) {
        buses[1].handle = NULL;
    }

    for (size_t i = 0; i < sizeof(buses) / sizeof(buses[0]); i++) {
        scan_bus(&buses[i]);
        if (vfd_bus == NULL && buses[i].has_vfd) vfd_bus = &buses[i];
    }

    if (vfd_bus == NULL) {
        ESP_LOGE(TAG, "No VFD at 0x%02X on any I2C bus", VFD_ADDRESS);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "Using VFD at 0x%02X on %s", VFD_ADDRESS, vfd_bus->name);

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = VFD_ADDRESS,
        .scl_speed_hz    = VFD_SPEED_HZ,
    };
    esp_err_t res = i2c_master_bus_add_device(vfd_bus->handle, &dev_cfg, &vfd_dev);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add VFD device: %s", esp_err_to_name(res));
        return res;
    }

    // Enable display and set brightness
    vfd_write_reg(VFD_REG_CONTROL, VFD_CTRL_ENABLE);
    vfd_write_reg(VFD_REG_BRIGHTNESS, 110);  // DON'T GO MUCH HIGHER, THIS WILL REDUCE THE LIFESPAN DRASTICALLY
    return ESP_OK;
}

static void vfd_shutdown(void) {
    if (vfd_dev == NULL) return;
    vfd_write_text("");
    vfd_write_reg(VFD_REG_CONTROL, 0);
    i2c_master_bus_rm_device(vfd_dev);
    vfd_dev = NULL;
}

static bool escape_pressed(TickType_t wait) {
    bsp_input_event_t event;
    while (xQueueReceive(input_event_queue, &event, wait) == pdTRUE) {
        wait = 0;  // Drain any further queued events without blocking
        if (event.type == INPUT_EVENT_TYPE_NAVIGATION && event.args_navigation.key == BSP_INPUT_NAVIGATION_KEY_ESC &&
            event.args_navigation.state) {
            return true;
        }
    }
    return false;
}

void app_main(void) {
    // Initialize the Non Volatile Storage partition
    esp_err_t res = nvs_flash_init();
    if (res == ESP_ERR_NVS_NO_FREE_PAGES || res == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        res = nvs_flash_init();
    }
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize NVS flash: %d", res);
        return;
    }

    // Initialize the Board Support Package
    const bsp_configuration_t bsp_configuration = {
        .display =
            {
                .requested_color_format = BSP_DISPLAY_COLOR_FORMAT_24_888RGB,
                .num_fbs                = 1,
            },
    };
    res = bsp_device_initialize(&bsp_configuration);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize BSP: %d", res);
        return;
    }

    // Set up the display framebuffer
    res = bsp_display_get_parameters(&display_h_res, &display_v_res, &display_color_format, &display_data_endian);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get display parameters: %d", res);
        return;
    }

    pax_buf_type_t format =
        display_color_format == BSP_DISPLAY_COLOR_FORMAT_16_565RGB ? PAX_BUF_16_565RGB : PAX_BUF_24_888RGB;

    pax_orientation_t orientation = PAX_O_UPRIGHT;
    switch (bsp_display_get_default_rotation()) {
        case BSP_DISPLAY_ROTATION_90:
            orientation = PAX_O_ROT_CCW;
            break;
        case BSP_DISPLAY_ROTATION_180:
            orientation = PAX_O_ROT_HALF;
            break;
        case BSP_DISPLAY_ROTATION_270:
            orientation = PAX_O_ROT_CW;
            break;
        default:
            break;
    }

    pax_buf_init(&fb, NULL, display_h_res, display_v_res, format);
    pax_buf_reversed(&fb, display_data_endian == BSP_DISPLAY_ENDIAN_BIG);
    pax_buf_set_orientation(&fb, orientation);

    // Get input event queue from graceloader (merges native + USB keyboard)
    ESP_ERROR_CHECK(gl_input_get_queue(&input_event_queue));

    // Give the debug monitor time to reconnect after the reboot into graceloader
    vTaskDelay(pdMS_TO_TICKS(10000));

    // The system clock starts at zero after the reboot into graceloader; load it from the coprocessor RTC.
    // No timezone is applied, so the clock shows UTC.
    res = bsp_rtc_update_time();
    if (res != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read RTC: %s", esp_err_to_name(res));
    }

    char const* scanning[] = {"Scanning I2C buses..."};
    show_lines(scanning, 1);

    if (vfd_init() != ESP_OK) {
        char catt_line[160];
        char internal_line[160];
        char text[64];
        snprintf(text, sizeof(text), "No VFD found at 0x%02X on any I2C bus", VFD_ADDRESS);
        snprintf(catt_line, sizeof(catt_line), "%s: %s", buses[0].name, buses[0].found);
        snprintf(internal_line, sizeof(internal_line), "%s: %s", buses[1].name, buses[1].found);
        char const* lines[] = {text, "", catt_line, internal_line};
        show_lines(lines, 4);
        while (!escape_pressed(portMAX_DELAY));
        bsp_device_restart_to_launcher();
        return;
    }

    char bus_line[64];
    snprintf(bus_line, sizeof(bus_line), "VFD:        0x%02X on %s", VFD_ADDRESS, vfd_bus->name);

    int last_second = -1;
    while (!escape_pressed(pdMS_TO_TICKS(100))) {
        time_t    now = time(NULL);
        struct tm timeinfo;
        gmtime_r(&now, &timeinfo);
        if (timeinfo.tm_sec == last_second) continue;
        last_second = timeinfo.tm_sec;

        // Format: "  HH MM SS  " (centered in 12 chars)
        char vfd_text[VFD_CHARS + 1];
        snprintf(vfd_text, sizeof(vfd_text), "  %02d %02d %02d  ", timeinfo.tm_hour, timeinfo.tm_min,
                 timeinfo.tm_sec);
        esp_err_t vfd_res = vfd_write_text(vfd_text);

        char time_line[64];
        char write_line[64];
        snprintf(time_line, sizeof(time_line), "Time (UTC): %02d:%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min,
                 timeinfo.tm_sec);
        snprintf(write_line, sizeof(write_line), "VFD write:  %s", esp_err_to_name(vfd_res));
        char const* lines[] = {time_line, bus_line, write_line};
        show_lines(lines, 3);
    }

    vfd_shutdown();
    bsp_device_restart_to_launcher();
}
