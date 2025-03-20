/*
 * Copyright (c) 2023 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_kscan_74hc164_matrix

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/kscan.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/pm/device.h>

#include <zmk/debounce.h>

LOG_MODULE_REGISTER(kscan_74hc164_matrix, CONFIG_KSCAN_LOG_LEVEL);

#define INST_ROWS_LEN(n) DT_INST_PROP(n, rows)
#define INST_COLS_LEN(n) DT_INST_PROP(n, columns)
#define INST_MATRIX_LEN(n) (INST_ROWS_LEN(n) * INST_COLS_LEN(n))

#define INST_DEBOUNCE_PRESS_MS(n) \
    DT_INST_PROP_OR(n, debounce_press_ms, DT_INST_PROP_OR(n, debounce_period, 10))

#define INST_DEBOUNCE_RELEASE_MS(n) \
    DT_INST_PROP_OR(n, debounce_release_ms, DT_INST_PROP_OR(n, debounce_period, 10))

struct kscan_74hc164_config {
    struct gpio_dt_spec data_gpio;
    struct gpio_dt_spec clk_gpio;
    struct gpio_dt_spec reset_gpio;
    struct gpio_dt_spec power_gpio;
    struct gpio_dt_spec *row_gpios;
    uint8_t row_count;
    uint8_t col_count;
    struct zmk_debounce_config debounce_config;
    int32_t debounce_scan_period_ms;
    int32_t poll_period_ms;
};

struct kscan_74hc164_data {
    const struct device *dev;
    kscan_callback_t callback;
    struct k_work_delayable work;
    int64_t scan_time;
    struct zmk_debounce_state *matrix_state;
    bool scan_enabled;
};

static void kscan_74hc164_shift_out(const struct device *dev, uint8_t column) {
    const struct kscan_74hc164_config *config = dev->config;
    
    // Reset the shift register
    gpio_pin_set_dt(&config->reset_gpio, 0);
    k_busy_wait(1);
    gpio_pin_set_dt(&config->reset_gpio, 1);
    
    // Initialize all bits to 1 (no active column)
    gpio_pin_set_dt(&config->data_gpio, 1);
    for (int i = 0; i < config->col_count; i++) {
        gpio_pin_set_dt(&config->clk_gpio, 1);
        k_busy_wait(1);
        gpio_pin_set_dt(&config->clk_gpio, 0);
        k_busy_wait(1);
    }
    
    // Set the active column (a single 0 bit)
    gpio_pin_set_dt(&config->data_gpio, 0);
    k_busy_wait(1);
    gpio_pin_set_dt(&config->clk_gpio, 1);
    k_busy_wait(1);
    gpio_pin_set_dt(&config->clk_gpio, 0);
    
    // Reset data line to 1 for remaining columns
    gpio_pin_set_dt(&config->data_gpio, 1);
    
    // Shift to desired column position
    for (int i = 0; i < column; i++) {
        gpio_pin_set_dt(&config->clk_gpio, 1);
        k_busy_wait(1);
        gpio_pin_set_dt(&config->clk_gpio, 0);
        k_busy_wait(1);
    }
}

static void kscan_74hc164_matrix_read(const struct device *dev) {
    const struct kscan_74hc164_config *config = dev->config;
    struct kscan_74hc164_data *data = dev->data;
    
    // Power on the matrix
    gpio_pin_set_dt(&config->power_gpio, 1);
    k_busy_wait(10);  // Give power time to stabilize
    
    // Scan each column
    for (int col = 0; col < config->col_count; col++) {
        // Set current column
        kscan_74hc164_shift_out(dev, col);
        
        // Small delay to allow signal to propagate
        k_busy_wait(5);
        
        // Read rows
        for (int row = 0; row < config->row_count; row++) {
            // Active low input - key is pressed when input is low
            bool pressed = !gpio_pin_get_dt(&config->row_gpios[row]);
            
            // Update debouncer state
            int index = (col * config->row_count) + row;
            zmk_debounce_update(&data->matrix_state[index], pressed, 
                               config->debounce_scan_period_ms, 
                               &config->debounce_config);
        }
    }
    
    // Power off the matrix to save power
    gpio_pin_set_dt(&config->power_gpio, 0);
    
    // Report any key state changes and check if we should continue scanning
    bool continue_scan = false;
    
    for (int row = 0; row < config->row_count; row++) {
        for (int col = 0; col < config->col_count; col++) {
            int index = (col * config->row_count) + row;
            struct zmk_debounce_state *state = &data->matrix_state[index];
            
            if (zmk_debounce_get_changed(state)) {
                bool pressed = zmk_debounce_is_pressed(state);
                data->callback(dev, row, col, pressed);
            }
            
            continue_scan = continue_scan || zmk_debounce_is_active(state);
        }
    }
    
    // Schedule next scan based on state
    if (continue_scan) {
        // At least one key is pressed or being debounced - scan again quickly
        data->scan_time += config->debounce_scan_period_ms;
        k_work_schedule(&data->work, K_TIMEOUT_ABS_MS(data->scan_time));
    } else {
        // All keys are stable and released - return to normal polling rate
        data->scan_time += config->poll_period_ms;
        k_work_schedule(&data->work, K_TIMEOUT_ABS_MS(data->scan_time));
    }
}

static void kscan_74hc164_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = CONTAINER_OF(work, struct k_work_delayable, work);
    struct kscan_74hc164_data *data = CONTAINER_OF(dwork, struct kscan_74hc164_data, work);
    
    if (data->scan_enabled) {
        kscan_74hc164_matrix_read(data->dev);
    }
}

static int kscan_74hc164_configure(const struct device *dev, kscan_callback_t callback) {
    struct kscan_74hc164_data *data = dev->data;
    data->callback = callback;
    return 0;
}

static int kscan_74hc164_enable(const struct device *dev) {
    struct kscan_74hc164_data *data = dev->data;
    data->scan_enabled = true;
    data->scan_time = k_uptime_get();
    k_work_schedule(&data->work, K_MSEC(1));
    return 0;
}

static int kscan_74hc164_disable(const struct device *dev) {
    struct kscan_74hc164_data *data = dev->data;
    data->scan_enabled = false;
    k_work_cancel_delayable(&data->work);
    return 0;
}

static int kscan_74hc164_init(const struct device *dev) {
    const struct kscan_74hc164_config *config = dev->config;
    struct kscan_74hc164_data *data = dev->data;
    
    data->dev = dev;
    
    // Initialize work
    k_work_init_delayable(&data->work, kscan_74hc164_work_handler);

    // Initialize GPIOs for shift register control
    if (!gpio_is_ready_dt(&config->data_gpio) ||
        !gpio_is_ready_dt(&config->clk_gpio) ||
        !gpio_is_ready_dt(&config->reset_gpio) ||
        !gpio_is_ready_dt(&config->power_gpio)) {
        LOG_ERR("Control GPIOs not ready");
        return -ENODEV;
    }

    if (gpio_pin_configure_dt(&config->data_gpio, GPIO_OUTPUT_INACTIVE) ||
        gpio_pin_configure_dt(&config->clk_gpio, GPIO_OUTPUT_INACTIVE) ||
        gpio_pin_configure_dt(&config->reset_gpio, GPIO_OUTPUT_ACTIVE) ||
        gpio_pin_configure_dt(&config->power_gpio, GPIO_OUTPUT_INACTIVE)) {
        LOG_ERR("Failed to configure control GPIOs");
        return -EIO;
    }

    // Initialize row GPIOs as inputs with pull-up (since columns are active low)
    for (int i = 0; i < config->row_count; i++) {
        if (!gpio_is_ready_dt(&config->row_gpios[i])) {
            LOG_ERR("Row GPIO %d not ready", i);
            return -ENODEV;
        }
        
        if (gpio_pin_configure_dt(&config->row_gpios[i], GPIO_INPUT | GPIO_PULL_UP)) {
            LOG_ERR("Failed to configure row GPIO %d", i);
            return -EIO;
        }
    }
    
    return 0;
}

#if IS_ENABLED(CONFIG_PM_DEVICE)
static int kscan_74hc164_pm_action(const struct device *dev, enum pm_device_action action) {
    struct kscan_74hc164_data *data = dev->data;
    const struct kscan_74hc164_config *config = dev->config;
    int ret = 0;
    
    switch (action) {
    case PM_DEVICE_ACTION_SUSPEND:
        // Disable scanning
        kscan_74hc164_disable(dev);
        
        // Power off the matrix
        gpio_pin_set_dt(&config->power_gpio, 0);
        
        // Disconnect GPIOs to save power
        gpio_pin_configure_dt(&config->data_gpio, GPIO_DISCONNECTED);
        gpio_pin_configure_dt(&config->clk_gpio, GPIO_DISCONNECTED);
        gpio_pin_configure_dt(&config->reset_gpio, GPIO_DISCONNECTED);
        gpio_pin_configure_dt(&config->power_gpio, GPIO_DISCONNECTED);
        
        for (int i = 0; i < config->row_count; i++) {
            gpio_pin_configure_dt(&config->row_gpios[i], GPIO_DISCONNECTED);
        }
        break;
        
    case PM_DEVICE_ACTION_RESUME:
        // Re-initialize GPIOs
        ret = gpio_pin_configure_dt(&config->data_gpio, GPIO_OUTPUT_INACTIVE);
        if (ret) return ret;
        
        ret = gpio_pin_configure_dt(&config->clk_gpio, GPIO_OUTPUT_INACTIVE);
        if (ret) return ret;
        
        ret = gpio_pin_configure_dt(&config->reset_gpio, GPIO_OUTPUT_ACTIVE);
        if (ret) return ret;
        
        ret = gpio_pin_configure_dt(&config->power_gpio, GPIO_OUTPUT_INACTIVE);
        if (ret) return ret;
        
        for (int i = 0; i < config->row_count; i++) {
            ret = gpio_pin_configure_dt(&config->row_gpios[i], GPIO_INPUT | GPIO_PULL_UP);
            if (ret) return ret;
        }
        
        // Re-enable scanning
        kscan_74hc164_enable(dev);
        break;
        
    default:
        return -ENOTSUP;
    }
    
    return 0;
}
#endif

static const struct kscan_driver_api kscan_74hc164_api = {
    .config = kscan_74hc164_configure,
    .enable_callback = kscan_74hc164_enable,
    .disable_callback = kscan_74hc164_disable,
};

#define KSCAN_74HC164_INIT(n)                                                           \
    static struct gpio_dt_spec row_gpios_##n[] = {                                      \
        DT_FOREACH_PROP_ELEM_SEP(DT_DRV_INST(n), row_gpios, GPIO_DT_SPEC_GET_BY_IDX, (, )) \
    };                                                                                  \
                                                                                        \
    /* Statically allocate matrix state */                                              \
    /* Using regular C comment instead of # preprocessor directive */                   \
    static struct zmk_debounce_state matrix_state_##n[INST_MATRIX_LEN(n)];            \
                                                                                        \
    static const struct kscan_74hc164_config kscan_74hc164_config_##n = {              \
        .data_gpio = GPIO_DT_SPEC_INST_GET(n, data_gpios),                             \
        .clk_gpio = GPIO_DT_SPEC_INST_GET(n, clk_gpios),                               \
        .reset_gpio = GPIO_DT_SPEC_INST_GET(n, reset_gpios),                           \
        .power_gpio = GPIO_DT_SPEC_INST_GET(n, power_gpios),                           \
        .row_gpios = row_gpios_##n,                                                     \
        .row_count = DT_INST_PROP(n, rows),                                            \
        .col_count = DT_INST_PROP(n, columns),                                         \
        .debounce_config = {                                                           \
            .debounce_press_ms = INST_DEBOUNCE_PRESS_MS(n),                            \
            .debounce_release_ms = INST_DEBOUNCE_RELEASE_MS(n),                        \
        },                                                                              \
        .debounce_scan_period_ms = DT_INST_PROP_OR(n, debounce_scan_period_ms, 1),     \
        .poll_period_ms = DT_INST_PROP_OR(n, poll_period_ms, 10),                      \
    };                                                                                  \
                                                                                        \
    static struct kscan_74hc164_data kscan_74hc164_data_##n = {                        \
        .matrix_state = matrix_state_##n,                                              \
    };                                                                                  \
                                                                                        \
    PM_DEVICE_DT_INST_DEFINE(n, kscan_74hc164_pm_action);                              \
                                                                                        \
    DEVICE_DT_INST_DEFINE(n, kscan_74hc164_init, PM_DEVICE_DT_INST_GET(n),             \
                      &kscan_74hc164_data_##n, &kscan_74hc164_config_##n,              \
                      POST_KERNEL, CONFIG_KSCAN_INIT_PRIORITY, &kscan_74hc164_api);

DT_INST_FOREACH_STATUS_OKAY(KSCAN_74HC164_INIT)
