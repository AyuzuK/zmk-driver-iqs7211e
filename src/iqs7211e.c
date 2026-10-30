/*
 * Copyright 2025 sekigon-gonnoc
 * SPDX-License-Identifier: GPL-2.0 or later
 */

#include <stdint.h>
#include <stdlib.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/sys/util.h>
#include <zephyr/devicetree.h>

#include "../include/iqs7211e_reg.h"

LOG_MODULE_REGISTER(iqs7211e, CONFIG_ZMK_LOG_LEVEL);

#define DT_DRV_COMPAT azoteq_iqs7211e

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

#define IQS7211E_INIT_DATA_LEN 217
#define IQS7211E_TIMEOUT_MS 100
#define IQS7211E_RESET_DELAY_MS 50
#define IQS7211E_ATI_TIMEOUT_CYCLES 600 // 30 seconds at 50ms intervals
#define IQS7211E_INERTIA_TICK_MS 10
#define IQS7211E_Q8_SHIFT 8
#define IQS7211E_Q8_ONE (1 << IQS7211E_Q8_SHIFT)
#define IQS7211E_SCROLL_AXIS_NONE 0
#define IQS7211E_SCROLL_AXIS_VERTICAL 1
#define IQS7211E_SCROLL_AXIS_HORIZONTAL 2

// Gesture mode signal codes
#define IQS7211E_GESTURE_CODE_TAP INPUT_BTN_5
#define IQS7211E_GESTURE_CODE_DOUBLE_TAP INPUT_BTN_6
#define IQS7211E_GESTURE_CODE_SWIPE_UP INPUT_BTN_7
#define IQS7211E_GESTURE_CODE_SWIPE_DOWN INPUT_BTN_8
// Click signal: pressed at a first tap release, or held during a second touch (drag)
// Not INPUT_BTN_0-4, which the input listener treats as mouse buttons when not mapped
#define IQS7211E_GESTURE_CODE_CLICK INPUT_BTN_9
#define IQS7211E_GESTURE_CODE_LONG_PRESS INPUT_BTN_MODE

struct iqs7211e_config {
    struct i2c_dt_spec i2c;
    struct gpio_dt_spec irq_gpio;
    struct gpio_dt_spec power_gpio;
    const uint8_t *init_data;
    size_t init_len;
    bool scroller_mode;
    bool gesture_mode;
    bool v_invert;
    bool h_invert;
};

struct iqs7211e_data {
    const struct device *dev;
    struct k_work motion_work;
    struct k_work_delayable click_work;
    struct gpio_callback motion_cb;
    uint16_t product_number;
    bool init_complete;
    int16_t previous_x;
    int16_t previous_y;
    bool previous_valid;
    // Gesture state tracking
    int64_t last_touch_time;
    int64_t last_tap_time;
    bool is_clicking;
    bool double_tap_hold;
    uint8_t tap_count;
    int16_t tap_start_x, tap_start_y;
    int16_t finger_2_prev_x, finger_2_prev_y;
    bool finger_2_prev_valid;
    uint8_t pending_click_type; // 0=none, 1=left, 2=right
    bool scroll_was_active;
    uint16_t x_resolution;
    uint16_t y_resolution;
    bool resolution_valid;
    bool gesture_started_near_edge;
    uint8_t scroller_axis_lock;
    bool motion_started;
    // Gesture mode state
    struct k_work_delayable gesture_tap_work;
    struct k_work_delayable gesture_release_work;
    bool gesture_tap_pending;
    uint16_t gesture_pressed_code; // 0=none
    bool gesture_multi_touch;
    struct k_work_delayable gesture_click_release_work;
    bool gesture_click_pressed;
    bool gesture_click_drag;
    struct k_work_delayable gesture_long_press_work;
    bool gesture_long_press_armed;
    bool gesture_long_press_fired;
#if defined(CONFIG_IQS7211E_SCROLLER_INERTIA) && CONFIG_IQS7211E_SCROLLER_INERTIA
    struct k_work_delayable inertia_work;
    int32_t inertia_wheel_velocity_q8;
    int32_t inertia_hwheel_velocity_q8;
    int32_t inertia_wheel_remainder_q8;
    int32_t inertia_hwheel_remainder_q8;
    int64_t inertia_last_wheel_time;
    int64_t inertia_last_hwheel_time;
    bool inertia_running;
#endif
};

static bool iqs7211e_is_near_edge(const struct iqs7211e_data *data, uint16_t x, uint16_t y) {
    uint32_t margin_x;
    uint32_t margin_y;

    if (CONFIG_IQS7211E_TAP_EDGE_MARGIN_PERMILLE == 0) {
        return false;
    }

    if (!data->resolution_valid || data->x_resolution == 0 || data->y_resolution == 0) {
        return false;
    }

    margin_x = ((uint32_t)data->x_resolution * CONFIG_IQS7211E_TAP_EDGE_MARGIN_PERMILLE) / 1000;
    margin_y = ((uint32_t)data->y_resolution * CONFIG_IQS7211E_TAP_EDGE_MARGIN_PERMILLE) / 1000;

    if (margin_x == 0) {
        margin_x = 1;
    }
    if (margin_y == 0) {
        margin_y = 1;
    }

    if (x <= margin_x || y <= margin_y) {
        return true;
    }

    if ((uint32_t)x >= ((uint32_t)data->x_resolution - margin_x) ||
        (uint32_t)y >= ((uint32_t)data->y_resolution - margin_y)) {
        return true;
    }

    return false;
}

static bool iqs7211e_is_hwheel_zone(const struct iqs7211e_data *data, uint16_t y) {
    uint32_t zone_min_permille = CONFIG_IQS7211E_SCROLLER_HWHEEL_ZONE_MIN_PERMILLE;
    uint32_t zone_max_permille = CONFIG_IQS7211E_SCROLLER_HWHEEL_ZONE_MAX_PERMILLE;
    uint32_t zone_start;
    uint32_t zone_end;

    if (!data->resolution_valid || data->y_resolution == 0) {
        return false;
    }

    if (zone_min_permille > zone_max_permille) {
        uint32_t tmp = zone_min_permille;
        zone_min_permille = zone_max_permille;
        zone_max_permille = tmp;
    }

    zone_start = ((uint32_t)data->y_resolution * zone_min_permille) / 1000;
    zone_end = ((uint32_t)data->y_resolution * zone_max_permille) / 1000;

    if (zone_start > data->y_resolution) {
        zone_start = data->y_resolution;
    }
    if (zone_end > data->y_resolution) {
        zone_end = data->y_resolution;
    }

    if (zone_end <= zone_start) {
        return false;
    }

    return (uint32_t)y >= zone_start && (uint32_t)y < zone_end;
}

static bool iqs7211e_suppress_motion_start(struct iqs7211e_data *data, uint16_t x, uint16_t y) {
    uint32_t distance;

    if (CONFIG_IQS7211E_MOTION_START_THRESHOLD == 0 || data->motion_started) {
        return false;
    }

    distance = abs((int32_t)x - data->tap_start_x) + abs((int32_t)y - data->tap_start_y);
    if (distance >= CONFIG_IQS7211E_MOTION_START_THRESHOLD) {
        data->motion_started = true;
    }

    // Discard movement up to and including the report that crosses the threshold
    return true;
}

static int16_t iqs7211e_vertical_scroll_delta(const struct iqs7211e_config *cfg, int16_t y_movement) {
    return cfg->v_invert ? y_movement : -y_movement;
}

static int16_t iqs7211e_horizontal_scroll_delta(const struct iqs7211e_config *cfg, int16_t x_movement) {
    return cfg->h_invert ? x_movement : -x_movement;
}

#if defined(CONFIG_IQS7211E_SCROLLER_INERTIA) && CONFIG_IQS7211E_SCROLLER_INERTIA
static int32_t iqs7211e_abs32(int32_t value) {
    return (value < 0) ? -value : value;
}

static void iqs7211e_select_inertia_axis(struct iqs7211e_data *data, uint16_t axis,
                                         int32_t **velocity_q8, int64_t **last_time) {
    if (axis == INPUT_REL_HWHEEL) {
        *velocity_q8 = &data->inertia_hwheel_velocity_q8;
        *last_time = &data->inertia_last_hwheel_time;
    } else {
        *velocity_q8 = &data->inertia_wheel_velocity_q8;
        *last_time = &data->inertia_last_wheel_time;
    }
}

static void iqs7211e_stop_inertia_scroll(struct iqs7211e_data *data) {
    data->inertia_running = false;
    data->inertia_wheel_velocity_q8 = 0;
    data->inertia_hwheel_velocity_q8 = 0;
    data->inertia_wheel_remainder_q8 = 0;
    data->inertia_hwheel_remainder_q8 = 0;
    data->inertia_last_wheel_time = 0;
    data->inertia_last_hwheel_time = 0;
    (void)k_work_cancel_delayable(&data->inertia_work);
}

static void iqs7211e_update_inertia_velocity(struct iqs7211e_data *data, uint16_t axis,
                                             int16_t wheel_delta, int64_t current_time) {
    int32_t *velocity_q8;
    int64_t *last_time;
    int32_t sample_q8;

    if (wheel_delta == 0) {
        return;
    }

    iqs7211e_select_inertia_axis(data, axis, &velocity_q8, &last_time);

    if (*last_time <= 0 || current_time <= *last_time) {
        sample_q8 = (int32_t)wheel_delta << IQS7211E_Q8_SHIFT;
    } else {
        int64_t dt_ms = current_time - *last_time;

        if (dt_ms < 1) {
            dt_ms = 1;
        }

        sample_q8 = ((int32_t)wheel_delta << IQS7211E_Q8_SHIFT) * IQS7211E_INERTIA_TICK_MS / (int32_t)dt_ms;
    }

    // A light moving-average filter prevents abrupt speed jumps.
    *velocity_q8 = (*velocity_q8 * 3 + sample_q8) / 4;
    *last_time = current_time;
}

static void iqs7211e_inertia_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct iqs7211e_data *data = CONTAINER_OF(dwork, struct iqs7211e_data, inertia_work);
    const struct device *dev = data->dev;
    int32_t wheel_delta;
    int32_t hwheel_delta;
    bool wheel_active;
    bool hwheel_active;

    if (!data->inertia_running) {
        return;
    }

    wheel_active = iqs7211e_abs32(data->inertia_wheel_velocity_q8) >=
                   CONFIG_IQS7211E_SCROLLER_INERTIA_STOP_THRESHOLD_Q8;
    hwheel_active = iqs7211e_abs32(data->inertia_hwheel_velocity_q8) >=
                    CONFIG_IQS7211E_SCROLLER_INERTIA_STOP_THRESHOLD_Q8;

    if (!wheel_active) {
        data->inertia_wheel_velocity_q8 = 0;
        data->inertia_wheel_remainder_q8 = 0;
    }
    if (!hwheel_active) {
        data->inertia_hwheel_velocity_q8 = 0;
        data->inertia_hwheel_remainder_q8 = 0;
    }

    if (!wheel_active && !hwheel_active) {
        iqs7211e_stop_inertia_scroll(data);
        return;
    }

    data->inertia_wheel_remainder_q8 += data->inertia_wheel_velocity_q8;
    wheel_delta = data->inertia_wheel_remainder_q8 / IQS7211E_Q8_ONE;
    data->inertia_wheel_remainder_q8 -= wheel_delta * IQS7211E_Q8_ONE;

    data->inertia_hwheel_remainder_q8 += data->inertia_hwheel_velocity_q8;
    hwheel_delta = data->inertia_hwheel_remainder_q8 / IQS7211E_Q8_ONE;
    data->inertia_hwheel_remainder_q8 -= hwheel_delta * IQS7211E_Q8_ONE;

    if (wheel_delta != 0) {
        input_report_rel(dev, INPUT_REL_WHEEL, wheel_delta, true, K_FOREVER);
    }
    if (hwheel_delta != 0) {
        input_report_rel(dev, INPUT_REL_HWHEEL, hwheel_delta, true, K_FOREVER);
    }

    data->inertia_wheel_velocity_q8 =
        (data->inertia_wheel_velocity_q8 * CONFIG_IQS7211E_SCROLLER_INERTIA_DECAY_PERMILLE) / 1000;
    data->inertia_hwheel_velocity_q8 =
        (data->inertia_hwheel_velocity_q8 * CONFIG_IQS7211E_SCROLLER_INERTIA_DECAY_PERMILLE) / 1000;

    if (iqs7211e_abs32(data->inertia_wheel_velocity_q8) <
            CONFIG_IQS7211E_SCROLLER_INERTIA_STOP_THRESHOLD_Q8 &&
        iqs7211e_abs32(data->inertia_hwheel_velocity_q8) <
            CONFIG_IQS7211E_SCROLLER_INERTIA_STOP_THRESHOLD_Q8) {
        iqs7211e_stop_inertia_scroll(data);
        return;
    }

    (void)k_work_schedule(&data->inertia_work, K_MSEC(IQS7211E_INERTIA_TICK_MS));
}

static void iqs7211e_start_inertia_scroll(struct iqs7211e_data *data) {
    bool wheel_active = iqs7211e_abs32(data->inertia_wheel_velocity_q8) >=
                        CONFIG_IQS7211E_SCROLLER_INERTIA_STOP_THRESHOLD_Q8;
    bool hwheel_active = iqs7211e_abs32(data->inertia_hwheel_velocity_q8) >=
                         CONFIG_IQS7211E_SCROLLER_INERTIA_STOP_THRESHOLD_Q8;

    if (!wheel_active) {
        data->inertia_wheel_velocity_q8 = 0;
        data->inertia_wheel_remainder_q8 = 0;
    }
    if (!hwheel_active) {
        data->inertia_hwheel_velocity_q8 = 0;
        data->inertia_hwheel_remainder_q8 = 0;
    }

    if (!wheel_active && !hwheel_active) {
        return;
    }

    data->inertia_running = true;
    (void)k_work_schedule(&data->inertia_work, K_MSEC(IQS7211E_INERTIA_TICK_MS));
}
#endif

static void iqs7211e_report_scroll(struct iqs7211e_data *data, uint16_t axis,
                                   int16_t wheel_delta, int64_t current_time) {
    if (wheel_delta == 0) {
        return;
    }

    input_report_rel(data->dev, axis, wheel_delta, true, K_FOREVER);
    data->scroll_was_active = true;

#if defined(CONFIG_IQS7211E_SCROLLER_INERTIA) && CONFIG_IQS7211E_SCROLLER_INERTIA
    iqs7211e_update_inertia_velocity(data, axis, wheel_delta, current_time);
#else
    ARG_UNUSED(axis);
    ARG_UNUSED(current_time);
#endif
}

static void iqs7211e_process_scroller_motion(struct iqs7211e_data *data,
                                             const struct iqs7211e_config *cfg,
                                             bool hwheel_zone,
                                             int16_t x_movement,
                                             int16_t y_movement,
                                             int64_t current_time) {
    if (data->scroller_axis_lock == IQS7211E_SCROLL_AXIS_NONE) {
        if (hwheel_zone && x_movement != 0 && abs(x_movement) > abs(y_movement)) {
            data->scroller_axis_lock = IQS7211E_SCROLL_AXIS_HORIZONTAL;
        } else if (y_movement != 0) {
            data->scroller_axis_lock = IQS7211E_SCROLL_AXIS_VERTICAL;
        } else if (hwheel_zone && x_movement != 0) {
            data->scroller_axis_lock = IQS7211E_SCROLL_AXIS_HORIZONTAL;
        }
    }

    if (data->scroller_axis_lock == IQS7211E_SCROLL_AXIS_HORIZONTAL) {
        int16_t h_delta = iqs7211e_horizontal_scroll_delta(cfg, x_movement);
        if (h_delta != 0) {
            iqs7211e_report_scroll(data, INPUT_REL_HWHEEL, h_delta, current_time);
        }
    } else if (data->scroller_axis_lock == IQS7211E_SCROLL_AXIS_VERTICAL) {
        int16_t v_delta = iqs7211e_vertical_scroll_delta(cfg, y_movement);
        if (v_delta != 0) {
            iqs7211e_report_scroll(data, INPUT_REL_WHEEL, v_delta, current_time);
        }
    }
}

static int iqs7211e_i2c_read_reg(const struct device *dev, uint8_t reg, uint8_t *data, uint8_t len) {
    const struct iqs7211e_config *cfg = dev->config;
    return i2c_burst_read_dt(&cfg->i2c, reg, data, len);
}

static int iqs7211e_i2c_write_reg(const struct device *dev, uint8_t reg, const uint8_t *data, uint8_t len) {
    const struct iqs7211e_config *cfg = dev->config;
    return i2c_burst_write_dt(&cfg->i2c, reg, data, len);
}

static bool iqs7211e_is_ready(const struct device *dev) {
    const struct iqs7211e_config *cfg = dev->config;
    
    if (!gpio_is_ready_dt(&cfg->irq_gpio)) {
        return true; // Assume ready if no IRQ pin configured
    }
    
    // RDY pin is active LOW, so device is ready when pin is LOW
    return !gpio_pin_get_dt(&cfg->irq_gpio);
}

static void iqs7211e_wait_for_ready(const struct device *dev, uint16_t timeout_ms) {
    uint16_t elapsed = 0;
    
    while (!iqs7211e_is_ready(dev) && elapsed < timeout_ms) {
        k_sleep(K_MSEC(1));
        elapsed++;
    }
    
    if (elapsed >= timeout_ms) {
        LOG_WRN("RDY timeout after %dms", timeout_ms);
    }
}

static int iqs7211e_get_base_data(const struct device *dev, azoteq_iqs7211e_base_data_t *base_data) {
    uint8_t transfer_bytes[8];
    int ret;
    
    iqs7211e_wait_for_ready(dev, 50);
    
    if (!iqs7211e_is_ready(dev)) {
        LOG_WRN("Device not ready for data read");
        return -EIO;
    }
    
    ret = iqs7211e_i2c_read_reg(dev, IQS7211E_MM_INFO_FLAGS, transfer_bytes, 8);
    if (ret < 0) {
        return ret;
    }
    
    base_data->info_flags[0] = transfer_bytes[0];
    base_data->info_flags[1] = transfer_bytes[1];
    base_data->finger_1_x.l = transfer_bytes[2];
    base_data->finger_1_x.h = transfer_bytes[3];
    base_data->finger_1_y.l = transfer_bytes[4];
    base_data->finger_1_y.h = transfer_bytes[5];
    base_data->finger_2_x.l = transfer_bytes[6];
    base_data->finger_2_x.h = transfer_bytes[7];
    
    return 0;
}

static int iqs7211e_reset(const struct device *dev) {
    uint8_t transfer_bytes[2];
    int ret;
    
    ret = iqs7211e_i2c_read_reg(dev, IQS7211E_MM_SYS_CONTROL, transfer_bytes, 2);
    if (ret < 0) {
        LOG_ERR("Failed to read system control: %d", ret);
        return ret;
    }
    
    transfer_bytes[1] |= (1 << IQS7211E_SW_RESET_BIT);
    
    return iqs7211e_i2c_write_reg(dev, IQS7211E_MM_SYS_CONTROL, transfer_bytes, 2);
}

static int iqs7211e_set_event_mode(const struct device *dev, bool enabled) {
    uint8_t transfer_bytes[2];
    int ret;
    
    ret = iqs7211e_i2c_read_reg(dev, IQS7211E_MM_CONFIG_SETTINGS, transfer_bytes, 2);
    if (ret < 0) {
        return ret;
    }
    
    if (enabled) {
        transfer_bytes[1] |= (1 << IQS7211E_EVENT_MODE_BIT);
    } else {
        transfer_bytes[1] &= ~(1 << IQS7211E_EVENT_MODE_BIT);
    }
    
    return iqs7211e_i2c_write_reg(dev, IQS7211E_MM_CONFIG_SETTINGS, transfer_bytes, 2);
}

static int iqs7211e_acknowledge_reset(const struct device *dev) {
    uint8_t transfer_bytes[2];
    int ret;
    
    iqs7211e_wait_for_ready(dev, 50);
    
    ret = iqs7211e_i2c_read_reg(dev, IQS7211E_MM_SYS_CONTROL, transfer_bytes, 2);
    if (ret < 0) {
        return ret;
    }
    
    iqs7211e_wait_for_ready(dev, 50);
    transfer_bytes[0] |= (1 << IQS7211E_ACK_RESET_BIT);
    
    ret = iqs7211e_i2c_write_reg(dev, IQS7211E_MM_SYS_CONTROL, transfer_bytes, 2);
    LOG_DBG("Acknowledged reset, status %d", ret);
    
    return ret;
}

static int iqs7211e_reati(const struct device *dev) {
    uint8_t transfer_bytes[2];
    int ret;
    
    iqs7211e_wait_for_ready(dev, 100);
    
    ret = iqs7211e_i2c_read_reg(dev, IQS7211E_MM_SYS_CONTROL, transfer_bytes, 2);
    if (ret < 0) {
        return ret;
    }
    
    iqs7211e_wait_for_ready(dev, 100);
    transfer_bytes[0] |= (1 << IQS7211E_TP_RE_ATI_BIT);
    
    ret = iqs7211e_i2c_write_reg(dev, IQS7211E_MM_SYS_CONTROL, transfer_bytes, 2);
    LOG_DBG("RE-ATI enabled, status %d", ret);
    
    return ret;
}

static uint16_t iqs7211e_get_product(const struct device *dev) {
    struct iqs7211e_data *data = dev->data;
    uint8_t transfer_bytes[2];
    int ret;
    
    iqs7211e_wait_for_ready(dev, 100);
    
    if (!iqs7211e_is_ready(dev)) {
        data->product_number = 0xff;
        LOG_WRN("Device not ready for product read");
        return 0;
    }
    
    ret = iqs7211e_i2c_read_reg(dev, IQS7211E_MM_PROD_NUM, transfer_bytes, 2);
    if (ret == 0) {
        data->product_number = transfer_bytes[0] | (transfer_bytes[1] << 8);
    }
    
    LOG_DBG("Product number %u, status %d", data->product_number, ret);
    return data->product_number;
}

static const uint8_t *iqs7211e_find_init_record(const struct iqs7211e_config *cfg, uint8_t reg, uint8_t *out_len) {
    if (!cfg || !cfg->init_data) {
        return NULL;
    }
    size_t pos = 0;
    size_t data_len = cfg->init_len ? cfg->init_len : IQS7211E_INIT_DATA_LEN;
    while (pos + 2 <= data_len) {
        uint8_t addr = cfg->init_data[pos++];
        uint8_t len = cfg->init_data[pos++];
        if (pos + len > data_len) {
            return NULL;
        }
        if (addr == reg) {
            if (out_len) *out_len = len;
            return &cfg->init_data[pos];
        }
        pos += len;
    }
    return NULL;
}

static bool iqs7211e_load_resolution_from_init(const struct iqs7211e_config *cfg,
                                               struct iqs7211e_data *data) {
    uint8_t len = 0;
    const uint8_t *rec = iqs7211e_find_init_record(cfg, IQS7211E_MM_TP_RX_SETTINGS, &len);

    if (rec == NULL || len < 8) {
        data->resolution_valid = false;
        data->x_resolution = 0;
        data->y_resolution = 0;
        return false;
    }

    data->x_resolution = ((uint16_t)rec[5] << 8) | rec[4];
    data->y_resolution = ((uint16_t)rec[7] << 8) | rec[6];
    data->resolution_valid = (data->x_resolution > 0) && (data->y_resolution > 0);

    return data->resolution_valid;
}

static int iqs7211e_write_memory_map(const struct device *dev) {
    int ret = 0;
    const struct iqs7211e_config *cfg = dev->config;

    if (!cfg->init_data) {
        LOG_ERR("No init data provided in config");
        return -EINVAL;
    }

    size_t pos = 0;
    size_t data_len = cfg->init_len ? cfg->init_len : IQS7211E_INIT_DATA_LEN;
    while (pos + 2 <= data_len) {
        uint8_t addr = cfg->init_data[pos++];
        uint8_t count = cfg->init_data[pos++];
        if (pos + count > data_len) {
            LOG_ERR("Init data truncated");
            return -EINVAL;
        }
        iqs7211e_wait_for_ready(dev, 100);
        ret |= iqs7211e_i2c_write_reg(dev, addr, &cfg->init_data[pos], count);
        pos += count;
    }

    LOG_DBG("Memory map write complete, status: %d", ret);
    return ret;
}

static int iqs7211e_check_reset(const struct device *dev) {
    uint8_t transfer_bytes[2];
    int ret;
    
    iqs7211e_wait_for_ready(dev, 50);
    
    if (!iqs7211e_is_ready(dev)) {
        LOG_WRN("Device not ready for reset check");
        return -EIO;
    }
    
    ret = iqs7211e_i2c_read_reg(dev, IQS7211E_MM_INFO_FLAGS, transfer_bytes, 2);
    if (ret < 0) {
        return ret;
    }
    
    return (transfer_bytes[0] & (1 << IQS7211E_SHOW_RESET_BIT)) ? 0 : -EAGAIN;
}

static bool iqs7211e_read_ati_active(const struct device *dev) {
    uint8_t transfer_bytes[2];
    int ret;
    
    iqs7211e_wait_for_ready(dev, 500);
    
    if (!iqs7211e_is_ready(dev)) {
        LOG_WRN("Device not ready for ATI check");
        return true;
    }
    
    ret = iqs7211e_i2c_read_reg(dev, IQS7211E_MM_SYS_CONTROL, transfer_bytes, 2);
    if (ret < 0) {
        return true;
    }
    
    LOG_DBG("ATI active check, flags: 0x%02X", transfer_bytes[0]);
    return (transfer_bytes[0] & (1 << IQS7211E_TP_RE_ATI_BIT)) != 0;
}

static void iqs7211e_suspend(const struct device *dev) {
    uint8_t transfer_bytes[2];
    iqs7211e_i2c_read_reg(dev, IQS7211E_MM_SYS_CONTROL, transfer_bytes, 2);
    transfer_bytes[1] |= (1 << 3);
    iqs7211e_i2c_write_reg(dev, IQS7211E_MM_SYS_CONTROL, transfer_bytes, 2);
    LOG_DBG("Device suspended");
}

static void iqs7211e_resume(const struct device *dev) {
    uint8_t transfer_bytes[2];
    iqs7211e_i2c_read_reg(dev, IQS7211E_MM_SYS_CONTROL, transfer_bytes, 2);
    transfer_bytes[1] &= ~(1 << 3);
    iqs7211e_i2c_write_reg(dev, IQS7211E_MM_SYS_CONTROL, transfer_bytes, 2);
    LOG_DBG("Device resumed");
}

static void iqs7211e_click_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct iqs7211e_data *data = CONTAINER_OF(dwork, struct iqs7211e_data, click_work);
    const struct device *dev = data->dev;
    
    if (data->pending_click_type == 1) {
        // Left click release
        LOG_DBG("Single tap - release");
        input_report_key(dev, INPUT_BTN_0, 0, true, K_FOREVER);
    } else if (data->pending_click_type == 2) {
        // Right click release
        LOG_DBG("Two finger tap - release");
        input_report_key(dev, INPUT_BTN_1, 0, true, K_FOREVER);
    }
    data->pending_click_type = 0;
}

static void iqs7211e_gesture_release(struct iqs7211e_data *data) {
    if (data->gesture_pressed_code != 0) {
        input_report_key(data->dev, data->gesture_pressed_code, 0, true, K_FOREVER);
        data->gesture_pressed_code = 0;
    }
}

static void iqs7211e_gesture_send(struct iqs7211e_data *data, uint16_t code) {
    // Release the previous signal first so that signals never overlap
    (void)k_work_cancel_delayable(&data->gesture_release_work);
    iqs7211e_gesture_release(data);

    input_report_key(data->dev, code, 1, true, K_FOREVER);
    data->gesture_pressed_code = code;
    (void)k_work_schedule(&data->gesture_release_work,
                          K_MSEC(CONFIG_IQS7211E_GESTURE_SIGNAL_RELEASE_MS));
}

static void iqs7211e_gesture_release_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct iqs7211e_data *data = CONTAINER_OF(dwork, struct iqs7211e_data, gesture_release_work);

    iqs7211e_gesture_release(data);
}

// The click signal has its own release timer, so other signals never release it
static void iqs7211e_gesture_click_release(struct iqs7211e_data *data) {
    (void)k_work_cancel_delayable(&data->gesture_click_release_work);
    if (data->gesture_click_pressed) {
        input_report_key(data->dev, IQS7211E_GESTURE_CODE_CLICK, 0, true, K_FOREVER);
        data->gesture_click_pressed = false;
    }
    data->gesture_click_drag = false;
}

static void iqs7211e_gesture_click_press(struct iqs7211e_data *data) {
    // Release a click waiting for its release first, so that press and release keep their order
    iqs7211e_gesture_click_release(data);

    input_report_key(data->dev, IQS7211E_GESTURE_CODE_CLICK, 1, true, K_FOREVER);
    data->gesture_click_pressed = true;
}

static void iqs7211e_gesture_click_release_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct iqs7211e_data *data =
        CONTAINER_OF(dwork, struct iqs7211e_data, gesture_click_release_work);

    iqs7211e_gesture_click_release(data);
}

static void iqs7211e_gesture_long_press_stop(struct iqs7211e_data *data) {
    (void)k_work_cancel_delayable(&data->gesture_long_press_work);
    data->gesture_long_press_armed = false;
}

// Long press is judged by a timer, because a still finger sends no report in event mode
static void iqs7211e_gesture_long_press_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct iqs7211e_data *data =
        CONTAINER_OF(dwork, struct iqs7211e_data, gesture_long_press_work);

    if (!data->gesture_long_press_armed) {
        return;
    }
    data->gesture_long_press_armed = false;

    // Only while one finger still touches
    if (!data->previous_valid || data->finger_2_prev_valid || data->gesture_multi_touch) {
        return;
    }

    data->gesture_long_press_fired = true;
    iqs7211e_gesture_send(data, IQS7211E_GESTURE_CODE_LONG_PRESS);
}

// Cancel the long press when the finger moved too far from the touch start
static void iqs7211e_gesture_long_press_check_motion(struct iqs7211e_data *data, uint16_t x,
                                                     uint16_t y) {
    uint32_t distance;

    if (!data->gesture_long_press_armed) {
        return;
    }

    distance = abs((int32_t)x - data->tap_start_x) + abs((int32_t)y - data->tap_start_y);
    if (distance >= CONFIG_IQS7211E_GESTURE_LONG_PRESS_DISTANCE) {
        iqs7211e_gesture_long_press_stop(data);
    }
}

static void iqs7211e_gesture_tap_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct iqs7211e_data *data = CONTAINER_OF(dwork, struct iqs7211e_data, gesture_tap_work);

    if (!data->gesture_tap_pending) {
        return;
    }

    data->gesture_tap_pending = false;
    iqs7211e_gesture_send(data, IQS7211E_GESTURE_CODE_TAP);
}

// Send a pending single tap immediately (keeps the order: tap -> next gesture)
static void iqs7211e_gesture_flush_pending_tap(struct iqs7211e_data *data) {
    if (!data->gesture_tap_pending) {
        return;
    }

    (void)k_work_cancel_delayable(&data->gesture_tap_work);
    data->gesture_tap_pending = false;
    iqs7211e_gesture_send(data, IQS7211E_GESTURE_CODE_TAP);
}

// Drop a pending single tap and release a pressed signal (used on suspend)
static void iqs7211e_gesture_cancel(struct iqs7211e_data *data) {
    (void)k_work_cancel_delayable(&data->gesture_tap_work);
    data->gesture_tap_pending = false;
    (void)k_work_cancel_delayable(&data->gesture_release_work);
    iqs7211e_gesture_release(data);
    iqs7211e_gesture_click_release(data);
    iqs7211e_gesture_long_press_stop(data);
    data->gesture_long_press_fired = false;
}

static void iqs7211e_gesture_touch_start(struct iqs7211e_data *data) {
    data->gesture_multi_touch = false;
    data->gesture_long_press_fired = false;

    // Give the second touch until the tap time to finish as a tap.
    // If it lasts longer, it is not a tap, so the pending single tap is sent then.
    if (data->gesture_tap_pending) {
        (void)k_work_reschedule(&data->gesture_tap_work,
                                K_MSEC(CONFIG_IQS7211E_GESTURE_TAP_TIME_MS));

        // Second touch within the double tap wait: hold the click until the finger leaves
        // (a quick release makes a double click, moving makes a drag)
        iqs7211e_gesture_click_press(data);
        data->gesture_click_drag = true;
    } else if (CONFIG_IQS7211E_GESTURE_LONG_PRESS_MS > 0) {
        // Not during the double tap wait: wait for a long press
        data->gesture_long_press_armed = true;
        (void)k_work_reschedule(&data->gesture_long_press_work,
                                K_MSEC(CONFIG_IQS7211E_GESTURE_LONG_PRESS_MS));
    }
}

static void iqs7211e_gesture_touch_end(struct iqs7211e_data *data, int64_t touch_duration,
                                       bool tap_allowed) {
    int32_t dx;
    int32_t dy;
    uint32_t tap_distance;
    uint32_t swipe_distance;

    // The click held from the touch start (double click or drag) is released first
    if (data->gesture_click_drag) {
        iqs7211e_gesture_click_release(data);
    }

    iqs7211e_gesture_long_press_stop(data);
    if (data->gesture_long_press_fired) {
        // The long press signal was sent for this touch: no other signal
        data->gesture_long_press_fired = false;
        return;
    }

    if (data->finger_2_prev_valid || data->gesture_multi_touch) {
        // Two finger contact (scroll or two finger tap) sends no gesture signal
        iqs7211e_gesture_flush_pending_tap(data);
        return;
    }

    dx = (int32_t)data->previous_x - data->tap_start_x;
    dy = (int32_t)data->previous_y - data->tap_start_y;
    tap_distance = abs(dx) + abs(dy);

    if (tap_allowed && touch_duration < CONFIG_IQS7211E_GESTURE_TAP_TIME_MS &&
        tap_distance < CONFIG_IQS7211E_GESTURE_TAP_DISTANCE) {
        if (data->gesture_tap_pending) {
            (void)k_work_cancel_delayable(&data->gesture_tap_work);
            data->gesture_tap_pending = false;
            iqs7211e_gesture_send(data, IQS7211E_GESTURE_CODE_DOUBLE_TAP);
        } else {
            data->gesture_tap_pending = true;
            (void)k_work_reschedule(&data->gesture_tap_work,
                                    K_MSEC(CONFIG_IQS7211E_GESTURE_DOUBLE_TAP_WAIT_MS));

            // First tap: click at the release without waiting for the double tap
            iqs7211e_gesture_click_press(data);
            (void)k_work_schedule(&data->gesture_click_release_work,
                                  K_MSEC(CONFIG_IQS7211E_GESTURE_SIGNAL_RELEASE_MS));
        }
        return;
    }

    // Not a tap: send a pending single tap first
    iqs7211e_gesture_flush_pending_tap(data);

    if (!data->resolution_valid || touch_duration > CONFIG_IQS7211E_GESTURE_SWIPE_MAX_TIME_MS) {
        return;
    }

    swipe_distance =
        ((uint32_t)data->y_resolution * CONFIG_IQS7211E_GESTURE_SWIPE_DISTANCE_PERMILLE) / 1000;
    if (swipe_distance == 0) {
        swipe_distance = 1;
    }

    // Edge margin is not applied to swipes
    if ((uint32_t)abs(dy) >= swipe_distance && abs(dy) > abs(dx)) {
        iqs7211e_gesture_send(data, (dy < 0) ? IQS7211E_GESTURE_CODE_SWIPE_UP
                                             : IQS7211E_GESTURE_CODE_SWIPE_DOWN);
    }
}

static int iqs7211e_interrupt_configure(const struct device *dev, gpio_flags_t flags) {
    const struct iqs7211e_config *cfg = dev->config;

    if (!gpio_is_ready_dt(&cfg->irq_gpio)) {
        return 0;
    }

    return gpio_pin_interrupt_configure_dt(&cfg->irq_gpio, flags);
}

static void iqs7211e_interrupt_enable(const struct device *dev) {
    int ret = iqs7211e_interrupt_configure(dev, GPIO_INT_LEVEL_LOW);

    if (ret < 0) {
        LOG_ERR("Failed to re-enable IRQ interrupt: %d", ret);
    }
}

static void iqs7211e_interrupt_disable(const struct device *dev) {
    int ret = iqs7211e_interrupt_configure(dev, GPIO_INT_DISABLE);

    if (ret < 0) {
        LOG_WRN("Failed to mask IRQ interrupt: %d", ret);
    }
}

static void iqs7211e_motion_work_handler(struct k_work *work) {
    struct iqs7211e_data *data = CONTAINER_OF(work, struct iqs7211e_data, motion_work);
    const struct device *dev = data->dev;
    const struct iqs7211e_config *cfg = dev->config;
    azoteq_iqs7211e_base_data_t base_data = {0};
    int ret;
    int64_t current_time = k_uptime_get();
    
    LOG_DBG("Motion work handler started");
    if (!data->init_complete) {
        LOG_WRN("Device not initialized, skipping motion handling");
        iqs7211e_interrupt_enable(dev);
        return;
    }
    
    // Only read data if device is ready
    if (!iqs7211e_is_ready(dev)) {
        LOG_WRN("Device not ready for motion data");
        iqs7211e_interrupt_enable(dev);
        return;
    }
    
    ret = iqs7211e_get_base_data(dev, &base_data);
    if (ret < 0) {
        LOG_WRN("Get report failed, status: %d", ret);
        iqs7211e_interrupt_enable(dev);
        return;
    }
 
    uint8_t finger_count = base_data.info_flags[1] & 0x03;

#if defined(CONFIG_IQS7211E_SCROLLER_INERTIA) && CONFIG_IQS7211E_SCROLLER_INERTIA
    if (finger_count > 0 && data->inertia_running) {
        iqs7211e_stop_inertia_scroll(data);
    }
#endif
    
    if (finger_count == 1) {
        // Single finger handling
        uint16_t finger_1_x = AZOTEQ_IQS7211E_COMBINE_H_L_BYTES(base_data.finger_1_x.h, base_data.finger_1_x.l);
        uint16_t finger_1_y = AZOTEQ_IQS7211E_COMBINE_H_L_BYTES(base_data.finger_1_y.h, base_data.finger_1_y.l);
        
        if (!data->previous_valid) {
            // Touch start
            data->tap_start_x = finger_1_x;
            data->tap_start_y = finger_1_y;
            data->last_touch_time = current_time;
            data->scroll_was_active = false;
            data->gesture_started_near_edge = iqs7211e_is_near_edge(data, finger_1_x, finger_1_y);
            data->scroller_axis_lock = IQS7211E_SCROLL_AXIS_NONE;
            data->motion_started = false;
            if (cfg->gesture_mode) {
                iqs7211e_gesture_touch_start(data);
            }
        } else if (data->finger_2_prev_valid) {
            // Transitioning from two finger to one finger - reset position reference
            LOG_DBG("Transition from two finger to one finger - reset position");
            data->tap_start_x = finger_1_x;
            data->tap_start_y = finger_1_y;
            data->last_touch_time = current_time;
            data->scroll_was_active = false;
            data->gesture_started_near_edge = iqs7211e_is_near_edge(data, finger_1_x, finger_1_y);
            data->scroller_axis_lock = IQS7211E_SCROLL_AXIS_NONE;
            data->motion_started = false;
            // Reset tap state to allow normal single finger gestures
            data->tap_count = 0;
            data->double_tap_hold = false;
            data->is_clicking = false;
        } else {
            // Normal single finger movement
            int16_t x = finger_1_x - data->previous_x;
            int16_t y = finger_1_y - data->previous_y;

            if (cfg->gesture_mode) {
                iqs7211e_gesture_long_press_check_motion(data, finger_1_x, finger_1_y);
            }

            if (cfg->scroller_mode) {
                bool hwheel_zone = iqs7211e_is_hwheel_zone(data, finger_1_y);
                iqs7211e_process_scroller_motion(data, cfg, hwheel_zone, x, y, current_time);
            } else if (iqs7211e_suppress_motion_start(data, finger_1_x, finger_1_y)) {
                // Movement is discarded until the motion start threshold is reached
            } else {
                LOG_DBG("Movement: x=%4d y=%4d", x, y);
                input_report_rel(dev, INPUT_REL_X, x, false, K_FOREVER);
                input_report_rel(dev, INPUT_REL_Y, y, true, K_FOREVER);
            }
        }
        
        data->previous_x = finger_1_x;
        data->previous_y = finger_1_y;
        data->previous_valid = true;
        data->finger_2_prev_valid = false;
        
    } else if (finger_count == 2) {
        // Two finger handling
        data->gesture_multi_touch = true;
        if (cfg->gesture_mode) {
            iqs7211e_gesture_long_press_stop(data);
        }
        uint16_t finger_1_x = AZOTEQ_IQS7211E_COMBINE_H_L_BYTES(base_data.finger_1_x.h, base_data.finger_1_x.l);
        uint16_t finger_1_y = AZOTEQ_IQS7211E_COMBINE_H_L_BYTES(base_data.finger_1_y.h, base_data.finger_1_y.l);
        uint16_t finger_2_x = AZOTEQ_IQS7211E_COMBINE_H_L_BYTES(base_data.finger_2_x.h, base_data.finger_2_x.l);
        
        // Read finger 2 Y coordinate (need additional read)
        uint8_t finger_2_y_bytes[2];
        ret = iqs7211e_i2c_read_reg(dev, IQS7211E_MM_FINGER_2_Y, finger_2_y_bytes, 2);
        uint16_t finger_2_y = (ret == 0) ? AZOTEQ_IQS7211E_COMBINE_H_L_BYTES(finger_2_y_bytes[1], finger_2_y_bytes[0]) : 0;
        
        if (!data->finger_2_prev_valid) {
            // Two finger touch start
            data->last_touch_time = current_time;
            data->scroll_was_active = false;
            data->gesture_started_near_edge = iqs7211e_is_near_edge(data, finger_1_x, finger_1_y) ||
                                              iqs7211e_is_near_edge(data, finger_2_x, finger_2_y);
            data->scroller_axis_lock = IQS7211E_SCROLL_AXIS_NONE;
            // Reset single finger tap state when starting two finger gesture
            data->tap_count = 0;
            data->double_tap_hold = false;
            if (data->is_clicking) {
                data->is_clicking = false;
                input_report_key(dev, INPUT_BTN_0, 0, true, K_FOREVER);
            }
        } else {
            // Two finger movement - scroll
            int16_t y_movement = (finger_1_y + finger_2_y) / 2 - (data->previous_y + data->finger_2_prev_y) / 2;
            int16_t x_movement = (finger_1_x + finger_2_x) / 2 - (data->previous_x + data->finger_2_prev_x) / 2;

            if (cfg->scroller_mode) {
                uint16_t avg_y = (finger_1_y + finger_2_y) / 2;
                bool hwheel_zone = iqs7211e_is_hwheel_zone(data, avg_y);
                iqs7211e_process_scroller_motion(data, cfg, hwheel_zone, x_movement, y_movement,
                                                current_time);
            } else {
                if (abs(y_movement) > 0) {
                    LOG_DBG("Scroll Y: %d", -y_movement);
                    input_report_rel(dev, INPUT_REL_WHEEL, -y_movement, true, K_FOREVER);
                }
                if (abs(x_movement) > 0) {
                    LOG_DBG("Scroll X: %d", x_movement);
                    input_report_rel(dev, INPUT_REL_HWHEEL, x_movement, true, K_FOREVER);
                }
            }
        }
        
        data->previous_x = finger_1_x;
        data->previous_y = finger_1_y;
        data->finger_2_prev_x = finger_2_x;
        data->finger_2_prev_y = finger_2_y;
        data->previous_valid = true;
        data->finger_2_prev_valid = true;
        
    } else {
        // No fingers - handle touch end events
        if (data->previous_valid || data->finger_2_prev_valid) {
            int64_t touch_duration = current_time - data->last_touch_time;
            bool ended_near_edge = false;
            bool tap_allowed;

            if (data->previous_valid) {
                ended_near_edge = iqs7211e_is_near_edge(data, data->previous_x, data->previous_y);
            }
            if (data->finger_2_prev_valid) {
                ended_near_edge = ended_near_edge ||
                                  iqs7211e_is_near_edge(data, data->finger_2_prev_x, data->finger_2_prev_y);
            }

            tap_allowed = !(data->gesture_started_near_edge || ended_near_edge);
            
            if (cfg->gesture_mode) {
                // Gesture signals instead of clicks; the common reset below still runs
                iqs7211e_gesture_touch_end(data, touch_duration, tap_allowed);
            } else if (data->finger_2_prev_valid) {
                // Two finger tap - right click
                if (touch_duration < 200) { // Quick tap
                    LOG_DBG("Two finger tap - press");
                    input_report_key(dev, INPUT_BTN_1, 1, true, K_FOREVER);
                    data->pending_click_type = 2;
                    k_work_schedule(&data->click_work, K_MSEC(50));
                }
            } else if (data->previous_valid) {
                // Single finger tap handling
                int16_t tap_distance = abs(data->previous_x - data->tap_start_x) + abs(data->previous_y - data->tap_start_y);

                LOG_DBG("Touch duration: %lld ms, tap distance: %d, tap count: %d", touch_duration, tap_distance, data->tap_count);
                
                if (tap_allowed && touch_duration < 200 && tap_distance < 50) { // Quick tap with minimal movement
                    int64_t tap_interval = current_time - data->last_tap_time;
                    
                    if (tap_interval < 400 && data->tap_count == 1) { // Double tap
                        LOG_DBG("Double tap - start hold click");
                        data->double_tap_hold = true;
                        data->is_clicking = true;
                        input_report_key(dev, INPUT_BTN_0, 1, true, K_FOREVER);
                        data->tap_count = 0;
                    } else {
                        // Single tap
                        if (!data->double_tap_hold) {
                            LOG_DBG("Single tap - press");
                            input_report_key(dev, INPUT_BTN_0, 1, true, K_FOREVER);
                            data->pending_click_type = 1;
                            k_work_schedule(&data->click_work, K_MSEC(50));
                        }
                        data->tap_count = 1;
                    }
                    data->last_tap_time = current_time;
                } else if (data->double_tap_hold && data->is_clicking) {
                    // Release double-tap hold
                    LOG_DBG("Release double tap hold");
                    data->double_tap_hold = false;
                    data->is_clicking = false;
                    input_report_key(dev, INPUT_BTN_0, 0, true, K_FOREVER);
                } else if (!tap_allowed) {
                    LOG_DBG("Single finger tap ignored near sensor edge");
                    data->tap_count = 0;
                    data->double_tap_hold = false;
                }
                
                // Reset tap count if too much time passed
                if (current_time - data->last_tap_time > 600) {
                    data->tap_count = 0;
                }
            }
        }

#if defined(CONFIG_IQS7211E_SCROLLER_INERTIA) && CONFIG_IQS7211E_SCROLLER_INERTIA
        if (cfg->scroller_mode && data->scroll_was_active) {
            iqs7211e_start_inertia_scroll(data);
        }
#endif
        
        data->previous_valid = false;
        data->finger_2_prev_valid = false;
        data->scroll_was_active = false;
        data->gesture_started_near_edge = false;
        data->scroller_axis_lock = IQS7211E_SCROLL_AXIS_NONE;
        data->gesture_multi_touch = false;
    }

    iqs7211e_interrupt_enable(dev);
}

static void iqs7211e_motion_handler(const struct device *gpio_dev, struct gpio_callback *cb, uint32_t pins) {
    struct iqs7211e_data *data = CONTAINER_OF(cb, struct iqs7211e_data, motion_cb);

    ARG_UNUSED(gpio_dev);
    ARG_UNUSED(pins);

    iqs7211e_interrupt_disable(data->dev);

    k_work_submit(&data->motion_work);
}

static int iqs7211e_configure(const struct device *dev) {
    struct iqs7211e_data *data = dev->data;
    const struct iqs7211e_config *cfg = dev->config;
    int ret;
    
    LOG_DBG("Initialization started");
    
    // Wait for device to be ready
    iqs7211e_wait_for_ready(dev, 100);
    
    // Software reset
    ret = iqs7211e_reset(dev);
    if (ret < 0) {
        LOG_ERR("Reset failed: %d", ret);
        return ret;
    }
    
    k_sleep(K_MSEC(IQS7211E_RESET_DELAY_MS));
    
    // Wait for device to be ready after reset
    iqs7211e_wait_for_ready(dev, 200);
    
    // Check product number
    if (iqs7211e_get_product(dev) == AZOTEQ_IQS7211E_PRODUCT_NUM) {
        LOG_DBG("Device found");
        
        // Check if reset occurred
        if (iqs7211e_check_reset(dev) == 0) {
            LOG_DBG("Reset event confirmed");
            
            // Write all settings from init file
            ret = iqs7211e_write_memory_map(dev);
            if (ret == 0) {
                // Acknowledge reset
                ret = iqs7211e_acknowledge_reset(dev);
                if (ret < 0) {
                    return ret;
                }
                
                k_sleep(K_MSEC(100));
                
                // Run ATI
                ret = iqs7211e_reati(dev);
                if (ret < 0) {
                    return ret;
                }
                
                // Wait for ATI to complete
                int ati_timeout = IQS7211E_ATI_TIMEOUT_CYCLES;
                while (iqs7211e_read_ati_active(dev) && ati_timeout > 0) {
                    k_sleep(K_MSEC(50));
                    ati_timeout--;
                }
                
                if (ati_timeout > 0) {
                    LOG_DBG("ATI completed");
                    
                    // Wait for device to be ready before setting event mode
                    iqs7211e_wait_for_ready(dev, 500);
                    
                    // Set event mode
                    ret = iqs7211e_set_event_mode(dev, true);
                    if (ret < 0) {
                        return ret;
                    }
                    
                    /* Sleep using the active mode report rate from init data if present */
                    {
                        uint8_t len = 0;
                        const uint8_t *rec = iqs7211e_find_init_record(cfg, IQS7211E_MM_ACTIVE_MODE_RR, &len);
                        uint32_t sleep_ms = (uint32_t)rec[0] + 1;
                        k_sleep(K_MSEC(sleep_ms));
                    }

                    data->init_complete = true;
                    LOG_DBG("Init complete");
                } else {
                    LOG_ERR("ATI timeout");
                    return -ETIMEDOUT;
                }
            } else {
                LOG_ERR("Memory map write failed");
                return ret;
            }
        } else {
            LOG_ERR("No reset event detected");
            return -EIO;
        }
    } else {
        LOG_ERR("Device not found, product: 0x%04x", data->product_number);
        return -ENODEV;
    }
    
    return 0;
}

static int iqs7211e_init(const struct device *dev) {
    const struct iqs7211e_config *cfg = dev->config;
    struct iqs7211e_data *data = dev->data;
    int ret;
    
    if (!device_is_ready(cfg->i2c.bus)) {
        LOG_ERR("I2C bus %s is not ready", cfg->i2c.bus->name);
        return -ENODEV;
    }
    
    data->dev = dev;
    data->init_complete = false;
    data->previous_valid = false;
    data->pending_click_type = 0;
    data->scroll_was_active = false;
    data->x_resolution = 0;
    data->y_resolution = 0;
    data->resolution_valid = false;
    data->gesture_started_near_edge = false;
    data->scroller_axis_lock = IQS7211E_SCROLL_AXIS_NONE;
    data->motion_started = false;
    data->gesture_tap_pending = false;
    data->gesture_pressed_code = 0;
    data->gesture_multi_touch = false;
    data->gesture_click_pressed = false;
    data->gesture_click_drag = false;
    data->gesture_long_press_armed = false;
    data->gesture_long_press_fired = false;

    if (!iqs7211e_load_resolution_from_init(cfg, data)) {
        LOG_WRN("Failed to parse X/Y resolution from init data, edge tap suppression disabled");
    }

#if defined(CONFIG_IQS7211E_SCROLLER_INERTIA) && CONFIG_IQS7211E_SCROLLER_INERTIA
    data->inertia_wheel_velocity_q8 = 0;
    data->inertia_hwheel_velocity_q8 = 0;
    data->inertia_wheel_remainder_q8 = 0;
    data->inertia_hwheel_remainder_q8 = 0;
    data->inertia_last_wheel_time = 0;
    data->inertia_last_hwheel_time = 0;
    data->inertia_running = false;
#endif
    
    k_work_init(&data->motion_work, iqs7211e_motion_work_handler);
    k_work_init_delayable(&data->click_work, iqs7211e_click_work_handler);
    k_work_init_delayable(&data->gesture_tap_work, iqs7211e_gesture_tap_work_handler);
    k_work_init_delayable(&data->gesture_release_work, iqs7211e_gesture_release_work_handler);
    k_work_init_delayable(&data->gesture_click_release_work,
                          iqs7211e_gesture_click_release_work_handler);
    k_work_init_delayable(&data->gesture_long_press_work,
                          iqs7211e_gesture_long_press_work_handler);
#if defined(CONFIG_IQS7211E_SCROLLER_INERTIA) && CONFIG_IQS7211E_SCROLLER_INERTIA
    k_work_init_delayable(&data->inertia_work, iqs7211e_inertia_work_handler);
#endif
    
#if DT_INST_NODE_HAS_PROP(0, power_gpios)
    if (gpio_is_ready_dt(&cfg->power_gpio)) {
        ret = gpio_pin_configure_dt(&cfg->power_gpio, GPIO_OUTPUT_INACTIVE);
        if (ret != 0) {
            LOG_ERR("Power pin configuration failed: %d", ret);
            return ret;
        }
        
        k_sleep(K_MSEC(500));
        
        ret = gpio_pin_set_dt(&cfg->power_gpio, 1);
        if (ret != 0) {
            LOG_ERR("Power pin set failed: %d", ret);
            return ret;
        }
        
        k_sleep(K_MSEC(10));
    }
#endif
    
    if (gpio_is_ready_dt(&cfg->irq_gpio)) {
        ret = gpio_pin_configure_dt(&cfg->irq_gpio, GPIO_INPUT);
        if (ret != 0) {
            LOG_ERR("IRQ pin configuration failed: %d", ret);
            return ret;
        }
        
        gpio_init_callback(&data->motion_cb, iqs7211e_motion_handler, BIT(cfg->irq_gpio.pin));
        
        ret = gpio_add_callback_dt(&cfg->irq_gpio, &data->motion_cb);
        if (ret < 0) {
            LOG_ERR("Could not set motion callback: %d", ret);
            return ret;
        }

        LOG_DBG("IRQ pin configured, pin: %d", cfg->irq_gpio.pin);
    }
    
    ret = iqs7211e_configure(dev);
    if (ret != 0) {
        LOG_ERR("Device configuration failed: %d", ret);
        return ret;
    }
    
    if (gpio_is_ready_dt(&cfg->irq_gpio)) {
        ret = iqs7211e_interrupt_configure(dev, GPIO_INT_LEVEL_LOW);
        if (ret != 0) {
            LOG_ERR("Motion interrupt configuration failed: %d", ret);
            return ret;
        }
    }
    
    ret = pm_device_runtime_enable(dev);
    if (ret < 0) {
        LOG_ERR("Failed to enable runtime power management: %d", ret);
        return ret;
    }
    
    return 0;
}

#ifdef CONFIG_PM_DEVICE
static int iqs7211e_pm_action(const struct device *dev, enum pm_device_action action) {
    const struct iqs7211e_config *cfg = dev->config;
    struct iqs7211e_data *data = dev->data;
    int ret;
    
    switch (action) {
    case PM_DEVICE_ACTION_SUSPEND:
        if (gpio_is_ready_dt(&cfg->irq_gpio)) {
            ret = gpio_pin_interrupt_configure_dt(&cfg->irq_gpio, GPIO_INT_DISABLE);
            if (ret < 0) {
                LOG_ERR("Failed to disable IRQ interrupt: %d", ret);
                return ret;
            }
            
            ret = gpio_pin_configure_dt(&cfg->irq_gpio, GPIO_DISCONNECTED);
            if (ret < 0) {
                LOG_ERR("Failed to disconnect IRQ GPIO: %d", ret);
                return ret;
            }
        }

        iqs7211e_suspend(dev);

    #if defined(CONFIG_IQS7211E_SCROLLER_INERTIA) && CONFIG_IQS7211E_SCROLLER_INERTIA
        iqs7211e_stop_inertia_scroll(data);
    #endif

        iqs7211e_gesture_cancel(data);
        
        data->init_complete = false;
        break;
        
    case PM_DEVICE_ACTION_RESUME:

        iqs7211e_resume(dev);
        
        if (gpio_is_ready_dt(&cfg->irq_gpio)) {
            ret = gpio_pin_configure_dt(&cfg->irq_gpio, GPIO_INPUT);
            if (ret < 0) {
                LOG_ERR("Failed to configure IRQ GPIO: %d", ret);
                return ret;
            }
            
            ret = iqs7211e_interrupt_configure(dev, GPIO_INT_LEVEL_LOW);
            if (ret < 0) {
                LOG_ERR("Failed to enable IRQ interrupt: %d", ret);
                return ret;
            }
        }
        
        ret = iqs7211e_configure(dev);
        if (ret < 0) {
            LOG_ERR("Failed to reconfigure device: %d", ret);
            return ret;
        }
        break;
        
    default:
        return -ENOTSUP;
    }
    
    return 0;
}
#endif

/* If a device-tree node provides `init-symbol` and `init-length`,
 * use that C symbol as the init data for the instance. The
 * DT property name in DTS is `init-symbol` but DT macros use
 * an underscore variant `init_symbol`.
 */
#define IQS7211E_INIT(n)                                                                           \
    extern const uint8_t iqs7211e_init_default[];                                                   \
    COND_CODE_1(DT_INST_NODE_HAS_PROP(n, init_symbol),                                           \
        (extern const uint8_t DT_PROP(DT_DRV_INST(n), init_symbol_STRING_UNQUOTED )[];),                           \
        ())                                                                                       \
    static const struct iqs7211e_config iqs7211e_cfg_##n = {                                      \
        .i2c = I2C_DT_SPEC_INST_GET(n),                                                           \
        .irq_gpio = GPIO_DT_SPEC_INST_GET(n, irq_gpios),                                          \
        .power_gpio = GPIO_DT_SPEC_INST_GET_OR(n, power_gpios, {0}),                              \
        .init_data = COND_CODE_1(DT_INST_NODE_HAS_PROP(n, init_symbol),                           \
            (DT_PROP(DT_DRV_INST(n), init_symbol_STRING_UNQUOTED )),                                               \
            (iqs7211e_init_default)),                                                             \
        .init_len = COND_CODE_1(DT_INST_NODE_HAS_PROP(n, init_symbol),                             \
            (DT_PROP(DT_DRV_INST(n), init_length)),                                               \
            IQS7211E_INIT_DATA_LEN),                                                         \
        .scroller_mode = DT_INST_PROP_OR(n, scroller_mode, false),                                  \
        .gesture_mode = DT_INST_PROP_OR(n, gesture_mode, false),                                    \
        .v_invert = DT_INST_PROP_OR(n, v_invert, false),                                        \
        .h_invert = DT_INST_PROP_OR(n, h_invert, false),                                        \
    };                                                                                             \
                                                                                                   \
    static struct iqs7211e_data iqs7211e_data_##n;                                                \
                                                                                                   \
    PM_DEVICE_DT_INST_DEFINE(n, iqs7211e_pm_action);                                              \
                                                                                                   \
    DEVICE_DT_INST_DEFINE(n, iqs7211e_init, PM_DEVICE_DT_INST_GET(n), &iqs7211e_data_##n,        \
                          &iqs7211e_cfg_##n, POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(IQS7211E_INIT)

#endif // DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)