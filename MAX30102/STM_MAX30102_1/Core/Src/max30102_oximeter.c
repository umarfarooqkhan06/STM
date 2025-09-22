#include "max30102_oximeter.h"
#include <limits.h>
#include <math.h>
#include <string.h>

#define MAX30102_I2C_ADDR                 (0x57U << 1)
#define MAX30102_FIFO_DATA                0x07U
#define MAX30102_FIFO_WR_PTR              0x04U
#define MAX30102_FIFO_RD_PTR              0x06U
#define MAX30102_OVF_COUNTER              0x05U
#define MAX30102_FIFO_CONFIG              0x08U
#define MAX30102_MODE_CONFIG              0x09U
#define MAX30102_SPO2_CONFIG              0x0AU
#define MAX30102_LED1_PA                  0x0CU
#define MAX30102_LED2_PA                  0x0DU
#define MAX30102_MULTI_LED_CTRL1          0x11U
#define MAX30102_MULTI_LED_CTRL2          0x12U
#define MAX30102_DIE_TEMP_CONFIG          0x21U
#define MAX30102_DIE_TEMP_INT             0x1FU
#define MAX30102_DIE_TEMP_FRAC            0x20U

#define MAX30102_FIFO_DEPTH               32U
#define MAX30102_SAMPLE_BYTES             6U

#define MAX30102_MULTI_LED_MODE           0x07U

#define MAX30102_SR_100HZ                 (0x02U << 2)
#define MAX30102_ADC_2048                 (0x00U << 5)
#define MAX30102_PW_16BIT                 (0x01U)

#define MAX30102_SAMPLE_AVG_8             (0x03U << 5)
#define MAX30102_ROLL_OVER_ENABLE         (1U << 4)

#define MAX30102_INT_EN1                  0x02U
#define MAX30102_INT_EN2                  0x03U
#define MAX30102_INT_STATUS2              0x01U
#define MAX30102_INT1_DATA_RDY_MASK       (1U << 6)
#define MAX30102_INT2_TEMP_RDY_MASK       (1U << 1)

#define MAX30102_MODE_RESET               (1U << 6)

#define MAX30102_TEMP_LSB                 0.0625f

#define PULSE_STATE_IDLE                  0U
#define PULSE_STATE_TRACE_UP              1U
#define PULSE_STATE_TRACE_DOWN            2U

#define PULSE_MIN_THRESHOLD               100.0f
#define PULSE_MAX_THRESHOLD               2000.0f
#define BPM_SAMPLES                       10U
#define SPO2_RESET_PULSE_INTERVAL         4U
#define ALPHA                             0.95f
#define MEAN_FILTER_SIZE                  15U

static void max30102_write(MAX30102_Oximeter *ctx, uint8_t reg, const uint8_t *buffer, uint16_t len) {
    uint8_t payload[1 + 6];
    payload[0] = reg;
    if (len && buffer) {
        memcpy(&payload[1], buffer, len);
    }
    HAL_I2C_Master_Transmit(ctx->hi2c, MAX30102_I2C_ADDR, payload, len + 1U, MAX30102_I2C_TIMEOUT_MS);
}

static void max30102_read(MAX30102_Oximeter *ctx, uint8_t reg, uint8_t *buffer, uint16_t len) {
    HAL_I2C_Master_Transmit(ctx->hi2c, MAX30102_I2C_ADDR, &reg, 1U, MAX30102_I2C_TIMEOUT_MS);
    HAL_I2C_Master_Receive(ctx->hi2c, MAX30102_I2C_ADDR, buffer, len, MAX30102_I2C_TIMEOUT_MS);
}

static void max30102_reset(MAX30102_Oximeter *ctx) {
    uint8_t cfg = MAX30102_MODE_RESET;
    max30102_write(ctx, MAX30102_MODE_CONFIG, &cfg, 1U);
    HAL_Delay(10U);
}

static void max30102_clear_fifo(MAX30102_Oximeter *ctx) {
    uint8_t zero = 0U;
    max30102_write(ctx, MAX30102_FIFO_WR_PTR, &zero, 1U);
    max30102_write(ctx, MAX30102_FIFO_RD_PTR, &zero, 1U);
    max30102_write(ctx, MAX30102_OVF_COUNTER, &zero, 1U);
}

static void max30102_configure(MAX30102_Oximeter *ctx) {
    uint8_t cfg;

    cfg = MAX30102_SAMPLE_AVG_8 | MAX30102_ROLL_OVER_ENABLE | 0x00U;
    max30102_write(ctx, MAX30102_FIFO_CONFIG, &cfg, 1U);

    cfg = MAX30102_SR_100HZ | MAX30102_ADC_2048 | MAX30102_PW_16BIT;
    max30102_write(ctx, MAX30102_SPO2_CONFIG, &cfg, 1U);

    cfg = 0x32U; /* ~10mA */
    max30102_write(ctx, MAX30102_LED1_PA, &cfg, 1U);
    max30102_write(ctx, MAX30102_LED2_PA, &cfg, 1U);

    /* Slot 1 -> RED LED2, Slot 2 -> IR LED1 */
    cfg = 0x21U;
    max30102_write(ctx, MAX30102_MULTI_LED_CTRL1, &cfg, 1U);
    cfg = 0x00U;
    max30102_write(ctx, MAX30102_MULTI_LED_CTRL2, &cfg, 1U);

    cfg = MAX30102_MULTI_LED_MODE;
    max30102_write(ctx, MAX30102_MODE_CONFIG, &cfg, 1U);

    cfg = MAX30102_INT1_DATA_RDY_MASK;
    max30102_write(ctx, MAX30102_INT_EN1, &cfg, 1U);

    cfg = MAX30102_INT2_TEMP_RDY_MASK;
    max30102_write(ctx, MAX30102_INT_EN2, &cfg, 1U);
}

static void max30102_request_die_temp(MAX30102_Oximeter *ctx) {
    uint8_t start = 0x01U;
    max30102_write(ctx, MAX30102_DIE_TEMP_CONFIG, &start, 1U);
}

static float dc_removal(float x, float *w) {
    float result = x - *w;
    *w = x - result * ALPHA;
    return result;
}

static float mean_diff_filter(MAX30102_Oximeter *ctx, float sample) {
    ctx->mean_ir_sum -= ctx->mean_ir_values[ctx->mean_ir_index];
    ctx->mean_ir_values[ctx->mean_ir_index] = sample;
    ctx->mean_ir_sum += sample;
    ctx->mean_ir_index = (ctx->mean_ir_index + 1U) % MEAN_FILTER_SIZE;
    if (ctx->mean_ir_count < MEAN_FILTER_SIZE) {
        ctx->mean_ir_count++;
    }
    const float avg = ctx->mean_ir_count ? (ctx->mean_ir_sum / ctx->mean_ir_count) : sample;
    return avg - sample;
}

static float low_pass_butterworth(MAX30102_Oximeter *ctx, float x) {
    ctx->butterworth_v[0] = ctx->butterworth_v[1];
    ctx->butterworth_v[1] = (0.24523727525278560f * x) + (0.50952544949442879f * ctx->butterworth_v[0]);
    ctx->butterworth_result = ctx->butterworth_v[0] + ctx->butterworth_v[1];
    return ctx->butterworth_result;
}

static void reset_spo2_window(MAX30102_Oximeter *ctx) {
    ctx->ir_ac_sq_sum = 0.0f;
    ctx->red_ac_sq_sum = 0.0f;
    ctx->samples_recorded = 0U;
}

static bool detect_pulse(MAX30102_Oximeter *ctx, float sensor_value, uint32_t now_ms) {
    if (sensor_value > PULSE_MAX_THRESHOLD) {
        ctx->pulse_state       = PULSE_STATE_IDLE;
        ctx->prev_sensor_value = 0.0f;
        ctx->last_beat_ms      = 0U;
        ctx->current_beat_ms   = 0U;
        ctx->values_went_down  = 0U;
        ctx->last_peak_value   = 0.0f;
        return false;
    }

    switch (ctx->pulse_state) {
    case PULSE_STATE_IDLE:
        if (sensor_value >= PULSE_MIN_THRESHOLD) {
            ctx->pulse_state = PULSE_STATE_TRACE_UP;
            ctx->values_went_down = 0U;
        }
        break;

    case PULSE_STATE_TRACE_UP:
        if (sensor_value > ctx->prev_sensor_value) {
            ctx->current_beat_ms = now_ms;
            ctx->last_peak_value = sensor_value;
        } else {
            uint32_t beat_duration = ctx->current_beat_ms - ctx->last_beat_ms;
            ctx->last_beat_ms = ctx->current_beat_ms;

            float raw_bpm = 0.0f;
            if (beat_duration > 0U) {
                raw_bpm = 60000.0f / (float)beat_duration;
            }

            ctx->bpm_sum -= ctx->bpm_buffer[ctx->bpm_index];
            ctx->bpm_buffer[ctx->bpm_index] = raw_bpm;
            ctx->bpm_sum += raw_bpm;

            if (ctx->bpm_count < BPM_SAMPLES) {
                ctx->bpm_count++;
            }
            ctx->bpm_index = (ctx->bpm_index + 1U) % BPM_SAMPLES;

            if (ctx->bpm_count > 0U) {
                ctx->heart_bpm = ctx->bpm_sum / ctx->bpm_count;
            }

            ctx->pulse_state = PULSE_STATE_TRACE_DOWN;
            return true;
        }
        break;

    case PULSE_STATE_TRACE_DOWN:
        if (sensor_value < ctx->prev_sensor_value) {
            if (ctx->values_went_down < UINT8_MAX) {
                ctx->values_went_down++;
            }
        }
        if (sensor_value < PULSE_MIN_THRESHOLD) {
            ctx->pulse_state = PULSE_STATE_IDLE;
        }
        break;

    default:
        ctx->pulse_state = PULSE_STATE_IDLE;
        break;
    }

    ctx->prev_sensor_value = sensor_value;
    return false;
}

static bool process_sample(MAX30102_Oximeter *ctx, uint32_t ir_raw, uint32_t red_raw, MAX30102_Measurement *out) {
    const float ir = (float)(ir_raw & 0x3FFFFU);
    const float red = (float)(red_raw & 0x3FFFFU);
    const uint32_t t = HAL_GetTick();

    const float ir_dc_removed = dc_removal(ir, &ctx->dc_ir_w);
    const float red_dc_removed = dc_removal(red, &ctx->dc_red_w);

    const float mean_ir = mean_diff_filter(ctx, ir_dc_removed);
    const float filtered_ir = low_pass_butterworth(ctx, mean_ir);

    ctx->ir_ac_sq_sum += ir_dc_removed * ir_dc_removed;
    ctx->red_ac_sq_sum += red_dc_removed * red_dc_removed;
    ctx->samples_recorded++;

    bool pulse_detected = detect_pulse(ctx, filtered_ir, t);
    if (pulse_detected && ctx->samples_recorded > 0U) {
        ctx->pulses_detected++;

        const float ir_rms = sqrtf(ctx->ir_ac_sq_sum / (float)ctx->samples_recorded);
        const float red_rms = sqrtf(ctx->red_ac_sq_sum / (float)ctx->samples_recorded);
        if ((ir_rms > 0.0f) && (red_rms > 0.0f)) {
            const float ratio_rms = logf(red_rms) / logf(ir_rms);
            ctx->spo2 = 110.0f - 18.0f * ratio_rms;
            if (ctx->spo2 > 100.0f) {
                ctx->spo2 = 100.0f;
            }
            if (ctx->spo2 < 0.0f) {
                ctx->spo2 = 0.0f;
            }
        }

        if ((ctx->pulses_detected % SPO2_RESET_PULSE_INTERVAL) == 0U) {
            reset_spo2_window(ctx);
        }
    }

    if (out) {
        uint8_t status;
        max30102_read(ctx, MAX30102_INT_STATUS2, &status, 1U);
        if (status & MAX30102_INT2_TEMP_RDY_MASK) {
            uint8_t int_part = 0U;
            uint8_t frac_part = 0U;
            max30102_read(ctx, MAX30102_DIE_TEMP_INT, &int_part, 1U);
            max30102_read(ctx, MAX30102_DIE_TEMP_FRAC, &frac_part, 1U);
            ctx->last_temperature_c = (int8_t)int_part + MAX30102_TEMP_LSB * (float)frac_part;
            max30102_request_die_temp(ctx);
        }

        out->pulse_detected = pulse_detected;
        out->heart_rate_bpm = ctx->heart_bpm;
        out->spo2_percent = ctx->spo2;
        out->ir_dc = ctx->dc_ir_w;
        out->red_dc = ctx->dc_red_w;
        out->ir_filtered = filtered_ir;
        out->last_peak_value = ctx->last_peak_value;
        out->timestamp_ms = t;
        out->temperature_c = ctx->last_temperature_c;
    }

    return true;
}

void MAX30102_Oximeter_Init(MAX30102_Oximeter *ctx, I2C_HandleTypeDef *hi2c) {
    memset(ctx, 0, sizeof(*ctx));
    ctx->hi2c = hi2c;

    HAL_Delay(10U);
    max30102_reset(ctx);
    max30102_clear_fifo(ctx);
    max30102_configure(ctx);
    max30102_request_die_temp(ctx);
}

bool MAX30102_Oximeter_Poll(MAX30102_Oximeter *ctx, MAX30102_Measurement *out) {
    uint8_t write_ptr = 0U;
    uint8_t read_ptr = 0U;
    max30102_read(ctx, MAX30102_FIFO_WR_PTR, &write_ptr, 1U);
    max30102_read(ctx, MAX30102_FIFO_RD_PTR, &read_ptr, 1U);

    int16_t available = (int16_t)write_ptr - (int16_t)read_ptr;
    if (available < 0) {
        available += MAX30102_FIFO_DEPTH;
    }
    if (available <= 0) {
        return false;
    }

    bool updated = false;
    while (available-- > 0) {
        uint8_t raw[MAX30102_SAMPLE_BYTES];
        max30102_read(ctx, MAX30102_FIFO_DATA, raw, sizeof(raw));

        const uint32_t red = ((uint32_t)raw[0] << 16) | ((uint32_t)raw[1] << 8) | raw[2];
        const uint32_t ir  = ((uint32_t)raw[3] << 16) | ((uint32_t)raw[4] << 8) | raw[5];

        if (process_sample(ctx, ir, red, out)) {
            updated = true;
        }
    }

    return updated;
}
