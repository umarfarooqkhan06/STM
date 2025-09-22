#ifndef MAX30102_OXIMETER_H
#define MAX30102_OXIMETER_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MAX30102_I2C_TIMEOUT_MS 1000U

typedef struct {
    bool     pulse_detected;
    float    heart_rate_bpm;
    float    spo2_percent;
    float    temperature_c;
    float    ir_dc;
    float    red_dc;
    float    ir_filtered;
    float    last_peak_value;
    uint32_t timestamp_ms;
} MAX30102_Measurement;

typedef struct {
    I2C_HandleTypeDef *hi2c;

    /* Internal processing state */
    float    dc_ir_w;
    float    dc_red_w;
    float    mean_ir_values[15];
    uint8_t  mean_ir_index;
    float    mean_ir_sum;
    uint8_t  mean_ir_count;
    float    butterworth_v[2];
    float    butterworth_result;
    float    ir_ac_sq_sum;
    float    red_ac_sq_sum;
    uint16_t samples_recorded;
    uint16_t pulses_detected;
    float    spo2;
    float    bpm_buffer[10];
    float    bpm_sum;
    uint8_t  bpm_index;
    uint8_t  bpm_count;
    float    heart_bpm;
    float    last_peak_value;
    float    prev_sensor_value;
    uint8_t  values_went_down;
    uint32_t current_beat_ms;
    uint32_t last_beat_ms;
    uint8_t  pulse_state;
    float    last_temperature_c;
} MAX30102_Oximeter;

void MAX30102_Oximeter_Init(MAX30102_Oximeter *ctx, I2C_HandleTypeDef *hi2c);
bool MAX30102_Oximeter_Poll(MAX30102_Oximeter *ctx, MAX30102_Measurement *out);

#ifdef __cplusplus
}
#endif

#endif /* MAX30102_OXIMETER_H */
