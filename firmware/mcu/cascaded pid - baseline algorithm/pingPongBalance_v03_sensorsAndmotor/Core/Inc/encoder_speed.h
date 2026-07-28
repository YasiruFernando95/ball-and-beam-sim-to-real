/*
 * encoder_speed.h
 *
 *  Created on: Nov 15, 2025
 *      Author: USER-PC
 */

#ifndef ENCODER_SPEED_H
#define ENCODER_SPEED_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

typedef struct
{
    TIM_HandleTypeDef *htim;

    uint32_t counts_per_rev;
    float    gear_ratio;
    float    dt_s;

    uint32_t prev_count;
    int32_t  delta_counts;
    float    motor_rps;        // unfiltered
    float    output_rps;       // unfiltered

    // --- NEW: low-pass filter state ---
    float    motor_rps_filt;   // filtered motor speed [rev/s]
    float    output_rps_filt;  // filtered output speed [rev/s]
    float    lpf_alpha;        // filter coefficient in [0,1]
} EncoderSpeed_Handle_t;
/**
 * @brief Initialise encoder speed handle.
 *
 * @param h          Pointer to handle.
 * @param htim       Pointer to timer handle (already configured in encoder mode).
 * @param counts_per_rev  Encoder counts per motor rev (e.g. PPR * 4).
 * @param gear_ratio motor_rev / output_rev (e.g. 30 for 30:1).
 * @param dt_s       Sample period in seconds (time between Update() calls).
 */
void EncoderSpeed_Init(EncoderSpeed_Handle_t *h,
                       TIM_HandleTypeDef *htim,
                       uint32_t counts_per_rev,
                       float gear_ratio,
                       float dt_s);

/**
 * @brief Call this at fixed period dt_s to update speed.
 */
void EncoderSpeed_Update(EncoderSpeed_Handle_t *h);

/**
 * @brief Get current position as signed int32 (wrapped).
 */
static inline int32_t EncoderSpeed_GetPosition(const EncoderSpeed_Handle_t *h)
{
    // Wrap the 32-bit counter to signed position
    return (int32_t)__HAL_TIM_GET_COUNTER(h->htim);
}

/**
 * @brief Get last delta counts (signed).
 */
static inline int32_t EncoderSpeed_GetDeltaCounts(const EncoderSpeed_Handle_t *h)
{
    return h->delta_counts;
}

/**
 * @brief Get last motor speed in revolutions per second.
 */
static inline float EncoderSpeed_GetMotorRPS(const EncoderSpeed_Handle_t *h)
{
    return h->motor_rps;
}

/**
 * @brief Get last motor speed in RPM.
 */
static inline float EncoderSpeed_GetMotorRPM(const EncoderSpeed_Handle_t *h)
{
    return h->motor_rps * 60.0f;
}

/**
 * @brief Get last output shaft speed in revolutions per second.
 */
static inline float EncoderSpeed_GetOutputRPS(const EncoderSpeed_Handle_t *h)
{
    return h->output_rps;
}

/**
 * @brief Get last output shaft speed in RPM.
 */
static inline float EncoderSpeed_GetOutputRPM(const EncoderSpeed_Handle_t *h)
{
    return h->output_rps * 60.0f;
}

static inline float EncoderSpeed_GetMotorRadPerSec(const EncoderSpeed_Handle_t *h)
{
    return h->motor_rps * 2.0f * M_PI;
}

static inline float EncoderSpeed_GetOutputRadPerSec(const EncoderSpeed_Handle_t *h)
{
    return h->output_rps * 2.0f * M_PI;
}

static inline float EncoderSpeed_GetOutputRadPerSecFilt(const EncoderSpeed_Handle_t *h)
{
    return h->output_rps_filt * 2.0f * M_PI;
}


/**
 * @brief Configure a first-order low-pass filter on speed.
 *
 * cutoff_hz <= 0  => filter disabled (alpha = 1, no smoothing).
 */
static inline void EncoderSpeed_SetLowPass(EncoderSpeed_Handle_t *h, float cutoff_hz)
{
    if (h == NULL) return;

    if (cutoff_hz <= 0.0f || h->dt_s <= 0.0f)
    {
        // No filtering: y(k) = x(k)
        h->lpf_alpha = 1.0f;
        return;
    }

    // RC = 1 / (2π f_c)
    float rc = 1.0f / (2.0f * M_PI * cutoff_hz);

    // alpha = dt / (RC + dt)
    h->lpf_alpha = h->dt_s / (rc + h->dt_s);
}

static inline float EncoderSpeed_GetMotorRPS_Filt(const EncoderSpeed_Handle_t *h)
{
    return h->motor_rps_filt;
}

static inline float EncoderSpeed_GetMotorRPM_Filt(const EncoderSpeed_Handle_t *h)
{
    return h->motor_rps_filt * 60.0f;
}

static inline float EncoderSpeed_GetOutputRPS_Filt(const EncoderSpeed_Handle_t *h)
{
    return h->output_rps_filt;
}

static inline float EncoderSpeed_GetOutputRPM_Filt(const EncoderSpeed_Handle_t *h)
{
    return h->output_rps_filt * 60.0f;
}

#ifdef __cplusplus
}
#endif

#endif // ENCODER_SPEED_H
