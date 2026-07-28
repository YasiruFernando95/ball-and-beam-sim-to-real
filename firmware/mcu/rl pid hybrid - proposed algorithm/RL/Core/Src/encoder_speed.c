/*
 * encoder_speed.c
 *
 *  Created on: Nov 15, 2025
 *      Author: USER-PC
 */

#include "encoder_speed.h"


void EncoderSpeed_Init(EncoderSpeed_Handle_t *h,
                       TIM_HandleTypeDef *htim,
                       uint32_t counts_per_rev,
                       float gear_ratio,
                       float dt_s)
{
    if (h == NULL) return;

    h->htim           = htim;
    h->counts_per_rev = (counts_per_rev == 0u) ? 1u : counts_per_rev; // avoid div0
    h->gear_ratio     = (gear_ratio == 0.0f) ? 1.0f : gear_ratio;
    h->dt_s           = (dt_s <= 0.0f) ? 1.0f : dt_s;

    h->prev_count   = __HAL_TIM_GET_COUNTER(htim);
    h->delta_counts = 0;
    h->motor_rps    = 0.0f;
    h->output_rps   = 0.0f;

    // LPF defaults: no filtering
    h->motor_rps_filt  = 0.0f;
    h->output_rps_filt = 0.0f;
    h->lpf_alpha       = 1.0f;   // y = x until user calls SetLowPass()
}

void EncoderSpeed_Update(EncoderSpeed_Handle_t *h)
{
    if (h == NULL || h->htim == NULL) return;

    uint32_t now = __HAL_TIM_GET_COUNTER(h->htim);
    uint32_t prev = h->prev_count;

    // Unsigned difference (wrap-safe modulo 2^32)
    uint32_t diff_u = now - prev;

    // Reinterpret difference as signed. This handles:
    //  - forward movement (small positive diff_u -> positive int32)
    //  - backward movement (in encoder mode) -> negative
    //  - wrap-around, as long as |delta| < 2^31 counts between samples.
    int32_t diff = -(int32_t)diff_u;

    h->prev_count   = now;
    h->delta_counts = diff;

    // Counts per motor revolution
    float cpr = (float)h->counts_per_rev;

    // Motor speed [rev/s]
    float motor_rps = diff / (cpr * h->dt_s);
    h->motor_rps    = motor_rps;

    // Output shaft speed [rev/s]
    // gear_ratio = motor_rev / output_rev
    h->output_rps   = motor_rps / h->gear_ratio;

    // --- NEW: first-order low-pass filter on motor_rps ---
    float alpha  = h->lpf_alpha;          // in [0,1]
    float x      = motor_rps;             // input
    float y_prev = h->motor_rps_filt;     // previous output
    float y      = y_prev + alpha * (x - y_prev);

    h->motor_rps_filt  = y;
    h->output_rps_filt = y / h->gear_ratio;
}
