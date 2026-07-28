/*
 * dist_adc.h
 *
 *  Created on: Sep 29, 2025
 *      Author: USER-PC
 */

#ifndef DIST_ADC_H
#define DIST_ADC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stddef.h>

typedef struct
{
    ADC_HandleTypeDef *hadc;

    // DMA buffer provided by user (must remain valid)
    uint16_t *dma_buf;
    size_t    dma_len;

    // ADC & voltage
    float     vref;                  // e.g., 3.3f
    uint8_t   adc_bits;              // e.g., 12 for STM32F4 (0..4095)
    uint32_t  max_counts;            // (1<<adc_bits) - 1

    float     voltage_raw;           // latest averaged raw pin voltage (before LPF)
    float     voltage_v;             // low-pass-filtered pin voltage

    // Low-pass filter: y(k) = y(k-1) + alpha * (x(k) - y(k-1))
    // alpha in (0,1]; alpha=1 => no filtering (y=x)
    float     lp_alpha;

    // Distance scaling (uses voltage_v)
    // Piecewise linear across v_low -> v_mid -> v_high
    // Maps to [0, 0.5, 1] respectively (then optional invert and gamma)
    float     v_low;
    float     v_mid;
    float     v_high;
    float     gamma;                 // >= 0.1, typical 1.0 (linear), >1 compresses, <1 expands
    uint8_t   invert;                // 0 = normal, 1 = invert (1 - x)
    float     distance_01;           // 0..1 scaled value

    // (Timing fields kept for backward compatibility; not used internally)
    uint32_t  compute_period_ms;
    uint32_t  last_compute_ms;

} AdcDistance_t;

/**
 * @brief  Initialize the reader (no hardware start here).
 * @param  ctx                 Module context (allocated by user)
 * @param  hadc                Configured ADC handle (continuous + DMA)
 * @param  dma_buf             Buffer for DMA (uint16_t samples)
 * @param  dma_len             Buffer length (>= 2 recommended)
 * @param  vref                ADC reference voltage (e.g., 3.3f)
 * @param  adc_bits            ADC resolution bits (e.g., 12)
 * @param  v_low               Lower bound voltage (maps to 0.0)
 * @param  v_mid               Midpoint voltage (maps to 0.5). If not strictly between
 *                             v_low and v_high, it will be auto-set to (v_low+v_high)/2.
 * @param  v_high              Upper bound voltage (maps to 1.0)
 * @param  invert              0 = normal, 1 = invert scale
 * @param  gamma               Nonlinearity exponent (>=0.1, usually 1.0)
 * @param  compute_period_ms   Unused inside module (kept for API compatibility)
 * @return HAL status
 */
HAL_StatusTypeDef AdcDist_Init(AdcDistance_t *ctx,
                               ADC_HandleTypeDef *hadc,
                               uint16_t *dma_buf, size_t dma_len,
                               float vref, uint8_t adc_bits,
                               float v_low, float v_mid, float v_high,
                               uint8_t invert, float gamma,
                               uint32_t compute_period_ms);

/**
 * @brief  Start ADC in DMA circular mode.
 * @return HAL status
 */
HAL_StatusTypeDef AdcDist_Start(AdcDistance_t *ctx);

/**
 * @brief  Call from main loop or a fixed-rate timer interrupt.
 *         Averages the DMA buffer, applies low-pass filter, and updates
 *         voltage & distance. No HAL calls inside; ISR-safe.
 */
void AdcDist_Update(AdcDistance_t *ctx);

/**
 * @brief  Set scaling parameters at runtime (piecewise linear v_low, v_mid, v_high).
 */
void AdcDist_SetScale(AdcDistance_t *ctx, float v_low, float v_mid, float v_high, uint8_t invert, float gamma);

/**
 * @brief  Configure low-pass filter alpha (0 < alpha <= 1).
 *         alpha = 1.0f => no filtering (voltage_v follows voltage_raw).
 */
void AdcDist_SetFilterAlpha(AdcDistance_t *ctx, float alpha);

/**
 * @brief  Getters
 */
static inline float AdcDist_GetVoltageRaw(const AdcDistance_t *ctx) { return ctx->voltage_raw; }
static inline float AdcDist_GetVoltage(const AdcDistance_t *ctx)    { return ctx->voltage_v;   }
static inline float AdcDist_GetDistance(const AdcDistance_t *ctx)   { return ctx->distance_01; }

#ifdef __cplusplus
}
#endif

#endif // DIST_ADC_H
