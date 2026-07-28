/*
 * dist_adc.c
 *
 *  Created on: Sep 29, 2025
 *      Author: USER-PC
 */

#include "dist_adc.h"
#include <math.h>

static float clamp01(float x)
{
    if (x < 0.0f) return 0.0f;
    if (x > 1.0f) return 1.0f;
    return x;
}

static float safe_mid(float low, float mid, float high)
{
    if (high <= low) return 0.5f * (low + high);
    if (!(low < mid && mid < high)) return 0.5f * (low + high);
    return mid;
}

HAL_StatusTypeDef AdcDist_Init(AdcDistance_t *ctx,
                               ADC_HandleTypeDef *hadc,
                               uint16_t *dma_buf, size_t dma_len,
                               float vref, uint8_t adc_bits,
                               float v_low, float v_mid, float v_high,
                               uint8_t invert, float gamma,
                               uint32_t compute_period_ms)
{
    if (!ctx || !hadc || !dma_buf || dma_len < 2U) return HAL_ERROR;

    ctx->hadc      = hadc;
    ctx->dma_buf   = dma_buf;
    ctx->dma_len   = dma_len;
    ctx->vref      = (vref > 0.0f) ? vref : 3.3f;
    ctx->adc_bits  = (adc_bits > 0U && adc_bits <= 16U) ? adc_bits : 12U;
    ctx->max_counts = (1UL << ctx->adc_bits) - 1UL;

    // Raw and filtered voltages
    ctx->voltage_raw = 0.0f;
    ctx->voltage_v   = 0.0f;

    // LPF: default alpha = 1.0 (no filtering)
    ctx->lp_alpha    = 1.0f;

    // Scaling (piecewise linear)
    // Ensure a sane range; if v_high <= v_low, fall back to [0, vref] with mid at half
    if (v_high <= v_low) {
        ctx->v_low  = 0.0f;
        ctx->v_high = ctx->vref;
        ctx->v_mid  = 0.5f * (ctx->v_low + ctx->v_high);
    } else {
        ctx->v_low  = v_low;
        ctx->v_high = v_high;
        ctx->v_mid  = safe_mid(v_low, v_mid, v_high);
    }

    ctx->invert      = invert ? 1U : 0U;
    ctx->gamma       = (gamma < 0.1f) ? 0.1f : gamma;
    ctx->distance_01 = 0.0f;

    // Timing fields kept for backward compatibility (not used)
    ctx->compute_period_ms = compute_period_ms;
    ctx->last_compute_ms   = 0U;

    return HAL_OK;
}

HAL_StatusTypeDef AdcDist_Start(AdcDistance_t *ctx)
{
    if (!ctx || !ctx->hadc || !ctx->dma_buf || ctx->dma_len < 2U) return HAL_ERROR;

    // Start ADC DMA in circular mode. CubeMX should have ADC->DMA configured for circular.
    return HAL_ADC_Start_DMA(ctx->hadc, (uint32_t*)ctx->dma_buf, ctx->dma_len);
}

void AdcDist_SetScale(AdcDistance_t *ctx, float v_low, float v_mid, float v_high, uint8_t invert, float gamma)
{
    if (!ctx) return;

    if (v_high <= v_low) {
        // Keep current if bad inputs; or reset to full-scale
        ctx->v_low  = 0.0f;
        ctx->v_high = ctx->vref;
        ctx->v_mid  = 0.5f * (ctx->v_low + ctx->v_high);
    } else {
        ctx->v_low  = v_low;
        ctx->v_high = v_high;
        ctx->v_mid  = safe_mid(v_low, v_mid, v_high);
    }

    ctx->invert = invert ? 1U : 0U;
    ctx->gamma  = (gamma < 0.1f) ? 0.1f : gamma;
}

void AdcDist_SetFilterAlpha(AdcDistance_t *ctx, float alpha)
{
    if (!ctx) return;

    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    ctx->lp_alpha = alpha;
}

void AdcDist_Update(AdcDistance_t *ctx)
{
    if (!ctx) return;

    // Average the DMA buffer
    uint32_t acc = 0U;
    for (size_t i = 0; i < ctx->dma_len; ++i) {
        acc += ctx->dma_buf[i];
    }
    float avg_counts = (float)acc / (float)ctx->dma_len;

    // Convert to voltage at the ADC pin (raw)
    float v = (avg_counts / (float)ctx->max_counts) * ctx->vref;
    ctx->voltage_raw = v;

    // Apply low-pass filter: y(k) = y(k-1) + alpha * (x(k) - y(k-1))
    float alpha = ctx->lp_alpha;
    if (alpha <= 0.0f) {
        // freeze at previous
    } else if (alpha >= 1.0f) {
        ctx->voltage_v = v;
    } else {
        ctx->voltage_v = ctx->voltage_v + alpha * (v - ctx->voltage_v);
    }

    // Piecewise linear mapping using v_low -> v_mid -> v_high
    const float vf   = ctx->voltage_v;
    const float vl   = ctx->v_low;
    const float vm   = ctx->v_mid;
    const float vh   = ctx->v_high;

    float x;
    if (vf <= vl) {
        x = 0.0f;
    } else if (vf >= vh) {
        x = 1.0f;
    } else if (vf <= vm) {
        float den = (vm - vl);
        if (den <= 0.0f) {
            // Degenerate mid; treat as direct low->high mapping
            x = (vf - vl) / (vh - vl);
        } else {
            x = 0.5f * (vf - vl) / den;   // maps [vl..vm] -> [0..0.5]
        }
    } else { // vf > vm && vf < vh
        float den = (vh - vm);
        if (den <= 0.0f) {
            x = (vf - vl) / (vh - vl);
        } else {
            x = 0.5f + 0.5f * (vf - vm) / den; // maps [vm..vh] -> [0.5..1]
        }
    }

    // Clamp, invert, gamma-shape (optional)
    x = clamp01(x);
    if (ctx->invert) x = 1.0f - x;
    x = powf(x, ctx->gamma);
    ctx->distance_01 = clamp01(x);
}
