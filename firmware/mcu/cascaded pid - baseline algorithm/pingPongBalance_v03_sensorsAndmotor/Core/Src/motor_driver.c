/*
 * motor_driver.c
 *
 *  Created on: Nov 16, 2025
 *      Author: USER-PC
 */


#include "motor_driver.h"
#include <math.h>

static uint32_t clamp_u32(uint32_t val, uint32_t min, uint32_t max)
{
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

void MotorDriver_Init(MotorDriver_HandleTypeDef *hmotor,
                      TIM_HandleTypeDef *htim,
                      uint32_t channel,
                      GPIO_TypeDef *dir_port,
                      uint16_t dir_pin,
                      uint32_t pwm_period,
                      GPIO_PinState forwardLevel)
{
    hmotor->htim            = htim;
    hmotor->channel         = channel;
    hmotor->dir_port        = dir_port;
    hmotor->dir_pin         = dir_pin;
    hmotor->pwm_period      = pwm_period;
    hmotor->forwardDirLevel = forwardLevel;

    // Optional: default duty = 0
    __HAL_TIM_SET_COMPARE(hmotor->htim, hmotor->channel, 0U);
}

HAL_StatusTypeDef MotorDriver_Start(MotorDriver_HandleTypeDef *hmotor)
{
    return HAL_TIM_PWM_Start(hmotor->htim, hmotor->channel);
}

HAL_StatusTypeDef MotorDriver_Stop(MotorDriver_HandleTypeDef *hmotor)
{
    // Force duty to zero before stopping
    __HAL_TIM_SET_COMPARE(hmotor->htim, hmotor->channel, 0U);
    return HAL_TIM_PWM_Stop(hmotor->htim, hmotor->channel);
}

void MotorDriver_SetDuty(MotorDriver_HandleTypeDef *hmotor, float duty)
{
    if (duty < 0.0f) duty = 0.0f;
    if (duty > 1.0f) duty = 1.0f;

    float compare_f = duty * (float)hmotor->pwm_period;
    uint32_t compare = (uint32_t)(compare_f + 0.5f);

    compare = clamp_u32(compare, 0U, hmotor->pwm_period);
    __HAL_TIM_SET_COMPARE(hmotor->htim, hmotor->channel, compare);
}

void MotorDriver_SetSpeed(MotorDriver_HandleTypeDef *hmotor, float speed)
{
    // Clamp to [-1, 1]
    if (speed > 1.0f)  speed = 1.0f;
    if (speed < -1.0f) speed = -1.0f;

    // Set direction according to sign
    if (speed > 0.0f)
    {
        HAL_GPIO_WritePin(hmotor->dir_port, hmotor->dir_pin, hmotor->forwardDirLevel);
    }
    else if (speed < 0.0f)
    {
        HAL_GPIO_WritePin(hmotor->dir_port, hmotor->dir_pin,
                          (hmotor->forwardDirLevel == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }
    else
    {
        // speed == 0 -> duty 0. Direction pin left unchanged (you can change this if needed).
    }

    // Magnitude -> PWM duty
    float mag = fabsf(speed);
    MotorDriver_SetDuty(hmotor, mag);
}
