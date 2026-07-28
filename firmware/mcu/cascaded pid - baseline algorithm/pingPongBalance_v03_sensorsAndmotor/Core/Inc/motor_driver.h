/*
 * motor_driver.h
 *
 *  Created on: Nov 16, 2025
 *      Author: USER-PC
 */

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/**
 * Direction logic:
 *   If speed > 0   -> forwardDirLevel is written to dir_pin
 *   If speed < 0   -> !forwardDirLevel is written to dir_pin
 *   If speed == 0  -> PWM = 0 (direction pin unchanged by default)
 */
typedef struct
{
    TIM_HandleTypeDef *htim;     // PWM timer handle
    uint32_t           channel;  // e.g. TIM_CHANNEL_1

    GPIO_TypeDef      *dir_port;
    uint16_t           dir_pin;

    uint32_t           pwm_period;       // Timer ARR value (counter period)
    GPIO_PinState      forwardDirLevel;  // GPIO_PIN_SET or GPIO_PIN_RESET
} MotorDriver_HandleTypeDef;

/**
 * @brief  Initialize motor driver handle.
 * @param  hmotor          Pointer to handle.
 * @param  htim            Timer handle configured for PWM.
 * @param  channel         PWM channel (e.g. TIM_CHANNEL_1).
 * @param  dir_port        GPIO port for direction pin.
 * @param  dir_pin         GPIO pin for direction pin.
 * @param  pwm_period      Timer period (ARR). Example: __HAL_TIM_GET_AUTORELOAD(htim)
 * @param  forwardLevel    Logic level for "forward" direction (GPIO_PIN_SET / RESET).
 */
void MotorDriver_Init(MotorDriver_HandleTypeDef *hmotor,
                      TIM_HandleTypeDef *htim,
                      uint32_t channel,
                      GPIO_TypeDef *dir_port,
                      uint16_t dir_pin,
                      uint32_t pwm_period,
                      GPIO_PinState forwardLevel);

/**
 * @brief  Start PWM output.
 */
HAL_StatusTypeDef MotorDriver_Start(MotorDriver_HandleTypeDef *hmotor);

/**
 * @brief  Stop PWM output (duty forced to 0).
 */
HAL_StatusTypeDef MotorDriver_Stop(MotorDriver_HandleTypeDef *hmotor);

/**
 * @brief  Set motor speed command in range [-1.0f, 1.0f].
 *         Magnitude -> PWM duty, sign -> direction pin.
 */
void MotorDriver_SetSpeed(MotorDriver_HandleTypeDef *hmotor, float speed);

/**
 * @brief  Directly set duty as a fraction [0.0f, 1.0f] without changing direction.
 */
void MotorDriver_SetDuty(MotorDriver_HandleTypeDef *hmotor, float duty);

#ifdef __cplusplus
}
#endif

#endif // MOTOR_DRIVER_H

