/*
 * mt6816.h
 *
 *  Created on: Nov 15, 2025
 *      Author: USER-PC
 */

#ifndef MT6816_H
#define MT6816_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    SPI_HandleTypeDef *hspi;

    uint8_t       tx_buf[3];
    uint8_t       rx_buf[3];

    volatile uint8_t spi_busy;
    volatile uint8_t new_data;

    uint16_t      raw_value;       // 14-bit position (0..16383)

    uint32_t      last_update_ms;
    uint32_t      update_period_ms; // e.g. 5 ms
} MT6816_Handle_t;

/**
 * @brief Initialise MT6816 handle.
 *
 * @param henc            Pointer to encoder handle.
 * @param hspi            SPI handle (e.g. &hspi1).
 * @param update_period_ms Minimum time between DMA reads (ms).
 */
void MT6816_Init(MT6816_Handle_t *henc,
                 SPI_HandleTypeDef *hspi,
                 uint32_t update_period_ms);

/**
 * @brief Call regularly (e.g. in main loop) with current time in ms.
 *
 * Non-blocking. Starts a DMA transfer when:
 *  - enough time has passed since last update, and
 *  - SPI is not already busy.
 */
void MT6816_Update(MT6816_Handle_t *henc, uint32_t now_ms);

/**
 * @brief Call this from HAL_SPI_TxRxCpltCallback().
 *
 * Example:
 *   void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
 *   {
 *       MT6816_SpiTxRxCpltCallback(&mt6816, hspi);
 *   }
 */
void MT6816_SpiTxRxCpltCallback(MT6816_Handle_t *henc, SPI_HandleTypeDef *hspi);

/**
 * @brief Get last raw 14-bit value if new data is available.
 *
 * @param henc      Encoder handle.
 * @param out_value Pointer to store raw value (0..16383).
 * @return true if a new value was returned, false if no new data.
 */
bool MT6816_GetRaw(MT6816_Handle_t *henc, uint16_t *out_value);

#ifdef __cplusplus
}
#endif

#endif // MT6816_H
