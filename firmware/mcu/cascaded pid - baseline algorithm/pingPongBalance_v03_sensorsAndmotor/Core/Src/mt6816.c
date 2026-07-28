/*
 * mt6816.c
 *
 *  Created on: Nov 15, 2025
 *      Author: USER-PC
 */

#include "mt6816.h"

void MT6816_Init(MT6816_Handle_t *henc,
                 SPI_HandleTypeDef *hspi,
                 uint32_t update_period_ms)
{
    if (henc == NULL)
        return;

    henc->hspi = hspi;

    henc->spi_busy        = 0u;
    henc->new_data        = 0u;
    henc->raw_value       = 0u;
    henc->last_update_ms  = 0u;
    henc->update_period_ms = update_period_ms;

    // Prepare command bytes (same as your example)
    henc->tx_buf[0] = 0x03;
    henc->tx_buf[0] |= 0x80;
    henc->tx_buf[0] |= 0x40;

    henc->tx_buf[1] = 0x04;
    henc->tx_buf[1] |= 0x80;
    henc->tx_buf[1] |= 0x40;

    henc->tx_buf[2] = 0x00;
}

void MT6816_Update(MT6816_Handle_t *henc, uint32_t now_ms)
{
    if (henc == NULL || henc->hspi == NULL)
        return;

    if (henc->spi_busy)
        return;

    // timing check
    if ((now_ms - henc->last_update_ms) < henc->update_period_ms)
        return;

    henc->last_update_ms = now_ms;

    henc->spi_busy = 1u;
    henc->new_data = 0u;

    HAL_StatusTypeDef status =
        HAL_SPI_TransmitReceive_DMA(henc->hspi,
                                    henc->tx_buf,
                                    henc->rx_buf,
                                    3);

    if (status != HAL_OK)
    {
        henc->spi_busy = 0u;
    }
}

void MT6816_SpiTxRxCpltCallback(MT6816_Handle_t *henc, SPI_HandleTypeDef *hspi)
{
    if (henc == NULL || henc->hspi == NULL)
        return;

    // Check that this callback is for our SPI instance
    if (hspi != henc->hspi)
        return;

    henc->spi_busy = 0u;

    // Convert RX bytes to 14-bit angle:
    // Same as your example:
    // absEnc_value = (((uint16_t)enc_rx_buf[1] << 8) | (uint16_t)enc_rx_buf[2]) >> 2;
    uint16_t raw = ((uint16_t)henc->rx_buf[0] << 8) | //This encoder is actually not working correctly and always sends the data at byte 0
                   (uint16_t)henc->rx_buf[1];

    raw >>= 2; // 14-bit angle in bits 13:0

    henc->raw_value = raw;
    henc->new_data  = 1u;
}

bool MT6816_GetRaw(MT6816_Handle_t *henc, uint16_t *out_value)
{
    if (henc == NULL)
        return false;

    if (!henc->new_data)
        return false;

    if (out_value != NULL)
    {
        *out_value = henc->raw_value;
    }

    henc->new_data = 0u;
    return true;
}
