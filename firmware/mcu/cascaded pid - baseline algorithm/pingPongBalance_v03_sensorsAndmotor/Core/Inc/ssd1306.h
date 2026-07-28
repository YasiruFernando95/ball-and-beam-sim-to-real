#ifndef _SSD1306_H
#define _SSD1306_H

#include "stm32f4xx_hal.h"
#include "fonts.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// I2C address (8-bit form for HAL)
#ifndef SSD1306_I2C_ADDR
#define SSD1306_I2C_ADDR        0x78
#endif

#ifndef SSD1306_WIDTH
#define SSD1306_WIDTH           128
#endif

#ifndef SSD1306_HEIGHT
#define SSD1306_HEIGHT          64
#endif

#ifndef SSD1306_COM_LR_REMAP
#define SSD1306_COM_LR_REMAP    0
#endif

#ifndef SSD1306_COM_ALTERNATIVE_PIN_CONFIG
#define SSD1306_COM_ALTERNATIVE_PIN_CONFIG    1
#endif

// Colours
typedef enum {
    Black = 0x00,
    White = 0x01,
} SSD1306_COLOR;

// State
typedef struct {
    uint16_t CurrentX;
    uint16_t CurrentY;
    uint8_t  Inverted;
    uint8_t  Initialized;
} SSD1306_t;

// ---- API (blocking drawing, non‑blocking screen update via I2C DMA) ----
uint8_t ssd1306_Init(I2C_HandleTypeDef *hi2c);
void    ssd1306_Fill(SSD1306_COLOR color);
void    ssd1306_DrawPixel(uint8_t x, uint8_t y, SSD1306_COLOR color);
char    ssd1306_WriteChar(char ch, FontDef Font, SSD1306_COLOR color);
char    ssd1306_WriteString(const char* str, FontDef Font, SSD1306_COLOR color);
void    ssd1306_SetCursor(uint8_t x, uint8_t y);
void    ssd1306_InvertColors(void);

// ---- DMA screen update API ----
// Starts a non‑blocking full‑screen flush (8 pages). Returns 0 on success, 1 if busy, 2 on start error.
uint8_t ssd1306_UpdateScreen_DMA(I2C_HandleTypeDef *hi2c);

// True while a DMA flush is in progress
uint8_t ssd1306_IsBusy(void);

// Busy‑wait (polling HAL_GetTick) until flush completes or timeout_ms elapses. Returns 0 if done, 1 on timeout.
uint8_t ssd1306_WaitReady(uint32_t timeout_ms);

// Optional: register a user callback invoked after the final page has been transferred.
void    ssd1306_RegisterUpdateDoneCallback(void (*cb)(void));

// Must be called from your global HAL_I2C_MemTxCpltCallback (or HAL_I2C_MasterTxCpltCallback)
// Example:
// void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c) { ssd1306_I2C_TxCpltCallback(hi2c); }
void    ssd1306_I2C_TxCpltCallback(I2C_HandleTypeDef *hi2c);

#ifdef __cplusplus
}
#endif

#endif // _SSD1306_H
