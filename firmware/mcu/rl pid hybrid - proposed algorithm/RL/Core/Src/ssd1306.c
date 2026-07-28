#include "ssd1306.h"

// ---------------- Internal buffer & state ----------------
static uint8_t SSD1306_Buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];
static SSD1306_t SSD1306;

// DMA update state
static volatile uint8_t ssd1306_dma_busy = 0;
static uint8_t ssd1306_dma_page = 0;          // 0..7
static I2C_HandleTypeDef *ssd1306_hi2c = NULL;
static void (*ssd1306_user_done_cb)(void) = NULL;

// ---------------- Local helpers ----------------
static uint8_t ssd1306_WriteCommand(I2C_HandleTypeDef *hi2c, uint8_t command)
{
    return (HAL_I2C_Mem_Write(hi2c, SSD1306_I2C_ADDR, 0x00, 1, &command, 1, 10) == HAL_OK) ? 0 : 1;
}

static uint8_t ssd1306_SetPageColumn(I2C_HandleTypeDef *hi2c, uint8_t page)
{
    // Set page and reset column to 0
    uint8_t ok = 0;
    ok |= ssd1306_WriteCommand(hi2c, 0xB0 + page);
    ok |= ssd1306_WriteCommand(hi2c, 0x00);   // low col = 0
    ok |= ssd1306_WriteCommand(hi2c, 0x10);   // high col = 0
    return ok;
}

// Kick one page via DMA (data control byte 0x40). Returns 0 on started.
static uint8_t ssd1306_StartPageDMA(I2C_HandleTypeDef *hi2c, uint8_t page)
{
    if (ssd1306_SetPageColumn(hi2c, page)) return 1;
    uint8_t *ptr = &SSD1306_Buffer[SSD1306_WIDTH * page];
    HAL_StatusTypeDef st = HAL_I2C_Mem_Write_DMA(hi2c, SSD1306_I2C_ADDR, 0x40, 1, ptr, SSD1306_WIDTH);
    return (st == HAL_OK) ? 0 : 2;
}

// ---------------- Public API ----------------
uint8_t ssd1306_Init(I2C_HandleTypeDef *hi2c)
{
    HAL_Delay(100);
    int status = 0;

    status += ssd1306_WriteCommand(hi2c, 0xAE);   // display off
    status += ssd1306_WriteCommand(hi2c, 0x20);   // memory mode
    status += ssd1306_WriteCommand(hi2c, 0x10);   // page addressing
    status += ssd1306_WriteCommand(hi2c, 0xB0);   // start page 0
    status += ssd1306_WriteCommand(hi2c, 0xC8);   // COM scan dir remap
    status += ssd1306_WriteCommand(hi2c, 0x00);   // low col
    status += ssd1306_WriteCommand(hi2c, 0x10);   // high col
    status += ssd1306_WriteCommand(hi2c, 0x40);   // start line 0
    status += ssd1306_WriteCommand(hi2c, 0x81);   // contrast
    status += ssd1306_WriteCommand(hi2c, 0xFF);
    status += ssd1306_WriteCommand(hi2c, 0xA1);   // segment remap
    status += ssd1306_WriteCommand(hi2c, 0xA6);   // normal display

    status += ssd1306_WriteCommand(hi2c, 0xA8);   // multiplex
    status += ssd1306_WriteCommand(hi2c, SSD1306_HEIGHT - 1);

    status += ssd1306_WriteCommand(hi2c, 0xA4);   // display follows RAM
    status += ssd1306_WriteCommand(hi2c, 0xD3);   // display offset
    status += ssd1306_WriteCommand(hi2c, 0x00);
    status += ssd1306_WriteCommand(hi2c, 0xD5);   // clk divide
    status += ssd1306_WriteCommand(hi2c, 0xF0);
    status += ssd1306_WriteCommand(hi2c, 0xD9);   // precharge
    status += ssd1306_WriteCommand(hi2c, 0x22);

    status += ssd1306_WriteCommand(hi2c, 0xDA);   // COM pins
    status += ssd1306_WriteCommand(hi2c, (SSD1306_COM_LR_REMAP << 5) | (SSD1306_COM_ALTERNATIVE_PIN_CONFIG << 4) | 0x02);

    status += ssd1306_WriteCommand(hi2c, 0xDB);   // vcomh
    status += ssd1306_WriteCommand(hi2c, 0x20);
    status += ssd1306_WriteCommand(hi2c, 0x8D);   // charge pump
    status += ssd1306_WriteCommand(hi2c, 0x14);
    status += ssd1306_WriteCommand(hi2c, 0xAF);   // display on

    if (status != 0) return 1;

    ssd1306_Fill(Black);

    SSD1306.CurrentX = 0;
    SSD1306.CurrentY = 0;
    SSD1306.Initialized = 1;

    // optional first clear via DMA
    (void)ssd1306_UpdateScreen_DMA(hi2c);
    return 0;
}

void ssd1306_Fill(SSD1306_COLOR color)
{
    for (uint32_t i = 0; i < sizeof(SSD1306_Buffer); i++) {
        SSD1306_Buffer[i] = (color == Black) ? 0x00 : 0xFF;
    }
}

// Blocking update kept for reference (can be removed)
__attribute__((unused)) static void ssd1306_UpdateScreen_Blocking(I2C_HandleTypeDef *hi2c)
{
    for (uint8_t i = 0; i < 8; i++) {
        (void)ssd1306_SetPageColumn(hi2c, i);
        (void)HAL_I2C_Mem_Write(hi2c, SSD1306_I2C_ADDR, 0x40, 1, &SSD1306_Buffer[SSD1306_WIDTH * i], SSD1306_WIDTH, 100);
    }
}

uint8_t ssd1306_UpdateScreen_DMA(I2C_HandleTypeDef *hi2c)
{
    if (ssd1306_dma_busy) return 1; // busy
    ssd1306_dma_busy = 1;
    ssd1306_dma_page = 0;
    ssd1306_hi2c = hi2c;
    return ssd1306_StartPageDMA(hi2c, ssd1306_dma_page);
}

uint8_t ssd1306_IsBusy(void)
{
    return ssd1306_dma_busy;
}

uint8_t ssd1306_WaitReady(uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    while (ssd1306_IsBusy()) {
        if ((HAL_GetTick() - t0) > timeout_ms) return 1;
    }
    return 0;
}

void ssd1306_RegisterUpdateDoneCallback(void (*cb)(void))
{
    ssd1306_user_done_cb = cb;
}

void ssd1306_I2C_TxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c != ssd1306_hi2c) return; // not ours
    if (!ssd1306_dma_busy) return;

    // Advance to next page or finish
    if (++ssd1306_dma_page < 8) {
        // start next page
        if (ssd1306_StartPageDMA(hi2c, ssd1306_dma_page)) {
            // failure: abort sequence
            ssd1306_dma_busy = 0;
        }
    } else {
        ssd1306_dma_busy = 0;
        if (ssd1306_user_done_cb) ssd1306_user_done_cb();
    }
}

// ---------------- Drawing primitives ----------------
void ssd1306_DrawPixel(uint8_t x, uint8_t y, SSD1306_COLOR color)
{
    if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) return;
    if (SSD1306.Inverted) color = (SSD1306_COLOR)!color;

    uint32_t index = x + (y / 8) * SSD1306_WIDTH;
    uint8_t  mask  = 1u << (y % 8);
    if (color == White) SSD1306_Buffer[index] |=  mask;
    else                SSD1306_Buffer[index] &= ~mask;
}

char ssd1306_WriteChar(char ch, FontDef Font, SSD1306_COLOR color)
{
    if (SSD1306_WIDTH <= (SSD1306.CurrentX + Font.FontWidth) ||
        SSD1306_HEIGHT <= (SSD1306.CurrentY + Font.FontHeight)) {
        return 0;
    }

    for (uint32_t i = 0; i < Font.FontHeight; i++) {
        uint32_t b = Font.data[(ch - 32) * Font.FontHeight + i];
        for (uint32_t j = 0; j < Font.FontWidth; j++) {
            if ((b << j) & 0x8000) ssd1306_DrawPixel(SSD1306.CurrentX + j, SSD1306.CurrentY + i, color);
            else                   ssd1306_DrawPixel(SSD1306.CurrentX + j, SSD1306.CurrentY + i, (SSD1306_COLOR)!color);
        }
    }

    SSD1306.CurrentX += Font.FontWidth;
    return ch;
}

char ssd1306_WriteString(const char* str, FontDef Font, SSD1306_COLOR color)
{
    while (*str) {
        if (ssd1306_WriteChar(*str, Font, color) != *str) return *str;
        str++;
    }
    return *str;
}

void ssd1306_InvertColors(void)
{
    SSD1306.Inverted = !SSD1306.Inverted;
}

void ssd1306_SetCursor(uint8_t x, uint8_t y)
{
    SSD1306.CurrentX = x;
    SSD1306.CurrentY = y;
}
