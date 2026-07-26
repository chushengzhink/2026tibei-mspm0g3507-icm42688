#ifndef ML_OLED_H
#define ML_OLED_H

#include <stdint.h>

#include "ml_common.h"

#define OLED_WIDTH             (128U)
#define OLED_PAGE_COUNT        (8U)
#define OLED_TEXT_LINE_COUNT   (4U)
#define OLED_TEXT_COLUMN_COUNT (16U)
#define OLED_I2C_ADDRESS       (0x3CU)

ml_status_t OLED_Init(void);
ml_status_t OLED_Clear(void);
ml_status_t OLED_WriteCommand(uint8_t command);
ml_status_t OLED_WriteData(uint8_t data);
ml_status_t OLED_SetCursor(uint8_t page, uint8_t x);
ml_status_t OLED_ShowChar(uint8_t line, uint8_t column, char character);
ml_status_t OLED_ShowString(
    uint8_t line, uint8_t column, const char *string);
ml_status_t OLED_ShowNum(
    uint8_t line, uint8_t column, uint32_t number, uint8_t length);
ml_status_t OLED_ShowSignedNum(
    uint8_t line, uint8_t column, int32_t number, uint8_t length);
ml_status_t OLED_ShowHexNum(
    uint8_t line, uint8_t column, uint32_t number, uint8_t length);
ml_status_t OLED_ShowBinNum(
    uint8_t line, uint8_t column, uint32_t number, uint8_t length);
ml_status_t OLED_ShowFloat(uint8_t line, uint8_t column, float number,
    uint8_t integer_length, uint8_t fractional_length);
ml_status_t OLED_DrawBMP(uint8_t x0, uint8_t page0, uint8_t x1,
    uint8_t page1, const uint8_t bitmap[]);

#endif
