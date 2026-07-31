#ifndef TEST_BALL_APP_ML_OLED_H
#define TEST_BALL_APP_ML_OLED_H

#include <stdint.h>

#include "ml_common.h"

#define OLED_TEXT_LINE_COUNT   (4U)
#define OLED_TEXT_COLUMN_COUNT (16U)

ml_status_t OLED_Init(void);
ml_status_t OLED_Clear(void);
ml_status_t OLED_ShowLine(uint8_t line, const char *string);

#endif
