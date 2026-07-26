#include "ml_oled.h"

#include "ml_delay.h"
#include "ml_i2c.h"
#include "ml_oled_font.h"

static ml_soft_i2c_bus_t g_oled_i2c_bus = {
    ML_OLED_PORT,
    ML_OLED_SCL_PIN,
    ML_OLED_SCL_IOMUX,
    ML_OLED_SDA_PIN,
    ML_OLED_SDA_IOMUX,
    ML_SOFT_I2C_HALF_PERIOD_US,
    ML_SOFT_I2C_TIMEOUT_US,
    false
};

static ml_status_t oled_write_payload(
    uint8_t control, const uint8_t *data, uint32_t length)
{
    uint8_t packet[OLED_WIDTH + 1U];
    uint32_t index;

    if ((data == 0) || (length == 0U) || (length > OLED_WIDTH)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    packet[0] = control;
    for (index = 0U; index < length; ++index) {
        packet[index + 1U] = data[index];
    }
    return soft_i2c_write(
        &g_oled_i2c_bus, OLED_I2C_ADDRESS, packet, length + 1U);
}

static ml_status_t oled_show_unsigned(uint8_t line, uint8_t column,
    uint32_t number, uint8_t length, uint8_t base)
{
    uint32_t divisor = 1U;
    uint8_t index;
    ml_status_t status;

    if ((length == 0U) || (length > OLED_TEXT_COLUMN_COUNT) ||
        (column == 0U) ||
        ((uint32_t) column + length - 1U > OLED_TEXT_COLUMN_COUNT) ||
        ((base != 2U) && (base != 10U) && (base != 16U))) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    for (index = 1U; index < length; ++index) {
        if (divisor > (UINT32_MAX / base)) {
            return ML_STATUS_UNSUPPORTED;
        }
        divisor *= base;
    }
    for (index = 0U; index < length; ++index) {
        uint8_t digit = (uint8_t) ((number / divisor) % base);
        char character = (digit < 10U) ?
            (char) ('0' + digit) : (char) ('A' + digit - 10U);

        status = OLED_ShowChar(line, (uint8_t) (column + index), character);
        if (status != ML_STATUS_OK) {
            return status;
        }
        if (divisor > 1U) {
            divisor /= base;
        }
    }
    return ML_STATUS_OK;
}

ml_status_t OLED_WriteCommand(uint8_t command)
{
    return oled_write_payload(0x00U, &command, 1U);
}

ml_status_t OLED_WriteData(uint8_t data)
{
    return oled_write_payload(0x40U, &data, 1U);
}

ml_status_t OLED_SetCursor(uint8_t page, uint8_t x)
{
    uint8_t commands[3];

    if ((page >= OLED_PAGE_COUNT) || (x >= OLED_WIDTH)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    commands[0] = (uint8_t) (0xB0U | page);
    commands[1] = (uint8_t) (0x10U | ((x >> 4) & 0x0FU));
    commands[2] = (uint8_t) (x & 0x0FU);
    return oled_write_payload(0x00U, commands, sizeof(commands));
}

ml_status_t OLED_Clear(void)
{
    uint8_t zeros[OLED_WIDTH] = {0};
    uint8_t page;
    ml_status_t status;

    for (page = 0U; page < OLED_PAGE_COUNT; ++page) {
        status = OLED_SetCursor(page, 0U);
        if (status == ML_STATUS_OK) {
            status = oled_write_payload(0x40U, zeros, sizeof(zeros));
        }
        if (status != ML_STATUS_OK) {
            return status;
        }
    }
    return ML_STATUS_OK;
}

ml_status_t OLED_ShowChar(uint8_t line, uint8_t column, char character)
{
    uint8_t glyph_index;
    ml_status_t status;

    if ((line == 0U) || (line > OLED_TEXT_LINE_COUNT) ||
        (column == 0U) || (column > OLED_TEXT_COLUMN_COUNT) ||
        ((uint8_t) character < (uint8_t) ' ') ||
        ((uint8_t) character > (uint8_t) '~')) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    glyph_index = (uint8_t) character - (uint8_t) ' ';
    status = OLED_SetCursor(
        (uint8_t) ((line - 1U) * 2U), (uint8_t) ((column - 1U) * 8U));
    if (status == ML_STATUS_OK) {
        status = oled_write_payload(
            0x40U, &OLED_F8x16[glyph_index][0], 8U);
    }
    if (status == ML_STATUS_OK) {
        status = OLED_SetCursor((uint8_t) (((line - 1U) * 2U) + 1U),
            (uint8_t) ((column - 1U) * 8U));
    }
    if (status == ML_STATUS_OK) {
        status = oled_write_payload(
            0x40U, &OLED_F8x16[glyph_index][8], 8U);
    }
    return status;
}

ml_status_t OLED_ShowString(
    uint8_t line, uint8_t column, const char *string)
{
    uint8_t length = 0U;
    uint8_t index;
    ml_status_t status;

    if ((string == 0) || (column == 0U) ||
        (column > OLED_TEXT_COLUMN_COUNT)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    while (string[length] != '\0') {
        if ((uint32_t) column + length > OLED_TEXT_COLUMN_COUNT) {
            return ML_STATUS_INVALID_ARGUMENT;
        }
        ++length;
    }
    for (index = 0U; index < length; ++index) {
        status = OLED_ShowChar(
            line, (uint8_t) (column + index), string[index]);
        if (status != ML_STATUS_OK) {
            return status;
        }
    }
    return ML_STATUS_OK;
}

ml_status_t OLED_ShowNum(
    uint8_t line, uint8_t column, uint32_t number, uint8_t length)
{
    return oled_show_unsigned(line, column, number, length, 10U);
}

ml_status_t OLED_ShowSignedNum(
    uint8_t line, uint8_t column, int32_t number, uint8_t length)
{
    uint32_t magnitude;
    ml_status_t status;

    if ((length == 0U) ||
        ((uint32_t) column + length > OLED_TEXT_COLUMN_COUNT)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    if (number < 0) {
        magnitude = (uint32_t) (-(number + 1)) + 1U;
        status = OLED_ShowChar(line, column, '-');
    } else {
        magnitude = (uint32_t) number;
        status = OLED_ShowChar(line, column, '+');
    }
    if (status != ML_STATUS_OK) {
        return status;
    }
    return oled_show_unsigned(
        line, (uint8_t) (column + 1U), magnitude, length, 10U);
}

ml_status_t OLED_ShowHexNum(
    uint8_t line, uint8_t column, uint32_t number, uint8_t length)
{
    return oled_show_unsigned(line, column, number, length, 16U);
}

ml_status_t OLED_ShowBinNum(
    uint8_t line, uint8_t column, uint32_t number, uint8_t length)
{
    return oled_show_unsigned(line, column, number, length, 2U);
}

ml_status_t OLED_ShowFloat(uint8_t line, uint8_t column, float number,
    uint8_t integer_length, uint8_t fractional_length)
{
    float absolute;
    uint32_t scale = 1U;
    uint32_t whole;
    uint32_t fraction;
    uint8_t index;
    bool negative;
    ml_status_t status;

    if ((number != number) || (integer_length == 0U) ||
        (fractional_length == 0U) || (fractional_length > 9U) ||
        ((uint32_t) column + integer_length + fractional_length + 1U >
            OLED_TEXT_COLUMN_COUNT)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    negative = number < 0.0f;
    absolute = negative ? -number : number;
    if (absolute >= 4294967296.0f) {
        return ML_STATUS_UNSUPPORTED;
    }
    for (index = 0U; index < fractional_length; ++index) {
        scale *= 10U;
    }
    whole = (uint32_t) absolute;
    fraction = (uint32_t) (((absolute - (float) whole) * scale) + 0.5f);
    if (fraction >= scale) {
        if (whole == UINT32_MAX) {
            return ML_STATUS_UNSUPPORTED;
        }
        ++whole;
        fraction = 0U;
    }

    status = OLED_ShowChar(line, column, negative ? '-' : '+');
    if (status == ML_STATUS_OK) {
        status = OLED_ShowNum(
            line, (uint8_t) (column + 1U), whole, integer_length);
    }
    if (status == ML_STATUS_OK) {
        status = OLED_ShowChar(line,
            (uint8_t) (column + integer_length + 1U), '.');
    }
    if (status == ML_STATUS_OK) {
        status = OLED_ShowNum(line,
            (uint8_t) (column + integer_length + 2U),
            fraction, fractional_length);
    }
    return status;
}

ml_status_t OLED_Init(void)
{
    static const uint8_t init_commands[] = {
        0xAEU, 0xD5U, 0x80U, 0xA8U, 0x3FU, 0xD3U, 0x00U, 0x40U,
        0xA1U, 0xC8U, 0xDAU, 0x12U, 0x81U, 0xCFU, 0xD9U, 0xF1U,
        0xDBU, 0x30U, 0xA4U, 0xA6U, 0x8DU, 0x14U, 0xAFU
    };
    ml_status_t status;

    delay_ms(100U);
    status = soft_i2c_init(&g_oled_i2c_bus);
    if (status == ML_STATUS_OK) {
        status = oled_write_payload(
            0x00U, init_commands, sizeof(init_commands));
    }
    if (status == ML_STATUS_OK) {
        status = OLED_Clear();
    }
    return status;
}

ml_status_t OLED_DrawBMP(uint8_t x0, uint8_t page0, uint8_t x1,
    uint8_t page1, const uint8_t bitmap[])
{
    uint8_t page;
    uint32_t offset = 0U;
    uint32_t width;
    ml_status_t status;

    if ((bitmap == 0) || (x0 >= x1) || (x1 > OLED_WIDTH) ||
        (page0 >= page1) || (page1 > OLED_PAGE_COUNT)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    width = (uint32_t) x1 - x0;
    for (page = page0; page < page1; ++page) {
        status = OLED_SetCursor(page, x0);
        if (status == ML_STATUS_OK) {
            status = oled_write_payload(0x40U, &bitmap[offset], width);
        }
        if (status != ML_STATUS_OK) {
            return status;
        }
        offset += width;
    }
    return ML_STATUS_OK;
}
