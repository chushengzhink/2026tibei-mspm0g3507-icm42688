#include "chassis_telemetry_uart.h"

#include "chassis_telemetry.h"
#include "ml_board.h"
#include "ml_uart.h"

static bool g_telemetry_export_active;

static char telemetry_hex_digit(uint8_t value)
{
    value &= 0x0FU;
    return (value < 10U) ? (char) ('0' + value) :
        (char) ('A' + (value - 10U));
}

static void telemetry_put_four_digits(char text[], uint8_t offset,
    uint32_t value)
{
    uint32_t divisor = 1000U;
    uint8_t index;

    if (value > 9999U) {
        value = 9999U;
    }
    for (index = 0U; index < 4U; ++index) {
        text[offset + index] =
            (char) ('0' + ((value / divisor) % 10U));
        divisor /= 10U;
    }
}

static ml_status_t telemetry_uart_writer(
    const char *data, uint16_t length, void *context)
{
    uint16_t index;
    ml_status_t status = ML_STATUS_OK;

    (void) context;
    for (index = 0U; (index < length) &&
         (status == ML_STATUS_OK); ++index) {
        status = uart_sendbyte(UART0, (uint8_t) data[index]);
    }
    return status;
}

ml_status_t chassis_telemetry_uart0_handle_byte(
    uint8_t byte, bool chassis_stopped)
{
    if (!chassis_stopped) {
        return ML_STATUS_BUSY;
    }
    if ((byte == (uint8_t) 'D') || (byte == (uint8_t) 'd')) {
        ml_status_t status;

        if (g_telemetry_export_active) {
            return ML_STATUS_BUSY;
        }
        g_telemetry_export_active = true;
        status = chassis_telemetry_export_csv(
            telemetry_uart_writer, 0);
        g_telemetry_export_active = false;
        return status;
    }
    if ((byte == (uint8_t) 'C') || (byte == (uint8_t) 'c')) {
        chassis_telemetry_clear();
        return ML_STATUS_OK;
    }
    return ML_STATUS_INVALID_ARGUMENT;
}

ml_status_t chassis_uart0_send_busy(void)
{
    static const char line[] = "BUSY\r\n";

    return telemetry_uart_writer(
        line, (uint16_t) (sizeof(line) - 1U), 0);
}

ml_status_t chassis_uart0_send_diagnostic_banner(uint32_t sequence)
{
    char line[] = "UART0,115200,TX,0000\r\n";

    telemetry_put_four_digits(line, 16U, sequence);
    return telemetry_uart_writer(
        line, (uint16_t) (sizeof(line) - 1U), 0);
}

ml_status_t chassis_uart0_send_diagnostic_rx(
    uint8_t byte, uint8_t error_flags)
{
    char line[] = "RX,00,ERR,00\r\n";

    line[3] = telemetry_hex_digit((uint8_t) (byte >> 4U));
    line[4] = telemetry_hex_digit(byte);
    line[10] = telemetry_hex_digit(
        (uint8_t) (error_flags >> 4U));
    line[11] = telemetry_hex_digit(error_flags);
    return telemetry_uart_writer(
        line, (uint16_t) (sizeof(line) - 1U), 0);
}
