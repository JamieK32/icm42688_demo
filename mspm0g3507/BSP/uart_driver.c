#include "uart_driver.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define MAX_TX_BUFFER_SIZE 256

static char buffer[MAX_TX_BUFFER_SIZE];

void usart_send_bytes(UART_Regs* uart, const uint8_t* data, size_t length) {
    for (size_t i = 0; i < length; i++) {
        DL_UART_Main_transmitDataBlocking(uart, data[i]);
    }
}

void usart_printf(UART_Regs* uart, const char* format, ...)
{
    va_list args;
    va_start(args, format);

    int n = vsnprintf(buffer, sizeof(buffer), format, args);

    va_end(args);

    if (n < 0) {
        // 格式化失败，避免 strlen 越界
        return;
    }

    // vsnprintf 返回“本应写入的长度”（不含 '\0'）
    // 若截断，n 可能 >= sizeof(buffer)
    size_t len = (n >= (int)sizeof(buffer)) ? (sizeof(buffer) - 1) : (size_t)n;

    usart_send_bytes(uart, (const uint8_t*)buffer, len);
}