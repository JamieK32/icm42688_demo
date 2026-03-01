#include "icm_spi_driver.h"

#define ICM_CS_LOW()     DL_GPIO_clearPins(ICM42688_PORT, ICM42688_CS_PIN)
#define ICM_CS_HIGH()    DL_GPIO_setPins(ICM42688_PORT, ICM42688_CS_PIN)

// SPI 写字节函数
static uint8_t spi_read_write_byte(SPI_Regs *spi_inst, uint8_t byte)
{
    // 等待 SPI 传输完成 (如果你使用的是阻塞模式的 transmitData8，这可能不需要)
    while (DL_SPI_isBusy(spi_inst));
    DL_SPI_transmitData8(spi_inst, byte);
    // 等待接收 FIFO 非空，表示数据已接收 (对于纯发送可能不需要接收数据)
    while(DL_SPI_isRXFIFOEmpty(spi_inst));
    // 返回接收到的数据 (如果不需要接收，可以返回 dummy 值)
    return DL_SPI_receiveData8(spi_inst); // 如果不需要接收，可以注释掉
}

/**
 * @brief SPI 连续读函数实现
 * @param user_ctx 用户上下文（通常传 NULL 或 SPI 句柄，此处由于你代码中硬编码了 SPI_0_INST，可不使用）
 * @param reg      寄存器地址
 * @param data     接收数据的缓冲区指针
 * @param len      读取数据的长度
 * @return int     成功返回 0
 */
int my_spi_read(void *user_ctx, uint8_t reg, uint8_t *data, uint32_t len) {
		SPI_Regs *spi_inst = (SPI_Regs *)user_ctx;
    // 1. 拉低片选，开始通信
    ICM_CS_LOW();
    
    // 2. 发送寄存器地址，MSB 置 1 表示读操作 (reg | 0x80)
    spi_read_write_byte(spi_inst, reg | 0x80);
    
    // 3. 循环读取数据
    for (uint32_t i = 0; i < len; i++) {
        // SPI 是交换协议，发送 0x00 以产生时钟并获取数据
        data[i] = spi_read_write_byte(spi_inst, 0x00);
    }
    
    // 4. 拉高片选，结束通信
    ICM_CS_HIGH();
    
    return 0; 
}

/**
 * @brief SPI 连续写函数实现
 * @param user_ctx 用户上下文
 * @param reg      寄存器地址
 * @param data     要写入的数据缓冲区指针
 * @param len      写入数据的长度
 * @return int     成功返回 0
 */
int my_spi_write(void *user_ctx, uint8_t reg, const uint8_t *data, uint32_t len) {
		SPI_Regs *spi_inst = (SPI_Regs *)user_ctx;
    // 1. 拉低片选
    ICM_CS_LOW();
    
    // 2. 发送寄存器地址，MSB 置 0 表示写操作 (reg & 0x7F)
    spi_read_write_byte(spi_inst, reg & 0x7F);
    
    // 3. 循环写入数据
    for (uint32_t i = 0; i < len; i++) {
        spi_read_write_byte(spi_inst, data[i]);
    }
    
    // 4. 拉高片选
    ICM_CS_HIGH();
    
    return 0; 
}
