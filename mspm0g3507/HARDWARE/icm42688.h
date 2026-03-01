#ifndef ICM42688_H
#define ICM42688_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// 宏定义
#define ICM42688_WHO_AM_I_VAL      0x47
#define ICM42688_NUM_CALIB_SAMPLES 1000
#define ICM42688_TEMP_SCALE        132.48f
#define ICM42688_TEMP_OFFSET       25.0f

// 枚举类型定义
typedef enum {
    ICM42688_GYRO_FS_2000dps  = 0x00,
    ICM42688_GYRO_FS_1000dps  = 0x01,
    ICM42688_GYRO_FS_500dps   = 0x02,
    ICM42688_GYRO_FS_250dps   = 0x03,
    ICM42688_GYRO_FS_125dps   = 0x04,
    ICM42688_GYRO_FS_62_5dps  = 0x05,
    ICM42688_GYRO_FS_31_25dps = 0x06,
    ICM42688_GYRO_FS_15_625dps= 0x07
} icm42688_gyro_fs_t;

typedef enum {
    ICM42688_ACCEL_FS_16g = 0x00,
    ICM42688_ACCEL_FS_8g  = 0x01,
    ICM42688_ACCEL_FS_4g  = 0x02,
    ICM42688_ACCEL_FS_2g  = 0x03
} icm42688_accel_fs_t;

typedef enum {
    ICM42688_ODR_32k    = 0x01, // LN mode only
    ICM42688_ODR_16k    = 0x02, // LN mode only
    ICM42688_ODR_8k     = 0x03, // LN mode only
    ICM42688_ODR_4k     = 0x04, // LN mode only
    ICM42688_ODR_2k     = 0x05, // LN mode only
    ICM42688_ODR_1k     = 0x06, // LN mode only
    ICM42688_ODR_200    = 0x07,
    ICM42688_ODR_100    = 0x08,
    ICM42688_ODR_50     = 0x09,
    ICM42688_ODR_25     = 0x0A,
    ICM42688_ODR_12_5   = 0x0B,
    ICM42688_ODR_6a25   = 0x0C, // LP mode only (accel only)
    ICM42688_ODR_3a125  = 0x0D, // LP mode only (accel only)
    ICM42688_ODR_1a5625 = 0x0E, // LP mode only (accel only)
    ICM42688_ODR_500    = 0x0F,
} icm42688_odr_t;

typedef enum {
    ICM42688_NFBW_1449Hz = 0x00,
    ICM42688_NFBW_680Hz  = 0x01,
    ICM42688_NFBW_329Hz  = 0x02,
    ICM42688_NFBW_162Hz  = 0x03,
    ICM42688_NFBW_80Hz   = 0x04,
    ICM42688_NFBW_40Hz   = 0x05,
    ICM42688_NFBW_20Hz   = 0x06,
    ICM42688_NFBW_10Hz   = 0x07,
} icm42688_gyro_nfbw_t;

typedef enum {
    ICM42688_FILT_ORD_1ST = 0x00,
    ICM42688_FILT_ORD_2ND = 0x01,
    ICM42688_FILT_ORD_3RD = 0x02,
} icm42688_ui_filt_ord_t;

// 通信接口函数指针类型定义
// 成功应返回 0，失败返回非0
typedef int (*icm42688_read_cb_t)(void *user_ctx, uint8_t reg, uint8_t *data, uint32_t len);
typedef int (*icm42688_write_cb_t)(void *user_ctx, uint8_t reg, const uint8_t *data, uint32_t len);
typedef void (*icm42688_delay_us_cb_t)(uint32_t us);

// ICM42688 设备上下文结构体
typedef struct {
    // 硬件抽象层接口
    icm42688_read_cb_t     read_cb;
    icm42688_write_cb_t    write_cb;
    icm42688_delay_us_cb_t delay_us_cb;
    void                  *user_ctx;     // 用户自定义数据，如 SPI 句柄，I2C 地址等

    // 内部状态和数据缓冲
    uint8_t buffer[64];
    uint8_t bank;

    float t;
    float acc[3];
    float gyr[3];

    int16_t raw_t;
    int16_t raw_acc[3];
    int16_t raw_gyr[3];

    int32_t raw_acc_bias[3];
    int32_t raw_gyr_bias[3];
    int16_t acc_offset[3];
    int16_t gyr_offset[3];

    float accel_scale;
    float gyro_scale;

    icm42688_accel_fs_t accel_fs;
    icm42688_gyro_fs_t  gyro_fs;

    float acc_b[3];
    float acc_s[3];
    float gyr_b[3];

    // FIFO 状态
    bool   en_fifo_accel;
    bool   en_fifo_gyro;
    bool   en_fifo_temp;
    bool   en_fifo_timestamp;
    bool   en_fifo_header;
    size_t fifo_size;
    size_t fifo_frame_size;
    
    float  ax_fifo[128];
    float  ay_fifo[128];
    float  az_fifo[128];
    size_t a_size;
    
    float  gx_fifo[128];
    float  gy_fifo[128];
    float  gz_fifo[128];
    size_t g_size;
    
    float  t_fifo[256];
    size_t t_size;
		
} icm42688_t;

// 初始化 HAL 接口
void icm42688_init_hal(icm42688_t *dev, icm42688_read_cb_t read_cb, icm42688_write_cb_t write_cb, icm42688_delay_us_cb_t delay_us_cb, void *user_ctx);

// 基础功能
int icm42688_begin(icm42688_t *dev);
void icm42688_reset(icm42688_t *dev);
int icm42688_who_am_i(icm42688_t *dev, uint8_t *who_am_i);

// 传感器配置
int icm42688_set_accel_fs(icm42688_t *dev, icm42688_accel_fs_t fssel);
int icm42688_get_accel_fs(icm42688_t *dev);
int icm42688_set_gyro_fs(icm42688_t *dev, icm42688_gyro_fs_t fssel);
int icm42688_get_gyro_fs(icm42688_t *dev);
int icm42688_set_accel_odr(icm42688_t *dev, icm42688_odr_t odr);
int icm42688_set_gyro_odr(icm42688_t *dev, icm42688_odr_t odr);
int icm42688_set_filters(icm42688_t *dev, bool gyro_filters, bool acc_filters);

// 中断功能
int icm42688_enable_data_ready_interrupt(icm42688_t *dev);
int icm42688_disable_data_ready_interrupt(icm42688_t *dev);

// 获取数据 (计算值)
int icm42688_get_agt(icm42688_t *dev);
// 获取数据 (原始值)
int icm42688_get_raw_agt(icm42688_t *dev);

// 零偏校准与Offset
int icm42688_calibrate_gyro(icm42688_t *dev);
int icm42688_calibrate_accel(icm42688_t *dev);
int icm42688_compute_offsets(icm42688_t *dev);
int icm42688_set_all_offsets(icm42688_t *dev);

int icm42688_set_acc_x_offset(icm42688_t *dev, int16_t offset);
int icm42688_set_acc_y_offset(icm42688_t *dev, int16_t offset);
int icm42688_set_acc_z_offset(icm42688_t *dev, int16_t offset);
int icm42688_set_gyr_x_offset(icm42688_t *dev, int16_t offset);
int icm42688_set_gyr_y_offset(icm42688_t *dev, int16_t offset);
int icm42688_set_gyr_z_offset(icm42688_t *dev, int16_t offset);

// 滤波器相关
int icm42688_set_ui_filter_block(icm42688_t *dev, icm42688_ui_filt_ord_t gyro_ord, icm42688_ui_filt_ord_t acc_ord);
int icm42688_set_gyro_notch_filter(icm42688_t *dev, float freq_x, float freq_y, float freq_z, icm42688_gyro_nfbw_t bw);

// 分辨率获取
float icm42688_get_accel_res(icm42688_t *dev);
float icm42688_get_gyro_res(icm42688_t *dev);

// FIFO 相关
int icm42688_fifo_enable(icm42688_t *dev, bool accel, bool gyro, bool temp);
int icm42688_fifo_stream_to_fifo(icm42688_t *dev);
int icm42688_fifo_read(icm42688_t *dev);
void icm42688_fifo_get_accel_x(icm42688_t *dev, size_t *size, float *data);
void icm42688_fifo_get_accel_y(icm42688_t *dev, size_t *size, float *data);
void icm42688_fifo_get_accel_z(icm42688_t *dev, size_t *size, float *data);
void icm42688_fifo_get_gyro_x(icm42688_t *dev, size_t *size, float *data);
void icm42688_fifo_get_gyro_y(icm42688_t *dev, size_t *size, float *data);
void icm42688_fifo_get_gyro_z(icm42688_t *dev, size_t *size, float *data);
void icm42688_fifo_get_temp(icm42688_t *dev, size_t *size, float *data);


#ifdef __cplusplus
}
#endif

#endif // ICM42688_H
