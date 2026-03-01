#ifndef MAHONY_AHRS_H
#define MAHONY_AHRS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

// 针对 Cortex-M0 精简后的 AHRS 上下文结构体
typedef struct {
    float q[4];             // 四元数 [w, x, y, z]
    float gyro_bias[3];     // 陀螺仪零偏估算值 [bx, by, bz] (rad/s)
    float e_int[3];         // 误差积分 [ex, ey, ez]

    // 算法参数
    float kp;               // 基础比例增益
    float ki;               // 积分增益
    bool  use_dyn_kp;       // 是否启用动态 Kp
    float acc_sigma;        // 动态 Kp: 线性衰减阈值 (原高斯的sigma)
    float g0;               // 重力加速度常量

    // 状态标志
    float trust;            // 当前的加速度置信度 (0.0 ~ 1.0)
} mahony_ahrs_t;


// 获取真实的陀螺仪数据 (单位: rad/s)
typedef void (*mahony_read_gyro_cb)(float *gx, float *gy, float *gz);
// 毫秒级延时函数
typedef void (*mahony_delay_ms_cb)(uint32_t ms);

void mahony_ahrs_init(mahony_ahrs_t *ahrs, float ax, float ay, float az);

void mahony_ahrs_update(mahony_ahrs_t *ahrs, float ax, float ay, float az, float gx, float gy, float gz, float dt);

void mahony_ahrs_get_euler(const mahony_ahrs_t *ahrs, float *roll, float *pitch, float *yaw);

void mahony_ahrs_calibrate_gyro(mahony_ahrs_t *ahrs, mahony_read_gyro_cb read_cb, mahony_delay_ms_cb delay_cb, uint32_t samples, uint32_t delay_time);

#ifdef __cplusplus
}
#endif

#endif // MAHONY_AHRS_H