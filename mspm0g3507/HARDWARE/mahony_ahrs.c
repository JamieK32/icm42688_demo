#include "mahony_ahrs.h"
#include <math.h>
#include "stdlib.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// Cortex-M0 性能核心：快速平方根倒数 (Fast Inverse Square Root)
// 使用 union 避免 strict-aliasing 编译警告和潜在的优化 bug
static float invSqrt(float x) {
    float halfx = 0.5f * x;
    union {
        float f;
        uint32_t i;
    } conv = {x};
    
    conv.i = 0x5f3759df - (conv.i >> 1);
    conv.f *= (1.5f - (halfx * conv.f * conv.f));
    // Cortex-M0 上一次牛顿迭代的精度已足够用于 AHRS
    return conv.f; 
}

void mahony_ahrs_init(mahony_ahrs_t *ahrs, float ax, float ay, float az) {
    // 设置默认参数
    ahrs->kp = 1.0f;
    ahrs->ki = 0.01f;
    ahrs->use_dyn_kp = true;
    ahrs->acc_sigma = 0.15f; 
    ahrs->g0 = 9.80665f;
    ahrs->trust = 0.0f;

    ahrs->e_int[0] = 0.0f; ahrs->e_int[1] = 0.0f; ahrs->e_int[2] = 0.0f;
    ahrs->gyro_bias[0] = 0.0f; ahrs->gyro_bias[1] = 0.0f; ahrs->gyro_bias[2] = 0.0f;

    // 初始姿态对齐
    float roll = atan2f(ay, az);
    float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

    float cy = cosf(0.0f), sy = sinf(0.0f);
    float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
    float cr = cosf(roll * 0.5f), sr = sinf(roll * 0.5f);

    ahrs->q[0] = cr * cp * cy + sr * sp * sy;
    ahrs->q[1] = sr * cp * cy - cr * sp * sy;
    ahrs->q[2] = cr * sp * cy + sr * cp * sy;
    ahrs->q[3] = cr * cp * sy - sr * sp * cy;

    // 初始归一化也使用快速平方根倒数和乘法
    float recipNorm = invSqrt(ahrs->q[0]*ahrs->q[0] + ahrs->q[1]*ahrs->q[1] + ahrs->q[2]*ahrs->q[2] + ahrs->q[3]*ahrs->q[3]);
    ahrs->q[0] *= recipNorm; 
    ahrs->q[1] *= recipNorm; 
    ahrs->q[2] *= recipNorm; 
    ahrs->q[3] *= recipNorm;
}

void mahony_ahrs_calibrate_gyro(mahony_ahrs_t *ahrs, mahony_read_gyro_cb read_cb, mahony_delay_ms_cb delay_cb, uint32_t samples, uint32_t delay_time) {
    if (samples == 0 || read_cb == NULL) return;

    float sum_gx = 0.0f;
    float sum_gy = 0.0f;
    float sum_gz = 0.0f;
    float gx = 0.0f, gy = 0.0f, gz = 0.0f;

    // 清零旧的 bias
    ahrs->gyro_bias[0] = 0.0f;
    ahrs->gyro_bias[1] = 0.0f;
    ahrs->gyro_bias[2] = 0.0f;

    for (uint32_t i = 0; i < samples; i++) {
        // 1. 调用用户提供的回调函数读取最新陀螺仪数据
        read_cb(&gx, &gy, &gz);
        
        // 2. 累加数据
        sum_gx += gx;
        sum_gy += gy;
        sum_gz += gz;
        
        // 3. 调用用户提供的延时函数等待下一次采样
        if (delay_cb != NULL && delay_time > 0) {
            delay_cb(delay_time);
        }
    }

    // 计算平均值，作为零偏保存到 ahrs 结构体中
    ahrs->gyro_bias[0] = sum_gx / (float)samples;
    ahrs->gyro_bias[1] = sum_gy / (float)samples;
    ahrs->gyro_bias[2] = sum_gz / (float)samples;
}

void mahony_ahrs_update(mahony_ahrs_t *ahrs, float ax, float ay, float az, float gx, float gy, float gz, float dt) {
    // 1. 去除陀螺仪零偏 (此时静态补偿需由外部提前标定并写入)
    float gu_x = gx - ahrs->gyro_bias[0];
    float gu_y = gy - ahrs->gyro_bias[1];
    float gu_z = gz - ahrs->gyro_bias[2];

    float current_kp = ahrs->kp;

    // 2. 计算加速度模长的平方
    float acc_sq = ax * ax + ay * ay + az * az;

    // 仅在加速度计有效时引入加速度补偿
    if (acc_sq > 1e-6f) {
        float recip_acc_norm = invSqrt(acc_sq);

        // 3. Cortex-M0 优化的动态 Kp 计算
        if (ahrs->use_dyn_kp) {
            // 用乘法求得真实的模长，避免耗时的 sqrtf
            float acc_norm = acc_sq * recip_acc_norm; 
            float acc_dev_ratio = fabsf(acc_norm - ahrs->g0) / ahrs->g0;
            
            // 使用线性分段函数代替极其耗时的 expf() 函数
            float trust = 1.0f - (acc_dev_ratio / ahrs->acc_sigma);
            if (trust < 0.0f) trust = 0.0f;
            ahrs->trust = trust;

            current_kp = ahrs->kp * (0.1f + 0.9f * trust);
        }

        // 归一化加速度计 (乘法代替除法)
        ax *= recip_acc_norm; 
        ay *= recip_acc_norm; 
        az *= recip_acc_norm;

        float q0 = ahrs->q[0], q1 = ahrs->q[1], q2 = ahrs->q[2], q3 = ahrs->q[3];

        // 提取当前四元数所代表的估算重力方向
        float vx = 2.0f * (q1 * q3 - q0 * q2);
        float vy = 2.0f * (q0 * q1 + q2 * q3);
        float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

        // 测量方向与估算方向的叉积 (求误差)
        float ex = (ay * vz - az * vy);
        float ey = (az * vx - ax * vz);
        float ez = (ax * vy - ay * vx);

        // 积分误差积累
        if (ahrs->ki > 0.0f) {
            ahrs->e_int[0] += ex * dt;
            ahrs->e_int[1] += ey * dt;
            ahrs->e_int[2] += ez * dt;
        } else {
            ahrs->e_int[0] = 0.0f; ahrs->e_int[1] = 0.0f; ahrs->e_int[2] = 0.0f;
        }

        // 将 PI 补偿项叠加到陀螺仪读数上
        gu_x += current_kp * ex + ahrs->ki * ahrs->e_int[0];
        gu_y += current_kp * ey + ahrs->ki * ahrs->e_int[1];
        gu_z += current_kp * ez + ahrs->ki * ahrs->e_int[2];
    }

    // 4. 使用最终的角速度更新四元数 (一阶龙格-库塔积分)
    float q0 = ahrs->q[0], q1 = ahrs->q[1], q2 = ahrs->q[2], q3 = ahrs->q[3];
    float half_dt = 0.5f * dt;
    
    ahrs->q[0] = q0 + (-q1 * gu_x - q2 * gu_y - q3 * gu_z) * half_dt;
    ahrs->q[1] = q1 + ( q0 * gu_x + q2 * gu_z - q3 * gu_y) * half_dt;
    ahrs->q[2] = q2 + ( q0 * gu_y - q1 * gu_z + q3 * gu_x) * half_dt;
    ahrs->q[3] = q3 + ( q0 * gu_z + q1 * gu_y - q2 * gu_x) * half_dt;

    // 5. 重新归一化四元数 (使用快速平方根倒数)
    float recip_q_norm = invSqrt(ahrs->q[0]*ahrs->q[0] + ahrs->q[1]*ahrs->q[1] + ahrs->q[2]*ahrs->q[2] + ahrs->q[3]*ahrs->q[3]);
    ahrs->q[0] *= recip_q_norm;
    ahrs->q[1] *= recip_q_norm;
    ahrs->q[2] *= recip_q_norm;
    ahrs->q[3] *= recip_q_norm;
}

void mahony_ahrs_get_euler(const mahony_ahrs_t *ahrs, float *roll, float *pitch, float *yaw) {
    float q0 = ahrs->q[0], q1 = ahrs->q[1], q2 = ahrs->q[2], q3 = ahrs->q[3];
    
    // Roll (X轴)
    *roll  = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 180.0f / M_PI;
    
    // Pitch (Y轴) - 限制在 -90 到 90 度之间以防 asin 域错误
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (sinp >= 1.0f) *pitch = 90.0f;
    else if (sinp <= -1.0f) *pitch = -90.0f;
    else *pitch = asinf(sinp) * 180.0f / M_PI;
    
    // Yaw (Z轴)
    *yaw   = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 180.0f / M_PI;
}