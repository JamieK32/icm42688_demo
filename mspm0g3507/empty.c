#include "icm42688.h"
#include "uart_driver.h"
#include "delay.h"
#include "icm_spi_driver.h"
#include <stdbool.h>
#include "ti_msp_dl_config.h"
#include "mahony_ahrs.h"

// 宏定义单位转换常数
#define G_TO_MPS2    9.80665f
#define DPS_TO_RADPS 0.0174532925f

#define FIXED_DT 0.01f

icm42688_t imu;
mahony_ahrs_t ahrs;

// 缓冲区
float ax[128], ay[128], az[128];
float gx[128], gy[128], gz[128];

volatile bool imu_int_flag = false;

void my_get_gyro_data(float *gx1, float *gy1, float *gz1) {
    icm42688_get_agt(&imu); 
    *gx1 = imu.gyr[0] * DPS_TO_RADPS;
    *gy1 = imu.gyr[1] * DPS_TO_RADPS;
    *gz1 = imu.gyr[2] * DPS_TO_RADPS;
}

int main(void)
{
    SYSCFG_DL_init();
    NVIC_EnableIRQ(ICM42688_INT_IRQN); 

    delay_ms(200);
    
    icm42688_init_hal(&imu, my_spi_read, my_spi_write, delay_us, SPI_0_INST);
        
    if (icm42688_begin(&imu) < 0) {
        usart_printf(UART0, "ICM42688 Initialization Failed!\r\n");
        while(1); 
    }
    usart_printf(UART0, "ICM42688 Initialized Successfully!\r\n");

    // 1. 设置 ODR 100Hz
    icm42688_set_accel_odr(&imu, ICM42688_ODR_100);
    icm42688_set_gyro_odr(&imu,  ICM42688_ODR_100);
    
    // 2. 初始对齐
    icm42688_get_agt(&imu); 
    mahony_ahrs_init(&ahrs, imu.acc[0] * G_TO_MPS2, 
                            imu.acc[1] * G_TO_MPS2, 
                            imu.acc[2] * G_TO_MPS2);
    
    // 3. 静态校准 (增加到 1000 次)
    usart_printf(UART0, "Keep still, calibrating...\r\n");
    mahony_ahrs_calibrate_gyro(&ahrs, my_get_gyro_data, delay_ms, 1000, 5);
    usart_printf(UART0, "Calibration done!\r\n");

    // 4. 开启 FIFO (不再依赖 Timestamp)
    icm42688_fifo_enable(&imu, true, true, false);
    icm42688_fifo_stream_to_fifo(&imu);
    icm42688_enable_data_ready_interrupt(&imu);
    
    imu_int_flag = false;

    while (1) {
        if (imu_int_flag == true) {
            imu_int_flag = false; 

            // A. 读取 FIFO
            if (icm42688_fifo_read(&imu) > 0) {
                
                size_t frame_count;
                
                // B. 提取传感器数据 (必须保留，否则 ax[i] 为 0 会触发 HardFault)
                icm42688_fifo_get_accel_x(&imu, &frame_count, ax);
                icm42688_fifo_get_accel_y(&imu, &frame_count, ay);
                icm42688_fifo_get_accel_z(&imu, &frame_count, az);
                icm42688_fifo_get_gyro_x(&imu, &frame_count, gx);
                icm42688_fifo_get_gyro_y(&imu, &frame_count, gy);
                icm42688_fifo_get_gyro_z(&imu, &frame_count, gz);

                // C. 使用固定 dt 循环积分
                for (size_t i = 0; i < frame_count; i++) {
                    
                    // D. 安全检查：模长太小不计算，防止 invSqrt 崩溃
                    float acc_mag_sq = ax[i]*ax[i] + ay[i]*ay[i] + az[i]*az[i];
                    if (acc_mag_sq > 0.001f) {
                        mahony_ahrs_update(&ahrs, 
                                           ax[i] * G_TO_MPS2, ay[i] * G_TO_MPS2, az[i] * G_TO_MPS2, 
                                           gx[i] * DPS_TO_RADPS, gy[i] * DPS_TO_RADPS, gz[i] * DPS_TO_RADPS, 
                                           FIXED_DT);
                    }
                }

                // E. 计算并打印欧拉角
                float roll, pitch, yaw;
                mahony_ahrs_get_euler(&ahrs, &roll, &pitch, &yaw);
                usart_printf(UART0, "%.2f,%.2f,%.2f\r\n", roll, pitch, yaw);
            }
        }
    }
}


void GROUP1_IRQHandler(void)
{
    switch( DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1) )
    {
        case ICM42688_INT_IIDX:
            imu_int_flag = true;
            break;
        default:
            break;
    }
}



void HardFault_Handler(void)

{
    usart_printf(UART0, "HardFault!\r\n");
    while(1);

}