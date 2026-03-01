#include "icm42688.h"
#include "registers.h"
#include <math.h>
#include <string.h>


#ifndef PI
#define PI 3.14159265358979323846f
#endif

// FIFO 相关宏常量定义补充（如果 registers.h 中没有的话）
#define FIFO_EN 0x5F
#define FIFO_TEMP_EN 0x04
#define FIFO_GYRO 0x02
#define FIFO_ACCEL 0x01
#define GYRO_NF_ENABLE 0x00
#define GYRO_NF_DISABLE 0x01
#define GYRO_AAF_ENABLE 0x00
#define GYRO_AAF_DISABLE 0x02
#define ACCEL_AAF_ENABLE 0x00
#define ACCEL_AAF_DISABLE 0x01

// 内部函数声明
static int icm42688_write_register(icm42688_t *dev, uint8_t reg, uint8_t data);
static int icm42688_read_registers(icm42688_t *dev, uint8_t reg, uint8_t count,
                                   uint8_t *dest);
static int icm42688_set_bank(icm42688_t *dev, uint8_t bank);

// 初始化 HAL 层指针
void icm42688_init_hal(icm42688_t *dev, icm42688_read_cb_t read_cb,
                       icm42688_write_cb_t write_cb,
                       icm42688_delay_us_cb_t delay_us_cb, void *user_ctx) {
  memset(dev, 0, sizeof(icm42688_t));
  dev->read_cb = read_cb;
  dev->write_cb = write_cb;
  dev->delay_us_cb = delay_us_cb;
  dev->user_ctx = user_ctx;

  // 默认比例初始化
  dev->acc_s[0] = 1.0f;
  dev->acc_s[1] = 1.0f;
  dev->acc_s[2] = 1.0f;
}

// 内部低级读写封装
static int icm42688_write_register(icm42688_t *dev, uint8_t reg, uint8_t data) {
  if (!dev->write_cb)
    return -1;

  int ret = dev->write_cb(dev->user_ctx, reg, &data, 1);
  if (ret != 0)
    return -1;

  if (dev->delay_us_cb) {
    dev->delay_us_cb(10000); // 延时 10ms
  }

  // 读回验证
  uint8_t read_back = 0;
  icm42688_read_registers(dev, reg, 1, &read_back);
  return (read_back == data) ? 1 : -1;
}

static int icm42688_read_registers(icm42688_t *dev, uint8_t reg, uint8_t count,
                                   uint8_t *dest) {
  if (!dev->read_cb)
    return -1;

  // 注意：如果使用 SPI，通常要求寄存器地址最高位置 1(即 reg|0x80)
  // 这里的抽象建议将这一步留给用户的 read_cb 实现去判断。
  // 如果想要在库内部处理，可改写这里。
  int ret = dev->read_cb(dev->user_ctx, reg, dest, count);
  return (ret == 0) ? 1 : -1;
}

static int icm42688_set_bank(icm42688_t *dev, uint8_t bank) {
  if (dev->bank == bank) {
    return 1;
  }
  dev->bank = bank;
  return icm42688_write_register(dev, REG_BANK_SEL, bank);
}

void icm42688_reset(icm42688_t *dev) {
  icm42688_set_bank(dev, 0);
  icm42688_write_register(dev, UB0_REG_DEVICE_CONFIG, 0x01);
  if (dev->delay_us_cb) {
    dev->delay_us_cb(1000); // delay 1ms
  }
}

int icm42688_who_am_i(icm42688_t *dev, uint8_t *who_am_i) {
  icm42688_set_bank(dev, 0);
  if (icm42688_read_registers(dev, UB0_REG_WHO_AM_I, 1, dev->buffer) < 0) {
    return -1;
  }
  *who_am_i = dev->buffer[0];
  return 1;
}

int icm42688_begin(icm42688_t *dev) {
  icm42688_reset(dev);

  uint8_t who = 0;
  if (icm42688_who_am_i(dev, &who) < 0 || who != ICM42688_WHO_AM_I_VAL) {
    return -3;
  }

  // 开启加速计和陀螺仪的低噪声模式
  if (icm42688_write_register(dev, UB0_REG_PWR_MGMT0, 0x0F) < 0) {
    return -4;
  }

  if (icm42688_set_accel_fs(dev, ICM42688_ACCEL_FS_16g) < 0) {
    return -5;
  }

  if (icm42688_set_gyro_fs(dev, ICM42688_GYRO_FS_2000dps) < 0) {
    return -6;
  }

  if (icm42688_set_filters(dev, false, false) < 0) {
    return -7;
  }

  if (icm42688_calibrate_gyro(dev) < 0) {
    return -8;
  }

  return 1;
}

int icm42688_set_accel_fs(icm42688_t *dev, icm42688_accel_fs_t fssel) {
  icm42688_set_bank(dev, 0);
  uint8_t reg;
  if (icm42688_read_registers(dev, UB0_REG_ACCEL_CONFIG0, 1, &reg) < 0) {
    return -1;
  }

  reg = (fssel << 5) | (reg & 0x1F);
  if (icm42688_write_register(dev, UB0_REG_ACCEL_CONFIG0, reg) < 0) {
    return -2;
  }

  dev->accel_scale = (float)(1 << (4 - fssel)) / 32768.0f;
  dev->accel_fs = fssel;
  return 1;
}

int icm42688_get_accel_fs(icm42688_t *dev) {
  icm42688_set_bank(dev, 0);
  uint8_t reg;
  if (icm42688_read_registers(dev, UB0_REG_ACCEL_CONFIG0, 1, &reg) < 0) {
    return -1;
  }
  return (reg & 0xE0) >> 5;
}

int icm42688_set_gyro_fs(icm42688_t *dev, icm42688_gyro_fs_t fssel) {
  icm42688_set_bank(dev, 0);
  uint8_t reg;
  if (icm42688_read_registers(dev, UB0_REG_GYRO_CONFIG0, 1, &reg) < 0) {
    return -1;
  }

  reg = (fssel << 5) | (reg & 0x1F);
  if (icm42688_write_register(dev, UB0_REG_GYRO_CONFIG0, reg) < 0) {
    return -2;
  }

  dev->gyro_scale = (2000.0f / (float)(1 << fssel)) / 32768.0f;
  dev->gyro_fs = fssel;
  return 1;
}

int icm42688_get_gyro_fs(icm42688_t *dev) {
  icm42688_set_bank(dev, 0);
  uint8_t reg;
  if (icm42688_read_registers(dev, UB0_REG_GYRO_CONFIG0, 1, &reg) < 0) {
    return -1;
  }
  return (reg & 0xE0) >> 5;
}

int icm42688_set_accel_odr(icm42688_t *dev, icm42688_odr_t odr) {
  icm42688_set_bank(dev, 0);
  uint8_t reg;
  if (icm42688_read_registers(dev, UB0_REG_ACCEL_CONFIG0, 1, &reg) < 0) {
    return -1;
  }
  reg = odr | (reg & 0xF0);
  return (icm42688_write_register(dev, UB0_REG_ACCEL_CONFIG0, reg) < 0) ? -2
                                                                        : 1;
}

int icm42688_set_gyro_odr(icm42688_t *dev, icm42688_odr_t odr) {
  icm42688_set_bank(dev, 0);
  uint8_t reg;
  if (icm42688_read_registers(dev, UB0_REG_GYRO_CONFIG0, 1, &reg) < 0) {
    return -1;
  }
  reg = odr | (reg & 0xF0);
  return (icm42688_write_register(dev, UB0_REG_GYRO_CONFIG0, reg) < 0) ? -2 : 1;
}

int icm42688_set_filters(icm42688_t *dev, bool gyro_filters, bool acc_filters) {
  if (icm42688_set_bank(dev, 1) < 0)
    return -1;

  if (gyro_filters) {
    if (icm42688_write_register(dev, UB1_REG_GYRO_CONFIG_STATIC2,
                                GYRO_NF_ENABLE | GYRO_AAF_ENABLE) < 0)
      return -2;
  } else {
    if (icm42688_write_register(dev, UB1_REG_GYRO_CONFIG_STATIC2,
                                GYRO_NF_DISABLE | GYRO_AAF_DISABLE) < 0)
      return -3;
  }

  if (icm42688_set_bank(dev, 2) < 0)
    return -4;

  if (acc_filters) {
    if (icm42688_write_register(dev, UB2_REG_ACCEL_CONFIG_STATIC2,
                                ACCEL_AAF_ENABLE) < 0)
      return -5;
  } else {
    if (icm42688_write_register(dev, UB2_REG_ACCEL_CONFIG_STATIC2,
                                ACCEL_AAF_DISABLE) < 0)
      return -6;
  }

  if (icm42688_set_bank(dev, 0) < 0)
    return -7;
  return 1;
}

int icm42688_enable_data_ready_interrupt(icm42688_t *dev) {
  if (icm42688_write_register(dev, UB0_REG_INT_CONFIG, 0x18 | 0x03) < 0)
    return -1;

  uint8_t reg;
  if (icm42688_read_registers(dev, UB0_REG_INT_CONFIG1, 1, &reg) < 0)
    return -2;
  reg &= ~0x10;
  if (icm42688_write_register(dev, UB0_REG_INT_CONFIG1, reg) < 0)
    return -3;

  if (icm42688_write_register(dev, UB0_REG_INT_SOURCE0, 0x18) < 0)
    return -4;
  return 1;
}

int icm42688_disable_data_ready_interrupt(icm42688_t *dev) {
  uint8_t reg;
  if (icm42688_read_registers(dev, UB0_REG_INT_CONFIG1, 1, &reg) < 0)
    return -1;
  reg |= 0x10;
  if (icm42688_write_register(dev, UB0_REG_INT_CONFIG1, reg) < 0)
    return -2;
  if (icm42688_write_register(dev, UB0_REG_INT_SOURCE0, 0x10) < 0)
    return -3;
  return 1;
}

int icm42688_get_agt(icm42688_t *dev) {
  if (icm42688_get_raw_agt(dev) < 0) {
    return -1;
  }

  dev->t = ((float)dev->raw_t / ICM42688_TEMP_SCALE) + ICM42688_TEMP_OFFSET;

  dev->acc[0] =
      ((dev->raw_acc[0] * dev->accel_scale) - dev->acc_b[0]) * dev->acc_s[0];
  dev->acc[1] =
      ((dev->raw_acc[1] * dev->accel_scale) - dev->acc_b[1]) * dev->acc_s[1];
  dev->acc[2] =
      ((dev->raw_acc[2] * dev->accel_scale) - dev->acc_b[2]) * dev->acc_s[2];

  dev->gyr[0] = (dev->raw_gyr[0] * dev->gyro_scale) - dev->gyr_b[0];
  dev->gyr[1] = (dev->raw_gyr[1] * dev->gyro_scale) - dev->gyr_b[1];
  dev->gyr[2] = (dev->raw_gyr[2] * dev->gyro_scale) - dev->gyr_b[2];

  return 1;
}

int icm42688_get_raw_agt(icm42688_t *dev) {
  if (icm42688_read_registers(dev, UB0_REG_TEMP_DATA1, 14, dev->buffer) < 0) {
    return -1;
  }

  int16_t raw_meas[7];
  for (size_t i = 0; i < 7; i++) {
    raw_meas[i] = ((int16_t)dev->buffer[i * 2] << 8) | dev->buffer[i * 2 + 1];
  }

  dev->raw_t = raw_meas[0];
  dev->raw_acc[0] = raw_meas[1];
  dev->raw_acc[1] = raw_meas[2];
  dev->raw_acc[2] = raw_meas[3];
  dev->raw_gyr[0] = raw_meas[4];
  dev->raw_gyr[1] = raw_meas[5];
  dev->raw_gyr[2] = raw_meas[6];

  return 1;
}

int icm42688_calibrate_gyro(icm42688_t *dev) {
  icm42688_gyro_fs_t current_fssel = dev->gyro_fs;
  if (icm42688_set_gyro_fs(dev, ICM42688_GYRO_FS_250dps) < 0) {
    return -1;
  }

  float gyro_bd[3] = {0, 0, 0};
  for (size_t i = 0; i < ICM42688_NUM_CALIB_SAMPLES; i++) {
    icm42688_get_agt(dev);
    gyro_bd[0] += (dev->gyr[0] + dev->gyr_b[0]) / ICM42688_NUM_CALIB_SAMPLES;
    gyro_bd[1] += (dev->gyr[1] + dev->gyr_b[1]) / ICM42688_NUM_CALIB_SAMPLES;
    gyro_bd[2] += (dev->gyr[2] + dev->gyr_b[2]) / ICM42688_NUM_CALIB_SAMPLES;
    if (dev->delay_us_cb)
      dev->delay_us_cb(1000);
  }

  dev->gyr_b[0] = gyro_bd[0];
  dev->gyr_b[1] = gyro_bd[1];
  dev->gyr_b[2] = gyro_bd[2];

  if (icm42688_set_gyro_fs(dev, current_fssel) < 0) {
    return -4;
  }
  return 1;
}

int icm42688_calibrate_accel(icm42688_t *dev) {
  icm42688_accel_fs_t current_fssel = dev->accel_fs;
  if (icm42688_set_accel_fs(dev, ICM42688_ACCEL_FS_2g) < 0) {
    return -1;
  }

  float acc_bd[3] = {0, 0, 0};
  float acc_max[3] = {0, 0, 0};
  float acc_min[3] = {0, 0, 0};

  for (size_t i = 0; i < ICM42688_NUM_CALIB_SAMPLES; i++) {
    icm42688_get_agt(dev);
    acc_bd[0] += (dev->acc[0] / dev->acc_s[0] + dev->acc_b[0]) /
                 ICM42688_NUM_CALIB_SAMPLES;
    acc_bd[1] += (dev->acc[1] / dev->acc_s[1] + dev->acc_b[1]) /
                 ICM42688_NUM_CALIB_SAMPLES;
    acc_bd[2] += (dev->acc[2] / dev->acc_s[2] + dev->acc_b[2]) /
                 ICM42688_NUM_CALIB_SAMPLES;
    if (dev->delay_us_cb)
      dev->delay_us_cb(1000);
  }

  if (acc_bd[0] > 0.9f)
    acc_max[0] = acc_bd[0];
  if (acc_bd[1] > 0.9f)
    acc_max[1] = acc_bd[1];
  if (acc_bd[2] > 0.9f)
    acc_max[2] = acc_bd[2];

  if (acc_bd[0] < -0.9f)
    acc_min[0] = acc_bd[0];
  if (acc_bd[1] < -0.9f)
    acc_min[1] = acc_bd[1];
  if (acc_bd[2] < -0.9f)
    acc_min[2] = acc_bd[2];

  if ((fabsf(acc_min[0]) > 0.9f) && (fabsf(acc_max[0]) > 0.9f)) {
    dev->acc_b[0] = (acc_min[0] + acc_max[0]) / 2.0f;
    dev->acc_s[0] = 1 / ((fabsf(acc_min[0]) + fabsf(acc_max[0])) / 2.0f);
  }
  if ((fabsf(acc_min[1]) > 0.9f) && (fabsf(acc_max[1]) > 0.9f)) {
    dev->acc_b[1] = (acc_min[1] + acc_max[1]) / 2.0f;
    dev->acc_s[1] = 1 / ((fabsf(acc_min[1]) + fabsf(acc_max[1])) / 2.0f);
  }
  if ((fabsf(acc_min[2]) > 0.9f) && (fabsf(acc_max[2]) > 0.9f)) {
    dev->acc_b[2] = (acc_min[2] + acc_max[2]) / 2.0f;
    dev->acc_s[2] = 1 / ((fabsf(acc_min[2]) + fabsf(acc_max[2])) / 2.0f);
  }

  if (icm42688_set_accel_fs(dev, current_fssel) < 0) {
    return -4;
  }
  return 1;
}

int icm42688_compute_offsets(icm42688_t *dev) {
  icm42688_accel_fs_t current_acc_fs = dev->accel_fs;
  icm42688_gyro_fs_t current_gyr_fs = dev->gyro_fs;

  icm42688_set_accel_fs(dev, ICM42688_ACCEL_FS_2g);
  icm42688_set_gyro_fs(dev, ICM42688_GYRO_FS_250dps);
  int16_t fs_acc = 2;
  int16_t fs_gyr = 250;

  icm42688_set_bank(dev, 4);
  icm42688_write_register(dev, UB4_REG_OFFSET_USER5, 0);
  icm42688_write_register(dev, UB4_REG_OFFSET_USER6, 0);
  icm42688_write_register(dev, UB4_REG_OFFSET_USER8, 0);
  icm42688_write_register(dev, UB4_REG_OFFSET_USER2, 0);
  icm42688_write_register(dev, UB4_REG_OFFSET_USER3, 0);
  icm42688_write_register(dev, UB4_REG_OFFSET_USER0, 0);
  icm42688_write_register(dev, UB4_REG_OFFSET_USER4, 0);
  icm42688_write_register(dev, UB4_REG_OFFSET_USER7, 0);
  icm42688_write_register(dev, UB4_REG_OFFSET_USER1, 0);
  icm42688_set_bank(dev, 0);

  for (size_t ii = 0; ii < 3; ii++) {
    dev->raw_acc_bias[ii] = 0;
    dev->raw_gyr_bias[ii] = 0;
  }

  for (size_t i = 0; i < ICM42688_NUM_CALIB_SAMPLES; i++) {
    icm42688_get_raw_agt(dev);
    dev->raw_acc_bias[0] += dev->raw_acc[0];
    dev->raw_acc_bias[1] += dev->raw_acc[1];
    dev->raw_acc_bias[2] += dev->raw_acc[2];
    dev->raw_gyr_bias[0] += dev->raw_gyr[0];
    dev->raw_gyr_bias[1] += dev->raw_gyr[1];
    dev->raw_gyr_bias[2] += dev->raw_gyr[2];
    if (dev->delay_us_cb)
      dev->delay_us_cb(1000);
  }

  dev->raw_acc_bias[0] = (int32_t)((double)dev->raw_acc_bias[0] /
                                   (double)ICM42688_NUM_CALIB_SAMPLES);
  dev->raw_acc_bias[1] = (int32_t)((double)dev->raw_acc_bias[1] /
                                   (double)ICM42688_NUM_CALIB_SAMPLES);
  dev->raw_acc_bias[2] = (int32_t)((double)dev->raw_acc_bias[2] /
                                   (double)ICM42688_NUM_CALIB_SAMPLES);
  dev->raw_gyr_bias[0] = (int32_t)((double)dev->raw_gyr_bias[0] /
                                   (double)ICM42688_NUM_CALIB_SAMPLES);
  dev->raw_gyr_bias[1] = (int32_t)((double)dev->raw_gyr_bias[1] /
                                   (double)ICM42688_NUM_CALIB_SAMPLES);
  dev->raw_gyr_bias[2] = (int32_t)((double)dev->raw_gyr_bias[2] /
                                   (double)ICM42688_NUM_CALIB_SAMPLES);

  for (size_t ii = 0; ii < 3; ii++) {
    dev->acc_offset[ii] =
        (int16_t)(-(dev->raw_acc_bias[ii]) * (fs_acc / 32768.0f * 2048));
    if (dev->raw_acc_bias[ii] * fs_acc > 26000) {
      dev->acc_offset[ii] =
          (int16_t)(-(dev->raw_acc_bias[ii] - 32768 / fs_acc) *
                    (fs_acc / 32768.0f * 2048));
    }
    if (dev->raw_acc_bias[ii] * fs_acc < -26000) {
      dev->acc_offset[ii] =
          (int16_t)(-(dev->raw_acc_bias[ii] + 32768 / fs_acc) *
                    (fs_acc / 32768.0f * 2048));
    }
    dev->gyr_offset[ii] =
        (int16_t)((-dev->raw_gyr_bias[ii]) * (fs_gyr / 32768.0f * 32));
  }

  if (icm42688_set_accel_fs(dev, current_acc_fs) < 0)
    return -4;
  if (icm42688_set_gyro_fs(dev, current_gyr_fs) < 0)
    return -4;
  return 1;
}

int icm42688_set_all_offsets(icm42688_t *dev) {
  icm42688_set_bank(dev, 4);
  uint8_t reg;

  icm42688_write_register(dev, UB4_REG_OFFSET_USER0, 0);
  icm42688_write_register(dev, UB4_REG_OFFSET_USER1, 0);
  icm42688_write_register(dev, UB4_REG_OFFSET_USER2, 0);
  icm42688_write_register(dev, UB4_REG_OFFSET_USER3, 0);
  icm42688_write_register(dev, UB4_REG_OFFSET_USER4, 0);
  icm42688_write_register(dev, UB4_REG_OFFSET_USER5, 0);
  icm42688_write_register(dev, UB4_REG_OFFSET_USER6, 0);
  icm42688_write_register(dev, UB4_REG_OFFSET_USER7, 0);

  reg = dev->acc_offset[0] & 0x00FF;
  icm42688_write_register(dev, UB4_REG_OFFSET_USER5, reg);
  reg = dev->acc_offset[1] & 0x00FF;
  icm42688_write_register(dev, UB4_REG_OFFSET_USER6, reg);
  reg = dev->acc_offset[2] & 0x00FF;
  icm42688_write_register(dev, UB4_REG_OFFSET_USER8, reg);

  reg = dev->gyr_offset[1] & 0x00FF;
  icm42688_write_register(dev, UB4_REG_OFFSET_USER2, reg);
  reg = dev->gyr_offset[2] & 0x00FF;
  icm42688_write_register(dev, UB4_REG_OFFSET_USER3, reg);
  reg = dev->gyr_offset[0] & 0x00FF;
  icm42688_write_register(dev, UB4_REG_OFFSET_USER0, reg);

  reg = (dev->acc_offset[0] & 0x0F00) >> 4 | (dev->gyr_offset[2] & 0x0F00) >> 8;
  icm42688_write_register(dev, UB4_REG_OFFSET_USER4, reg);
  reg = (dev->acc_offset[2] & 0x0F00) >> 4 | (dev->acc_offset[1] & 0x0F00) >> 8;
  icm42688_write_register(dev, UB4_REG_OFFSET_USER7, reg);
  reg = (dev->gyr_offset[1] & 0x0F00) >> 4 | (dev->gyr_offset[0] & 0x0F00) >> 8;
  icm42688_write_register(dev, UB4_REG_OFFSET_USER1, reg);

  icm42688_set_bank(dev, 0);
  return 1;
}

int icm42688_set_acc_x_offset(icm42688_t *dev, int16_t offset) {
  icm42688_set_bank(dev, 4);
  uint8_t reg1 = (offset & 0x00FF);
  uint8_t reg2;
  if (icm42688_read_registers(dev, UB4_REG_OFFSET_USER4, 1, &reg2) < 0)
    return -1;
  reg2 = (reg2 & 0x0F) | ((offset & 0x0F00) >> 4);
  if (icm42688_write_register(dev, UB4_REG_OFFSET_USER5, reg1) < 0)
    return -2;
  if (icm42688_write_register(dev, UB4_REG_OFFSET_USER4, reg2) < 0)
    return -2;
  icm42688_set_bank(dev, 0);
  return 1;
}

int icm42688_set_acc_y_offset(icm42688_t *dev, int16_t offset) {
  icm42688_set_bank(dev, 4);

  uint8_t reg1 = (offset & 0x00FF);
  uint8_t reg2;

  if (icm42688_read_registers(dev, UB4_REG_OFFSET_USER7, 1, &reg2) < 0)
    return -1;

  reg2 = (reg2 & 0xF0) | ((offset & 0x0F00) >> 8);

  if (icm42688_write_register(dev, UB4_REG_OFFSET_USER6, reg1) < 0)
    return -2;

  if (icm42688_write_register(dev, UB4_REG_OFFSET_USER7, reg2) < 0)
    return -2;

  icm42688_set_bank(dev, 0);
  return 1;
}

int icm42688_set_acc_z_offset(icm42688_t *dev, int16_t offset) {
  icm42688_set_bank(dev, 4);

  uint8_t reg1 = (offset & 0x00FF);
  uint8_t reg2;

  if (icm42688_read_registers(dev, UB4_REG_OFFSET_USER7, 1, &reg2) < 0)
    return -1;

  reg2 = (reg2 & 0x0F) | ((offset & 0x0F00) >> 4);

  if (icm42688_write_register(dev, UB4_REG_OFFSET_USER8, reg1) < 0)
    return -2;

  if (icm42688_write_register(dev, UB4_REG_OFFSET_USER7, reg2) < 0)
    return -2;

  icm42688_set_bank(dev, 0);
  return 1;
}

int icm42688_set_gyr_x_offset(icm42688_t *dev, int16_t offset) {
  icm42688_set_bank(dev, 4);

  uint8_t reg1 = (offset & 0x00FF);
  uint8_t reg2;

  if (icm42688_read_registers(dev, UB4_REG_OFFSET_USER1, 1, &reg2) < 0)
    return -1;

  reg2 = (reg2 & 0xF0) | ((offset & 0x0F00) >> 8);

  if (icm42688_write_register(dev, UB4_REG_OFFSET_USER0, reg1) < 0)
    return -2;

  if (icm42688_write_register(dev, UB4_REG_OFFSET_USER1, reg2) < 0)
    return -2;

  icm42688_set_bank(dev, 0);
  return 1;
}

int icm42688_set_gyr_y_offset(icm42688_t *dev, int16_t offset) {
  icm42688_set_bank(dev, 4);

  uint8_t reg1 = (offset & 0x00FF);
  uint8_t reg2;

  if (icm42688_read_registers(dev, UB4_REG_OFFSET_USER1, 1, &reg2) < 0)
    return -1;

  reg2 = (reg2 & 0x0F) | ((offset & 0x0F00) >> 4);

  if (icm42688_write_register(dev, UB4_REG_OFFSET_USER2, reg1) < 0)
    return -2;

  if (icm42688_write_register(dev, UB4_REG_OFFSET_USER1, reg2) < 0)
    return -2;

  icm42688_set_bank(dev, 0);
  return 1;
}

int icm42688_set_gyr_z_offset(icm42688_t *dev, int16_t offset) {
  icm42688_set_bank(dev, 4);

  uint8_t reg1 = (offset & 0x00FF);
  uint8_t reg2;

  if (icm42688_read_registers(dev, UB4_REG_OFFSET_USER4, 1, &reg2) < 0)
    return -1;

  reg2 = (reg2 & 0xF0) | ((offset & 0x0F00) >> 8);

  if (icm42688_write_register(dev, UB4_REG_OFFSET_USER3, reg1) < 0)
    return -2;

  if (icm42688_write_register(dev, UB4_REG_OFFSET_USER4, reg2) < 0)
    return -2;

  icm42688_set_bank(dev, 0);
  return 1;
}

int icm42688_set_gyro_notch_filter(icm42688_t *dev, float freq_x, float freq_y,
                                   float freq_z, icm42688_gyro_nfbw_t bw) {
  icm42688_set_bank(dev, 3);
  uint8_t reg;
  if (icm42688_read_registers(dev, UB0_REG_GYRO_CONFIG0, 1, &reg) < 0)
    return -1;

  uint8_t clkdiv = reg & 0x3F;
  icm42688_set_bank(dev, 1);

  uint16_t nf_coswz;
  uint8_t gyro_nf_coswz_low[3] = {0};
  uint8_t buff = 0;
  float fdrv = 19200.0f / (clkdiv * 10.0f);
  const float fdesired[3] = {freq_x, freq_y, freq_z};

  for (size_t ii = 0; ii < 3; ii++) {
    float coswz = cos(2 * PI * fdesired[ii] / fdrv);
    if (coswz <= 0.875f) {
      nf_coswz = (uint16_t)round(coswz * 256.0f);
      gyro_nf_coswz_low[ii] = (uint8_t)(nf_coswz & 0x00FF);
      buff = buff | (((nf_coswz & 0xFF00) >> 8) << ii);
    } else {
      buff = buff | (1 << (3 + ii));
      if (coswz > 0.875f) {
        nf_coswz = (uint16_t)round(8.0f * (1.0f - coswz) * 256.0f);
        gyro_nf_coswz_low[ii] = (uint8_t)(nf_coswz & 0x00FF);
        buff = buff | (((nf_coswz & 0xFF00) >> 8) << ii);
      } else if (coswz < -0.875f) {
        nf_coswz = (uint16_t)round(-8.0f * (1.0f - coswz) * 256.0f);
        gyro_nf_coswz_low[ii] = (uint8_t)(nf_coswz & 0x00FF);
        buff = buff | (((nf_coswz & 0xFF00) >> 8) << ii);
      }
    }
  }

  icm42688_write_register(dev, UB1_REG_GYRO_CONFIG_STATIC6,
                          gyro_nf_coswz_low[0]);
  icm42688_write_register(dev, UB1_REG_GYRO_CONFIG_STATIC7,
                          gyro_nf_coswz_low[1]);
  icm42688_write_register(dev, UB1_REG_GYRO_CONFIG_STATIC8,
                          gyro_nf_coswz_low[2]);
  icm42688_write_register(dev, UB1_REG_GYRO_CONFIG_STATIC9, buff);
  icm42688_write_register(dev, UB1_REG_GYRO_CONFIG_STATIC10, bw);

  icm42688_set_bank(dev, 0);
  return 1;
}

float icm42688_get_accel_res(icm42688_t *dev) {
  int current_acc_fs = icm42688_get_accel_fs(dev);
  switch (current_acc_fs) {
  case ICM42688_ACCEL_FS_2g:
    return 16.0f / 32768.0f; // Note: Logic from original code
  case ICM42688_ACCEL_FS_4g:
    return 4.0f / 32768.0f;
  case ICM42688_ACCEL_FS_8g:
    return 8.0f / 32768.0f;
  case ICM42688_ACCEL_FS_16g:
    return 16.0f / 32768.0f;
  default:
    return 0.0f;
  }
}

float icm42688_get_gyro_res(icm42688_t *dev) {
  int current_gyro_fs = icm42688_get_gyro_fs(dev);
  switch (current_gyro_fs) {
  case ICM42688_GYRO_FS_2000dps:
    return 2000.0f / 32768.0f;
  case ICM42688_GYRO_FS_1000dps:
    return 1000.0f / 32768.0f;
  case ICM42688_GYRO_FS_500dps:
    return 500.0f / 32768.0f;
  case ICM42688_GYRO_FS_250dps:
    return 250.0f / 32768.0f;
  case ICM42688_GYRO_FS_125dps:
    return 125.0f / 32768.0f;
  case ICM42688_GYRO_FS_62_5dps:
    return 62.5f / 32768.0f;
  case ICM42688_GYRO_FS_31_25dps:
    return 31.25f / 32768.0f;
  case ICM42688_GYRO_FS_15_625dps:
    return 15.625f / 32768.0f;
  default:
    return 0.0f;
  }
}

// ====================== FIFO Functions ======================
int icm42688_fifo_enable(icm42688_t *dev, bool accel, bool gyro, bool temp) {
  dev->en_fifo_accel = accel;
  dev->en_fifo_gyro = gyro;
  dev->en_fifo_temp = true; // 原逻辑保持
  dev->en_fifo_timestamp = accel && gyro;
  dev->en_fifo_header = accel || gyro;
  dev->fifo_frame_size = dev->en_fifo_header * 1 + dev->en_fifo_accel * 6 +
                         dev->en_fifo_gyro * 6 + dev->en_fifo_temp +
                         dev->en_fifo_timestamp * 2;

  if (icm42688_write_register(dev, FIFO_EN,
                              (dev->en_fifo_accel * FIFO_ACCEL) |
                                  (dev->en_fifo_gyro * FIFO_GYRO) |
                                  (dev->en_fifo_temp * FIFO_TEMP_EN)) < 0) {
    return -2;
  }
  return 1;
}

int icm42688_fifo_stream_to_fifo(icm42688_t *dev) {
  if (icm42688_write_register(dev, UB0_REG_FIFO_CONFIG, 1 << 6) < 0) {
    return -2;
  }
  return 1;
}

int icm42688_fifo_read(icm42688_t *dev) {
  icm42688_read_registers(dev, UB0_REG_FIFO_COUNTH, 2, dev->buffer);
  dev->fifo_size =
      (((uint16_t)(dev->buffer[0] & 0x0F)) << 8) + ((uint16_t)dev->buffer[1]);

  size_t num_frames = dev->fifo_size / dev->fifo_frame_size;
  size_t acc_index = 1;
  size_t gyro_index = acc_index + dev->en_fifo_accel * 6;
  size_t temp_index = gyro_index + dev->en_fifo_gyro * 6;

  for (size_t i = 0; i < num_frames; i++) {
    if (icm42688_read_registers(dev, UB0_REG_FIFO_DATA, dev->fifo_frame_size,
                                dev->buffer) < 0) {
      return -1;
    }
    if (dev->en_fifo_accel) {
      int16_t raw_meas[3];
      raw_meas[0] = (((int16_t)dev->buffer[0 + acc_index]) << 8) |
                    dev->buffer[1 + acc_index];
      raw_meas[1] = (((int16_t)dev->buffer[2 + acc_index]) << 8) |
                    dev->buffer[3 + acc_index];
      raw_meas[2] = (((int16_t)dev->buffer[4 + acc_index]) << 8) |
                    dev->buffer[5 + acc_index];
      dev->ax_fifo[i] =
          ((raw_meas[0] * dev->accel_scale) - dev->acc_b[0]) * dev->acc_s[0];
      dev->ay_fifo[i] =
          ((raw_meas[1] * dev->accel_scale) - dev->acc_b[1]) * dev->acc_s[1];
      dev->az_fifo[i] =
          ((raw_meas[2] * dev->accel_scale) - dev->acc_b[2]) * dev->acc_s[2];
      dev->a_size = num_frames;
    }
    if (dev->en_fifo_temp) {
      int8_t raw_meas = dev->buffer[temp_index + 0];
      dev->t_fifo[i] =
          ((float)raw_meas / ICM42688_TEMP_SCALE) + ICM42688_TEMP_OFFSET;
      dev->t_size = num_frames;
    }
    if (dev->en_fifo_gyro) {
      int16_t raw_meas[3];
      raw_meas[0] = (((int16_t)dev->buffer[0 + gyro_index]) << 8) |
                    dev->buffer[1 + gyro_index];
      raw_meas[1] = (((int16_t)dev->buffer[2 + gyro_index]) << 8) |
                    dev->buffer[3 + gyro_index];
      raw_meas[2] = (((int16_t)dev->buffer[4 + gyro_index]) << 8) |
                    dev->buffer[5 + gyro_index];
      dev->gx_fifo[i] = (raw_meas[0] * dev->gyro_scale) - dev->gyr_b[0];
      dev->gy_fifo[i] = (raw_meas[1] * dev->gyro_scale) - dev->gyr_b[1];
      dev->gz_fifo[i] = (raw_meas[2] * dev->gyro_scale) - dev->gyr_b[2];
      dev->g_size = num_frames;
    }
  }
  return 1;
}

void icm42688_fifo_get_accel_x(icm42688_t *dev, size_t *size, float *data) {
  *size = dev->a_size;
  memcpy(data, dev->ax_fifo, dev->a_size * sizeof(float));
}

void icm42688_fifo_get_accel_y(icm42688_t *dev, size_t *size, float *data) {
  *size = dev->a_size;
  memcpy(data, dev->ay_fifo, dev->a_size * sizeof(float));
}

void icm42688_fifo_get_accel_z(icm42688_t *dev, size_t *size, float *data) {
  *size = dev->a_size;
  memcpy(data, dev->az_fifo, dev->a_size * sizeof(float));
}

void icm42688_fifo_get_gyro_x(icm42688_t *dev, size_t *size, float *data) {
  *size = dev->g_size;
  memcpy(data, dev->gx_fifo, dev->g_size * sizeof(float));
}

void icm42688_fifo_get_gyro_y(icm42688_t *dev, size_t *size, float *data) {
  *size = dev->g_size;
  memcpy(data, dev->gy_fifo, dev->g_size * sizeof(float));
}

void icm42688_fifo_get_gyro_z(icm42688_t *dev, size_t *size, float *data) {
  *size = dev->g_size;
  memcpy(data, dev->gz_fifo, dev->g_size * sizeof(float));
}

void icm42688_fifo_get_temp(icm42688_t *dev, size_t *size, float *data) {
  *size = dev->t_size;
  memcpy(data, dev->t_fifo, dev->t_size * sizeof(float));
}