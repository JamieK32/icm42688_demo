#ifndef ICM42688_REGISTERS_H_
#define ICM42688_REGISTERS_H_

#include <stdint.h>

// Accesible from all user banks
static uint8_t REG_BANK_SEL = 0x76;

// User Bank 0
static uint8_t UB0_REG_DEVICE_CONFIG = 0x11;
// break
static uint8_t UB0_REG_DRIVE_CONFIG = 0x13;
static uint8_t UB0_REG_INT_CONFIG   = 0x14;
// break
static uint8_t UB0_REG_FIFO_CONFIG = 0x16;
// break
static uint8_t UB0_REG_TEMP_DATA1    = 0x1D;
static uint8_t UB0_REG_TEMP_DATA0    = 0x1E;
static uint8_t UB0_REG_ACCEL_DATA_X1 = 0x1F;
static uint8_t UB0_REG_ACCEL_DATA_X0 = 0x20;
static uint8_t UB0_REG_ACCEL_DATA_Y1 = 0x21;
static uint8_t UB0_REG_ACCEL_DATA_Y0 = 0x22;
static uint8_t UB0_REG_ACCEL_DATA_Z1 = 0x23;
static uint8_t UB0_REG_ACCEL_DATA_Z0 = 0x24;
static uint8_t UB0_REG_GYRO_DATA_X1  = 0x25;
static uint8_t UB0_REG_GYRO_DATA_X0  = 0x26;
static uint8_t UB0_REG_GYRO_DATA_Y1  = 0x27;
static uint8_t UB0_REG_GYRO_DATA_Y0  = 0x28;
static uint8_t UB0_REG_GYRO_DATA_Z1  = 0x29;
static uint8_t UB0_REG_GYRO_DATA_Z0  = 0x2A;
static uint8_t UB0_REG_TMST_FSYNCH   = 0x2B;
static uint8_t UB0_REG_TMST_FSYNCL   = 0x2C;
static uint8_t UB0_REG_INT_STATUS    = 0x2D;
static uint8_t UB0_REG_FIFO_COUNTH   = 0x2E;
static uint8_t UB0_REG_FIFO_COUNTL   = 0x2F;
static uint8_t UB0_REG_FIFO_DATA     = 0x30;
static uint8_t UB0_REG_APEX_DATA0    = 0x31;
static uint8_t UB0_REG_APEX_DATA1    = 0x32;
static uint8_t UB0_REG_APEX_DATA2    = 0x33;
static uint8_t UB0_REG_APEX_DATA3    = 0x34;
static uint8_t UB0_REG_APEX_DATA4    = 0x35;
static uint8_t UB0_REG_APEX_DATA5    = 0x36;
static uint8_t UB0_REG_INT_STATUS2   = 0x37;
static uint8_t UB0_REG_INT_STATUS3   = 0x38;
// break
static uint8_t UB0_REG_SIGNAL_PATH_RESET  = 0x4B;
static uint8_t UB0_REG_INTF_CONFIG0       = 0x4C;
static uint8_t UB0_REG_INTF_CONFIG1       = 0x4D;
static uint8_t UB0_REG_PWR_MGMT0          = 0x4E;
static uint8_t UB0_REG_GYRO_CONFIG0       = 0x4F;
static uint8_t UB0_REG_ACCEL_CONFIG0      = 0x50;
static uint8_t UB0_REG_GYRO_CONFIG1       = 0x51;
static uint8_t UB0_REG_GYRO_ACCEL_CONFIG0 = 0x52;
static uint8_t UB0_REG_ACCEFL_CONFIG1     = 0x53;
static uint8_t UB0_REG_TMST_CONFIG        = 0x54;
// break
static uint8_t UB0_REG_APEX_CONFIG0 = 0x56;
static uint8_t UB0_REG_SMD_CONFIG   = 0x57;
// break
static uint8_t UB0_REG_FIFO_CONFIG1 = 0x5F;
static uint8_t UB0_REG_FIFO_CONFIG2 = 0x60;
static uint8_t UB0_REG_FIFO_CONFIG3 = 0x61;
static uint8_t UB0_REG_FSYNC_CONFIG = 0x62;
static uint8_t UB0_REG_INT_CONFIG0  = 0x63;
static uint8_t UB0_REG_INT_CONFIG1  = 0x64;
static uint8_t UB0_REG_INT_SOURCE0  = 0x65;
static uint8_t UB0_REG_INT_SOURCE1  = 0x66;
// break
static uint8_t UB0_REG_INT_SOURCE3 = 0x68;
static uint8_t UB0_REG_INT_SOURCE4 = 0x69;
// break
static uint8_t UB0_REG_FIFO_LOST_PKT0 = 0x6C;
static uint8_t UB0_REG_FIFO_LOST_PKT1 = 0x6D;
// break
static uint8_t UB0_REG_SELF_TEST_CONFIG = 0x70;
// break
static uint8_t UB0_REG_WHO_AM_I = 0x75;

// User Bank 1
static uint8_t UB1_REG_SENSOR_CONFIG0 = 0x03;
// break
static uint8_t UB1_REG_GYRO_CONFIG_STATIC2  = 0x0B;
static uint8_t UB1_REG_GYRO_CONFIG_STATIC3  = 0x0C;
static uint8_t UB1_REG_GYRO_CONFIG_STATIC4  = 0x0D;
static uint8_t UB1_REG_GYRO_CONFIG_STATIC5  = 0x0E;
static uint8_t UB1_REG_GYRO_CONFIG_STATIC6  = 0x0F;
static uint8_t UB1_REG_GYRO_CONFIG_STATIC7  = 0x10;
static uint8_t UB1_REG_GYRO_CONFIG_STATIC8  = 0x11;
static uint8_t UB1_REG_GYRO_CONFIG_STATIC9  = 0x12;
static uint8_t UB1_REG_GYRO_CONFIG_STATIC10 = 0x13;
// break
static uint8_t UB1_REG_XG_ST_DATA = 0x5F;
static uint8_t UB1_REG_YG_ST_DATA = 0x60;
static uint8_t UB1_REG_ZG_ST_DATA = 0x61;
static uint8_t UB1_REG_TMSTVAL0   = 0x62;
static uint8_t UB1_REG_TMSTVAL1   = 0x63;
static uint8_t UB1_REG_TMSTVAL2   = 0x64;
// break
static uint8_t UB1_REG_INTF_CONFIG4 = 0x7A;
static uint8_t UB1_REG_INTF_CONFIG5 = 0x7B;
static uint8_t UB1_REG_INTF_CONFIG6 = 0x7C;

// User Bank 2
static uint8_t UB2_REG_ACCEL_CONFIG_STATIC2 = 0x03;
static uint8_t UB2_REG_ACCEL_CONFIG_STATIC3 = 0x04;
static uint8_t UB2_REG_ACCEL_CONFIG_STATIC4 = 0x05;
// break
static uint8_t UB2_REG_XA_ST_DATA = 0x3B;
static uint8_t UB2_REG_YA_ST_DATA = 0x3C;
static uint8_t UB2_REG_ZA_ST_DATA = 0x3D;

// User Bank 4
static uint8_t UB4_REG_APEX_CONFIG1 = 0x40;
static uint8_t UB4_REG_APEX_CONFIG2 = 0x41;
static uint8_t UB4_REG_APEX_CONFIG3 = 0x42;
static uint8_t UB4_REG_APEX_CONFIG4 = 0x43;
static uint8_t UB4_REG_APEX_CONFIG5 = 0x44;
static uint8_t UB4_REG_APEX_CONFIG6 = 0x45;
static uint8_t UB4_REG_APEX_CONFIG7 = 0x46;
static uint8_t UB4_REG_APEX_CONFIG8 = 0x47;
static uint8_t UB4_REG_APEX_CONFIG9 = 0x48;
// break
static uint8_t UB4_REG_ACCEL_WOM_X_THR = 0x4A;
static uint8_t UB4_REG_ACCEL_WOM_Y_THR = 0x4B;
static uint8_t UB4_REG_ACCEL_WOM_Z_THR = 0x4C;
static uint8_t UB4_REG_INT_SOURCE6     = 0x4D;
static uint8_t UB4_REG_INT_SOURCE7     = 0x4E;
static uint8_t UB4_REG_INT_SOURCE8     = 0x4F;
static uint8_t UB4_REG_INT_SOURCE9     = 0x50;
static uint8_t UB4_REG_INT_SOURCE10    = 0x51;
// break
static uint8_t UB4_REG_OFFSET_USER0 = 0x77;
static uint8_t UB4_REG_OFFSET_USER1 = 0x78;
static uint8_t UB4_REG_OFFSET_USER2 = 0x79;
static uint8_t UB4_REG_OFFSET_USER3 = 0x7A;
static uint8_t UB4_REG_OFFSET_USER4 = 0x7B;
static uint8_t UB4_REG_OFFSET_USER5 = 0x7C;
static uint8_t UB4_REG_OFFSET_USER6 = 0x7D;
static uint8_t UB4_REG_OFFSET_USER7 = 0x7E;
static uint8_t UB4_REG_OFFSET_USER8 = 0x7F;


#endif  // ICM42688_REGISTERS_H_