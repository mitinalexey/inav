/*
 * This file is part of INAV.
 *
 * INAV is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * INAV is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with INAV.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#include "common/axis.h"
#include "common/maths.h"

#include "drivers/system.h"
#include "drivers/time.h"
#include "drivers/bus.h"

#include "drivers/sensor.h"
#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/accgyro/accgyro_qmi8658.h"

#if defined(USE_IMU_QMI8658)

#define GYRO_SCALE_2048DPS (2048.0f / (1 << 15))   //     16 dps/lsb scalefactor for 2048dps sensors

// QMI8658 registers (not the complete list)
typedef enum {
    QMI8658_REG_WHO_AM_I = 0x00,       // chip id, should be 0x05
    QMI8658_REG_REVISION_ID = 0x01,    // chip revision, should be 0x7C
    QMI8658_REG_CTRL1 = 0x02,          // SPI Interface and Sensor Enable
    QMI8658_REG_CTRL2 = 0x03,          // Accelerometer: Output Data Rate, Full Scale, Self-Test
    QMI8658_REG_CTRL3 = 0x04,          // Gyroscope: Output Data Rate, Full Scale, Self-Test
    QMI8658_REG_CTRL5 = 0x06,          // Low pass filter setting
    QMI8658_REG_CTRL7 = 0x08,          // Enable Sensors
    QMI8658_REG_CTRL9 = 0x0A,          // Host Commands
    QMI8658_REG_STATUS0 = 0x2E,        // Output Data Over Run and Data Availability
    QMI8658_REG_TEMP_L = 0x33,         // Temperature sensor low byte.
    QMI8658_REG_TEMP_H = 0x34,         // Temperature sensor high byte.
    QMI8658_REG_AX_L = 0x35,           // Accelerometer X axis LSB
    QMI8658_REG_AX_H = 0x36,           // Accelerometer X axis MSB
    QMI8658_REG_AY_L = 0x37,           // Accelerometer Y axis LSB
    QMI8658_REG_AY_H = 0x38,           // Accelerometer Y axis MSB
    QMI8658_REG_AZ_L = 0x39,           // Accelerometer Z axis LSB
    QMI8658_REG_AZ_H = 0x3A,           // Accelerometer Z axis MSB
    QMI8658_REG_GX_L = 0x3B,           // Gyroscope X axis LSB
    QMI8658_REG_GX_H = 0x3C,           // Gyroscope X axis MSB
    QMI8658_REG_GY_L = 0x3D,           // Gyroscope Y axis LSB
    QMI8658_REG_GY_H = 0x3E,           // Gyroscope Y axis MSB
    QMI8658_REG_GZ_L = 0x3F,           // Gyroscope Z axis LSB
    QMI8658_REG_GZ_H = 0x40,           // Gyroscope Z axis MSB
    QMI8658_REG_RESET = 0x60,           // Soft Reset Register
} qmi8658Register_e;

// QMI8658 register configuration values
/*
typedef enum {
    QMI8658_VAL_CTRL1 = 0x76,                           // 4 wire, address auto increment, int2 output, fifo on int1
    QMI8658_VAL_CTRL2_ENABLE_ACC_SELF_TEST = BIT(7),    // bit 7, enable acc self test
    QMI8658_VAL_CTRL2_ACC_FS_16G = 0x03,                // bit[6:4], acc full scale 16g
    QMI8658_VAL_CTRL2_ACC_ODR_896 = 0x03,               // bit[3:0], 896.8Hz ODR
    QMI8658_VAL_CTRL3_ENABLE_GYRO_SELF_TEST = BIT(7),   // bit 7, enable gyro self test
    QMI8658_VAL_CTRL3_GYRO_FS_2048DPS = 0x07,           // bit[6:4], gyro full scale 2048dps
    QMI8658_VAL_CTRL3_GYRO_ODR_7174 = 0x00,             // bit[3:0], 7174.4Hz ODR
    QMI8658_VAL_CTRL5_GLPF_ODR_266 = 0x00,              // bit[6:5], 2.66% ODR, 190.5Hz
    QMI8658_VAL_CTRL5_GLPF_ODR_363 = 0x01,              // bit[6:5], 3.63% ODR, 260.4Hz
    QMI8658_VAL_CTRL5_GLPF_ODR_539 = 0x02,              // bit[6:5], 5.39% ODR, 386.7Hz
    QMI8658_VAL_CTRL5_GLPF_ODR_1337 = 0x03,             // bit[6:5], 13.37% ODR, 959.2Hz
    QMI8658_VAL_CTRL5_GLPF_EN = BIT(4),                 // bit 4, enable gyro low pass filter
    QMI8658_VAL_CTRL5_ALPF_ODR_1337 = 0x03,             // bit[2:1], 13.37% ODR, 119.9Hz
    QMI8658_VAL_CTRL5_ALPF_EN = BIT(0),                 // bit 0, enable acc low pass filter
    QMI8658_VAL_CTRL7_G_EN = BIT(1),                    // bit 1, enable gyro
    QMI8658_VAL_CTRL7_A_EN = BIT(0),                    // bit 0, enable acc
    QMI8658_VAL_CTRL9_CMD_NOP = 0x00,                   // no operation
    QMI8658_VAL_CTRL9_CMD_ON_DEMAND_CALI = 0xA2,        // on demand cali
    QMI8658_VAL_RESET = 0x0B,                            // reset the device immediately
} qmi8658ConfigReg_e;
 */
typedef enum {
    QMI8658_VAL_CTRL1 = 0x78, //0x76, //    0111 0110                       // 4 wire, address auto increment, int2 output, fifo on int1
    QMI8658_VAL_CTRL2 = 0xB3, //0xB3,//0x33,//
    QMI8658_VAL_CTRL3 = 0xF3, //0xF3,//0x63,//
    QMI8658_VAL_CTRL5 = 0x00, //0x77,//0x77,//0x11,//0x33,//
    QMI8658_VAL_CTRL7 = 0x03, //    10000011
    QMI8658_VAL_CTRL2_ENABLE_ACC_SELF_TEST = 0x80,    // bit 7, enable acc self test
    QMI8658_VAL_CTRL2_ACC_FS_16G = 0x03,                // bit[6:4], acc full scale 16g
    QMI8658_VAL_CTRL2_ACC_ODR_896 = 0x03,               // bit[3:0], 896.8Hz ODR
    QMI8658_VAL_CTRL3_ENABLE_GYRO_SELF_TEST = 0x80,   // bit 7, enable gyro self test
    QMI8658_VAL_CTRL3_GYRO_FS_2048DPS = 0x07,           // bit[6:4], gyro full scale 2048dps
    QMI8658_VAL_CTRL3_GYRO_ODR_7174 = 0x00,             // bit[3:0], 7174.4Hz ODR
    QMI8658_VAL_CTRL5_GLPF_ODR_266 = 0x00,              // bit[6:5], 2.66% ODR, 190.5Hz
    QMI8658_VAL_CTRL5_GLPF_ODR_363 = 0x01,              // bit[6:5], 3.63% ODR, 260.4Hz
    QMI8658_VAL_CTRL5_GLPF_ODR_539 = 0x02,              // bit[6:5], 5.39% ODR, 386.7Hz
    QMI8658_VAL_CTRL5_GLPF_ODR_1337 = 0x03,             // bit[6:5], 13.37% ODR, 959.2Hz
    QMI8658_VAL_CTRL5_GLPF_EN = 0x10,                 // bit 4, enable gyro low pass filter
    QMI8658_VAL_CTRL5_ALPF_ODR_1337 = 0x03,             // bit[2:1], 13.37% ODR, 119.9Hz
    QMI8658_VAL_CTRL5_ALPF_EN = 0x01,                 // bit 0, enable acc low pass filter
    QMI8658_VAL_CTRL7_G_EN = 0x02,                    // bit 1, enable gyro
    QMI8658_VAL_CTRL7_A_EN = 0x01,                    // bit 0, enable acc
    QMI8658_VAL_CTRL9_CMD_NOP = 0x00,                   // no operation
    QMI8658_VAL_CTRL9_CMD_ON_DEMAND_CALI = 0xA2,        // on demand cali
    QMI8658_VAL_RESET = 0x0B,                            // reset the device immediately
    QMI8658_VAL_STATUS0_G_EN = 0x02,                    // bit 1, Gyroscope new data available 0: No updates since last read. 1: New data available.
    QMI8658_VAL_STATUS0_A_EN = 0x01,                    // bit 0, Accelerometer new data available 0: No updates since last read. 1: New data available.
} qmi8658ConfigReg_e;

typedef enum {
    QMI8658_MASK_CTRL5 = 0x77,
    QMI8658_MASK_CTRL7 = 0x03,
} qmi8658ConfigMasks_e;


typedef enum {
    QMI8658_GYRO_HARDWARE_LPF_NORMAL,
    QMI8658_GYRO_HARDWARE_LPF_OPTION_1,
    QMI8658_GYRO_HARDWARE_LPF_OPTION_2,
    QMI8658_GYRO_HARDWARE_LPF_EXPERIMENTAL,
    QMI8658_GYRO_HARDWARE_LPF_COUNT
} q_gyroHardwareLpf_e;

#define QMI8658_CHIP_ID 0x05
uint8_t qmi_sens_data[12]={0,0,0, 0,0,0, 0,0,0, 0,0,0};

static void qmi8658WriteRegister(const busDevice_t *dev, qmi8658Register_e registerID, uint8_t value, unsigned delayMs)
{
    busWrite(dev, registerID, value);
    if (delayMs) {
        delay(delayMs);
    }
}

static void qmi8658WriteRegisterBits(const busDevice_t *dev, qmi8658Register_e registerID, qmi8658ConfigMasks_e mask, uint8_t value, unsigned delayMs)
{
    uint8_t newValue;
    if (busReadBuf(dev, registerID, &newValue, 1)) {
        delayMicroseconds(2);
        newValue = (newValue & ~mask) | value;
        qmi8658WriteRegister(dev, registerID, newValue, delayMs);
    }
}

static uint8_t getQmiDlpfBandwidth(gyroDev_t *gyro)
{
    switch (gyro->lpf) {
        case QMI8658_GYRO_HARDWARE_LPF_NORMAL:
            return QMI8658_VAL_CTRL5_GLPF_ODR_266;
        case QMI8658_GYRO_HARDWARE_LPF_OPTION_1:
            return QMI8658_VAL_CTRL5_GLPF_ODR_363;
        case QMI8658_GYRO_HARDWARE_LPF_OPTION_2:
            return QMI8658_VAL_CTRL5_GLPF_ODR_539;
        case QMI8658_GYRO_HARDWARE_LPF_EXPERIMENTAL:
            return QMI8658_VAL_CTRL5_GLPF_ODR_1337;
    }

    return 0;
}

static void qmi8658Config(gyroDev_t *gyro)
{
    busDevice_t *dev = gyro->busDev;

    // reset the device
    qmi8658WriteRegister(dev, QMI8658_REG_RESET, QMI8658_VAL_RESET, 20);

    // On demand cali
    qmi8658WriteRegister(dev, QMI8658_REG_CTRL9, QMI8658_VAL_CTRL9_CMD_ON_DEMAND_CALI, 2200);
    qmi8658WriteRegister(dev, QMI8658_REG_CTRL9, QMI8658_VAL_CTRL9_CMD_NOP, 100);

    // Configure the CTRL1
    qmi8658WriteRegister(dev, QMI8658_REG_CTRL1, QMI8658_VAL_CTRL1, 1);

    // Disable all sensors
    qmi8658WriteRegister(dev, QMI8658_REG_CTRL7, 0x00, 1);

    // Configure the CTRL2 - ACC configuration
    qmi8658WriteRegister(dev, QMI8658_REG_CTRL2, ((QMI8658_VAL_CTRL2_ACC_FS_16G << 4) | QMI8658_VAL_CTRL2_ACC_ODR_896), 1);

    // Configure the CTRL3 - GYRO configuration
    qmi8658WriteRegister(dev, QMI8658_REG_CTRL3, ((QMI8658_VAL_CTRL3_GYRO_FS_2048DPS << 4) | QMI8658_VAL_CTRL3_GYRO_ODR_7174), 1);

    // Configure the CTRL5 - GYRO LPF and ACC LPF configuration
    qmi8658WriteRegisterBits(dev, QMI8658_REG_CTRL5, QMI8658_MASK_CTRL5, ((getQmiDlpfBandwidth(gyro)<<5) | QMI8658_VAL_CTRL5_GLPF_EN | (QMI8658_VAL_CTRL5_ALPF_ODR_1337<<1) | QMI8658_VAL_CTRL5_ALPF_EN), 1);

    // Enable acc gyro
    qmi8658WriteRegisterBits(dev, QMI8658_REG_CTRL7, QMI8658_MASK_CTRL7, (QMI8658_VAL_CTRL7_G_EN | QMI8658_VAL_CTRL7_A_EN), 100);
}

bool qmi8658accCheckDataReady(accDev_t *acc)
{
    uint8_t status0;

    busRead(acc->busDev, QMI8658_REG_STATUS0, &status0);

    if (status0 & QMI8658_VAL_STATUS0_A_EN) {
        return true;
    }

    return false;
}

bool qmi8658AccRead(accDev_t *acc)
{
    uint8_t data[6];
    const bool ack = busReadBuf(acc->busDev, QMI8658_REG_AX_L, data, 6);

    if (!ack) {
        return false;
    }

    acc->ADCRaw[X] = (float)int16_val_little_endian(data, 0);
    acc->ADCRaw[Y] = (float)int16_val_little_endian(data, 1);
    acc->ADCRaw[Z] = (float)int16_val_little_endian(data, 2);

    return true;
}

bool qmi8658gyroCheckDataReady(gyroDev_t *gyro)
{
    uint8_t status0;

    busRead(gyro->busDev, QMI8658_REG_STATUS0, &status0);

    if (status0 & QMI8658_VAL_STATUS0_G_EN) {
        return true;
    }

    return false;
}

bool qmi8658GyroRead(gyroDev_t *gyro)
{
    uint8_t data[6];
    const bool ack = busReadBuf(gyro->busDev, QMI8658_REG_GX_L, data, 6);

    if (!ack) {
        return false;
    }

    gyro->gyroADCRaw[X] = (float)int16_val_little_endian(data, 0);
    gyro->gyroADCRaw[Y] = (float)int16_val_little_endian(data, 1);
    gyro->gyroADCRaw[Z] = (float)int16_val_little_endian(data, 2);

    return true;
}

static void qmi8658AccInit(accDev_t *acc)
{
    acc->acc_1G = 512 * 4;
}

bool qmi8658AccDetect(accDev_t *acc)
{
    acc->busDev = busDeviceOpen(BUSTYPE_SPI, DEVHW_QMI8658, acc->imuSensorToUse);
    if (acc->busDev == NULL) {
        return false;
     }

    mpuContextData_t * ctx = busDeviceGetScratchpadMemory(acc->busDev);
    if (ctx->chipMagicNumber != 0x8658) {
        return false;
    }

    acc->initFn = qmi8658AccInit;
    acc->readFn = qmi8658AccRead;
    acc->accAlign = acc->busDev->param;

    return true;
}

static void qmi8658AccAndGyroInit(gyroDev_t *gyro)
{
    busDevice_t *dev = gyro->busDev;
    
    qmi8658Config(gyro);

    busSetSpeed(dev, BUS_SPEED_ULTRAFAST);
}

static bool qmi8658DeviceDetect(busDevice_t * busDev)
{
    uint8_t chipID = 0;
    uint8_t i = 0;
    busSetSpeed(busDev, BUS_SPEED_INITIALIZATION);

    // reset the device
    qmi8658WriteRegister(busDev, QMI8658_REG_RESET, QMI8658_VAL_RESET, 20);

    while ((chipID != QMI8658_CHIP_ID) && (i++ < 5)) {
        busReadBuf(busDev, QMI8658_REG_WHO_AM_I, &chipID, 1);
        if ((i == 5) && (chipID != QMI8658_CHIP_ID)) {
            return false;
        }
    }
    busSetSpeed(busDev, BUS_SPEED_ULTRAFAST);

    return true;
}



#define TEMPERATURE_SENSOR_RESOLUTION   (float)(1.0/256.0)  // Telperature sensor resolution (ADC)

bool qmi8658mpuTemperatureRead(gyroDev_t *gyro, int16_t *temp)
{
    busDevice_t *dev = gyro->busDev;
    uint8_t data[2];

    const bool ack = busReadBuf(dev, QMI8658_REG_TEMP_L, data, 2);
    
    if (!ack) {
        return false;
    }
    // T = TEMP_H + (TEMP_L / 256)
    //float temperature = (float)data[1] + (float)data[0] * TEMPERATURE_SENSOR_RESOLUTION;
    int16_t temperature = (int16_t)data[1];

    *temp = (int16_t)(temperature); 

    return true;
}

bool qmi8658GyroDetect(gyroDev_t *gyro)
{
    gyro->busDev = busDeviceInit(BUSTYPE_SPI, DEVHW_QMI8658, gyro->imuSensorToUse, OWNER_MPU);

    if (gyro->busDev == NULL) {
        return false;
    }

    if (!qmi8658DeviceDetect(gyro->busDev)) {
        busDeviceDeInit(gyro->busDev);
        return false;
    }

    // Magic number for ACC detection to indicate that we have detected qmi8658 gyro
    mpuContextData_t * ctx = busDeviceGetScratchpadMemory(gyro->busDev);
    ctx->chipMagicNumber = 0x8658;

    gyro->initFn = qmi8658AccAndGyroInit;
    gyro->readFn = qmi8658GyroRead;
    gyro->intStatusFn = qmi8658gyroCheckDataReady;
    gyro->temperatureFn = qmi8658mpuTemperatureRead;
    gyro->scale = GYRO_SCALE_2048DPS ;   //     16 dps/lsb scalefactor for 2048dps sensors 1.0f /32.0f;// 16.4f;     // 16.4 dps/lsb scalefactor
    gyro->gyroAlign = gyro->busDev->param;

    return true;
}

#endif