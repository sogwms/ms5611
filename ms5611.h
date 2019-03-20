/*
 * Copyright (c) 2006-2019, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-03-12     sogwms       the first version
 */

#ifndef __MS5611_H__
#define __MS5611_H__

#include <rtthread.h>

/* the macro below will enable second order temperature compensation */
#define CFG_MS5611_USING_TEMPERATURE_COMPENSATION
/* the macro below will enable ms5611 PROM data crc check */
#define CFG_MS5611_CRC_ENABLE

#define MS5611_I2C_ADDR_CSB_L   (0xEC >> 1)
#define MS5611_I2C_ADDR_CSB_H   (0xEE >> 1)

/* Oversampling Ratio */
enum ms5611_osr_e
{                             //response time (max)
     MS5611_OSR_256 = 0,      //0.60ms
     MS5611_OSR_512 = 2,      //1.17ms
     MS5611_OSR_1024 = 4,     //2.28ms
     MS5611_OSR_2048 = 6,     //4.54ms
     MS5611_OSR_4096 = 8      //9.04ms
};

#define MS5611_OSR_DEFAULT      MS5611_OSR_4096

struct ms5611_param
{
     uint16_t cx[8];
     enum ms5611_osr_e osr_d1;  //pressure
     enum ms5611_osr_e osr_d2;  //temperature
};

struct ms5611_device
{
    struct rt_device *bus;
    struct ms5611_param ref;
    uint8_t i2c_addr;
    int32_t temp;   //range: -4000 - 8500  (2007=20.07`C)
    int32_t baro;   //range: 1000 - 120000 (100009=1000.09mbar)
    rt_mutex_t  measure_mutex;
};
typedef struct ms5611_device *ms5611_device_t;

ms5611_device_t ms5611_init(const char *dev_name, uint8_t param);
void ms5611_measure(ms5611_device_t dev);
int ms5611_reset_and_load_calibration(ms5611_device_t dev);
void ms5611_deinit(ms5611_device_t dev);

#endif