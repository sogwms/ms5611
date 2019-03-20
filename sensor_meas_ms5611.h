#ifndef SENSOR_MEAS_MS5611_H__
#define SENSOR_MEAS_MS5611_H__

#include "sensor.h"
#include "ms5611.h"

#define MS5611_USING_BARO
#define MS5611_USING_TEMP
int rt_hw_ms5611_init(const char *name, struct rt_sensor_config *cfg);

#endif