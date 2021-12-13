#pragma once

#include <stdbool.h>
#include "main.h"
#include "DWT_Delay.h"
#include "Vector.h"
#include "CRC.h"

typedef enum DS18B20Status {
    DS18B20_OK,
    DS18B20_RESPONSE_ERROR,
    DS18B20_NOT_FOUND_ERROR,
    DS18B20_CHECKSUM_ERROR
} DS18B20Status;

typedef struct DS18B20Sensor {
    GPIO_TypeDef *GPIOx;
    uint32_t pin;
    uint64_t address;
    DS18B20Status status;
    float temperature;
} DS18B20Sensor;

DS18B20Sensor *initDS18B20Sensor(GPIO_TypeDef *GPIOx, uint32_t pin);
void readTemperatureDS18B20(DS18B20Sensor *sensor);
void checkConnectionStatusDS18B20(DS18B20Sensor *sensor);
void deleteDS18B20(DS18B20Sensor *sensor);