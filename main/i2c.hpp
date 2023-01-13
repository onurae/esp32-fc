/*******************************************************************************************
 *                                                                                         *
 *    ESP32-FC                                                                             *
 *    I2C (Inter-Integrated Circuit)                                                       *
 *                                                                                         *
 *    Copyright (c) 2022 Onur AKIN <https://github.com/onurae>                             *
 *    Licensed under the MIT License.                                                      *
 *                                                                                         *
 ******************************************************************************************/

#ifndef I2C_HPP
#define I2C_HPP

#include "driver/i2c.h"

class I2c
{
private:
    uint8_t masterPort;
    uint8_t sclIO;
    uint8_t sdaIO;
    uint16_t timeoutMS;

public:
    I2c(uint8_t masterPort, uint8_t sclIO, uint8_t sdaIO, uint16_t timeoutMS = 1000);
    virtual ~I2c() = default;

    void MasterInit();
    esp_err_t Write(uint8_t address, uint8_t data);
    esp_err_t Write(uint8_t address, uint8_t registerAddress, uint8_t data);
    esp_err_t Read(uint8_t address, uint8_t registerAddress, uint8_t len, uint8_t *buffer);
};

#endif /* I2C_HPP */
