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

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"

class I2c
{
private:
    i2c_port_t masterPort;
    i2c_master_bus_handle_t busHandle;
    gpio_num_t sclIO;
    gpio_num_t sdaIO;
    uint16_t timeoutMS;

public:
    I2c(i2c_port_t masterPort, gpio_num_t sclIO, gpio_num_t sdaIO, uint16_t timeoutMS = 1000);
    virtual ~I2c() = default;

    void Init();
    void AddDevice(const uint8_t address, i2c_master_dev_handle_t* devHandle);
    esp_err_t Write(i2c_master_dev_handle_t* devHandle, uint8_t data);
    esp_err_t Write(i2c_master_dev_handle_t* devHandle, uint8_t registerAddress, uint8_t data);
    esp_err_t Read(i2c_master_dev_handle_t* devHandle, uint8_t registerAddress, uint8_t len, uint8_t *buffer);
};

#endif /* I2C_HPP */
