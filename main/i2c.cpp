/*******************************************************************************************
 *                                                                                         *
 *    ESP32-FC                                                                             *
 *    I2C (Inter-Integrated Circuit)                                                       *
 *                                                                                         *
 *    Copyright (c) 2022 Onur AKIN <https://github.com/onurae>                             *
 *    Licensed under the MIT License.                                                      *
 *                                                                                         *
 ******************************************************************************************/

#include "i2c.hpp"

I2c::I2c(i2c_port_t masterPort, uint8_t sclIO, uint8_t sdaIO, uint16_t timeoutMS) : 
    masterPort(masterPort),
    sclIO(sclIO),
    sdaIO(sdaIO),
    timeoutMS(timeoutMS)
{
}

void I2c::MasterInit()
{
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER,
    conf.sda_io_num = sdaIO,
    conf.scl_io_num = sclIO,
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE,
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE,
    conf.master.clk_speed = 400000,
    conf.clk_flags = 0;

    ESP_ERROR_CHECK(i2c_param_config(masterPort, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(masterPort, conf.mode, 0, 0, 0));
}

esp_err_t I2c::Write(uint8_t address, uint8_t data)
{
    uint8_t writeBuffer[1] = {data};
    return i2c_master_write_to_device(masterPort, address, writeBuffer, sizeof(writeBuffer), timeoutMS / portTICK_PERIOD_MS);
}

esp_err_t I2c::Write(uint8_t address, uint8_t registerAddress, uint8_t data)
{
    uint8_t writeBuffer[2] = {registerAddress, data};
    return i2c_master_write_to_device(masterPort, address, writeBuffer, sizeof(writeBuffer), timeoutMS / portTICK_PERIOD_MS);
}

esp_err_t I2c::Read(uint8_t address, uint8_t registerAddress, uint8_t len, uint8_t *buffer)
{
    return i2c_master_write_read_device(masterPort, address, &registerAddress, 1, buffer, len, timeoutMS / portTICK_PERIOD_MS);
}