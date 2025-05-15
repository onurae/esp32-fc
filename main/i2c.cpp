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

I2c::I2c(i2c_port_t masterPort, gpio_num_t sclIO, gpio_num_t sdaIO, uint16_t timeoutMS) : 
    masterPort(masterPort),
    sclIO(sclIO),
    sdaIO(sdaIO),
    timeoutMS(timeoutMS)
{
}

void I2c::Init()
{
    i2c_master_bus_config_t busConfig = {};
    busConfig.i2c_port = masterPort;
    busConfig.sda_io_num = sdaIO;
    busConfig.scl_io_num = sclIO;
    busConfig.clk_source = I2C_CLK_SRC_DEFAULT;
    busConfig.glitch_ignore_cnt = 7;
    busConfig.flags.enable_internal_pullup = true;
    ESP_ERROR_CHECK(i2c_new_master_bus(&busConfig, &busHandle));
}

void I2c::AddDevice(const uint8_t address, i2c_master_dev_handle_t* devHandle)
{
    i2c_device_config_t devConfig = {};
    devConfig.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    devConfig.device_address = address;
    devConfig.scl_speed_hz = 400000;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(busHandle, &devConfig, devHandle));
}

esp_err_t I2c::Write(i2c_master_dev_handle_t* devHandle, uint8_t data)
{
    uint8_t writeBuffer[1] = {data};
    return i2c_master_transmit(*devHandle, writeBuffer, sizeof(writeBuffer), timeoutMS / portTICK_PERIOD_MS);
}

esp_err_t I2c::Write(i2c_master_dev_handle_t* devHandle, uint8_t registerAddress, uint8_t data)
{
    uint8_t writeBuffer[2] = {registerAddress, data};
    return i2c_master_transmit(*devHandle, writeBuffer, sizeof(writeBuffer), timeoutMS / portTICK_PERIOD_MS);
}

esp_err_t I2c::Read(i2c_master_dev_handle_t* devHandle, uint8_t registerAddress, uint8_t len, uint8_t *buffer)
{
    return i2c_master_transmit_receive(*devHandle, &registerAddress, 1, buffer, len, timeoutMS / portTICK_PERIOD_MS);
}