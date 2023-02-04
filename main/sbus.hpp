/*******************************************************************************************
 *                                                                                         *
 *    ESP32-FC                                                                             *
 *    SBUS (Serial Bus, FrSky)                                                             *
 *                                                                                         *
 *    Copyright (c) 2023 Onur AKIN <https://github.com/onurae>                             *
 *    Licensed under the MIT License.                                                      *
 *                                                                                         *
 ******************************************************************************************/

#ifndef SBUS_HPP
#define SBUS_HPP

#include "driver/uart.h"
#include "driver/gpio.h"
#include <algorithm>

class Sbus
{
public:
    Sbus(uart_port_t uartPort = UART_NUM_1, gpio_num_t txPin = GPIO_NUM_4, gpio_num_t rxPin = GPIO_NUM_5, uint8_t period = 7);
    virtual ~Sbus() = default;
    struct SbusData
    {
        uint16_t ch[16];
        bool ch17;
        bool ch18;
        bool failSafe;
        bool frameLost;
    };
    bool Init();
    bool Read();
    int Write(const SbusData &txData);
    SbusData GetData();
    void PrintData();
    float GetAnalog(uint16_t channel, float rangeMin, float rangeMax);
    int GetSwitch2Pos(uint16_t channel);
    int GetSwitch3Pos(uint16_t channel);

private:
    uart_port_t uartPort;
    gpio_num_t txPin;
    gpio_num_t rxPin;
    uint8_t period; // Receiver output [ms]
    const uint8_t header = 0x0F;
    const uint8_t footer = 0x00;
    const uint8_t footer2 = 0x04;
    uint8_t buf[25];
    uint8_t rxBuf[25];
    SbusData rxData;
    uint8_t k = 0;
    uint8_t prevData = footer;
    float MapRange(uint16_t value, float minOut, float maxOut, float minIn = 192.0f, float maxIn = 1792.0f);
};

#endif /* SBUS_HPP */