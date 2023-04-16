/*******************************************************************************************
 *                                                                                         *
 *    ESP32-FC                                                                             *
 *    FrSKY (Smart Port)                                                                   *
 *                                                                                         *
 *    Copyright (c) 2022 Onur AKIN <https://github.com/onurae>                             *
 *    Licensed under the MIT License.                                                      *
 *                                                                                         *
 ******************************************************************************************/

#ifndef FRSKY_HPP
#define FRSKY_HPP

#include "driver/uart.h"
#include "driver/gpio.h"

class Frsky
{
private:
    uart_port_t uartPort;
    gpio_num_t txPin;
    gpio_num_t rxPin;
    const uint8_t header = 0x7E;
    uint8_t buf[2];

    // Sensor ID
    const uint8_t sensorID1 = 0x00;
    const uint8_t sensorID2 = 0xA1;
    const uint8_t sensorID3 = 0x22; // FAS40S
    const uint8_t sensorID4 = 0x83;
    const uint8_t sensorID5 = 0xE4;
    const uint8_t sensorID6 = 0x45;
    const uint8_t sensorID7 = 0xC6;
    const uint8_t sensorID8 = 0x67;
    const uint8_t sensorID9 = 0x48;
    const uint8_t sensorID10 = 0xE9;
    const uint8_t sensorID11 = 0x6A;
    const uint8_t sensorID12 = 0xCB;
    const uint8_t sensorID13 = 0xAC;
    const uint8_t sensorID14 = 0x0D;
    const uint8_t sensorID15 = 0x8E;
    const uint8_t sensorID16 = 0x2F;
    const uint8_t sensorID17 = 0xD0;
    const uint8_t sensorID18 = 0x71;
    const uint8_t sensorID19 = 0xF2;
    const uint8_t sensorID20 = 0x53;
    const uint8_t sensorID21 = 0x34;
    const uint8_t sensorID22 = 0x95;
    const uint8_t sensorID23 = 0x16;
    const uint8_t sensorID24 = 0xB7;
    const uint8_t sensorID25 = 0x98;
    const uint8_t sensorID26 = 0x39;
    const uint8_t sensorID27 = 0xBA;
    const uint8_t sensorID28 = 0x1B;

    // Data ID
    unsigned int i[28] = {};
    const uint16_t dataID_RSSI = 0xf101;
    const uint16_t dataID_ADC1 = 0xf102;
    const uint16_t dataID_ADC2 = 0xf103;
    const uint16_t dataID_BATT = 0xf104;
    const uint16_t dataID_SWR = 0xf105;
    const uint16_t dataID_T1 = 0x0400;
    const uint16_t dataID_T2 = 0x0410;
    const uint16_t dataID_RPM = 0x0500;
    const uint16_t dataID_FUEL = 0x0600;
    const uint16_t dataID_ALT = 0x0100;
    const uint16_t dataID_VARIO = 0x0110;
    const uint16_t dataID_ACCX = 0x0700;
    const uint16_t dataID_ACCY = 0x0710;
    const uint16_t dataID_ACCZ = 0x0720;
    const uint16_t dataID_CURR = 0x0200;
    const uint16_t dataID_VFAS = 0x0210;
    const uint16_t dataID_CELLS = 0x0300;
    const uint16_t dataID_GPS_LONG_LATI = 0x0800;
    const uint16_t dataID_GPS_ALT = 0x0820;
    const uint16_t dataID_GPS_SPEED = 0x0830;
    const uint16_t dataID_GPS_COURS = 0x0840;
    const uint16_t dataID_GPS_TIME_DATE = 0x0850;

    void SendData(uint16_t id, int32_t val, uint8_t type = 0x10);
    void SendByte(uint8_t c, uint16_t *crcp);

    float current = 0;
    float voltage = 0;

public:
    Frsky(uart_port_t uartPort = UART_NUM_1, gpio_num_t txPin = GPIO_NUM_17, gpio_num_t rxPin = GPIO_NUM_16);
    virtual ~Frsky() = default;

    void Init();
    void Flush();
    bool Operate();

    void SetCurrent(float c) { current = c; }
    void SetVoltage(float v) { voltage = v; }
};

#endif /* FRSKY_HPP */
