/*******************************************************************************************
 *                                                                                         *
 *    ESP32-FC                                                                             *
 *    FrSKY (Smart Port)                                                                   *
 *                                                                                         *
 *    Copyright (c) 2022 Onur AKIN <https://github.com/onurae>                             *
 *    Licensed under the MIT License.                                                      *
 *                                                                                         *
 ******************************************************************************************/

#include "frsky.hpp"

Frsky::Frsky(uart_port_t uartPort, gpio_num_t txPin, gpio_num_t rxPin) : uartPort(uartPort), txPin(txPin), rxPin(rxPin)
{
}

void Frsky::Init()
{
    uart_config_t conf = {};
    conf.baud_rate = 57600;
    conf.data_bits = UART_DATA_8_BITS;
    conf.parity = UART_PARITY_DISABLE;
    conf.stop_bits = UART_STOP_BITS_1;
    conf.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    conf.rx_flow_ctrl_thresh = 0;
    conf.source_clk = UART_SCLK_DEFAULT;
    ESP_ERROR_CHECK(uart_driver_install(uartPort, 1024 * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(uartPort, &conf));
    ESP_ERROR_CHECK(uart_set_line_inverse(uartPort, UART_SIGNAL_RXD_INV | UART_SIGNAL_TXD_INV));
    ESP_ERROR_CHECK(uart_set_pin(uartPort, txPin, rxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_flush(uartPort));

    printf("FrSKY telemetry initialization...\n");
    uart_flush_input(uartPort);
    uint8_t period = 11; // [ms]
    while (!Operate())
    {
        vTaskDelay(period / portTICK_PERIOD_MS);
    }
    printf("FrSKY telemetry ready.\n");
}

void Frsky::Flush()
{
    uart_flush(uartPort);
    uart_flush_input(uartPort);
}

bool Frsky::Operate()
{
    int length = 0;
    if (uart_get_buffered_data_len(uartPort, (size_t *)&length) == ESP_OK && length > 1)
    {
        if (length > 2)
        {
            uart_flush_input(uartPort);
            return false;
        }
        const int rxBytes = uart_read_bytes(uartPort, buf, length, 0);
        if (rxBytes == 2 && buf[0] == header)
        {
            switch (buf[1])
            {
            case 0x00: // ID1
                if (i[0] == 0)
                    SendData(dataID_ALT, altitude * 100); // Ex: 123.45m
                i[0]++;
                if (i[0] >= 1) { i[0] = 0; }
                return true;
                break;
            case 0xA1: // ID2
                break;
            case 0x22: // ID3 - FAS40S
                if (i[2] == 0)
                    SendData(dataID_CURR, current * 10); // Ex: 10.5A
                if (i[2] == 1)
                    SendData(dataID_VFAS, voltage * 100); // Ex: 10.55V
                i[2]++;
                if (i[2] >= 2) { i[2] = 0; }
                return true;
                break;
            case 0x83: // ID4
                break;
            case 0xE4: // ID5
                break;
            case 0x45: // ID6
                break;
            case 0xC6: // ID7
                break;
            case 0x67: // ID8
                break;
            case 0x48: // ID9
                break;
            case 0xE9: // ID10
                break;
            case 0x6A: // ID11
                break;
            case 0xCB: // ID12
                break;
            case 0xAC: // ID13
                break;
            case 0x0D: // ID14
                break;
            case 0x8E: // ID15
                break;
            case 0x2F: // ID16
                break;
            case 0xD0: // ID17
                break;
            case 0x71: // ID18
                break;
            case 0xF2: // ID19
                break;
            case 0x53: // ID20
                break;
            case 0x34: // ID21
                break;
            case 0x95: // ID22
                break;
            case 0x16: // ID23
                break;
            case 0xB7: // ID24
                break;
            case 0x98: // ID25
                break;
            case 0x39: // ID26
                break;
            case 0xBA: // ID27
                break;
            case 0x1B: // ID28
                break;
            default:
                break;
            }
        }
    }
    return false;
}

void Frsky::SendData(uint16_t id, int32_t val, uint8_t type)
{
    uint16_t crc = 0;
    uint8_t *u8p;

    // type
    SendByte(type, &crc);

    // id
    u8p = (uint8_t *)&id;
    SendByte(u8p[0], &crc);
    SendByte(u8p[1], &crc);

    // val
    u8p = (uint8_t *)&val;
    SendByte(u8p[0], &crc);
    SendByte(u8p[1], &crc);
    SendByte(u8p[2], &crc);
    SendByte(u8p[3], &crc);

    // crc
    SendByte(0xFF - (uint8_t)crc, nullptr);
}

void Frsky::SendByte(uint8_t c, uint16_t *crcp)
{
    // smart port escape sequence
    if (c == 0x7D || c == 0x7E)
    {
        uint8_t tx = 0x7D;
        uart_write_bytes(uartPort, &tx, 1);
        c ^= 0x20;
    }

    uint8_t tx = c;
    uart_write_bytes(uartPort, &tx, 1);

    if (crcp == nullptr)
        return;

    uint16_t crc = *crcp;
    crc += c;
    crc += crc >> 8;
    crc &= 0x00FF;
    *crcp = crc;
}