/*******************************************************************************************
 *                                                                                         *
 *    ESP32-FC                                                                             *
 *    SBUS (Serial Bus, FrSky)                                                             *
 *                                                                                         *
 *    Copyright (c) 2023 Onur AKIN <https://github.com/onurae>                             *
 *    Licensed under the MIT License.                                                      *
 *                                                                                         *
 ******************************************************************************************/

#include "sbus.hpp"

Sbus::Sbus(uart_port_t uartPort, gpio_num_t txPin, gpio_num_t rxPin, uint8_t period) : uartPort(uartPort), txPin(txPin), rxPin(rxPin), period(period)
{
}

bool Sbus::Init()
{
    uart_config_t conf = {};
    conf.baud_rate = 100000;
    conf.data_bits = UART_DATA_8_BITS;
    conf.parity = UART_PARITY_EVEN;
    conf.stop_bits = UART_STOP_BITS_2;
    conf.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    conf.rx_flow_ctrl_thresh = 0;
    conf.source_clk = UART_SCLK_DEFAULT;
    ESP_ERROR_CHECK(uart_driver_install(uartPort, 1024 * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(uartPort, &conf));
    ESP_ERROR_CHECK(uart_set_line_inverse(uartPort, UART_SIGNAL_RXD_INV | UART_SIGNAL_TXD_INV));
    ESP_ERROR_CHECK(uart_set_pin(uartPort, txPin, rxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_flush(uartPort));

    vTaskDelay((period * 2) / portTICK_PERIOD_MS); // Wait for data.
    if (Read())
    {
        printf("SBUS ready.\n");
        return true;
    }
    return false;
}

bool Sbus::Read()
{
    bool newPacket = false;
    int length = 0;
    while (uart_get_buffered_data_len(uartPort, (size_t *)&length) == ESP_OK && length > 0)
    {
        const int rxBytes = uart_read_bytes(uartPort, buf, (length > 25) ? 25 : length, 100 / portTICK_PERIOD_MS);
        for (int i = 0; i < rxBytes; i++)
        {
            if (k == 0)
            {
                if (buf[i] == header && (prevData == footer || ((prevData & 0x0F) == footer2)))
                {
                    rxBuf[k] = buf[i];
                    k += 1;
                }
            }
            else
            {
                rxBuf[k] = buf[i];
                k += 1;
                if (k == 25)
                {
                    k = 0;
                    prevData = buf[i];
                    if (buf[i] == footer || ((buf[i] & 0x0F) == footer2))
                    {
                        newPacket = true;
                        packetTime = esp_timer_get_time();
                        rxData.ch[0] = static_cast<uint16_t>(rxBuf[1] | ((rxBuf[2] << 8) & 0x07FF));
                        rxData.ch[1] = static_cast<uint16_t>((rxBuf[2] >> 3) | ((rxBuf[3] << 5) & 0x07FF));
                        rxData.ch[2] = static_cast<uint16_t>((rxBuf[3] >> 6) | (rxBuf[4] << 2) | ((rxBuf[5] << 10) & 0x07FF));
                        rxData.ch[3] = static_cast<uint16_t>((rxBuf[5] >> 1) | ((rxBuf[6] << 7) & 0x07FF));
                        rxData.ch[4] = static_cast<uint16_t>((rxBuf[6] >> 4) | ((rxBuf[7] << 4) & 0x07FF));
                        rxData.ch[5] = static_cast<uint16_t>((rxBuf[7] >> 7) | (rxBuf[8] << 1) | ((rxBuf[9] << 9) & 0x07FF));
                        rxData.ch[6] = static_cast<uint16_t>((rxBuf[9] >> 2) | ((rxBuf[10] << 6) & 0x07FF));
                        rxData.ch[7] = static_cast<uint16_t>((rxBuf[10] >> 5) | ((rxBuf[11] << 3) & 0x07FF));
                        rxData.ch[8] = static_cast<uint16_t>(rxBuf[12] | ((rxBuf[13] << 8) & 0x07FF));
                        rxData.ch[9] = static_cast<uint16_t>((rxBuf[13] >> 3) | ((rxBuf[14] << 5) & 0x07FF));
                        rxData.ch[10] = static_cast<uint16_t>((rxBuf[14] >> 6) | (rxBuf[15] << 2) | ((rxBuf[16] << 10) & 0x07FF));
                        rxData.ch[11] = static_cast<uint16_t>((rxBuf[16] >> 1) | ((rxBuf[17] << 7) & 0x07FF));
                        rxData.ch[12] = static_cast<uint16_t>((rxBuf[17] >> 4) | ((rxBuf[18] << 4) & 0x07FF));
                        rxData.ch[13] = static_cast<uint16_t>((rxBuf[18] >> 7) | (rxBuf[19] << 1) | ((rxBuf[20] << 9) & 0x07FF));
                        rxData.ch[14] = static_cast<uint16_t>((rxBuf[20] >> 2) | ((rxBuf[21] << 6) & 0x07FF));
                        rxData.ch[15] = static_cast<uint16_t>((rxBuf[21] >> 5) | ((rxBuf[22] << 3) & 0x07FF));
                        rxData.ch17 = rxBuf[23] & 1;
                        rxData.ch18 = rxBuf[23] >> 1 & 1;
                        rxData.frameLost = rxBuf[23] >> 2 & 1;
                        rxData.failSafe = rxBuf[23] >> 3 & 1;
                    }
                }
            }
            prevData = buf[i];
        }
    }
    return newPacket;
}

Sbus::SbusData Sbus::GetData()
{
    return rxData;
}

float Sbus::MapRange(uint16_t value, float minOut, float maxOut, float minIn, float maxIn)
{
    float out = (value - minIn) * (maxOut - minOut) / (maxIn - minIn) + minOut;
    return std::min(std::max(out, minOut < maxOut ? minOut : maxOut), minOut > maxOut ? minOut : maxOut);
}

float Sbus::GetAnalog(uint16_t channel, float rangeMin, float rangeMax)
{
    return MapRange(rxData.ch[channel - 1], rangeMin, rangeMax);
}

int Sbus::GetSwitch2Pos(uint16_t channel)
{
    float m = MapRange(rxData.ch[channel - 1], 0, 1);
    if (m > 0.5f) 
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

int Sbus::GetSwitch3Pos(uint16_t channel)
{
    float m = MapRange(rxData.ch[channel - 1], 0, 1);
    if (m > 0.66f)
    {
        return 2;
    }
    else if ( m > 0.33f)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

int Sbus::Write(const SbusData &txData)
{
    uint8_t txBuf[25] = {};
    txBuf[0] = header;
    txBuf[1] = static_cast<uint8_t>((txData.ch[0] & 0x07FF));
    txBuf[2] = static_cast<uint8_t>((txData.ch[0] & 0x07FF) >> 8 | (txData.ch[1] & 0x07FF) << 3);
    txBuf[3] = static_cast<uint8_t>((txData.ch[1] & 0x07FF) >> 5 | (txData.ch[2] & 0x07FF) << 6);
    txBuf[4] = static_cast<uint8_t>((txData.ch[2] & 0x07FF) >> 2);
    txBuf[5] = static_cast<uint8_t>((txData.ch[2] & 0x07FF) >> 10 | (txData.ch[3] & 0x07FF) << 1);
    txBuf[6] = static_cast<uint8_t>((txData.ch[3] & 0x07FF) >> 7 | (txData.ch[4] & 0x07FF) << 4);
    txBuf[7] = static_cast<uint8_t>((txData.ch[4] & 0x07FF) >> 4 | (txData.ch[5] & 0x07FF) << 7);
    txBuf[8] = static_cast<uint8_t>((txData.ch[5] & 0x07FF) >> 1);
    txBuf[9] = static_cast<uint8_t>((txData.ch[5] & 0x07FF) >> 9 | (txData.ch[6] & 0x07FF) << 2);
    txBuf[10] = static_cast<uint8_t>((txData.ch[6] & 0x07FF) >> 6 | (txData.ch[7] & 0x07FF) << 5);
    txBuf[11] = static_cast<uint8_t>((txData.ch[7] & 0x07FF) >> 3);
    txBuf[12] = static_cast<uint8_t>((txData.ch[8] & 0x07FF));
    txBuf[13] = static_cast<uint8_t>((txData.ch[8] & 0x07FF) >> 8 | (txData.ch[9] & 0x07FF) << 3);
    txBuf[14] = static_cast<uint8_t>((txData.ch[9] & 0x07FF) >> 5 | (txData.ch[10] & 0x07FF) << 6);
    txBuf[15] = static_cast<uint8_t>((txData.ch[10] & 0x07FF) >> 2);
    txBuf[16] = static_cast<uint8_t>((txData.ch[10] & 0x07FF) >> 10 | (txData.ch[11] & 0x07FF) << 1);
    txBuf[17] = static_cast<uint8_t>((txData.ch[11] & 0x07FF) >> 7 | (txData.ch[12] & 0x07FF) << 4);
    txBuf[18] = static_cast<uint8_t>((txData.ch[12] & 0x07FF) >> 4 | (txData.ch[13] & 0x07FF) << 7);
    txBuf[19] = static_cast<uint8_t>((txData.ch[13] & 0x07FF) >> 1);
    txBuf[20] = static_cast<uint8_t>((txData.ch[13] & 0x07FF) >> 9 | (txData.ch[14] & 0x07FF) << 2);
    txBuf[21] = static_cast<uint8_t>((txData.ch[14] & 0x07FF) >> 6 | (txData.ch[15] & 0x07FF) << 5);
    txBuf[22] = static_cast<uint8_t>((txData.ch[15] & 0x07FF) >> 3);
    txBuf[23] = 0x00 | (txData.ch17) | (txData.ch18 << 1) | (txData.frameLost << 2) | (txData.failSafe << 3);
    txBuf[24] = footer;
    const int txBytes = uart_write_bytes(uartPort, txBuf, sizeof(txBuf));
    return txBytes;
}

bool Sbus::CheckStatus(int timeout)
{
    // If there is no or corrupted data for a while, it means there is a receiver or cable connection problem.
    // If rxData.failsafe is true, there is no connection between Tx and Rx.
    // Test Note: After the Tx is switched off, rxData.frameLost is true almost immediately and rxData.failSafe is true after 1 second.
    if (rxData.failSafe == true || (esp_timer_get_time() - packetTime) > (timeout * 1000))
    {
        return false;
    }
    return true;
}

void Sbus::PrintData()
{
    printf("%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
           rxData.ch[0], rxData.ch[1], rxData.ch[2], rxData.ch[3], rxData.ch[4], rxData.ch[5], rxData.ch[6], rxData.ch[7],
           rxData.ch[8], rxData.ch[9], rxData.ch[10], rxData.ch[11], rxData.ch[12], rxData.ch[13], rxData.ch[14], rxData.ch[15],
           rxData.ch17, rxData.ch18, rxData.failSafe, rxData.frameLost);
}

void Sbus::PrintTest()
{
    Sbus::SbusData sbusData = GetData();
    uint16_t ch0_raw = sbusData.ch[0];
    float ch0 = GetAnalog(1, -1.0f, 1.0f);
    int arm = GetSwitch2Pos(5);
    int another = GetSwitch3Pos(6);
    uint16_t throttle_raw = sbusData.ch[2];
    float throttle = GetAnalog(3, 0.0f, 1.0f);
    bool fs = rxData.failSafe;
    bool fl = rxData.frameLost;
    // Test Note: After the Tx is switched off, rxData.frameLost is true almost immediately and rxData.failSafe is true after 1 second.
    // Both failsafe and framelost are set to false after the Tx is switched on.
    printf("%d %f %d %d %d %f %d %d\n", ch0_raw, ch0, arm, another, throttle_raw, throttle, fs, fl);
}