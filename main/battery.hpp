/*******************************************************************************************
 *                                                                                         *
 *    ESP32-FC                                                                             *
 *    Battery Voltage Measurement                                                          *
 *                                                                                         *
 *    Copyright (c) 2022 Onur AKIN <https://github.com/onurae>                             *
 *    Licensed under the MIT License.                                                      *
 *                                                                                         *
 ******************************************************************************************/

#ifndef BATTERY_HPP
#define BATTERY_HPP

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"

class Battery
{
private:
    int adcRaw;
    int voltage; // [mV]
    float k = 11.153f; // Voltage divider
    adc_oneshot_unit_handle_t adcHandle = nullptr;
    adc_channel_t adcChannel = ADC_CHANNEL_6; // ADC_CHANNEL_6 = Pin: 34
    adc_atten_t adcAtten = ADC_ATTEN_DB_2_5;  // 2.5 dB gives full-scale voltage 1.5 V
    adc_cali_handle_t adcCaliHandle = nullptr;

public:
    Battery() = default;
    virtual ~Battery() = default;

    void Init();
    int GetVoltage(); // [mV]
    bool Delete();
};

#endif /* BATTERY_HPP */
