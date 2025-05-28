/*******************************************************************************************
 *                                                                                         *
 *    ESP32-FC                                                                             *
 *    Battery Voltage Measurement                                                          *
 *                                                                                         *
 *    Copyright (c) 2022 Onur AKIN <https://github.com/onurae>                             *
 *    Licensed under the MIT License.                                                      *
 *                                                                                         *
 ******************************************************************************************/

#include "battery.hpp"
static const char *TAG = "Battery";

void Battery::Init()
{
    adc_oneshot_unit_init_cfg_t initConfig = {};
    initConfig.unit_id = ADC_UNIT_1;
    initConfig.ulp_mode = ADC_ULP_MODE_DISABLE;
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&initConfig, &adcHandle));

    adc_oneshot_chan_cfg_t config = {};
    config.bitwidth = ADC_BITWIDTH_DEFAULT;
    config.atten = adcAtten;
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adcHandle, adcChannel, &config));

    ESP_LOGI(TAG, "Calibration scheme version is line fitting");
    adc_cali_line_fitting_config_t caliConfig = {};
    caliConfig.unit_id = ADC_UNIT_1;
    caliConfig.atten = adcAtten;
    caliConfig.bitwidth = ADC_BITWIDTH_DEFAULT;
    ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&caliConfig, &adcCaliHandle));
}

int Battery::GetVoltage()
{
    esp_err_t ret;
    uint32_t adcReading = 0;
    int j = 0;
    for (int i = 0; i < 8; i++)
    {
        ret = adc_oneshot_read(adcHandle, adcChannel, &adcRaw);
        if (ret == ESP_OK)
        {
            j += 1;
            adcReading += adcRaw;
        }
    }
    if (j != 0)
    {
        adcReading /= j;
    }
    //ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data(Average): %ld", ADC_UNIT_1 + 1, adcChannel, adcReading);
    ret = adc_cali_raw_to_voltage(adcCaliHandle, adcReading, &voltage);
    //ESP_LOGI(TAG, "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, adcChannel, voltage);
    if (ret != ESP_OK)
    {
        return 0;
    }
    return voltage * k;
}

bool Battery::Delete()
{
    esp_err_t ret = adc_oneshot_del_unit(adcHandle);
    ESP_LOGI(TAG, "Deregister line fitting calibration scheme");
    ret = adc_cali_delete_scheme_line_fitting(adcCaliHandle);
    if (ret != ESP_OK)
    {
        return false;
    }
    return true;
}