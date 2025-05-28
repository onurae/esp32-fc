/*******************************************************************************************
 *                                                                                         *
 *    ESP32-FC                                                                             *
 *    Utility                                                                              *
 *                                                                                         *
 *    Copyright (c) 2022 Onur AKIN <https://github.com/onurae>                             *
 *    Licensed under the MIT License.                                                      *
 *                                                                                         *
 ******************************************************************************************/

#include "util.hpp"
static const char *TAG = "Util";

void PrintCountDown(uint8_t sec)
{
    for (int i = sec; i > 0; i--)
    {
        ESP_LOGI(TAG, "%d...", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void PrintCountDown(const char *s, uint8_t sec)
{
    ESP_LOGI(TAG, "%s", s);
    PrintCountDown(sec);
}

void Wait(const char *s, uint8_t sec)
{
    ESP_LOGI(TAG, "Wait: %s [%ds]", s, sec);
    for (int i = sec; i > 0; i--)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void WaitForever()
{
    ESP_LOGI(TAG, "Wait: Forever!");
    while(true)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

float LinearScaledDeadband(float value, float deadbandCutoff)
{
    if (std::abs(value) <= deadbandCutoff)
    {
        return 0;
    }
    else
    {
        return (value - (std::abs(value) / value) * deadbandCutoff) / (1.0 - deadbandCutoff);
    }
}

float DegToRad(float deg)
{
    return deg / 180.0f * M_PI;
}

float RadToDeg(float rad)
{
    return rad / M_PI * 180.0f;
}