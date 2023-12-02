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

void PrintCountDown(uint8_t sec)
{
    for (int i = sec; i > 0; i--)
    {
        printf("%d...\n", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void PrintCountDown(const char *s, uint8_t sec)
{
    printf("%s\n", s);
    PrintCountDown(sec);
}

void Wait(const char *s, uint8_t sec)
{
    printf("Wait: %s [%ds]\n", s, sec);
    for (int i = sec; i > 0; i--)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void WaitForever()
{
    printf("Wait: Forever!\n");
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