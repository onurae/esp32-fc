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

void PrintCountDown(const char* s, uint8_t sec)
{
    printf("%s\n", s);
    PrintCountDown(sec);
}

void LoopForever()
{
    while (true)
    {
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}