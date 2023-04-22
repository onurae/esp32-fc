/*******************************************************************************************
 *                                                                                         *
 *    ESP32-FC                                                                             *
 *    LED                                                                                  *
 *                                                                                         *
 *    Copyright (c) 2022 Onur AKIN <https://github.com/onurae>                             *
 *    Licensed under the MIT License.                                                      *
 *                                                                                         *
 ******************************************************************************************/

#include "led.hpp"

void Led::Init()
{
    gpio_set_direction(led, GPIO_MODE_OUTPUT);
    TurnOff();
}

void Led::BlinkForever(int intervalMS)
{
    printf("Infinite loop: Led ON/OFF\n");
    while(true)
    {
        state = !state;
        gpio_set_level(led, state);
        vTaskDelay(intervalMS / portTICK_PERIOD_MS);
    }
}