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

void Led::Blink(int numBlinks, int onTime, int offTime)
{
    for (int i = 0; i < numBlinks; i++)
    {
        TurnOff();
        vTaskDelay(offTime / portTICK_PERIOD_MS);
        TurnOn();
        vTaskDelay(onTime / portTICK_PERIOD_MS);
    }
    TurnOff();
}

void Led::BlinkForever(int intervalMS)
{
    printf("Infinite loop: Led ON/OFF\n");
    while (true)
    {
        Flip();
        vTaskDelay(intervalMS / portTICK_PERIOD_MS);
    }
}

void Led::TurnOn()
{
    if (gpio_set_level(led, false) == ESP_OK) // Led is on when the level is false.
    {
        state = true;
    }
}

void Led::TurnOff()
{
    if (gpio_set_level(led, true) == ESP_OK)
    {
        state = false;
    }
}

void Led::Flip()
{
    state ? TurnOff() : TurnOn();
}