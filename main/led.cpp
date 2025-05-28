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
static const char *TAG = "Led";

void Led::Init()
{
    gpio_set_direction(led, GPIO_MODE_OUTPUT);
    TurnOff();
    currentTime = esp_timer_get_time(); // [us]
    prevTime = currentTime;             // [us]
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
    ESP_LOGI(TAG, "Infinite loop: Led ON/OFF");
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

void Led::BlinkLoop(int onTime, int offTime)
{
    currentTime = esp_timer_get_time(); // [us]
    if (currentTime - prevTime > delay * 1000)
    {
        prevTime = esp_timer_get_time(); // [us]
        if (state == false)
        {
            TurnOn();
            delay = onTime; // [ms]
        }
        else
        {
            TurnOff();
            delay = offTime; // [ms]
        }
    }
}