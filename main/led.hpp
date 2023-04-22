/*******************************************************************************************
 *                                                                                         *
 *    ESP32-FC                                                                             *
 *    LED                                                                                  *
 *                                                                                         *
 *    Copyright (c) 2022 Onur AKIN <https://github.com/onurae>                             *
 *    Licensed under the MIT License.                                                      *
 *                                                                                         *
 ******************************************************************************************/

#ifndef LED_HPP
#define LED_HPP

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

class Led
{
private:
    gpio_num_t led;
    bool state = false;

public:
    Led(gpio_num_t led = GPIO_NUM_22) { this->led = led; }
    virtual ~Led() = default;

    void Init();
    void Blink(int numBlinks, int onTime, int offTime); // [ms]
    void BlinkForever(int intervalMS = 500);
    void TurnOn() { gpio_set_level(led, false); } // Led is on when the level is false.
    void TurnOff() { gpio_set_level(led, true); }
};

#endif /* LED_HPP */