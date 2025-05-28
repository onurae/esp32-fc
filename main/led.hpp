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
#include "esp_timer.h"
#include "esp_log.h"

class Led
{
private:
    gpio_num_t led;
    bool state = false; // ON: true, OFF: false

public:
    Led(gpio_num_t led = GPIO_NUM_22) { this->led = led; }
    virtual ~Led() = default;

    void Init();
    void Blink(int numBlinks, int onTime, int offTime); // [ms]
    void BlinkForever(int intervalMS = 500);
    void TurnOn();
    void TurnOff();
    void Flip();

    int64_t currentTime = 0;                 // Current time [us]
    int64_t prevTime = 0;                    // Previous time [us]
    int64_t delay = 0;                       // Delay [ms]
    void BlinkLoop(int onTime, int offTime); // [ms]
};

#endif /* LED_HPP */