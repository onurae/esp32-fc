/*******************************************************************************************
 *                                                                                         *
 *    ESP32-FC                                                                             *
 *    TIMER                                                                                *
 *                                                                                         *
 *    Copyright (c) 2025 Onur AKIN <https://github.com/onurae>                             *
 *    Licensed under the MIT License.                                                      *
 *                                                                                         *
 ******************************************************************************************/

#ifndef TIMER_HPP
#define TIMER_HPP

#include <driver/gptimer_types.h>
#include <driver/gptimer.h>

class Timer
{
private:
    gptimer_handle_t gptimer = nullptr;

public:
    Timer() = default;
    virtual ~Timer() = default;

    void Init(gptimer_alarm_cb_t onAlarm, int freq);
    void Start();
};

#endif /* TIMER_HPP */