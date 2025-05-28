/*******************************************************************************************
 *                                                                                         *
 *    ESP32-FC                                                                             *
 *    TIMER                                                                                *
 *                                                                                         *
 *    Copyright (c) 2025 Onur AKIN <https://github.com/onurae>                             *
 *    Licensed under the MIT License.                                                      *
 *                                                                                         *
 ******************************************************************************************/

#include "timer.hpp"

void Timer::Init(gptimer_alarm_cb_t onAlarm, int freq)
{
    gptimer_config_t timer_config = {};
    timer_config.clk_src = GPTIMER_CLK_SRC_DEFAULT;
    timer_config.direction = GPTIMER_COUNT_UP;
    timer_config.resolution_hz = 10 * 1000 * 1000; // 10MHz, 1 tick = 0.1us
    timer_config.intr_priority = 1;
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));

    gptimer_event_callbacks_t cbs = {};
    cbs.on_alarm = onAlarm;
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, nullptr));
    ESP_ERROR_CHECK(gptimer_enable(gptimer));

    gptimer_alarm_config_t alarm_config = {};
    alarm_config.reload_count = 0; // counter will reload with 0 on alarm event
    alarm_config.alarm_count = timer_config.resolution_hz / freq; // period in us @resolution 10MHz
    alarm_config.flags.auto_reload_on_alarm = true; // enable auto-reload
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));
}

void Timer::Start()
{
    ESP_ERROR_CHECK(gptimer_start(gptimer));
}