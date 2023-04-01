/*******************************************************************************************
 *                                                                                         *
 *    ESP32-FC                                                                             *
 *    ESC PWM (OneShot125)                                                                 *
 *                                                                                         *
 *    Copyright (c) 2023 Onur AKIN <https://github.com/onurae>                             *
 *    Licensed under the MIT License.                                                      *
 *                                                                                         *
 ******************************************************************************************/

#ifndef ESC_HPP
#define ESC_HPP

#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"

class EscTimer
{
private:
    mcpwm_timer_handle_t timer;
    mcpwm_timer_config_t timerConfig = {};

public:
    EscTimer(int groupId)
    {
        timerConfig.group_id = groupId;
        timerConfig.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
        timerConfig.resolution_hz = 8000000; // 8MHz, 1/8 us per tick
        timerConfig.period_ticks = 2000;     // 2000 ticks, 250 us
        timerConfig.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
        ESP_ERROR_CHECK(mcpwm_new_timer(&timerConfig, &timer));
    };
    virtual ~EscTimer() = default;
    mcpwm_timer_handle_t GetTimerHandle() { return timer; }
    int GetGroupId() { return timerConfig.group_id; }
    void EnableAndStartTimer()
    {
        ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
        ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
    }
};

class EscOperator
{
private:
    mcpwm_oper_handle_t oper;
    mcpwm_operator_config_t operatorConfig = {};

public:
    EscOperator(EscTimer *escTimer)
    {
        operatorConfig.group_id = escTimer->GetGroupId();
        ESP_ERROR_CHECK(mcpwm_new_operator(&operatorConfig, &oper));                     // Create operator, it must be in the same group to the timer
        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, escTimer->GetTimerHandle())); // Connect timer and operator
    }
    mcpwm_oper_handle_t GetHandle() { return oper; }
};

class Esc
{
private:
    EscOperator *escOper = nullptr;
    gpio_num_t pwmPin;
    mcpwm_cmpr_handle_t comparator;
    mcpwm_gen_handle_t generator;
    mcpwm_comparator_config_t comparatorConfig = {};
    mcpwm_generator_config_t generatorConfig = {};
    uint16_t initialValue = 1000;

public:
    Esc(EscOperator *oper, gpio_num_t pwmPin);
    virtual ~Esc() = default;
    bool Update(uint16_t value); // [1000-2000]
};

#endif /* ESC_HPP */