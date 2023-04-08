/*******************************************************************************************
 *                                                                                         *
 *    ESP32-FC                                                                             *
 *    Servo PWM                                                                            *
 *                                                                                         *
 *    Copyright (c) 2023 Onur AKIN <https://github.com/onurae>                             *
 *    Licensed under the MIT License.                                                      *
 *                                                                                         *
 ******************************************************************************************/

#ifndef SERVO_HPP
#define SERVO_HPP

#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"
#include "util.hpp"

class ServoTimer
{
private:
    mcpwm_timer_handle_t timer;
    mcpwm_timer_config_t timerConfig = {};

public:
    ServoTimer(int groupId)
    {
        timerConfig.group_id = groupId;
        timerConfig.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
        timerConfig.resolution_hz = 1000000; // 1MHz, 1 us per tick
        timerConfig.period_ticks = 20000;    // 20000 ticks, 20 ms
        timerConfig.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
        ESP_ERROR_CHECK(mcpwm_new_timer(&timerConfig, &timer));
    };
    virtual ~ServoTimer() = default;
    mcpwm_timer_handle_t GetTimerHandle() { return timer; }
    int GetGroupId() { return timerConfig.group_id; }
    void EnableAndStartTimer()
    {
        ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
        ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
    }
};

class ServoOperator
{
private:
    mcpwm_oper_handle_t oper;
    mcpwm_operator_config_t operatorConfig = {};

public:
    ServoOperator(ServoTimer *servoTimer)
    {
        operatorConfig.group_id = servoTimer->GetGroupId();
        ESP_ERROR_CHECK(mcpwm_new_operator(&operatorConfig, &oper));                       // Create operator, it must be in the same group to the timer
        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, servoTimer->GetTimerHandle())); // Connect timer and operator
    }
    mcpwm_oper_handle_t GetHandle() { return oper; }
};

class Servo
{
private:
    ServoOperator *servoOper = nullptr;
    gpio_num_t pwmPin;
    int minPulseWidth;
    int maxPulseWidth;
    int minAngleDeg;
    int maxAngleDeg;
    mcpwm_cmpr_handle_t comparator;
    mcpwm_gen_handle_t generator;
    mcpwm_comparator_config_t comparatorConfig = {};
    mcpwm_generator_config_t generatorConfig = {};

public:
    Servo(ServoOperator *oper, gpio_num_t pwmPin, uint16_t minPulseWidth, uint16_t maxPulseWidth, int minAngleDeg, int maxAngleDeg, uint16_t initialPulseWidth = 1500);
    virtual ~Servo() = default;
    bool Update(int angleDeg); // [deg]
};

void ServoTest(Servo* servo1, Servo* servo2, Servo* servo3, Servo* servo4);

#endif /* SERVO_HPP */