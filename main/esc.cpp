/*******************************************************************************************
 *                                                                                         *
 *    ESP32-FC                                                                             *
 *    ESC PWM (OneShot125)                                                                 *
 *                                                                                         *
 *    Copyright (c) 2023 Onur AKIN <https://github.com/onurae>                             *
 *    Licensed under the MIT License.                                                      *
 *                                                                                         *
 ******************************************************************************************/

#include "esc.hpp"

Esc::Esc(EscOperator* escOperator, gpio_num_t pwmPin) : pwmPin(pwmPin)
{
    // Create comparator from the operator
    comparatorConfig.flags.update_cmp_on_tez = true;
    ESP_ERROR_CHECK(mcpwm_new_comparator(escOperator->GetHandle(), &comparatorConfig, &comparator));

    // Create generator from the operator
    generatorConfig.gen_gpio_num = pwmPin;
    ESP_ERROR_CHECK(mcpwm_new_generator(escOperator->GetHandle(), &generatorConfig, &generator));

    // Set initial compare value
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, initialValue)); // Ex. 125 us, 1000 ticks.

    // High on counter empty
    mcpwm_gen_timer_event_action_t eventActTimer;
    eventActTimer.direction = MCPWM_TIMER_DIRECTION_UP;
    eventActTimer.event = MCPWM_TIMER_EVENT_EMPTY;
    eventActTimer.action = MCPWM_GEN_ACTION_HIGH;
    mcpwm_gen_timer_event_action_t eventActTimerEnd;
    eventActTimerEnd.event = MCPWM_TIMER_EVENT_INVALID;
    ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_timer_event(generator, eventActTimer, eventActTimerEnd));

    // Low on compare threshold
    mcpwm_gen_compare_event_action_t eventAct;
    eventAct.direction = MCPWM_TIMER_DIRECTION_UP;
    eventAct.comparator = comparator;
    eventAct.action = MCPWM_GEN_ACTION_LOW;
    mcpwm_gen_compare_event_action_t eventActEnd;
    eventActEnd.comparator = nullptr;
    ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_compare_event(generator, eventAct, eventActEnd));
}

bool Esc::Update(uint16_t value)
{
    if (mcpwm_comparator_set_compare_value(comparator, value) == ESP_OK) // 1/8 us per tick(value: [1000-2000] = [125-250]us)
    {
        return true;
    }
    return false;
}