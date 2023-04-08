/*******************************************************************************************
 *                                                                                         *
 *    ESP32-FC                                                                             *
 *    Servo PWM                                                                            *
 *                                                                                         *
 *    Copyright (c) 2023 Onur AKIN <https://github.com/onurae>                             *
 *    Licensed under the MIT License.                                                      *
 *                                                                                         *
 ******************************************************************************************/

#include "Servo.hpp"

Servo::Servo(ServoOperator* servoOperator, gpio_num_t pwmPin, uint16_t minPulseWidth, uint16_t maxPulseWidth, int minAngleDeg, int maxAngleDeg, uint16_t initialPulseWidth) :
    pwmPin(pwmPin), minPulseWidth(minPulseWidth), maxPulseWidth(maxPulseWidth), minAngleDeg(minAngleDeg), maxAngleDeg(maxAngleDeg)
{
    // Create comparator from the operator
    comparatorConfig.flags.update_cmp_on_tez = true;
    ESP_ERROR_CHECK(mcpwm_new_comparator(servoOperator->GetHandle(), &comparatorConfig, &comparator));

    // Create generator from the operator
    generatorConfig.gen_gpio_num = pwmPin;
    ESP_ERROR_CHECK(mcpwm_new_generator(servoOperator->GetHandle(), &generatorConfig, &generator));

    // Set initial compare value
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, initialPulseWidth)); // Ex. 1500 us, 1500 ticks.

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

bool Servo::Update(int angleDeg)
{
    uint32_t pulseWidth = (angleDeg - minAngleDeg) * (maxPulseWidth - minPulseWidth) / (maxAngleDeg - minAngleDeg) + minPulseWidth;
    if (mcpwm_comparator_set_compare_value(comparator, pulseWidth) == ESP_OK) // 1 us per tick
    {
        return true;
    }
    return false;
}

void ServoTest(Servo* servo1, Servo* servo2, Servo* servo3, Servo* servo4)
{
    int angle = 0;
    int step = 2;
    while(true)
    {
        servo1->Update(angle);
        servo2->Update(angle * -1.0f);
        servo3->Update(angle * 0.5f);
        servo4->Update(angle * -0.5f);
        printf("%d\n", angle);
        // Wait for servo to rotate, ex: @5V, 0.10s/60degree at no load.
        vTaskDelay(500 / portTICK_PERIOD_MS);
        if ((angle + step) > 45 || (angle + step) < -45)
        {
            step *= -1;
        }
        angle += step;
    }
}