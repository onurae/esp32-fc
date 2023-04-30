/*******************************************************************************************
 *                                                                                         *
 *    ESP32-FC                                                                             *
 *                                                                                         *
 *    Copyright (c) 2022 Onur AKIN <https://github.com/onurae>                             *
 *    Licensed under the MIT License.                                                      *
 *                                                                                         *
 ******************************************************************************************/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "led.hpp"
#include "ahrs.hpp"
#include "baro.hpp"
#include "sbus.hpp"
#include "control.hpp"
#include "servo.hpp"
#include "esc.hpp"
#include "battery.hpp"
#include "frsky.hpp"

extern "C" void Task3(void *params)
{
    printf("Main loop led task has been started!\n");
    size_t size = xPortGetFreeHeapSize();
    printf("FreeHeapSize: ");
    printf("%d\n", size);

    Led *p = (Led *)params;
    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(2));
        p->Blink(1, 100, 2000);
    }
}

extern "C" void Task2(void *params)
{
    printf("Telemetry task has been started!\n");
    size_t size = xPortGetFreeHeapSize();
    printf("FreeHeapSize: ");
    printf("%d\n", size);

    Battery battery;
    battery.Init();
    printf("Battery Voltage: %d\n", battery.GetVoltage());
    Frsky frsky;
    frsky.Init();
    frsky.Flush();

    int i = 0;
    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(2));
        i += 1;
        if (i > 500) // Approximately 500*2 = 1 second.
        {
            i = 0;
            int v = battery.GetVoltage(); // [mV]
            if (v >= 0 && v < 15000)      // 3S lipo
            {
                frsky.SetVoltage(v / 1000.0f);
            }
            // Sometimes Tx shows an unexpected voltage value.
            // If it happens, delete sensor and discover again.
        }
        frsky.Operate();
    }
}

extern "C" void Task1(void *params)
{
    printf("Main task has been started!\n");
    size_t size = xPortGetFreeHeapSize();
    printf("FreeHeapSize: ");
    printf("%d\n", size);

    Led led; // On-board Led
    led.Init();
    led.TurnOn();

    I2c i2c(I2C_NUM_0, 19, 23); // Port, SCL, SDA
    i2c.MasterInit();           // Initialize I2C

    const uint16_t freq = 500; // Main loop frequency [Hz]

    Ahrs ahrs(&i2c);          // Attitude and Heading Reference System
    if (!ahrs.Init(freq * 2)) // Sensor sample rate: (freq*2)
    {
        printf("%s", "Ahrs Error!\n");
        led.BlinkForever(); // Do not proceed.
    }
    Wait("Turn the heading to the north for fast sensor fusion convergence.", 3);
    ahrs.Converge(freq);    // Keep it steady.
    led.Blink(5, 100, 100); // Indicates convergence.
    led.TurnOn();

    // ahrs.CalibrateAccGyro();
    // ahrs.CalibrateMag();
    // ahrs.PrintMagForMotionCal(false); // false: raw, true: calibrated. 115200 bps.

    Baro baro(&i2c);
    if (!baro.Init()) // Pressure refresh rate: (freq / 2). Max 50Hz when the main loop freq is 100Hz and above.
    {
        printf("%s", "Baro Error!\n");
        led.BlinkForever(); // Do not proceed.
    }

    Sbus sbus;
    sbus.Init();
    led.Blink(5, 100, 100); // Indicates connected.
    led.TurnOn();
    Control contr(&sbus);
    contr.Init(freq);

    // ServoTimer servoTimer(0);
    // ServoOperator servoOperator1(&servoTimer);
    // Servo servo1(&servoOperator1, GPIO_NUM_27, 1000, 2000, -45, +45, 1200);
    // Servo servo2(&servoOperator1, GPIO_NUM_26, 1000, 2000, -45, +45, 1500);
    // ServoOperator servoOperator2(&servoTimer);
    // Servo servo3(&servoOperator2, GPIO_NUM_25, 1000, 2000, -45, +45, 1700);
    // Servo servo4(&servoOperator2, GPIO_NUM_33, 1000, 2000, -45, +45, 1400);
    // servoTimer.EnableAndStartTimer();
    // ServoTest(&servo1, &servo2, &servo3, &servo4);

    PrintCountDown("Esc arming in", 3);
    EscTimer escTimer(1);
    EscOperator escOperator1(&escTimer);
    Esc esc1(&escOperator1, GPIO_NUM_33);
    Esc esc2(&escOperator1, GPIO_NUM_27);
    EscOperator escOperator2(&escTimer);
    Esc esc3(&escOperator2, GPIO_NUM_25);
    Esc esc4(&escOperator2, GPIO_NUM_26);
    escTimer.EnableAndStartTimer();
    Wait("Esc arming...", 3);
    bool isArmed = true;
    led.Blink(5, 100, 100); // Indicates armed.
    // EscTest(&esc1, &esc2, &esc3, &esc4); // Do not proceed after test.

    xTaskCreate(Task2, "TelemetryTask", 4096, NULL, 1, NULL);   // Low priority.
    xTaskCreate(Task3, "MainLoopLedTask", 4096, &led, 0, NULL); // Very low priority.
    Wait("Task initialization...", 1);                          // Task2 & Task3 initializing...
    PrintCountDown("Entering loop in", 3);                      // Entering loop counter.
    led.Blink(3, 150, 75);                                      // Indicates entering loop.
    sbus.Flush();                                               // Clear sbus buffer.
    sbus.WaitForData(2);                                        // Wait for first sbus data.
    BaseType_t xWasDelayed;                                     // Deadline missed or not
    float dt = 0;                                               // Time step
    int64_t prevTime = 0;                                       // Previous time [us]
    int64_t elapsedTime = 0;                                    // Elapsed time [us]
    int64_t currentTime = esp_timer_get_time();                 // Current time [us]
    const int loopTime = 1000 / freq;                           // Loop time [ms]
    TickType_t xLastWakeTime = xTaskGetTickCount();             // Last wake time
    while (true)
    {
        xWasDelayed = xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(loopTime)); // Wait for the next cycle.
        prevTime = currentTime;
        currentTime = esp_timer_get_time();   // [us]
        elapsedTime = currentTime - prevTime; // [us]
        dt = elapsedTime / 1000000.0f;        // [s]
        // printf("%d\n", (uint16_t)(elapsedTime));
        if (xWasDelayed == pdFALSE)
        {
            printf("%s", "Deadline missed!\n");
        }

        ahrs.Update(dt);
        // ahrs.PrintAccRaw();
        // ahrs.PrintAccCalibrated();
        // ahrs.PrintGyroRaw();
        // ahrs.PrintGyroCalibrated();
        // ahrs.PrintMagRaw();
        // ahrs.PrintMagCalibrated();
        // ahrs.PrintTemp();
        // ahrs.PrintQuaternions();
        // ahrs.PrintEulerAngles();

        // baro.Update(dt);
        // baro.PrintAltVs();

        if (sbus.Read())
        {
            // sbus.PrintData();
            // sbus.PrintTest();
            contr.UpdateControlInput();
        }
        contr.CheckRxFailure();
        contr.UpdateRefInput(dt);
        // contr.PrintRef();

        /*
        if (radio.ch5_2po == 0) // Disarm
        {
            isArmed = false;
            esc1.Update(1000);
            esc2.Update(1000);
            esc3.Update(1000);
            esc4.Update(1000);
            // reset pid etc..
        }
        else if (radio.ch5_2po == 1 && radio.ch3_thr < 0.005f) // Arm
        {
            isArmed = true;
        }

        if (isArmed)
        {
            // PID
            esc1.Update(radio.ch3_thr * 1000 + 1000);
            esc2.Update(radio.ch3_thr * 1000 + 1000);
            esc3.Update(radio.ch3_thr * 1000 + 1000);
            esc4.Update(radio.ch3_thr * 1000 + 1000);
        }
        */

        // int64_t workTime = esp_timer_get_time(); // [us]
        // printf("w: %d\n", (uint16_t)(workTime - currentTime));
    }
}

extern "C" void app_main(void)
{
    PrintCountDown("Starting in", 10);
    xTaskCreate(Task1, "MainTask", 8192, NULL, 2, NULL); // High priority, periodic task.
    while (true)                                         // is this necessary?
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}