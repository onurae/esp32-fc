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
#include "servo.hpp"
#include "esc.hpp"
#include "battery.hpp"
#include "frsky.hpp"

extern "C" void Task2(void *params)
{
    printf("Telemetry task has been started!\n");

    size_t size = xPortGetFreeHeapSize();
    printf("FreeHeapSize: ");
    printf("%d\n", size);

    // int *p = (int *)params;
    // gpio_num_t led = GPIO_NUM_22;
    // gpio_set_direction(led, GPIO_MODE_OUTPUT);
    // int isOn = 0;

    Battery battery;
    battery.Init();
    //printf("Battery Voltage: %d\n", battery.GetVoltage());

    Frsky frsky;
    frsky.Init();
    frsky.Flush();

    int64_t currentTime = esp_timer_get_time();
    int i = 0;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (true)
    {
        int64_t workTime = esp_timer_get_time(); // [us]
        // printf("w: %d\n", (uint16_t)(workTime - currentTime));
        vTaskDelay(pdMS_TO_TICKS(2));
        currentTime = esp_timer_get_time(); // [us]

        // printf("Task1 is running!\n");
        // isOn = !isOn;
        // gpio_set_level(led, isOn);
        i += 1;
        if (i > 500) // approximately 500*2 = 1 seconds.
        {
            frsky.SetVoltage(battery.GetVoltage() / 1000.0f);
            i = 0;
        }
        frsky.Operate();
    }
    // vTaskDelete( NULL );
}

extern "C" void Task1(void *params)
{
    printf("Main task has been started!\n");

    Led led;
    led.Init();

    size_t size = xPortGetFreeHeapSize();
    printf("FreeHeapSize: ");
    printf("%d\n", size);

    I2c i2c(I2C_NUM_0, 19, 23); // Port, SCL, SDA
    i2c.MasterInit();   // Initialize I2C

    uint16_t freq = 500; // Main loop frequency [Hz]

    Ahrs ahrs(&i2c);
    if (!ahrs.Init(freq*2)) // Sensor sample rate: (freq*2)
    {
        printf("%s", "Ahrs Error!\n");
        led.BlinkForever();
    }

    // ahrs.CalibrateAccGyro();
    // ahrs.CalibrateMag();
    // ahrs.PrintMagForMotionCal(false); // false: raw, true: calibrated. 115200 bps.

    Baro baro(&i2c);
    if (!baro.Init()) // Pressure refresh rate: (freq / 2). Max 50Hz when the main loop freq is 100Hz and above.
    {
        printf("%s", "Baro Error!\n");
        led.BlinkForever();
    }
    
    //Sbus sbus;
    //sbus.Init();

    // ServoTimer servoTimer(0);
    // ServoOperator servoOperator1(&servoTimer);
    // Servo servo1(&servoOperator1, GPIO_NUM_27, 1000, 2000, -45, +45, 1200);
    // Servo servo2(&servoOperator1, GPIO_NUM_26, 1000, 2000, -45, +45, 1500);
    // ServoOperator servoOperator2(&servoTimer);
    // Servo servo3(&servoOperator2, GPIO_NUM_25, 1000, 2000, -45, +45, 1700);
    // Servo servo4(&servoOperator2, GPIO_NUM_33, 1000, 2000, -45, +45, 1400);
    // servoTimer.EnableAndStartTimer();
    // ServoTest(&servo1, &servo2, &servo3, &servo4);

    //PrintCountDown("Esc arming in", 3);
    //EscTimer escTimer(1);
    //EscOperator escOperator1(&escTimer);
    //Esc esc1(&escOperator1, GPIO_NUM_33);
    //Esc esc2(&escOperator1, GPIO_NUM_27);
    //EscOperator escOperator2(&escTimer);
    //Esc esc3(&escOperator2, GPIO_NUM_25);
    //Esc esc4(&escOperator2, GPIO_NUM_26);
    //escTimer.EnableAndStartTimer();
    //Wait("Esc arming...", 3);
    // EscTest(&esc1, &esc2, &esc3, &esc4);

    {
        bool isConverged = false;
        TickType_t xLastWakeTime = xTaskGetTickCount();
        while(isConverged == false)
        {
            xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000 / freq));
            ahrs.Update((1000 / freq) * 0.001f);
            ahrs.PrintEulerAngles();
            if (false)
            {
                isConverged = true;
            }
        }
    }
    
    PrintCountDown("Entering loop in", 3);
    int64_t prevTime = esp_timer_get_time();
    int64_t currentTime = prevTime;
    float dt = 0;

    //xTaskCreate(Task2, "mySecondTask", 4096, NULL, 0, NULL); // Low priority.
    //sbus.Flush();
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (true)
    {
        // int64_t workTime = esp_timer_get_time();         // [us]
        // printf("w: %d\n", (uint16_t)(workTime - currentTime));

        // Wait for the next cycle.
        BaseType_t xWasDelayed = xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000 / freq));
        prevTime = currentTime;
        currentTime = esp_timer_get_time();         // [us]
        dt = (currentTime - prevTime) / 1000000.0f; // [s]
        //int a = ((currentTime - prevTime) != 2000) ? printf("dt: %d\n", (uint16_t)(currentTime - prevTime)) : 0;
        if (xWasDelayed == pdFALSE)
        {
            printf("%s", "Deadline missed!\n");
            // led.BlinkForever();
        }

        ahrs.Update(dt);
        //  ahrs.PrintAccRaw();
        //  ahrs.PrintAccCalibrated();
        //  ahrs.PrintGyroRaw();
        //  ahrs.PrintGyroCalibrated();
        //  ahrs.PrintMagRaw();
        //  ahrs.PrintMagCalibrated();
        //  ahrs.PrintTemp();
        //  ahrs.PrintQuaternions();
        // ahrs.PrintEulerAngles();

        baro.Update(dt);
        // baro.PrintAltVs();

        //if (sbus.Read())
        //{
        //    // sbus.PrintData();
        //    //  sbus.PrintTest();
        //    float ch1 = sbus.GetAnalog(1, -1.0f, 1.0f);
        //}
        // printf("%d\n", sbus.CheckStatus());

        //bool throttleCut = false;
        //if (throttleCut == true)
        //{
        //    esc1.Update(1000);
        //    esc2.Update(1000);
        //    esc3.Update(1000);
        //    esc4.Update(1000);
        //}
    }
}

extern "C" void app_main(void)
{
    PrintCountDown("Starting in", 10);
    xTaskCreate(Task1, "myFirstTask", 4096, NULL, 1, NULL); // High priority, periodic task.
    // while ?
}