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
#include "ahrs.hpp"
#include "baro.hpp"

extern "C" void app_main(void)
{
    I2c i2c(0, 19, 23); // Port, SCL, SDA
    i2c.MasterInit();   // Initialize I2C

    Ahrs ahrs(&i2c);
    uint16_t freq = 500;  // [Hz]
    if (!ahrs.Init(freq)) // Sensor sample rate: (freq)
    {
        printf("%s", "Ahrs Error!\n");
        LoopForever();
    }

    // ahrs.CalibrateAccGyro();
    // ahrs.CalibrateMag();
    // ahrs.PrintMagForMotionCal(false); // false: raw, true: calibrated. 115200 bps.

    Baro baro(&i2c);
    if (!baro.Init()) // Pressure refresh rate: (freq / 2)
    {
       printf("%s", "Baro Error!\n");
       while (true)
       {
           vTaskDelay(1000 / portTICK_PERIOD_MS);
       }
    }

    PrintCountDown("Entering loop in", 3);
    int64_t prevTime = esp_timer_get_time();
    int64_t currentTime = prevTime;
    float dt = 0;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {
        // int64_t workTime = esp_timer_get_time();         // [us]
        // printf("w: %d\n", (uint16_t)(workTime - currentTime));

        // Wait for the next cycle.
        BaseType_t xWasDelayed = xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000 / freq));
        prevTime = currentTime;
        currentTime = esp_timer_get_time();         // [us]
        dt = (currentTime - prevTime) / 1000000.0f; // [s]
        // printf("dt: %d\t", (uint16_t)(currentTime - prevTime));
        if (xWasDelayed == pdFALSE)
        {
            printf("%s", "Deadline missed!\n");
            //LoopForever();
        }

        // ahrs.Update(dt);
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
        baro.PrintAltVs();
    }
}