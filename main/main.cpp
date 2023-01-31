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
#include "sbus.hpp"

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
        LoopForever();
    }

    Sbus sbus;
    if (!sbus.Init())
    {
        printf("%s", "Sbus Error!\n");
        LoopForever();
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

        //baro.Update(dt);
        //baro.PrintAltVs();

        if (sbus.Read())
        {
            Sbus::SbusData sbusData = sbus.GetData();
            //sbus.PrintData();
            uint16_t ch0_raw = sbusData.ch[0];
            float ch0 = sbus.GetAnalog(1, -1.0f, 1.0f); // add yaw,rc deadband around zero. add filter.
            int arm = sbus.GetSwitch2Pos(5);
            int another = sbus.GetSwitch3Pos(6);
            uint16_t throttle_raw = sbusData.ch[2];
            float throttle = sbus.GetAnalog(3, 0.0f, 1.0f);
            printf("%d %f %d %d %d %f\n", ch0_raw, ch0, arm, another, throttle_raw, throttle);
        }
    }
}