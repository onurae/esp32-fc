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
#include "servo.hpp"
#include "esc.hpp"

#define SERVOTEST 0
#define ESCTEST 1

extern "C" void app_main(void)
{
    PrintCountDown("Starting in", 10);
    //BlinkLedForever();
    
    I2c i2c(0, 19, 23); // Port, SCL, SDA
    i2c.MasterInit();   // Initialize I2C

    uint16_t freq = 500; // Main loop frequency [Hz]

    Ahrs ahrs(&i2c);
    if (!ahrs.Init(freq)) // Sensor sample rate: (freq)
    {
        printf("%s", "Ahrs Error!\n");
        BlinkLedForever();
    }

    // ahrs.CalibrateAccGyro();
    // ahrs.CalibrateMag();
    // ahrs.PrintMagForMotionCal(false); // false: raw, true: calibrated. 115200 bps.

    Baro baro(&i2c);
    if (!baro.Init()) // Pressure refresh rate: (freq / 2). Max 50Hz when the main loop freq is 100Hz and above.
    {
        printf("%s", "Baro Error!\n");
        BlinkLedForever();
    }

    Sbus sbus;
    sbus.Init();
    while(!sbus.IsReady())
    {
        Wait("Sbus data...", 3);
    }

    //ServoTimer servoTimer(0);
    //ServoOperator servoOperator1(&servoTimer);
    //Servo servo1(&servoOperator1, GPIO_NUM_27, 1000, 2000, -45, +45, 1200);
    //Servo servo2(&servoOperator1, GPIO_NUM_26, 1000, 2000, -45, +45, 1500);
    //ServoOperator servoOperator2(&servoTimer);
    //Servo servo3(&servoOperator2, GPIO_NUM_25, 1000, 2000, -45, +45, 1700);
    //Servo servo4(&servoOperator2, GPIO_NUM_33, 1000, 2000, -45, +45, 1400);
    //servoTimer.EnableAndStartTimer();

#if SERVOTEST
    int angle = 0;
    int step = 2;
#endif // SERVOTEST

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
            // BlinkLedForever();
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

        // baro.Update(dt);
        // baro.PrintAltVs();

        if (sbus.Read())
        {
            sbus.PrintData();
            // sbus.PrintTest();
            float ch1 = sbus.GetAnalog(1, -1.0f, 1.0f);
        }
        //printf("%d\n", sbus.CheckStatus());

#if SERVOTEST
        servo1.Update(angle);
        servo2.Update(angle * -1.0f);
        servo3.Update(angle * 0.5f);
        servo4.Update(angle * -0.5f);
        printf("%d\n", angle);
        // Wait for servo to rotate, ex: @5V, 0.10s/60degree at no load.
        vTaskDelay(pdMS_TO_TICKS(500));
        if ((angle + step) > 45 || (angle + step) < -45)
        {
            step *= -1;
        }
        angle += step;
#endif // SERVOTEST

#if ESCTEST
        PrintCountDown("Esc 1 Start", 3);
        esc1.Update(1200);
        PrintCountDown("Esc 1 Stop", 3);
        esc1.Update(1000);
        PrintCountDown("Esc 2 Start", 3);
        esc2.Update(1200);
        PrintCountDown("Esc 2 Stop", 3);
        esc2.Update(1000);
        PrintCountDown("Esc 3 Start", 3);
        esc3.Update(1200);
        PrintCountDown("Esc 3 Stop", 3);
        esc3.Update(1000);
        PrintCountDown("Esc 4 Start", 3);
        esc4.Update(1200);
        PrintCountDown("Esc 4 Stop", 3);
        esc4.Update(1000);
        BlinkLedForever();
#endif // ESCTEST

        bool throttleCut = false;
        if (throttleCut == true)
        {
            esc1.Update(1000);
            esc2.Update(1000);
            esc3.Update(1000);
            esc4.Update(1000);
        }
    }
}