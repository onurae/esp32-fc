/*******************************************************************************************
 *                                                                                         *
 *    ESP32-FC                                                                             *
 *                                                                                         *
 *    Copyright (c) 2022 Onur AKIN <https://github.com/onurae>                             *
 *    Licensed under the MIT License.                                                      *
 *                                                                                         *
 ******************************************************************************************/

#include "led.hpp"
#include "ahrs.hpp"
#include "baro.hpp"
#include "sbus.hpp"
#include "control.hpp"
#include "servo.hpp"
#include "esc.hpp"
#include "battery.hpp"
#include "frsky.hpp"
#include "rom/ets_sys.h"

extern "C" void app_main(void)
{
    PrintCountDown("Starting in", 10);

    Led led; // On-board Led
    led.Init();
    led.TurnOn();

    I2c i2c(I2C_NUM_0, GPIO_NUM_19, GPIO_NUM_23); // Port, SCL, SDA
    i2c.Init();   // Initialize I2C

    const uint16_t freq = 400; // Main loop frequency [Hz]

    Ahrs ahrs(&i2c);    // Attitude and Heading Reference System
    if (!ahrs.Init())
    {
        printf("Ahrs Error!\n");
        led.BlinkForever(); // Do not proceed.
    }
    Wait("Turn the heading to the north for fast sensor fusion convergence.", 3);
    ahrs.Converge();        // Keep it steady.
    led.Blink(5, 100, 100); // Indicates convergence.
    led.TurnOn();

    // ahrs.CalibrateAccGyro();
    // ahrs.CalibrateMag();
    // ahrs.PrintMagForMotionCal(false); // false: raw, true: calibrated. 115200 bps.

    Baro baro(&i2c);
    if (!baro.Init()) // Pressure refresh rate: (freq / 2). Max 50Hz when the main loop freq is 100Hz and above.
    {
        printf("Baro Error!\n");
        led.BlinkForever(); // Do not proceed.
    }

    Battery battery;
    battery.Init();
    printf("Battery Voltage: %d\n", battery.GetVoltage());

    Sbus sbus;
    sbus.Init();
    Frsky frsky;
    frsky.Init();
    led.Blink(5, 100, 100); // Indicates receiver connection.
    led.TurnOn();

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
    Esc esc3(&escOperator2, GPIO_NUM_26);
    Esc esc4(&escOperator2, GPIO_NUM_25);
    escTimer.EnableAndStartTimer();
    Wait("Esc arming...", 3);
    led.Blink(5, 100, 100); // Indicates armed.
    // EscTest(&esc1, &esc2, &esc3, &esc4); // Do not proceed after test.

    Control contr(&sbus);
    contr.Init(freq, &esc1, &esc2, &esc3, &esc4);

    PrintCountDown("Entering loop in", 3);          // Entering loop counter.
    led.Blink(3, 150, 75);                          // Indicates entering loop.
    sbus.Flush();                                   // Clear sbus buffer.
    sbus.WaitForData(2);                            // Wait for first sbus data.
    ahrs.Update(0.0f);                              // Warmup ahrs.
    baro.Update(0.0f);                              // Warmup baro.
    frsky.Flush();                                  // Clear telemetry buffer.
    int iFrsky = 0;                                 // Telemetry counter.
    uint8_t iMissedDeadline = 0;                    // Missed deadline flag.
    float dt = 0;                                   // Time step
    int64_t prevTime = 0;                           // Previous time [us]
    int64_t elapsedTime = 0;                        // Elapsed time [us]
    int64_t currentTime = esp_timer_get_time();     // Current time [us]
    const int64_t loopPeriodUs = 1000000 / freq;    // Loop period [us]
    int64_t nextWakeTime = esp_timer_get_time();    // [us]
    while (true)
    {
        nextWakeTime += loopPeriodUs;
        int64_t now = esp_timer_get_time();
        int64_t delayUs = nextWakeTime - now;
        if (delayUs > 0)
        {
            ets_delay_us(delayUs); // Busy-wait for sub-ms precision
        }
        else
        {
            printf("Deadline missed by %lld us!\n", -delayUs);
            iMissedDeadline = 1;
            nextWakeTime = now; // Reset to avoid drift
        }

        prevTime = currentTime;
        currentTime = esp_timer_get_time();   // [us]
        elapsedTime = currentTime - prevTime; // [us]
        dt = elapsedTime / 1000000.0f;        // [s]

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

        baro.Update(dt);
        // baro.PrintAlt();

        if (sbus.Read())
        {
            // sbus.PrintData();
            // sbus.PrintTest();
            contr.UpdateControlInput();
        }
        contr.CheckRxFailure();
        contr.UpdateRefInput(dt);
        // contr.PrintRef();
        contr.UpdateEscCmd(dt,
            ahrs.GetP(), ahrs.GetQ(), ahrs.GetR(),
            ahrs.GetPhi(), ahrs.GetTheta(), ahrs.GetPsi());

        // Telemetry
        iFrsky += 1;
        if (iFrsky >= freq) // Every 1 second.
        {
            iFrsky = 0;
            int v = battery.GetVoltage(); // [mV]
            if (v >= 0 && v < 15000)      // 3S lipo
            {
                frsky.SetVoltage(v / 1000.0f);
            }
        }
        frsky.Operate();

        // Loop Led
        led.BlinkLoop(100, iMissedDeadline == 0 ? 2000 : 500);

        // int64_t workTime = esp_timer_get_time(); // [us]
        // printf("w: %d\n", (uint16_t)(workTime - currentTime));
    }
}