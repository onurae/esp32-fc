/*******************************************************************************************
 *                                                                                         *
 *    ESP32-FC                                                                             *
 *                                                                                         *
 *    Copyright (c) 2025 Onur AKIN <https://github.com/onurae>                             *
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
#include "timer.hpp"
static const char *TAG = "Main";

#define MEASURE_EXECUTION_TIME 0    // Set it zero before flight.
static const uint16_t freq = 400;   // Main loop frequency [Hz]
static TaskHandle_t mainTaskHandle = nullptr;
static bool IRAM_ATTR timer_isr(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(mainTaskHandle, &xHigherPriorityTaskWoken);
    return xHigherPriorityTaskWoken == pdTRUE; // If true is returned, it will call portYIELD_FROM_ISR()
}

extern "C" void MainTask(void* parameter)
{
    PrintCountDown("Main starting in", 10);

    Timer timer; // Periodic timer
    timer.Init(timer_isr, freq);

    Led led; // On-board Led
    led.Init();
    led.TurnOn();

    I2c i2c(I2C_NUM_0, GPIO_NUM_19, GPIO_NUM_23); // Port, SCL, SDA
    i2c.Init();   // Initialize I2C

    Ahrs ahrs(&i2c);    // Attitude and Heading Reference System
    if (!ahrs.Init())
    {
        ESP_LOGI(TAG, "Ahrs Error!");
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
        ESP_LOGI(TAG, "Baro Error!");
        led.BlinkForever(); // Do not proceed.
    }

    Sbus sbus;
    sbus.Init();
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
    bool missedDeadline = false;                    // Missed deadline flag.
    float dt = 0.0f;                                // Time step [s]
    const int delta = 1000000 / freq;               // Time step [us]
    ahrs.Update(0.0f);                              // Warmup ahrs.
    baro.Update(0.0f);                              // Warmup baro.
    timer.Start();                                  // Start timer.
    int64_t pTime = esp_timer_get_time();           // Previous time.
    while (true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        int64_t sTime = esp_timer_get_time();
        int64_t dTime = sTime - pTime;
        pTime = sTime;
        if (dTime > delta && dt != 0.0f)
        {
            missedDeadline = true;
        }
        dt = dTime * 0.000001f;

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
        float fAlt = baro.GetFilteredAlt();

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

        // Loop Led
        led.BlinkLoop(100, missedDeadline == false ? 2000 : 300);

#if MEASURE_EXECUTION_TIME
        if (dTime != delta)
        {
            ESP_LOGI(TAG, "dTime: %lld us", dTime);
        }
        int64_t duration = esp_timer_get_time() - sTime;
        if (duration > (int64_t)(delta * 0.80f)) // change % value to print execution time.
        {
            ESP_LOGI(TAG, "eTime: %lld us", duration);
        }
        //ESP_LOGI(TAG, "Free stack: %d bytes", uxTaskGetStackHighWaterMark(nullptr) * sizeof(StackType_t));
#endif
    }
}

extern "C" void TelemetryTask(void* parameter)
{
    Battery battery;
    battery.Init();
    ESP_LOGI(TAG, "Battery Voltage: %d", battery.GetVoltage());
    
    Frsky frsky;
    frsky.Init();
    frsky.SetCurrent(0.0f);
    frsky.SetVoltage(battery.GetVoltage() / 1000.0f);
    frsky.Flush();  // Clear telemetry buffer.

    uint32_t currentTick = xTaskGetTickCount();
    uint32_t lastTick = currentTick;
    while(true)
    {
        currentTick = xTaskGetTickCount();
        if ((currentTick - lastTick) >= pdMS_TO_TICKS(1000)) // Every 1 second.
        {
            lastTick = currentTick;
            int v = battery.GetVoltage(); // [mV]
            if (v >= 0 && v < 15000)      // 3S lipo
            {
                frsky.SetVoltage(v / 1000.0f);
            }
        }
        frsky.Operate();
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

extern "C" void app_main(void)
{
    xTaskCreatePinnedToCore(TelemetryTask, "Telemetry Task", 4096, nullptr, 1, nullptr, 1);
    xTaskCreatePinnedToCore(MainTask, "Main Task", 8192, nullptr, 2, &mainTaskHandle, 1);
    vTaskDelete(nullptr);
}