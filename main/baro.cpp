/*******************************************************************************************
 *                                                                                         *
 *    ESP32-FC                                                                             *
 *    Barometric Pressure Sensor (MS5611)                                                  *
 *                                                                                         *
 *    Copyright (c) 2022 Onur AKIN <https://github.com/onurae>                             *
 *    Licensed under the MIT License.                                                      *
 *                                                                                         *
 ******************************************************************************************/

#include "baro.hpp"

bool Baro::Init()
{
    // I2C
    i2c->AddDevice(addressMS5611, &msHandle);
    
    // Reset
    if (i2c->Write(&msHandle, 0x1E) != ESP_OK)
    {
        printf("MS5611 not found!\n");
        return false;
    }
    printf("MS5611 found.\n");
    vTaskDelay(10 / portTICK_PERIOD_MS);

    for (int i = 0; i < 6; i++) // Read calibration data
    {
        uint8_t buf[2];
        ESP_ERROR_CHECK(i2c->Read(&msHandle, 0xA2 + (i * 2), 2, buf));
        cal[i] = ((uint16_t)buf[0]) << 8 | (uint16_t)buf[1];
    }
    printf(" C1: %d\n C2: %d\n C3: %d\n C4: %d\n C5: %d\n C6: %d\n", cal[0], cal[1], cal[2], cal[3], cal[4], cal[5]);
    tRef = (int64_t)cal[4] << 8;
    tOff1 = (int64_t)cal[1] << 16;
    tSens1 = (int64_t)cal[0] << 15;

    if (SendConvCmdTemp() == false) // Send temperature command
    {
        printf("MS5611 temperature command error.\n");
        return false;
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
    if (ReadTemperature() == false) // Read digital temperature value
    {
        printf("MS5611 temperature read error.\n");
        return false;
    }
    if (SendConvCmdPres() == false) // Send pressure command
    {
        printf("MS5611 pressure command error.\n");
        return false;
    }
    lastConvTick = xTaskGetTickCount();
    vTaskDelay(10 / portTICK_PERIOD_MS);
    CalculatePresTempAlt(); // Read digital pressure value, calculate pressure, temperature and altitude.
    altfp = alt;
    altf = alt;
    PrintPresTemp();
    PrintAlt();
    printf("MS5611 ready.\n");
    return true;
}

void Baro::Update(float dt)
{
    CalculatePresTempAlt();
    CalculateFilteredAlt(dt);
}

bool Baro::SendConvCmdPres()
{
    if (i2c->Write(&msHandle, addDpt[0]) != ESP_OK)
    {
        return false;
    }
    return true;
}

bool Baro::SendConvCmdTemp()
{
    if (i2c->Write(&msHandle, addDpt[1]) != ESP_OK)
    {
        return false;
    }
    return true;
}

bool Baro::ReadPressure()
{
    uint8_t buf[3];
    if (i2c->Read(&msHandle, 0x00, 3, buf) != ESP_OK)
    {
        return false;
    }
    dpt[0] = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | (uint32_t)buf[2];
    if (dpt[0] == 0) // if loop is fast, it returns 0.
    {
        return false;
    }
    return true;
}

bool Baro::ReadTemperature()
{
    uint8_t buf[3];
    if (i2c->Read(&msHandle, 0x00, 3, buf) != ESP_OK)
    {
        return false;
    }
    dpt[1] = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | (uint32_t)buf[2];
    if (dpt[1] == 0) // if loop is fast, it returns 0.
    {
        return false;
    }
    return true;
}

bool Baro::ReadPresTemp()
{
    uint32_t currentTick = xTaskGetTickCount();
    if ((currentTick - lastConvTick) < pdMS_TO_TICKS(10))
    {
        return false;
    }
    lastConvTick = currentTick;

    // If the sample rate is 100Hz, pressure & temperature refresh rate would be 50Hz.
    if (state == false)
    {
        bool pState = ReadPressure();
        bool tState = SendConvCmdTemp();
        if (pState && tState)
        {
            state = true;
            return true;
        }
    }
    else
    {
        bool tState = ReadTemperature();
        bool pState = SendConvCmdPres();
        if (tState && pState)
        {
            state = false;
            return true;
        }
    }
    return false;
}

void Baro::CalculatePresTempAlt()
{
    if (ReadPresTemp() == true)
    {
        tDiff = (int64_t)dpt[1] - tRef;
        tAct = 2000 + ((tDiff * cal[5]) >> 23);
        tOff = tOff1 + ((tDiff * cal[3]) >> 7);
        tSens = tSens1 + ((tDiff * cal[2]) >> 8);

        if (tAct < 2000) // Temperature < 20°C
        {
            tAct2 = (tDiff * tDiff) >> 31;
            int64_t x = tAct - 2000;
            x = 5 * x * x;
            tOff2 = x >> 1;
            tSens2 = x >> 2;
            if (tAct < -1500) // Temperature < -15°C
            {
                x = tAct + 1500;
                x = x * x;
                tOff2 += 7 * x;
                tSens2 += ((11 * x) >> 2);
            }
            tAct -= tAct2;
            tOff -= tOff2;
            tSens -= tSens2;
        }

        temperature = tAct / 100.0f;
        tCompPres = (((((int64_t)dpt[0] * tSens) >> 21) - tOff) >> 15);
        pressure = tCompPres / 100.0f;
        alt = (((tsl + 273.15f) / -0.0065f) * (std::pow(pressure / qnh, 0.1902632f) - 1.0f));
    }
}

float Baro::CalculateAlpha(float f, float dt)
{
    if (f < 0.0f)
    {
        return 1.0f;
    }
    float omega = f * 2.0f * M_PI;
    return (omega * dt / (1.0f + omega * dt));
}

void Baro::CalculateFilteredAlt(float dt)
{
    float altAlpha = CalculateAlpha(fCutAlt, dt);
    altf = (1.0f - altAlpha) * altfp + altAlpha * alt; // Filtered altitude
    altfp = altf;
}

void Baro::PrintPresTemp()
{
    printf("pres: %.2f, temp: %.2f", pressure, temperature);
}

void Baro::PrintAlt()
{
    printf("alt: %.2f, altf: %.2f\n", alt, altf);
}