/*******************************************************************************************
 *                                                                                         *
 *    ESP32-FC                                                                             *
 *    Barometric Pressure Sensor (MS5611)                                                  *
 *                                                                                         *
 *    Copyright (c) 2022 Onur AKIN <https://github.com/onurae>                             *
 *    Licensed under the MIT License.                                                      *
 *                                                                                         *
 ******************************************************************************************/

#ifndef BARO_HPP
#define BARO_HPP

#include "i2c.hpp"
#include "esp_log.h"
#include <cmath>

class Baro
{
private:
    i2c_master_dev_handle_t msHandle;
    I2c *i2c;                           // I2C
    const uint8_t addressMS5611 = 0x77; // CSB: Low
    uint16_t cal[7];                    // Calibration data
    uint32_t dpt[2];                    // Digital pressure & temperature value
    uint8_t addDpt[2] = {0x48, 0x58};   // OCR: 4096. Conversion time: 9.04 [ms] max.
    int64_t tDiff;                      // Difference between actual and reference temperature
    int32_t tAct;                       // Actual temperature
    int64_t tOff;                       // Offset at actual temperature
    int64_t tSens;                      // Sensitivity at actual temperature
    int32_t tCompPres;                  // Temperature compensated pressure
    int64_t tRef;                       // Reference temperature
    int64_t tOff1;
    int64_t tSens1;
    int32_t tAct2;
    int64_t tOff2;
    int64_t tSens2;
    bool SendConvCmdPres();
    bool SendConvCmdTemp();
    bool ReadPressure();
    bool ReadTemperature();
    uint32_t lastConvTick;
    bool ReadPresTemp(); // max 50Hz.
    void CalculatePresTempAlt();
    float temperature = 0; // [°C]
    float pressure = 0;    // [mbar]
    float qnh = 1019.00;   // Mean sea level pressure [mbar]
    float tsl = 15.0;      // Temperature sea level [°C]
    float alt = 0;         // Altitude sea level [m]
    bool state = false;    // false: Pres read, temp cmd. true: Pres cmd, temp read.
    void CalculateFilteredAlt(float dt);
    float CalculateAlpha(float f, float dt);
    float altf = 0;      // Filtered altitude [m]
    float altfp = 0;     // Previous filtered altitude
    float fCutAlt = 0.5; // [Hz]
public:
    Baro(I2c *i2c) { this->i2c = i2c; }
    virtual ~Baro() = default;
    bool Init();
    void Update(float dt);
    float GetFilteredAlt() { return altf; }
    void PrintPresTemp();
    void PrintAlt();
};

#endif /* BARO_HPP */