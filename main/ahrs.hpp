/*******************************************************************************************
 *                                                                                         *
 *    ESP32-FC                                                                             *
 *    Attitude and Heading Reference System (MPU9250)                                      *
 *                                                                                         *
 *    Copyright (c) 2022 Onur AKIN <https://github.com/onurae>                             *
 *    Licensed under the MIT License.                                                      *
 *                                                                                         *
 *    Madgwick, S. (2010)                                                                  *
 *    An efficient orientation filter for inertial and inertial/magnetic sensor arrays.    *
 *    Rep. X-Io Univ. Bristol (UK) 25, 113–118.                                            *
 *                                                                                         *
 ******************************************************************************************/

#ifndef AHRS_HPP
#define AHRS_HPP

#include "i2c.hpp"
#include "util.hpp"
#include <cmath>

class Ahrs
{
private:
    // Calibration data
    float axb = -0.015989; // Accelerometer bias
    float ayb = 0.002342;
    float azb = 0.013717;
    float gxb = -2.426104; // Gyroscope bias
    float gyb = 1.183609;
    float gzb = -0.518570;
    float mxb = 14.12; // Magnetometer bias
    float myb = 14.55;
    float mzb = -37.90;
    float mxs = 0.999; // Magnetometer scale
    float mys = 0.977;
    float mzs = 1.028;

    // I2C
    I2c *i2c;

    // MPU9250
    const uint8_t addressMPU9250 = 0x68; // AD0 Pin: Low
    const uint8_t addressAK8963 = 0x0C;
    uint8_t srd; // Sample rate divider.
    float gRes;
    float aRes;
    float mRes;
    float magXCoeff;
    float magYCoeff;
    float magZCoeff;

    // Raw data
    float axr = 0.0; // Accelerometer
    float ayr = 0.0;
    float azr = 0.0;
    float gxr = 0.0; // Gyroscope
    float gyr = 0.0;
    float gzr = 0.0;
    float mxr = 0.0; // Magnetometer
    float myr = 0.0;
    float mzr = 0.0;
    float tpr = 0.0; // Temperature
    bool ReadAccGyroTemp();
    bool ReadMag();

    // Calibrated data
    float axc = 0.0; // Accelerometer
    float ayc = 0.0;
    float azc = 0.0;
    float gxc = 0.0; // Gyroscope
    float gyc = 0.0;
    float gzc = 0.0;
    float mxc = 0.0; // Magnetometer
    float myc = 0.0;
    float mzc = 0.0;
    void CalculateCalibratedAccGyro();
    void CalculateCalibratedMag();

    // Filtered data
    float axf = 0.0; // Accelerometer
    float ayf = 0.0;
    float azf = 0.0;
    float gxf = 0.0; // Gyroscope
    float gyf = 0.0;
    float gzf = 0.0;
    float mxf = 0.0; // Magnetometer
    float myf = 0.0;
    float mzf = 0.0;
    void CalculateFilteredAccGyroMag(float dt);
    float fCutAcc = 80.0;  // [Hz]
    float fCutGyro = 80.0; // [Hz]
    float fCutMag = -1;    // -1 means no filter applied.
    float CalculateAlpha(float f, float dt);
    float axfp = 0.0; // Previous acc
    float ayfp = 0.0;
    float azfp = 0.0;
    float gxfp = 0.0; // Previous gyro
    float gyfp = 0.0;
    float gzfp = 0.0;
    float mxfp = 0.0; // Previous mag
    float myfp = 0.0;
    float mzfp = 0.0;

    // Fusion
    float beta = 0.041;
    float q0 = 1.0;
    float q1 = 0.0;
    float q2 = 0.0;
    float q3 = 0.0;
    float DegToRad(float deg);
    float RadToDeg(float rad);
    float InvSqrt(float x);
    void Madgwick9(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt);
    void Madgwick6(float gx, float gy, float gz, float ax, float ay, float az, float dt);
    void CalculateEulerAngles();
    float phi;   // Roll
    float theta; // Pitch
    float psi;   // Yaw

public:
    Ahrs(I2c *i2c) { this->i2c = i2c; }
    virtual ~Ahrs() = default;
    bool Init(uint16_t sampleRate);
    void Update(float dt);

    // Calibration
    void CalibrateAccGyro();
    void CalibrateMag();
    void PrintMagForMotionCal(bool cal); // false: raw, true: calibrated. 115200 bps.

    // Print
    void PrintAccRaw();
    void PrintGyroRaw();
    void PrintMagRaw();
    void PrintTemp();
    void PrintAccCalibrated();
    void PrintGyroCalibrated();
    void PrintMagCalibrated();
    void PrintAccGyroBias();
    void PrintMagBiasScale();
    void PrintQuaternions();
    void PrintEulerAngles();
};

#endif /* AHRS_HPP */