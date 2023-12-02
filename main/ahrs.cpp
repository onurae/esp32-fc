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

#include "Ahrs.hpp"

bool Ahrs::Init(uint16_t sampleRate)
{
    // Disable filters if the sample rate is below 500Hz.
    if (sampleRate < 500)
    {
        fCutAcc = -1;
        fCutGyro = -1;
        fCutMag = -1;
    }

    // MPU9250
    srd = 1000 / sampleRate - 1; // Calculate sample rate divider.
    uint8_t whoAmI;
    ESP_ERROR_CHECK(i2c->Read(addressMPU9250, 0x75, 1, &whoAmI)); // WhoAmI
    if (whoAmI != 0x71)
    {
        printf("MPU9250 not found!\n");
        return false;
    }
    printf("MPU9250 found.\n");
    ESP_ERROR_CHECK(i2c->Write(addressMPU9250, 0x6B, 0x80)); // Reset MPU9250
    vTaskDelay(100 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(i2c->Write(addressMPU9250, 0x6B, 0x01)); // Clock source
    vTaskDelay(200 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(i2c->Write(addressMPU9250, 0x19, srd));  // Sample rate divider. SAMPLE_RATE = Internal_Sample_Rate / (1 + SMPLRT_DIV)
    ESP_ERROR_CHECK(i2c->Write(addressMPU9250, 0x1A, 0x01)); // Gyro bandwitdh 184Hz, delay 2.9ms, fs 1kHz. Temperature bandwitdh 188Hz, delay 1.9ms.
    ESP_ERROR_CHECK(i2c->Write(addressMPU9250, 0x1B, 0x00)); // Gyro full scale select: +-250 deg/s
    gRes = 250.0 / 32768.0;                                  // +-250 deg/s
    ESP_ERROR_CHECK(i2c->Write(addressMPU9250, 0x1C, 0x00)); // Acc full scale select: +-2 g
    aRes = 2.0 / 32768.0;                                    // +-2 g
    ESP_ERROR_CHECK(i2c->Write(addressMPU9250, 0x1D, 0x01)); // Acc bandwitdh 184Hz, delay 5.8ms, rate 1kHz.
    ESP_ERROR_CHECK(i2c->Write(addressMPU9250, 0x37, 0x02)); // Enable bypass
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // AK8963
    ESP_ERROR_CHECK(i2c->Write(addressAK8963, 0x0A, 0x00)); // Power down
    vTaskDelay(100 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(i2c->Write(addressAK8963, 0x0A, 0x0F)); // Fuse ROM access mode
    vTaskDelay(100 / portTICK_PERIOD_MS);
    uint8_t data[3];
    ESP_ERROR_CHECK(i2c->Read(addressAK8963, 0x10, 3, data)); // Sensitivity adjustment values
    magXCoeff = (float)(data[0] - 128) / 256.0f + 1.0f;
    magYCoeff = (float)(data[1] - 128) / 256.0f + 1.0f;
    magZCoeff = (float)(data[2] - 128) / 256.0f + 1.0f;
    ESP_ERROR_CHECK(i2c->Write(addressAK8963, 0x0A, 0x00)); // Power down
    vTaskDelay(100 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(i2c->Write(addressAK8963, 0x0A, 0x16)); // 100Hz and 16-bit output.
    mRes = 4912.0 / 32760.0;                                // 16-bit resolution
    vTaskDelay(100 / portTICK_PERIOD_MS);
    printf(" MagXCoeff: %f\n MagYCoeff: %f\n MagZCoeff: %f\n", magXCoeff, magYCoeff, magZCoeff);
    printf("MPU9250 configured.\n");
    return true;
}

void Ahrs::Update(float dt)
{
    CalculateCalibratedAccGyro();
    CalculateCalibratedMag();
    CalculateFilteredAccGyroMag(dt);
    Madgwick9(gxf, -gyf, -gzf, -axf, ayf, azf, myf, -mxf, mzf, dt);
    CalculateEulerAngles();
}

void Ahrs::CalculateCalibratedAccGyro()
{
    if (ReadAccGyroTemp())
    {
        axc = axr - axb;
        ayc = ayr - ayb;
        azc = azr - azb;
        gxc = gxr - gxb;
        gyc = gyr - gyb;
        gzc = gzr - gzb;
    }
    else
    {
        printf("AccGyro data not ready\n");
    }
}

void Ahrs::CalculateCalibratedMag()
{
    if (ReadMag())
    {
        mxc = mxs * (mxr - mxb);
        myc = mys * (myr - myb);
        mzc = mzs * (mzr - mzb);
    }
    else
    {
        // printf("Mag data not ready\n");
    }
}

void Ahrs::CalculateFilteredAccGyroMag(float dt)
{
    float aAlpha = CalculateAlpha(fCutAcc, dt);
    float gAlpha = CalculateAlpha(fCutGyro, dt);
    float mAlpha = CalculateAlpha(fCutMag, dt);

    // Accelerometer
    axf = (1.0f - aAlpha) * axfp + aAlpha * axc;
    ayf = (1.0f - aAlpha) * ayfp + aAlpha * ayc;
    azf = (1.0f - aAlpha) * azfp + aAlpha * azc;
    axfp = axf;
    ayfp = ayf;
    azfp = azf;
    // Gyroscope
    gxf = (1.0f - gAlpha) * gxfp + gAlpha * gxc;
    gyf = (1.0f - gAlpha) * gyfp + gAlpha * gyc;
    gzf = (1.0f - gAlpha) * gzfp + gAlpha * gzc;
    gxfp = gxf;
    gyfp = gyf;
    gzfp = gzf;
    // Magnetometer
    mxf = (1.0f - mAlpha) * mxfp + mAlpha * mxc;
    myf = (1.0f - mAlpha) * myfp + mAlpha * myc;
    mzf = (1.0f - mAlpha) * mzfp + mAlpha * mzc;
    mxfp = mxf;
    myfp = myf;
    mzfp = mzf;
}

float Ahrs::CalculateAlpha(float f, float dt)
{
    if (f < 0.0f)
    {
        return 1.0f;
    }
    float omega = f * 2.0f * M_PI;
    return (omega * dt / (1.0f + omega * dt));
}

bool Ahrs::ReadAccGyroTemp()
{
    uint8_t dataReady;
    i2c->Read(addressMPU9250, 0x3A, 1, &dataReady);
    if (dataReady & 0x01)
    {
        uint8_t buffer[14];
        if (i2c->Read(addressMPU9250, 0x3B, 14, buffer) != ESP_OK)
        {
            return false;
        }
        int16_t axCount = ((int16_t)buffer[0] << 8) | buffer[1];
        int16_t ayCount = ((int16_t)buffer[2] << 8) | buffer[3];
        int16_t azCount = ((int16_t)buffer[4] << 8) | buffer[5];
        int16_t tpCount = ((int16_t)buffer[6] << 8) | buffer[7];
        int16_t gxCount = ((int16_t)buffer[8] << 8) | buffer[9];
        int16_t gyCount = ((int16_t)buffer[10] << 8) | buffer[11];
        int16_t gzCount = ((int16_t)buffer[12] << 8) | buffer[13];
        axr = (float)axCount * aRes;
        ayr = (float)ayCount * aRes;
        azr = (float)azCount * aRes;
        tpr = ((float)tpCount) / 333.87f + 21.0f;
        gxr = (float)gxCount * gRes;
        gyr = (float)gyCount * gRes;
        gzr = (float)gzCount * gRes;
        return true;
    }
    return false;
}

bool Ahrs::ReadMag()
{
    uint8_t dataReady;
    i2c->Read(addressAK8963, 0x02, 1, &dataReady);
    if (dataReady & 0x01)
    {
        uint8_t buffer[7];
        if (i2c->Read(addressAK8963, 0x03, 7, buffer) != ESP_OK)
        {
            return false;
        }
        if (!(buffer[6] & 0x08)) // If magnetic sensor is overflown or not. buffer[6]: ST2 register
        {
            int16_t mxCount = ((int16_t)buffer[1] << 8) | buffer[0];
            int16_t myCount = ((int16_t)buffer[3] << 8) | buffer[2];
            int16_t mzCount = ((int16_t)buffer[5] << 8) | buffer[4];
            mxr = (float)mxCount * mRes * magXCoeff;
            myr = (float)myCount * mRes * magYCoeff;
            mzr = (float)mzCount * mRes * magZCoeff;
            return true;
        }
    }
    return false;
}

void Ahrs::CalibrateAccGyro()
{
    ESP_ERROR_CHECK(i2c->Write(addressMPU9250, 0x19, 9)); // 100Hz. SAMPLE_RATE = Internal_Sample_Rate / (1 + SMPLRT_DIV)
    printf("%s", "\nAccelerometer and gyroscope calibration\n");
    printf("%s", "The sensor should be on flat surface! This takes 10 seconds.\n");
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    PrintCountDown("Beginning in", 3);
    printf("%s", "Do not move the sensor!\n");
    float bias[6] = {};
    uint16_t cycle = 0;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (cycle < 1000)
    {
        xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
        ReadAccGyroTemp();
        bias[0] += axr;
        bias[1] += ayr;
        bias[2] += azr;
        bias[3] += gxr;
        bias[4] += gyr;
        bias[5] += gzr;
        cycle++;
    }
    for (int i = 0; i < 6; i++)
    {
        bias[i] = bias[i] / cycle;
    }
    bias[2] -= 1.0f; // Gravity 1g

    axb = bias[0];
    ayb = bias[1];
    azb = bias[2];
    gxb = bias[3];
    gyb = bias[4];
    gzb = bias[5];
    printf("%s", "\nCalibration completed.\n");
    PrintAccGyroBias();

    ESP_ERROR_CHECK(i2c->Write(addressMPU9250, 0x19, srd)); // Set srd.
}

void Ahrs::CalibrateMag()
{
    printf("%s", "\nMagnetometer calibration\n");
    printf("%s", "Rotate the sensor about all axes until complete! This takes 30 seconds.\n");
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    PrintCountDown("Beginning in", 3);
    printf("%s", "Start rotating!\n");
    ReadMag();
    float mMax[3] = {mxr, myr, mzr};
    float mMin[3] = {mxr, myr, mzr};
    uint16_t cycle = 0;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (cycle < 3000)
    {
        xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10)); // 100Hz.
        ReadMag();
        float m[3] = {mxr, myr, mzr};
        for (int i = 0; i < 3; i++)
        {
            if (m[i] > mMax[i])
            {
                mMax[i] = m[i];
            }
            if (m[i] < mMin[i])
            {
                mMin[i] = m[i];
            }
        }
        cycle++;
    }
    printf("mMin = x: %f y: %f z: %f\n", mMin[0], mMin[1], mMin[2]);
    printf("mMax = x: %f y: %f z: %f\n", mMax[0], mMax[1], mMax[2]);

    mxb = (mMax[0] + mMin[0]) / 2.0f;
    myb = (mMax[1] + mMin[1]) / 2.0f;
    mzb = (mMax[2] + mMin[2]) / 2.0f;

    float mix = (mMax[0] - mMin[0]) / 2.0f;
    float miy = (mMax[1] - mMin[1]) / 2.0f;
    float miz = (mMax[2] - mMin[2]) / 2.0f;
    float av = (mix + miy + miz) / 3.0f;
    mxs = av / mix;
    mys = av / miy;
    mzs = av / miz;
    printf("%s", "\nCalibration completed.\n");
    PrintMagBiasScale();
}

void Ahrs::PrintMagForMotionCal(bool cal)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (true)
    {
        xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10)); // 100Hz.
        CalculateCalibratedMag();
        if (cal)
        {
            printf("Raw:0,0,0,0,0,0,%d,%d,%d\n", (int)(mxc * 10), (int)(myc * 10), (int)(mzc * 10));
        }
        else
        {
            printf("Raw:0,0,0,0,0,0,%d,%d,%d\n", (int)(mxr * 10), (int)(myr * 10), (int)(mzr * 10));
        }
    }
}

void Ahrs::PrintAccRaw()
{
    printf("%s%.2f, %s%.2f, %s%.2f\n", "axr: ", axr, "ayr: ", ayr, "azr: ", azr);
}

void Ahrs::PrintGyroRaw()
{
    printf("%s%.2f, %s%.2f, %s%.2f\n", "gxr: ", gxr, "gyr: ", gyr, "gzr: ", gzr);
}

void Ahrs::PrintMagRaw()
{
    printf("%s%.2f, %s%.2f, %s%.2f\n", "mxr: ", mxr, "myr: ", myr, "mzr: ", mzr);
}

void Ahrs::PrintTemp()
{
    printf("%s%.2f\n", "tpr: ", tpr);
}

void Ahrs::PrintAccCalibrated()
{
    printf("%s%.2f, %s%.2f, %s%.2f\n", "axc: ", axc, "ayc: ", ayc, "azc: ", azc);
}

void Ahrs::PrintGyroCalibrated()
{
    printf("%s%.2f, %s%.2f, %s%.2f\n", "gxc: ", gxc, "gyc: ", gyc, "gzc: ", gzc);
}

void Ahrs::PrintMagCalibrated()
{
    printf("%s%.2f, %s%.2f, %s%.2f\n", "mxc: ", mxc, "myc: ", myc, "mzc: ", mzc);
}

void Ahrs::PrintAccGyroBias()
{
    printf("%s", "Enter these values to the Ahrs.hpp file and comment out the CalibrateAccGyro function.\n");
    printf("%s%f\n", "biasAx = ", axb);
    printf("%s%f\n", "biasAy = ", ayb);
    printf("%s%f\n", "biasAz = ", azb);
    printf("%s%f\n", "biasGx = ", gxb);
    printf("%s%f\n", "biasGy = ", gyb);
    printf("%s%f\n", "biasGz = ", gzb);
    WaitForever();
}

void Ahrs::PrintMagBiasScale()
{
    printf("%s", "Enter these values to the Ahrs.hpp file and comment out the CalibrateMag function.\n");
    printf("%s", "Hard iron: \n");
    printf("Bias = x: %f y: %f z: %f\n", mxb, myb, mzb);
    printf("%s", "Soft iron: \n");
    printf("Scale = x: %f y: %f z: %f\n", mxs, mys, mzs);
    WaitForever();
}

void Ahrs::Madgwick9(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

    // Use 6DOF algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
    if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))
    {
        Madgwick6(gx, gy, gz, ax, ay, az, dt);
        return;
    }

    gx = DegToRad(gx);
    gy = DegToRad(gy);
    gz = DegToRad(gz);

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        // Normalise accelerometer measurement
        recipNorm = InvSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Normalise magnetometer measurement
        recipNorm = InvSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0mx = 2.0f * q0 * mx;
        _2q0my = 2.0f * q0 * my;
        _2q0mz = 2.0f * q0 * mz;
        _2q1mx = 2.0f * q1 * mx;
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _2q0q2 = 2.0f * q0 * q2;
        _2q2q3 = 2.0f * q2 * q3;
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

        // Reference direction of Earth's magnetic field
        hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
        hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
        _2bx = sqrtf(hx * hx + hy * hy);
        _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        // Gradient decent algorithm corrective step
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        recipNorm = InvSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    // Normalize quaternion
    recipNorm = InvSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

void Ahrs::Madgwick6(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    gx = DegToRad(gx);
    gy = DegToRad(gy);
    gz = DegToRad(gz);

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        // Normalise accelerometer measurement
        recipNorm = InvSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0;
        _4q1 = 4.0f * q1;
        _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1;
        _8q2 = 8.0f * q2;
        q0q0 = q0 * q0;
        q1q1 = q1 * q1;
        q2q2 = q2 * q2;
        q3q3 = q3 * q3;

        // Gradient decent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
        recipNorm = InvSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    // Normalise quaternion
    recipNorm = InvSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

float Ahrs::InvSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

void Ahrs::CalculateEulerAngles()
{
    phi = std::atan2(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2);
    double sinTheta = 2.0f * (q0 * q2 - q1 * q3);
    if (std::abs(sinTheta) >= 1)
    {
        theta = std::copysign(M_PI / 2.0f, sinTheta);
    }
    else
    {
        theta = std::asin(sinTheta);
    }
    psi = std::atan2(q0 * q3 + q1 * q2, 0.5f - q2 * q2 - q3 * q3);
}

void Ahrs::PrintQuaternions()
{
    printf("%s%.2f, %s%.2f, %s%.2f, %s%.2f\n", "q0: ", q0, "q1: ", q1, "q2: ", q2, "q3: ", q3);
}

void Ahrs::PrintEulerAngles()
{
    printf("%s%.1f, %s%.1f, %s%.1f\n", "phi: ", RadToDeg(phi), "theta: ", RadToDeg(theta), "psi: ", RadToDeg(psi));
}

void Ahrs::Converge(uint16_t freq)
{
    bool isConverged = false;
    float deltaPhi = 0;
    float deltaTheta = 0;
    float deltaPsi = 0;
    float prevPhi = 0;
    float prevTheta = 0;
    float prevPsi = 0;
    int64_t i = 0;

    BaseType_t xWasDelayed;                         // Deadline missed or not
    float dt = 0;                                   // Time step
    int64_t prevTime = 0;                           // Previous time [us]
    int64_t elapsedTime = 0;                        // Elapsed time [us]
    int64_t currentTime = esp_timer_get_time();     // Current time [us]
    const int loopTime = 1000 / freq;               // Loop time [ms]
    TickType_t xLastWakeTime = xTaskGetTickCount(); // Last wake time
    while (isConverged == false)
    {
        xWasDelayed = xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(loopTime)); // Wait for the next cycle.
        prevTime = currentTime;
        currentTime = esp_timer_get_time();   // [us]
        elapsedTime = currentTime - prevTime; // [us]
        dt = elapsedTime / 1000000.0f;        // [s]
        Update(dt);
        PrintEulerAngles();
        if (xWasDelayed == pdFALSE)
        {
            printf("%s", "Deadline missed!\n");
        }
        // Calculate delta values for every 2 sec.
        if (i > freq * 2)
        {
            i = 0;
            float v1 = GetPhi();
            float v2 = GetTheta();
            float v3 = GetPsi();
            deltaPhi = v1 - prevPhi;
            deltaTheta = v2 - prevTheta;
            deltaPsi = v3 - prevPsi;
            prevPhi = v1;
            prevTheta = v2;
            prevPsi = v3;
            float tol = 0.1f; // Tolerance [deg]
            if (abs(deltaPhi) < tol && abs(deltaTheta) < tol && abs(deltaPsi) < tol)
            {
                isConverged = true;
            }
        }
        i += 1;
    }
}