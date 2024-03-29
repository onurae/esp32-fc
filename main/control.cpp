/*******************************************************************************************
 *                                                                                         *
 *    ESP32-FC                                                                             *
 *    Flight Control System                                                                *
 *                                                                                         *
 *    Copyright (c) 2023 Onur AKIN <https://github.com/onurae>                             *
 *    Licensed under the MIT License.                                                      *
 *                                                                                         *
 ******************************************************************************************/

#include "control.hpp"

void Control::Init(uint16_t freq, Esc *esc1, Esc *esc2, Esc *esc3, Esc *esc4)
{
    // Disable ref filter if the frequency is below 100Hz.
    if (freq < 100)
    {
        fCutRef = -1;
    }
    this->esc1 = esc1;
    this->esc2 = esc2;
    this->esc3 = esc3;
    this->esc4 = esc4;
}

void Control::UpdateControlInput()
{
    failsafe = sbus->GetFailSafe();
    if (failsafe == false)
    {
        ch1_rud = sbus->GetAnalog(1, -1.0f, 1.0f);
        ch2_ele = sbus->GetAnalog(2, -1.0f, 1.0f);
        ch3_thr = sbus->GetAnalog(3, 0.0f, 1.0f);
        ch4_ail = sbus->GetAnalog(4, -1.0f, 1.0f);
        ch5_2po = sbus->GetSwitch2Pos(5);
        ch6_3po = sbus->GetSwitch3Pos(6);
    }
}

void Control::CheckRxFailure()
{
    // Receiver failure, Radio failsafe values. Radio setting: Hold.
    if (sbus->CheckStatus() == false || failsafe == true)
    {
        ch1_rud = 0;
        ch2_ele = 0;
        ch3_thr = 0;
        ch4_ail = 0;
        ch5_2po = 0; // Disarm, drop.
        ch6_3po = 0; // Not used.
        if (printState == true)
        {
            printf("Rx/Tx failure\n");
            printState = false;
        }
    }
    else
    {
        if (printState == false)
        {
            printf("Rx/Tx connected\n");
            printState = true;
        }
    }
}

float Control::CalculateAlpha(float f, float dt)
{
    if (f < 0.0f)
    {
        return 1.0f;
    }
    float omega = f * 2.0f * M_PI;
    return (omega * dt / (1.0f + omega * dt)); // [0-1]
}

void Control::UpdateRefInput(float dt)
{
    float alpha = CalculateAlpha(fCutRef, dt);
    ch1f = (1.0f - alpha) * ch1fp + alpha * ch1_rud;
    ch2f = (1.0f - alpha) * ch2fp + alpha * ch2_ele;
    ch3f = (1.0f - alpha) * ch3fp + alpha * ch3_thr;
    ch4f = (1.0f - alpha) * ch4fp + alpha * ch4_ail;
    ch1fp = ch1f;
    ch2fp = ch2f;
    ch3fp = ch3f;
    ch4fp = ch4f;

    rRef = ch1f * maxYawRate;
    thetaRef = -1.0f * ch2f * maxPitchAngle; // Revert theta reference.
    thr = ch3f;
    phiRef = ch4f * maxRollAngle;
}

void Control::PrintRef()
{
    printf("%.1f, %.1f, %.2f, %.1f\n ", rRef, thetaRef, thr, phiRef);
    // printf("%.3f, %.3f\n", ch1_rud, ch1f);
}

void Control::Arming()
{
    if (ch5_2po == 0) // Disarm
    {
        isArmed = false;
    }
    else if (ch5_2po == 1 && thr < 0.005f) // Arm
    {
        isArmed = true;
    }
}

float Control::Saturation(float value, float min, float max)
{
    return std::min(std::max(value, min), max);
}

void Control::UpdateEscCmd(float dt, float p, float q, float r, float phi, float theta, float psi)
{
    Arming();
    if (isArmed)
    {
        // Roll channel
        xiRollAngle = xiRollAnglePrev + (phiRef - phi) * dt;
        if (thr < threshold) { xiRollAngle = 0; }
        xiRollAngle = Saturation(xiRollAngle, -iLimit, iLimit);
        lat = kiRollAngle * xiRollAngle - phi * ksRollAngle - p * ksRollRate;

        // Pitch channel
        xiPitchAngle = xiPitchAnglePrev + (thetaRef - theta) * dt;
        if (thr < threshold) { xiPitchAngle = 0; }
        xiPitchAngle = Saturation(xiPitchAngle, -iLimit, iLimit);
        lon = kiPitchAngle * xiPitchAngle - theta * ksPitchAngle - q * ksPitchRate;

        // Yaw channel
        xiYawRate = xiYawRatePrev + (rRef - r) * dt;
        if (thr < threshold) { xiYawRate = 0; }
        xiYawRate = Saturation(xiYawRate, -iLimit, iLimit);
        pedal = kiYawRate * xiYawRate - r * ksYawRate;

        float m1 = ((kRoll * lat + kPitch * lon - pedal) / 2.0 * thr + thr);
        float m2 = ((-kRoll * lat + kPitch * lon + pedal) / 2.0 * thr + thr);
        float m3 = ((-kRoll * lat - kPitch * lon - pedal) / 2.0 * thr + thr);
        float m4 = ((kRoll * lat - kPitch * lon + pedal) / 2.0 * thr + thr);
        
        pwmMax = 1950.0f; // There is an esc problem at max value. pwmMax: 2000 -> 1950.
        esc1->Update((uint16_t)std::round(Saturation(m1 * pwmRange + pwmIdle, pwmIdle, pwmMax)));
        esc2->Update((uint16_t)std::round(Saturation(m2 * pwmRange + pwmIdle, pwmIdle, pwmMax)));
        esc3->Update((uint16_t)std::round(Saturation(m3 * pwmRange + pwmIdle, pwmIdle, pwmMax)));
        esc4->Update((uint16_t)std::round(Saturation(m4 * pwmRange + pwmIdle, pwmIdle, pwmMax)));
    }
    else
    {
        xiRollAngle = 0;
        xiPitchAngle = 0;
        xiYawRate = 0;
        esc1->Update(1000);
        esc2->Update(1000);
        esc3->Update(1000);
        esc4->Update(1000);
    }

    // Set previous state values.
    xiRollAnglePrev = xiRollAngle;
    xiPitchAnglePrev = xiPitchAngle;
    xiYawRatePrev = xiYawRate;
}