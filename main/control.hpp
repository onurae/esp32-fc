/*******************************************************************************************
 *                                                                                         *
 *    ESP32-FC                                                                             *
 *    Flight Control System                                                                *
 *                                                                                         *
 *    Copyright (c) 2023 Onur AKIN <https://github.com/onurae>                             *
 *    Licensed under the MIT License.                                                      *
 *                                                                                         *
 ******************************************************************************************/

#ifndef CONTROL_HPP
#define CONTROL_HPP

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cmath>
#include "sbus.hpp"
#include "esc.hpp"

class Control
{
private:
    // Sbus protocol
    Sbus *sbus;

    // Control inputs
    bool failsafe = false;
    bool printState = true;
    float ch1_rud = 0;
    float ch2_ele = 0;
    float ch3_thr = 0;
    float ch4_ail = 0;
    int ch5_2po = 0;
    int ch6_3po = -1;

    // Filters
    float ch1f = 0;
    float ch2f = 0;
    float ch3f = 0;
    float ch4f = 0;
    float fCutRef = 20; // [Hz]
    float CalculateAlpha(float f, float dt);
    float ch1fp = 0;
    float ch2fp = 0;
    float ch3fp = 0;
    float ch4fp = 0;

    // Max. parameters
    const float maxRollAngle = 30;  // [deg]
    const float maxPitchAngle = 30; // [deg]
    const float maxYawRate = 160;   // [deg/s]

    // Reference values
    float rRef = 0;     // Yaw rate
    float thetaRef = 0; // Pitch angle
    float thrRef = 0;   // Throttle
    float phiRef = 0;   // Roll angle

    // Attitude control

    float rollOut = 0;
    float pitchOut = 0;
    float yawOut = 0;

    // Mixer
    // 1 CW   2 CCW
    // 4 CCW  3 CW
    float kRoll = 10.10f / 10.10f; // 1-2: 20.2 cm, 1-RollAxis: 10.10 cm.
    float kPitch = 7.75f / 10.10f; // 1-4: 15.5 cm, 1-PitchAxis: 7.75 cm.
    float pwmIdle = 1000.0f;
    float pwmMax = 1950.0f; // There is an esc problem at full throttle. 1.0 -> 0.95.
    float pwmRange = pwmMax - pwmIdle;
    float Saturation(float value, float min, float max);

    // Esc
    bool isArmed = false;
    void Arming();
    Esc *esc1;
    Esc *esc2;
    Esc *esc3;
    Esc *esc4;

public:
    Control(Sbus *sbus) { this->sbus = sbus; }
    virtual ~Control() = default;

    void Init(uint16_t freq, Esc* esc1, Esc* esc2, Esc* esc3, Esc* esc4);
    void UpdateControlInput();
    void CheckRxFailure();
    void UpdateRefInput(float dt);
    void PrintRef();
    void UpdateEscCmd();
};

#endif /* CONTROL_HPP */