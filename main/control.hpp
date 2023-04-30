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

    // Filter
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

    // Motors
    float m[4];

public:
    Control(Sbus *sbus) { this->sbus = sbus; }
    virtual ~Control() = default;

    void Init(uint16_t freq);
    void UpdateControlInput();
    void CheckRxFailure();
    void UpdateRefInput(float dt);
    void PrintRef();
};

#endif /* CONTROL_HPP */