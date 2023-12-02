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
#include "util.hpp"

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
    const float maxRollAngle = DegToRad(30);  // [rad]
    const float maxPitchAngle = DegToRad(30); // [rad]
    const float maxYawRate = DegToRad(160);   // [rad/s]

    // Attitude control
    float threshold = 0.01; // Throttle threshold for integral reset.
    float xiRoll_OL = 0;    // Outer loop roll integral output.
    float xiRoll_OLp = 0;   // Outer loop roll integral previous output.
    float xiPitch_OL = 0;   // Outer loop pitch integral output.
    float xiPitch_OLp = 0;  // Outer loop pitch integral previous output.
    float xiRoll_IL = 0;    // Inner loop roll integral output.
    float xiRoll_ILp = 0;   // Inner loop roll integral previous output.
    float xiPitch_IL = 0;   // Inner loop pitch integral output.
    float xiPitch_ILp = 0;  // Inner loop pitch integral previous output.
    float xiYaw_IL = 0;     // Inner loop yaw integral output.
    float xiYaw_ILp = 0;    // Inner loop yaw integral previous output.

    float phiRef = 0;   // Roll angle reference [rad]
    float thetaRef = 0; // Pitch angle reference [rad]
    float pRef = 0;     // Roll rate reference [rad/s]
    float qRef = 0;     // Pitch rate reference [rad/s]
    float rRef = 0;     // Yaw rate reference [rad/s]

    float lat = 0;      // Lateral
    float lon = 0;      // Longitudinal
    float pedal = 0;    // Pedal 
    float thr = 0;      // Throttle

    // Gains, OL: Outer loop, IL: Inner loop
    float kiRoll_OL = 0;
    float ksRoll_OL = 0;
    float kiRoll_IL = 0;
    float ksRoll_IL = 30;
    float kiPitch_OL = 0;
    float ksPitch_OL = 30;
    float kiPitch_IL = 0;
    float ksPitch_IL = 0;
    float kiYaw_IL = 0;
    float ksYaw_IL = 30;

    // Mixer
    // 1 CW   2 CCW
    // 4 CCW  3 CW
    float kRoll = 7.75 / 10.10 * (0.001467 / 0.001867); 	// 1-2: 20.2 cm, 1-RollAxis: 10.10 cm. (inertia ratio: Ixx/Iyy)
    float kPitch = 1.0f; 									// 1-4: 15.5 cm, 1-PitchAxis: 7.75 cm.
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
    void UpdateEscCmd(float dt, float p, float q, float r, float phi, float theta, float psi);
};

#endif /* CONTROL_HPP */