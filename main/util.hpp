/*******************************************************************************************
 *                                                                                         *
 *    ESP32-FC                                                                             *
 *    Utility                                                                              *
 *                                                                                         *
 *    Copyright (c) 2022 Onur AKIN <https://github.com/onurae>                             *
 *    Licensed under the MIT License.                                                      *
 *                                                                                         *
 ******************************************************************************************/

#ifndef UTIL_HPP
#define UTIL_HPP

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cmath>

void PrintCountDown(uint8_t sec);
void PrintCountDown(const char* s, uint8_t sec);
void Wait(const char* s, uint8_t sec);
void WaitForever();
float LinearScaledDeadband(float value, float deadbandCutoff);
float DegToRad(float deg);
float RadToDeg(float rad);

#endif /* UTIL_HPP */