#pragma once
#include <Arduino.h>

void batteryInit();
void batterySetCharging(bool enabled);
float batteryGetADC();
uint16_t batteryGetPercent();
