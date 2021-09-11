
#pragma once

#include <Arduino.h>

#define C_TO_F(C) ((C) * 9.0 / 5.0 + 32)

namespace atmospherics {
uint16_t rel_to_abs_humidity(float tempC, float pressurehPa, float rhPercent);
float pressure_to_est_altitude(float pressurehPa);
}
