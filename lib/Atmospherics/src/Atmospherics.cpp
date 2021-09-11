
#include "Atmospherics.h"

#include <Arduino.h>

namespace atmospherics {

uint16_t rel_to_abs_humidity(float tempC, float pressurehPa, float rhPercent) {
    // Use this data to calculate the absolute humidity in g/m^3 as 8.8 fixed point
    // relative humidity https://planetcalc.com/2167/
    // saturation vapor pressure https://planetcalc.com/2161/
    float relative_humidity_percent = rhPercent;
    // Figure out the saturation vapor pressure for water vapor, given temperature and pressure
    // f(p)
    // empirical partial pressure of water vapor in air, I think
    float f_of_p = 1.0016 + 3.15e-6 * pressurehPa - 0.074 / pressurehPa;
    // e_w(t)
    // pure phase vapor pressure
    float e_w_of_t = 6.112 * exp(17.62 * tempC / (243.12 + tempC));
    float e_w_of_p_t = f_of_p * e_w_of_t;

    float saturation_vapor_pressure = e_w_of_p_t;
    float actual_vapor_pressure = saturation_vapor_pressure * relative_humidity_percent / 100;

    float R_sub_v = 461.5; // gas constant of water vapor

    float tempK = tempC + 273.15;
    float abs_hum = 1000/*kg/m^3 to g/m^3*/ * (actual_vapor_pressure * 100/*hPa to Pa*/) / (R_sub_v * tempK); // g / m^3
    return (uint16_t)((abs_hum * (float)(1<<8))); // 8.8 fixed point
}

float pressure_to_est_altitude(float pressurehPa) {
    return (1013.25 /* hPa at sea level */ - pressurehPa) / (11.3 /* Pa/m fall for first 1000m */ / 100 /* Pa/hPa */); // meters; TODO: do this with logarithm derivation    
}

}
