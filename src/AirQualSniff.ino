// This #include statement was automatically added by the Particle IDE.
#include "decimator.h"

// This #include statement was automatically added by the Particle IDE.
#include <photon-vbat.h>

// This #include statement was automatically added by the Particle IDE.
#include "sps30.h"

// This #include statement was automatically added by the Particle IDE.
#include <SparkFun_SCD30_Arduino_Library.h>

// This #include statement was automatically added by the Particle IDE.
#include "SparkFun_SGP30_Arduino_Library.h"

// This #include statement was automatically added by the Particle IDE.
#include "ssd1327.h"

// This #include statement was automatically added by the Particle IDE.
#include <SparkFun_LPS25HB_Arduino_Library.h>

static const int LED = D7;
#define TEXT_FOREGROUND 0x4
#define TEXT_BACKGROUND 0x0
#define TEXT_HEIGHT 8
#define TEXT_LINE_SPACING 1

LPS25HB pressureSensor;
bool pressureSensorPresent;
const float pressureSensorTempFOffset = -5.0;

// https://github.com/sparkfun/SparkFun_SGP30_Arduino_Library/tree/main/src
SGP30 vocSensor;
bool vocSensorPresent;
const int VOC_START_DELAY_MILLIS = 15*1000;
unsigned long vocSensorInitStart;
bool vocSensorInitDone;

SCD30 co2Sensor;
bool co2SensorPresent;
const float co2SensorTempFOffset = -1.0;
Decimator co2Decimator(3, 3, 3);

SPS30 pmSensor;
bool pmSensorPresent;

PhotonVBAT vbat(A0);

//Adafruit_SSD1327 display(128, 128, &Wire, -1, 400000, 400000);
//U8G2_SSD1327_EA_W128128_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

void setup() {
    Wire.begin();
    Wire.setSpeed(CLOCK_SPEED_100KHZ); // SPS30 only supports 100KHz

    Serial.begin(115200);

    pressureSensorPresent = pressureSensor.begin();

    co2SensorPresent = co2Sensor.begin();
    if (co2SensorPresent != false) {
        co2Sensor.setAutoSelfCalibration(true);
        co2Sensor.setMeasurementInterval(5);
    }

    vocSensorPresent = vocSensor.begin();
    if (vocSensorPresent != false)
    {
        vocSensor.setHumidity(0x0F80);
        vocSensor.initAirQuality();
        vocSensorInitDone = false;
        vocSensorInitStart = millis();
    }
    
    pmSensorPresent = pmSensor.begin();
    if (pmSensorPresent != false) {
        pmSensor.wake();
        pmSensor.reset();
        uint32_t interval;
        pmSensor.read_fan_cleaning_interval(&interval);
        if (interval != 7 * 24 * 60 * 60) {
            interval = 7 * 24 * 60 * 60;
            pmSensor.set_fan_cleaning_interval(interval);
        }
        pmSensor.start_measuring(SPS30_FORMAT_UINT16);
    }

    // int iType, int iAddr, int bFlip, int bInvert, int iSDAPin, int iSCLPin, int32_t iSpeed
    ssd1327Init(OLED_128x128, 0x3c, 0, 0, -1, -1, 100000L);
    ssd1327Power(true);
    ssd1327Fill(TEXT_BACKGROUND);
    //display.begin();

    Time.zone(-8.0);
    Time.setDSTOffset(+1.0);

    pinMode(LED, OUTPUT);
}

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

#define C_TO_F(C) ((C) * 9.0 / 5.0 + 32)

// uint8_t x, uint8_t y, char *szMsg, uint8_t iSize, int ucFGColor, int ucBGColor
#define PRINTLN(cstr) ssd1327WriteString(0, 0+(lineNo++)*(TEXT_HEIGHT+TEXT_LINE_SPACING), cstr, FONT_SMALL, TEXT_FOREGROUND, TEXT_BACKGROUND);

void loop() {
    static bool ledState = false;

    int lineNo = 0;

    //ledState = !ledState;
    digitalWrite(LED, ledState ? HIGH : LOW);

    // Measured values from LPS25HB (pressure sensor)
    float pressurehPa;
    float tempC_LPS25HB;
    // Calculated values from LPS25HB
    // using pressure to estimate altitude
    //https://en.wikipedia.org/wiki/Barometric_formula
    float tempC;
    float tempF;
    float altitudem;
    
    // Measured values from SCD30 (CO2 sensor)
    // requires pressure, (if available) altitude, and (need to read docs) temperature offset
    static float relativeHumidityPercent;
    static float tempC_SCD30;
    static uint16_t co2ppm;
    // Calculated values from SCD30
    float tempOffsetC_SCD30;

    // Composite calculated value
    uint16_t absoluteHumidity_g_m3_8_8 = 0xF80; // default value in SGP30

    // Measured values from SPS30 (particulate sensor)
    SPS30_DATA_INT pmData;
    
    // Measured values from SGP30 (VOC sensor)
    uint16_t TVOCppb;
    uint16_t eCO2ppm;

    PRINTLN("Air Quality Sniffer"); // 1

    PRINTLN("02"); // 2

    if (pressureSensorPresent != false) {
        tempC = tempC_LPS25HB = pressureSensor.getTemperature_degC() + pressureSensorTempFOffset * 5 / 9;
        pressurehPa = pressureSensor.getPressure_hPa();

        tempF = C_TO_F(tempC_LPS25HB);
        altitudem = pressure_to_est_altitude(pressurehPa);

        String val;
        val += String(tempC, 3);
        val += " C ";
        val += String(pressurehPa, 3);
        val += " hPa";
        PRINTLN(val.c_str()); // 3
        val = String(tempF, 3);
        val += " F ";
        val += String(altitudem, 0);
        val += " m";
        PRINTLN(val.c_str()); // 4
            co2Decimator.push(tempF);
            if (co2Decimator.is_full()) {
                float final_val;
                co2Decimator.decimate_and_clear(&final_val);
                Serial.printlnf("final_val: %0.1f, n=%d", final_val, co2Decimator.capacity());
            }
    } else {
        PRINTLN("LPS25HB not present");
    }

    if (co2SensorPresent != false) {
        co2Sensor.setAmbientPressure(pressurehPa /* 1.0 hPa/mb */);
        // altitude ignored when ambient pressure set

        if (co2Sensor.dataAvailable() != false) {
            relativeHumidityPercent = co2Sensor.getHumidity();
            tempC_SCD30 = co2Sensor.getTemperature() + co2SensorTempFOffset * 5 / 9;
            co2ppm = co2Sensor.getCO2();
        }

        String val;
        val += "co2:";
        val += String(co2ppm);
        val += "ppm rh:";
        val += String(relativeHumidityPercent, 0);
        val += "%       ";
        PRINTLN(val.c_str()); // 5
    } else {
        PRINTLN("SCD30 not present");
    }

    if (pressureSensorPresent != false && co2SensorPresent != false) {
        absoluteHumidity_g_m3_8_8 = rel_to_abs_humidity(tempC_LPS25HB, pressurehPa, relativeHumidityPercent);
    }

    if (vocSensorPresent != false) {
        String val = "";

        vocSensor.setHumidity(absoluteHumidity_g_m3_8_8);

        SGP30ERR sgperr = vocSensor.measureAirQuality();
        if (sgperr != SGP30_SUCCESS)
        {
            vocSensorPresent = false;
            val = "SGP30 measurement failed: ";
            val += String(sgperr);
        } else if (vocSensorInitDone == false) {
            vocSensorInitDone = millis() - vocSensorInitStart > VOC_START_DELAY_MILLIS;
            val = "SGP30 initializing";
        } else {
            uint16_t tvoc_ppb = vocSensor.TVOC;
            uint16_t eCO2_ppm = vocSensor.CO2;
            //Particle.publish("TVOC", String(tvoc_ppb));
            
            val = "TVOC:";
            val += String(tvoc_ppb);
            val += "ppb(";
            val += String(eCO2_ppm);
            val += "ppmCO2)   ";
        }
        PRINTLN(val.c_str()); // 6

        sgperr = vocSensor.measureRawSignals();
        if (sgperr == SGP30_SUCCESS) {
            val = "H2:";
            val += String(vocSensor.H2, HEX);
            val += " CH4:";
            val += String(vocSensor.ethanol, HEX);
            PRINTLN(val.c_str()); // 7
        } else {
            PRINTLN("SGP30 raw read failed");
        }
    } else {
        PRINTLN("SGP30 not present");
    }

    String decVal = "08";
    for (int idx = 0; idx < co2Decimator.fine.size(); idx++) {
        float val;
        decVal += " ";
        co2Decimator.fine.peek(idx, &val);
        decVal += String(val, 0);
    }
    decVal += ",";
    for (int idx = 0; idx < co2Decimator.mid.size(); idx++) {
        float val;
        decVal += " ";
        co2Decimator.mid.peek(idx, &val);
        decVal += String(val, 0);
    }
    decVal += ",";
    for (int idx = 0; idx < co2Decimator.coarse.size(); idx++) {
        float val;
        decVal += " ";
        co2Decimator.coarse.peek(idx, &val);
        decVal += String(val, 0);
    }
    PRINTLN(decVal.c_str());
    Serial.println(decVal.c_str());

    if (pmSensorPresent != false && pmSensor.is_data_ready() == SPS30_OK) {
        pmSensor.read_data_no_wait_int(&pmData);
        String val;
        val += String(pmData.pm_1_0_ug_m3);
        val += ",";
        val += String(pmData.pm_2_5_ug_m3);
        val += ",";
        val += String(pmData.pm_4_0_ug_m3);
        val += ",";
        val += String(pmData.pm_10_ug_m3);
        val += ",";
        val += String(pmData.typical_size_nm);
        val += "            ";
        PRINTLN(val.c_str()); // 9
        val = "";
        val += String(pmData.pm_0_5_n_cm3);
        val += ",";
        val += String(pmData.pm_1_0_n_cm3);
        val += ",";
        val += String(pmData.pm_2_5_n_cm3);
        val += ",";
        val += String(pmData.pm_4_0_n_cm3);
        val += ",";
        val += String(pmData.pm_10_n_cm3);
        val += "            ";
        PRINTLN(val.c_str()); // 10
    } else {
        PRINTLN("pm not present");
        PRINTLN("10");
    }

    if (Time.isValid()) {
        PRINTLN(Time.format("%F %T PST").c_str()); // 11
    } else {
        PRINTLN("Time not ready");
    }

    if (co2SensorPresent && pressureSensorPresent) {
        String str;
        str += "s:";
        str += String(C_TO_F(tempC_SCD30), 0);
        str += ",l:";
        str += String(C_TO_F(tempC_LPS25HB), 0);
        str += ",c:";
        str += String(C_TO_F(vbat.readTempC()), 0);
        str += ",v:";
        str += String(vbat.readVBAT(), 2);
        PRINTLN(str.c_str()); // 12
    }
    Serial.printlnf("scd: %0.3fF lps: %0.3fF CPU: %0.3fF", C_TO_F(tempC_SCD30), C_TO_F(tempC_LPS25HB), C_TO_F(vbat.readTempC()));
    Serial.printlnf("rH: %0.1f Pressure: %0.3fhPa", relativeHumidityPercent, pressurehPa);

    lineNo = 12;

    PRINTLN("Bld " __DATE__); // 13
    PRINTLN("Bld " __TIME__); // 14

    //Particle.publish("operating", NULL, 60, PRIVATE);

    // Arduino's Wire library uses 32-byte buffers; anything longer is truncated.
    // Since the SSD1327 requires a type byte first and pixels are 4 bits, this means we can only send 62-pixel chunks at a time.
    // This library already splits the transmission per y value, so it's just the x value that is limited.
    unsigned long drawStart = millis();
    ssd1327ShowBitmap(NULL, 0, 0, 0, 62, 128);
    ssd1327ShowBitmap(NULL, 0, 62, 0, 62, 128);
    ssd1327ShowBitmap(NULL, 0, 124, 0, 4, 128);
    unsigned long drawTime = millis() - drawStart;
    if (drawTime < 1000) {
        delay(1000 - drawTime);
    }
}
