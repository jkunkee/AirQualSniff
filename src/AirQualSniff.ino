
#include "decimator.h"
#include "DeltaClock.h"
#include "Atmospherics.h"

// This #include statement was automatically added by the Particle IDE.
#include <photon-vbat.h>

// This #include statement was automatically added by the Particle IDE.
#include "sps30.h"

// This #include statement was automatically added by the Particle IDE.
#include <SparkFun_SCD30_Arduino_Library.h>

// This #include statement was automatically added by the Particle IDE.
#include "SparkFun_SGP30_Arduino_Library.h"

// Split application execution into its own thread
SYSTEM_MODE(AUTOMATIC);

// This #include statement was automatically added by the Particle IDE.
#define OLD_DISPLAY 1
#if OLD_DISPLAY
#include "ssd1327.h"
#else
#include "U8g2lib.h"
#endif

// This #include statement was automatically added by the Particle IDE.
#include <SparkFun_LPS25HB_Arduino_Library.h>

static const int LED = D7;
#define TEXT_FOREGROUND 0x4
#define TEXT_BACKGROUND 0x0
#define TEXT_HEIGHT 8
#define TEXT_LINE_SPACING 1

DeltaClock deltaClock;

LPS25HB pressureSensor;
bool pressureSensorPresent = false;
// Device returns 83.2 deg F, TemporalScanner returns 80.3
// Offset is added
// 70 deg F in actively cooled airstream was -5 deg F
const float pressureSensorTempFOffset = 80.3 - 83.2; // actual - measured

// https://github.com/sparkfun/SparkFun_SGP30_Arduino_Library/tree/main/src
SGP30 vocSensor;
bool vocSensorPresent = false;
const int VOC_START_DELAY_MILLIS = 15*1000;
unsigned long vocSensorInitStart = 0;
bool vocSensorInitDone = false;

SCD30 co2Sensor;
bool co2SensorPresent = false;
// Device returns 79.0 deg F, TemporalScanner returns 80.1
// Offset is added
// 70 deg F in actively cooled airstream was -1 deg F
const float co2SensorTempFOffset = 80.1 - 79.0;
Decimator co2Decimator(3, 3, 3);

SPS30 pmSensor;
bool pmSensorPresent = false;

PhotonVBAT vbat(A0);

#if OLD_DISPLAY
#else
// FUll U8G2, SSD1327 controller, EA_128128 display, full framebuffer, First Arduino Hardware I2C, not rotated
U8G2_SSD1327_EA_W128128_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, U8X8_PIN_NONE, U8X8_PIN_NONE);
#endif

void BlinkActionFunc(void) {
    static bool ledState = false;
    ledState = !ledState;
    digitalWrite(LED, ledState ? HIGH : LOW);
}

DeltaClockEntry BlinkAction = { 0 };

// Data Collection Strategy
//
// What am I measuring?
// * barometric pressure
// * ambient temperature x2
// * CO2 concentration
// * tVOC concentration
// * non-normalized H2 concentration
// * non-normalized C2H6O concentration
// * relative humidity
//
// What measurements depend on what others?
// * tVOC, H2, and C2H6O depend on temperature, relative humidity and barometric pressure
// * CO2 depends on barometric pressure
//
// How often can I measure what?
// * barometric pressure - piezoelectric, 1, 7, 12.5, 25 Hz
//     averaging can help reduce LPS25HB RMS noise from 0.15 hPa to 0.008 hPa
// * ambient temperature x2
//     LPS25HB temp is described the same way as the pressure sensor, but no FIFO.
//      +/- 2 deg C accuracy across range, wow...
//     SCD30 tau-63 is >2s
// * CO2 concentration
//     SCD30 tau-63 is 20s
// * tVOC concentration
// * non-normalized H2 concentration and non-normalized C2H6O concentration
//     SGP30 can put out 40 Hz samples of H2/C2H6O, but tVOC/eCO2 is 1 Hz
// * relative humidity
//     SCD30 tau-63 is 8s
//
// How often shall I measure what?
// https://www.pmel.noaa.gov/ocs/sampling-rates
// * barometric pressure - every 10 min, take 2 min of 1 Hz samples
// * ambient temperature - every 10 min, take 2 min of 2 Hz samples
// * CO2 concentration - ALL GASSES - every 3 hours take 30 sec of 2 Hz samples
// * tVOC concentration
// * non-normalized H2 concentration
// * non-normalized C2H6O concentration
// * relative humidity - every 10 min, take 2 min of 1 Hz samples
// My Own SWAG
// I'm mostly interested in detecting and displaying changes in the weather
// as they happen. This means I want to sample frequently enough to catch
// the fastest changes, not necessarily as fast as the device can sample.
// * barometric pressure
//   https://sciencing.com/2-types-barometers-8524023.html
//   range 32.01 in-Hg (Agata, Siberia, 1968) to 25.9 in-Hg (Pacific Ocean, 1979)
//   "rapid" is .18 in-Hg per 3 h
//   "slow" is .003 to .04 in-Hg per 3 h
//   "steady" is <.003 in-Hg per 3 h
// * ambient temperature
//   world record change speed is +49 deg F in 2 min (Spearfish, SD)
//   this swings daily
//   I want to capture anything bigger than 0.5 deg C in, say, 5 min?
//   USCRN https://www.ncdc.noaa.gov/crn/measurements.html#temp
//   0.5 Hz samples -> 5 min avg -> hourly stats
// * gas concentrations
//   Breathing on it should produce a visible change, but most of the sensors are fairly slow
//   SGP30 datasheet 1.1 says 0.3-30 ppm C2H6O and 0.5-3ppm H2 are expected indoor air quality ranges
//

void setup() {
    deltaClock.begin();
    BlinkAction.action = &BlinkActionFunc;
    BlinkAction.interval = 1000;
    BlinkAction.repeating = true;
    deltaClock.insert(&BlinkAction);

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

#if OLD_DISPLAY
    // int iType, int iAddr, int bFlip, int bInvert, int iSDAPin, int iSCLPin, int32_t iSpeed
    ssd1327Init(OLED_128x128, 0x3c, 0, 0, -1, -1, 100000L);
    ssd1327Power(true);
    ssd1327Fill(TEXT_BACKGROUND);
#else
    //u8g2.setBusClock(CLOCK_SPEED_100KHZ);
    //u8g2.setI2CAddress(0x3c);
    u8g2.beginSimple();
    //u8g2.clearDisplay();
    //u8g2.setPowerSave(0);
#endif

    Time.zone(-8.0);
    Time.setDSTOffset(+1.0);

    pinMode(LED, OUTPUT);
}

#if OLD_DISPLAY
// uint8_t x, uint8_t y, char *szMsg, uint8_t iSize, int ucFGColor, int ucBGColor
#define PRINTLN(cstr) ssd1327WriteString(0, 0+(lineNo++)*(TEXT_HEIGHT+TEXT_LINE_SPACING), cstr, FONT_SMALL, TEXT_FOREGROUND, TEXT_BACKGROUND);
#else
//#define PRINTLN(cstr) u8g2.print(cstr);
#define PRINTLN(cstr)
#endif

void loop() {
    deltaClock.update();

#if OLD_DISPLAY
#else
    //u8g2.clearBuffer();
    //u8g2.home();
#endif

    int lineNo = 0;

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
    float tempOffsetC_SCD30; // TODO: figure out difference between input air and sensor temps as input for SCD30 correction

    // Composite calculated value
    uint16_t absoluteHumidity_g_m3_8_8 = 0xF80; // default value in SGP30

    // Measured values from SPS30 (particulate sensor)
    SPS30_DATA_INT pmData;
    
    // Measured values from SGP30 (VOC sensor)
    uint16_t TVOCppb = 0;
    uint16_t eCO2ppm = 0;

    PRINTLN("Air Quality Sniffer"); // 1

    PRINTLN("02"); // 2

    if (pressureSensorPresent != false) {
        tempC = tempC_LPS25HB = pressureSensor.getTemperature_degC() + pressureSensorTempFOffset * 5 / 9;
        pressurehPa = pressureSensor.getPressure_hPa();

        tempF = C_TO_F(tempC_LPS25HB);
        altitudem = atmospherics::pressure_to_est_altitude(pressurehPa);

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
        absoluteHumidity_g_m3_8_8 = atmospherics::rel_to_abs_humidity(tempC_LPS25HB, pressurehPa, relativeHumidityPercent);
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
            val += " C2H6O:";
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
        str += String(C_TO_F(tempC_SCD30), 2);
        str += ",l:";
        str += String(C_TO_F(tempC_LPS25HB), 2);
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
#if OLD_DISPLAY
    ssd1327ShowBitmap(NULL, 0, 0, 0, 62, 128);
    ssd1327ShowBitmap(NULL, 0, 62, 0, 62, 128);
    ssd1327ShowBitmap(NULL, 0, 124, 0, 4, 128);
#else
    //u8g2.updateDisplayArea(0, 0, u8g2.getBufferTileWidth()-1, u8g2.getBufferTileHeight()-1);
#endif
    unsigned long drawTime = millis() - drawStart;
    if (drawTime < 1000) {
        delay(1000 - drawTime);
    }
}
