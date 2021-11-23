
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

// This #include statement was automatically added by the Particle IDE.
#define OLD_DISPLAY 0
#if OLD_DISPLAY
#include "ssd1327.h"
#else
#include "U8g2lib.h"
#endif

// This #include statement was automatically added by the Particle IDE.
#include <SparkFun_LPS25HB_Arduino_Library.h>

#include <SparkFun_I2C_Mux_Arduino_Library.h>
#include <SparkFun_Qwiic_Humidity_AHT20.h>
#include <SparkFun_Qwiic_Joystick_Arduino_Library.h>

// Split application execution into its own thread
SYSTEM_THREAD(ENABLED);
// Always connect and stay connected
SYSTEM_MODE(AUTOMATIC);

static constexpr int LED = D7;
#define TEXT_FOREGROUND 0x4
#define TEXT_BACKGROUND 0x0
#define TEXT_HEIGHT 8
#define TEXT_LINE_SPACING 1

DeltaClock deltaClock;

QWIICMUX i2cMux;
bool i2cMuxPresent = false;

#define I2C_DEFAULT_SPEED CLOCK_SPEED_400KHZ
#define I2C_SAFE_SPEED    CLOCK_SPEED_100KHZ

LPS25HB pressureSensor;
bool pressureSensorPresent = false;
// Device returns 83.2 deg F, TemporalScanner returns 80.3
// Offset is added
// 70 deg F in actively cooled airstream was -5 deg F
constexpr float pressureSensorTempFOffset = 80.3 - 83.2; // actual - measured

AHT20 humiditySensor;
bool humiditySensorPresent = false;
constexpr float humiditySensorTempOffset = 0.0;

// https://github.com/sparkfun/SparkFun_SGP30_Arduino_Library/tree/main/src
SGP30 vocSensor;
bool vocSensorPresent = false;
constexpr int VOC_START_DELAY_MILLIS = 15*1000;
unsigned long vocSensorInitStart = 0;
bool vocSensorInitDone = false;

SCD30 co2Sensor;
bool co2SensorPresent = false;
// Device returns 79.0 deg F, TemporalScanner returns 80.1
// Offset is added
// 70 deg F in actively cooled airstream was -1 deg F
constexpr float co2SensorTempFOffset = 80.1 - 79.0;
Decimator co2Decimator(3, 3, 3);

SPS30 pmSensor;
bool pmSensorPresent = false;
constexpr uint8_t PM_MUX_PORT = 7;

JOYSTICK joystick;
bool joystickPresent = false;

PhotonVBAT vbat(A0);

#if OLD_DISPLAY
#else
// FUll U8G2, SSD1327 controller, EA_128128 display, full framebuffer, First Arduino Hardware I2C, not rotated
// Something about the C++ process causes lockups
//U8G2_SSD1327_EA_W128128_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, U8X8_PIN_NONE, U8X8_PIN_NONE);
u8g2_t u8g2 = { 0 };
// I thought something about u8x8_gpio_and_delay_arduino caused lockups too;
// replacing it with a do-nothing return 0; works too.
#endif

void BlinkActionFunc(void) {
    static bool ledState = false;
    ledState = !ledState;
    digitalWrite(LED, ledState ? HIGH : LOW);
}

DeltaClockEntry BlinkAction = { 0 };

// To use I2C buffers bigger than 32 bytes, we provide a function for allocating the buffers. Particle-specific.
// https://docs.staging.particle.io/cards/firmware/wire-i2c/acquirewirebuffer/
constexpr size_t I2C_BUFFER_SIZE = 512;

HAL_I2C_Config acquireWireBuffer() {
    HAL_I2C_Config config = {
        .size = sizeof(HAL_I2C_Config),
        .version = HAL_I2C_CONFIG_VERSION_1,
        .rx_buffer = new (std::nothrow) uint8_t[I2C_BUFFER_SIZE],
        .rx_buffer_size = I2C_BUFFER_SIZE,
        .tx_buffer = new (std::nothrow) uint8_t[I2C_BUFFER_SIZE],
        .tx_buffer_size = I2C_BUFFER_SIZE
    };
    return config;
}

void watchdogHandler(void) {
    System.reset(RESET_NO_WAIT);
}

ApplicationWatchdog *wd = NULL;

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
//   range 32.01 in-Hg (Agata, Siberia, 1968) to 25.9 in-Hg (Pacific Ocean, 1979) (B.S.)
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

//
// Data Flow Design
//
// Setup
// 1. Set up hardware (i2c config, device resets, etc.)
// 2. Schedule initialization routines
    // a. Initialize data-passing structs (events)
// 3. Schedule data production routines
// Loop
// 1. Crank delta clock
// 2. Crank event consumers
// 3. Render resulting state
//
// This turns into
// A bunch of structs to hold sensor events
// A bunch of functions to schedule to produce sensor events
// A bunch of 'if's to check for sensor events and process them
// A bunch of structs to hold processed sensor events
// A bunch of 'if's to check for processed sensor events and turn them into serial, network, and UI messages
// A bunch of structs to represent the outputs
// A bunch of 'if's to take those and process them
//
// This could be abstracted to be a bool pointer and a function pointer: if bool, call func.
// This doesn't allow for marking the event as consumed by multiple consumers.
// It requires void pointers and recasts to pass around all the different and potentially complex data types.
//

void setup() {
    deltaClock.begin();
    BlinkAction.action = &BlinkActionFunc;
    BlinkAction.interval = 1000;
    BlinkAction.repeating = true;
    deltaClock.insert(&BlinkAction);

    // setSpeed must be before begin()
    // https://docs.staging.particle.io/cards/firmware/wire-i2c/setspeed/
    Wire.setSpeed(I2C_SAFE_SPEED);
    Wire.begin();

    Serial.begin(115200);

    i2cMuxPresent = i2cMux.begin();
    if (i2cMuxPresent) {
        i2cMux.setPortState(0
            |0x1 // C02
            //|0x2 // nc
            //|0x4 // nc
            //|0x8 // nc
            //|0x10 // nc
            //|0x20 // nc
            //|0x40 // nc
            |0x80 // SCD30/PM
            );
    }

    joystickPresent = joystick.begin();

    pressureSensorPresent = pressureSensor.begin();

    co2SensorPresent = co2Sensor.begin();
    if (co2SensorPresent != false) {
        co2Sensor.setAutoSelfCalibration(true);
        co2Sensor.setMeasurementInterval(5);
    }

    humiditySensorPresent = humiditySensor.begin();

    vocSensorPresent = vocSensor.begin();
    if (vocSensorPresent != false)
    {
        vocSensor.setHumidity(0x0F80);
        vocSensor.initAirQuality();
        vocSensorInitDone = false;
        vocSensorInitStart = millis();
    }

    if (i2cMuxPresent) {
        // Slow down I2C
        Wire.end();
        Wire.setSpeed(I2C_SAFE_SPEED); // SPS30 only supports 100KHz
        Wire.begin();
        // Enable I2C mux
        i2cMux.enablePort(PM_MUX_PORT);
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
    if (i2cMuxPresent) {
        // Disable I2C mux
        i2cMux.disablePort(PM_MUX_PORT);
        // Speed up I2C
        Wire.end();
        Wire.setSpeed(I2C_DEFAULT_SPEED); // Display really needs 400KHz
        Wire.begin();
    }

#if OLD_DISPLAY
    // int iType, int iAddr, int bFlip, int bInvert, int iSDAPin, int iSCLPin, int32_t iSpeed
    ssd1327Init(OLED_128x128, 0x3c, 0, 0, -1, -1, I2C_DEFAULT_SPEED);
    ssd1327Power(true);
    ssd1327Fill(TEXT_BACKGROUND);
#else
    // Docs say U8G2_SSD1327_EA_W128128_F_HW_I2C, but it chops off the top and bottom 16 rows.
    // u8g2_Setup_ssd1327_i2c_ws_128x128_f seems to work well, at least empirically.
    u8g2_Setup_ssd1327_i2c_ws_128x128_f(&u8g2, U8G2_R0, u8x8_byte_arduino_hw_i2c, u8x8_gpio_and_delay_arduino);
    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0);
    u8g2_SetFont(&u8g2, u8g2_font_nerhoe_tf);
    u8g2_ClearBuffer(&u8g2);
    u8g2_SendBuffer(&u8g2);
#endif

    Time.zone(-8.0);
    Time.setDSTOffset(+1.0);

    pinMode(LED, OUTPUT);

    wd = new ApplicationWatchdog(3000U, &watchdogHandler);
}

#if OLD_DISPLAY
// uint8_t x, uint8_t y, char *szMsg, uint8_t iSize, int ucFGColor, int ucBGColor
#define PRINTLN(cstr) ssd1327WriteString(0, 0+(lineNo++)*(TEXT_HEIGHT+TEXT_LINE_SPACING), cstr, FONT_SMALL, TEXT_FOREGROUND, TEXT_BACKGROUND);
#else
#define PRINTLN(cstr) u8g2_DrawUTF8(&u8g2, 0, 0+(lineNo++)*(TEXT_HEIGHT+TEXT_LINE_SPACING), cstr);
#endif

void loop() {
    ApplicationWatchdog::checkin();
    deltaClock.update();

#if OLD_DISPLAY
#else
    u8g2_ClearBuffer(&u8g2);
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

    if (joystickPresent) {
        uint16_t x = joystick.getHorizontal();
        uint16_t y = joystick.getVertical();
        byte button = joystick.checkButton();
        String val;
        val += "joy:";
        val += "x=";
        val += x;
        val += ",y=";
        val += y;
        val += "b=";
        val += button;
        val += "         ";
        PRINTLN(val.c_str()); // 2
    } else {
        PRINTLN("Joystick not found"); // 2
    }

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

    if (humiditySensorPresent != false) {
        humiditySensor.triggerMeasurement();
        String val;
        val += "hum:h=";
        val += String(humiditySensor.getHumidity(), 0);
        val += ",T=";
        val += String(humiditySensor.getTemperature(), 0);
        val += ",s=";
        val += humiditySensor.getStatus();
        val += humiditySensor.isCalibrated() ? ",c" : ",nc";
        PRINTLN(val.c_str()); // ?
    } else {
        PRINTLN("AHT20 not found"); // ?
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
    decVal += "                    "; // ensure line gets blanked
    PRINTLN(decVal.c_str());
    Serial.println(decVal.c_str());

    if (i2cMuxPresent) {
        // Slow down I2C
        Wire.end();
        Wire.setSpeed(I2C_SAFE_SPEED);
        Wire.begin();
        // Enable I2C mux
        i2cMux.enablePort(PM_MUX_PORT);
    }
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
        PRINTLN("pm not present"); // 9
        String val;
        val += "10 ";
        val += pmSensor.device_status;
        val += " ";
        val += pmSensor.firmware_version[0];
        val += ".";
        val += pmSensor.firmware_version[1];
        val += " ";
        val += pmSensor.is_data_ready();
        PRINTLN(val.c_str()); // 10
    }
    if (i2cMuxPresent) {
        // Disable I2C mux
        i2cMux.disablePort(PM_MUX_PORT);
        // Speed up I2C
        Wire.end();
        Wire.setSpeed(I2C_DEFAULT_SPEED);
        Wire.begin();
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
    } else {
        PRINTLN("no summary avail"); // 12
    }
    Serial.printlnf("scd: %0.3fF lps: %0.3fF CPU: %0.3fF", C_TO_F(tempC_SCD30), C_TO_F(tempC_LPS25HB), C_TO_F(vbat.readTempC()));
    Serial.printlnf("rH: %0.1f Pressure: %0.3fhPa", relativeHumidityPercent, pressurehPa);

    if (joystickPresent) {
        String str = "joy x=";
        str += String(joystick.getHorizontal());
        str += ",y=";
        str += String(joystick.getVertical());
        str += "          ";
        PRINTLN(str.c_str()); // 14
    } else {
        //PRINTLN("Bld " __DATE__); // 13
        PRINTLN("Bld " __TIME__); // 14
    }

    //Particle.publish("operating", NULL, 60, PRIVATE);

    unsigned long drawStart = millis();
#if OLD_DISPLAY
    // Arduino's Wire library uses 32-byte buffers; anything longer is truncated.
    // Since the SSD1327 requires a type byte first and pixels are 4 bits, this means we can only send 62-pixel chunks at a time.
    // This library already splits the transmission per y value, so it's just the x value that is limited.
    //ssd1327ShowBitmap(NULL, 0, 0, 0, 62, 128);
    //ssd1327ShowBitmap(NULL, 0, 62, 0, 62, 128);
    //ssd1327ShowBitmap(NULL, 0, 124, 0, 4, 128);
    // Particle provides a mechanism, acquireWireBuffer, for making it larger.
    ssd1327ShowBitmap(NULL, 0, 0, 0, 128, 128);
#else
    {
        for (int n = 0; n < 128; n++) {
            if (n % 5 == 0) {
                u8g2_DrawPixel(&u8g2, n, 0);
                u8g2_DrawPixel(&u8g2, 0, n);
            }
            if (n % 10 == 0) {
                u8g2_DrawPixel(&u8g2, n, 1);
                u8g2_DrawPixel(&u8g2, 1, n);
            }
        }
        u8g2_DrawPixel(&u8g2, 126, 94);
        u8g2_DrawPixel(&u8g2, 127, 95);
        u8g2_DrawPixel(&u8g2, 128, 96);
        u8g2_DrawPixel(&u8g2, 126, 126);
        u8g2_DrawPixel(&u8g2, 127, 127);
        u8g2_DrawPixel(&u8g2, 128, 128);
    }
    u8g2_SendBuffer(&u8g2);
#endif
    unsigned long drawTime = millis() - drawStart;
    if (drawTime < 1000) {
        delay(1000 - drawTime);
    }
    Serial.printlnf("draw latency: %u ms", drawTime);
}
