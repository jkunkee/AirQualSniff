
// checked-in dependencies
#include "decimator.h"
#include "DeltaClock.h"
#include "Atmospherics.h"
#include "sps30.h"
#define EVENTHUB_DEBUG
#include "EventHub.h"
// checked-in 3p dependencies
#include "SparkFun_SGP30_Arduino_Library.h"
#include "U8g2lib.h"

// Particle-style dependencies
#include <photon-vbat.h>
#include <SparkFun_SCD30_Arduino_Library.h>
#include <SparkFun_LPS25HB_Arduino_Library.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h>
#include <SparkFun_Qwiic_Humidity_AHT20.h>
#include <SparkFun_Qwiic_Joystick_Arduino_Library.h>

/*****************************************************************************\
 * Particle system configuration and board facts
\*****************************************************************************/

// Split application execution into its own thread
SYSTEM_THREAD(ENABLED);
// Always connect and stay connected
SYSTEM_MODE(AUTOMATIC);

// To use I2C buffers bigger than 32 bytes, we provide a function for allocating the buffers. Particle-specific.
// https://docs.staging.particle.io/cards/firmware/wire-i2c/acquirewirebuffer/
static constexpr size_t I2C_BUFFER_SIZE = 512;

hal_i2c_config_t acquireWireBuffer() {
    hal_i2c_config_t config = {
        .size = sizeof(hal_i2c_config_t),
        .version = HAL_I2C_CONFIG_VERSION_1,
        .rx_buffer = new (std::nothrow) uint8_t[I2C_BUFFER_SIZE],
        .rx_buffer_size = I2C_BUFFER_SIZE,
        .tx_buffer = new (std::nothrow) uint8_t[I2C_BUFFER_SIZE],
        .tx_buffer_size = I2C_BUFFER_SIZE
    };
    return config;
}

static constexpr int LED = D7;

/*****************************************************************************/
/*****************************************************************************/


/*****************************************************************************\
 * Infrastructure structures
 * Delta Clock, Event Hub, Software Watchdog
\*****************************************************************************/

namespace infrastructure {

    static DeltaClock deltaClock;

    static Eventing::EventHub event_hub;

    void watchdogHandler(void) {
        System.reset(RESET_NO_WAIT);
    }

    static ApplicationWatchdog *wd = NULL;

    static void init();
    static void init() {
        deltaClock.begin();
        wd = new ApplicationWatchdog(3000U, &watchdogHandler);
    }

} // namespace infrastructure

/*****************************************************************************/
/*****************************************************************************/


/*****************************************************************************\
 * Peripherals
 * Non-sensor device structures and functions
\*****************************************************************************/

namespace peripherals {

static constexpr uint32_t I2C_DEFAULT_SPEED = CLOCK_SPEED_400KHZ;
static constexpr uint32_t I2C_SAFE_SPEED    = CLOCK_SPEED_100KHZ;
static constexpr uint8_t PM_MUX_PORT = 7;

static QWIICMUX i2cMux;
static bool i2cMuxPresent = false;

void SpeedUpI2c() {
    i2cMux.disablePort(PM_MUX_PORT);
    Wire.end();
    Wire.setSpeed(I2C_DEFAULT_SPEED); // SPS30 only supports 100KHz
    Wire.begin();
}
void SlowDownI2c() {
    Wire.end();
    Wire.setSpeed(I2C_SAFE_SPEED); // SPS30 only supports 100KHz
    Wire.begin();
    i2cMux.enablePort(PM_MUX_PORT);
}

namespace Display {
    // FUll U8G2, SSD1327 controller, EA_128128 display, full framebuffer, First Arduino Hardware I2C, not rotated
    // Something about the C++ process causes lockups
    //U8G2_SSD1327_EA_W128128_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, U8X8_PIN_NONE, U8X8_PIN_NONE);
    static u8g2_t u8g2 = { 0 };

    // I thought something about u8x8_gpio_and_delay_arduino caused lockups too;
    // replacing it with a do-nothing return 0; works too.

    static constexpr u8g2_uint_t WIDTH = 128;
    static constexpr u8g2_uint_t HEIGHT = 128;

    void u8g2_ssd1327_lock() {
        u8g2_SendF(&peripherals::Display::u8g2, "ca", 0xFD, 0x12 | (1<<2));
    }

    void u8g2_ssd1327_unlock() {
        u8g2_SendF(&peripherals::Display::u8g2, "ca", 0xFD, 0x12 | (0<<2));
    }

    void u8g2_ssd1327_register_reset() {
        // SSD1327 All Register Reset
        // Set Column Address
        u8g2_SendF(&peripherals::Display::u8g2, "caa", 0x15, 0x00, 0x3F);
        // Set Row Address
        u8g2_SendF(&peripherals::Display::u8g2, "caa", 0x75, 0x00, 0x7f);
        // Set Contrast Control
        u8g2_SendF(&peripherals::Display::u8g2, "ca", 0x81, 0x7F);
        // Set Re-map
        //u8g2_SendF(&u8g2, "ca", 0xA0, (0<<7)|(0<<6)|(0<<5)|(0<<4)|(0<<3)|(1<<2)|(0<<1)|(0<<0)); // screws it up more
        // Set Display Start Line
        u8g2_SendF(&peripherals::Display::u8g2, "ca", 0xA1, 0x00);
        // Set Display Offset
        u8g2_SendF(&peripherals::Display::u8g2, "ca", 0xA2, 0x00);
        // Set Display Mode
        u8g2_SendF(&peripherals::Display::u8g2, "c", 0xA4);
        // Set MUX Ratio
        u8g2_SendF(&peripherals::Display::u8g2, "ca", 0xA8, 127);
        // Function Selection A
        // Needs schematic analysis
        // Set Display ON/OFF
        u8g2_SendF(&peripherals::Display::u8g2, "c", 0xAF);
        // Set Phase Length
        // Needs datasheet analysis
        // Set Front Clock Divider/Oscillator Frequency
        // Needs datasheet analysis
        // GPIO
        u8g2_SendF(&peripherals::Display::u8g2, "ca", 0xB5, (1<<1) | (1<<0));
        // Set Second pre-charge Period (depends on 0xD5)
        // Needs datasheet analysis
        // Set Gray Scale Table
        //u8g2_SendF(&u8g2, "ca", 0xB8, ); // Needs datasheet analysis
        // Linear LUT
        u8g2_SendF(&peripherals::Display::u8g2, "caaaaaaaaaaaaaaaa", 0xB9,
            0,
            0,
            2,
            4,
            6,
            8,
            10,
            12,
            14,
            16,
            18,
            20,
            22,
            24,
            26,
            28);
        // Set Pre-charge voltage
        // Needs schematic analysis
        // Set Vcomh
        // Needs schematic analysis
        // Function Select B
        // Needs datasheet analysis
        // Set Command Lock
        //u8g2_SendF(&u8g2, "ca", 0xFD, 0x12 | (1<<2)); // lock isn't the issue
        // Continuous Horizontal Scroll Setup
        u8g2_SendF(&peripherals::Display::u8g2, "caaaaaaa", 0xB9,
            0, // dummy
            0, // start row
            0, // step freq
            0x7F, // end row
            0, // start column
            0x3F, // end column
            0); // dummy
        // Deactivate scroll
        u8g2_SendF(&peripherals::Display::u8g2, "c", 0x2E);
        //u8g2_SendF(&u8g2, "c", 0x2F); // yup, that was it

        // blink inverted so I know things are working
        u8g2_SendF(&peripherals::Display::u8g2, "c", 0xA7);
        delay(500);
        u8g2_SendF(&peripherals::Display::u8g2, "c", 0xA4);
    }

    void init() {
        // Docs say U8G2_SSD1327_EA_W128128_F_HW_I2C, but it chops off the top and bottom 16 rows.
        // u8g2_Setup_ssd1327_i2c_ws_128x128_f seems to work well, at least empirically.
        //u8g2_Setup_ssd1327_i2c_ws_128x128_f(&u8g2, U8G2_R0, u8x8_byte_arduino_hw_i2c, u8x8_gpio_and_delay_arduino);
        u8g2_Setup_ssd1327_i2c_midas_128x128_f(&u8g2, U8G2_R0, u8x8_byte_arduino_hw_i2c, u8x8_gpio_and_delay_arduino);
        //u8g2_Setup_ssd1327_i2c_ea_w128128_f(&u8g2, U8G2_R0, u8x8_byte_arduino_hw_i2c, u8x8_gpio_and_delay_arduino);
        u8g2_InitDisplay(&u8g2);
        u8g2_SetPowerSave(&u8g2, 0);
        u8g2_SetFont(&u8g2, u8g2_font_nerhoe_tf);
        //u8g2_SetDrawColor(&u8g2, 0x8);
        u8g2_ClearBuffer(&u8g2);
        u8g2_SendBuffer(&u8g2);
    }
} // namespace Display

void init() {
    Serial.begin(115200);

    Wire.setSpeed(I2C_SAFE_SPEED);
    Wire.begin();

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
            //|0x80 // SCD30/PM
            );
    }

    // The PM sensor has been muxed off, so speed up
    //SpeedUpI2c();
    Display::init();
}

} // namespace peripherals

/*****************************************************************************/
/*****************************************************************************/


/*****************************************************************************\
 * Sensors
 * Sensor device structures and functions
\*****************************************************************************/

namespace sensors {

static LPS25HB pressureSensor;
static bool lps25hb_pressure_sensor_present = false;
// Device returns 83.2 deg F, TemporalScanner returns 80.3
// Offset is added
// 70 deg F in actively cooled airstream was -5 deg F
static constexpr float pressureSensorTempFOffset = 80.3 - 83.2; // actual - measured

Eventing::Event LPS25HB_Pressure_Event(NULL, "LPS25HB Pressure hPa", Eventing::TRIGGER_NONE);
Eventing::Event LPS25HB_Altitude_Event(NULL, "LPS25HB Altitude m", Eventing::TRIGGER_NONE);
Eventing::Event LPS25HB_TempC_Event(NULL, "LPS25HB Temp C", Eventing::TRIGGER_NONE);
Eventing::Event LPS25HB_TempF_Event(NULL, "LPS25HB Temp F", Eventing::TRIGGER_NONE);

static bool LPS25HB_data_is_ready();
static bool LPS25HB_data_is_ready() {
    constexpr uint8_t STATUS_REG_P_DA = 0x2;
    constexpr uint8_t STATUS_REG_T_DA = 0x1;
    uint8_t status = pressureSensor.getStatus();
    return lps25hb_pressure_sensor_present &&
            (status & STATUS_REG_T_DA) &&
            (status & STATUS_REG_P_DA);
}

static void ReadLPS25HB();
static void ReadLPS25HB() {
    if (lps25hb_pressure_sensor_present != false && LPS25HB_data_is_ready()) {
        Eventing::EventData tempC;
        Eventing::EventData tempF;
        Eventing::EventData pressurehPa;
        Eventing::EventData altitudem;

        tempC.fl = pressureSensor.getTemperature_degC() + pressureSensorTempFOffset * 5 / 9;
        tempF.fl = C_TO_F(tempC.fl);
        pressurehPa.fl = pressureSensor.getPressure_hPa();
        altitudem.fl = atmospherics::pressure_to_est_altitude(pressurehPa.fl);

        infrastructure::event_hub.Fire(&LPS25HB_Pressure_Event, &pressurehPa);
        infrastructure::event_hub.Fire(&LPS25HB_Altitude_Event, &altitudem);
        infrastructure::event_hub.Fire(&LPS25HB_TempC_Event, &tempC);
        infrastructure::event_hub.Fire(&LPS25HB_TempF_Event, &tempF);
    }
}
static DeltaClockEntry LPS25HBTimer = {
    .action = ReadLPS25HB,
    .interval = 1000, // pre-decimated output is at 1 Hz
    .repeating = true,
};

static SCD30 co2Sensor;
static bool co2SensorPresent = false;
// Device returns 79.0 deg F, TemporalScanner returns 80.1
// Offset is added
// 70 deg F in actively cooled airstream was -1 deg F
static constexpr float co2SensorTempFOffset = 80.1 - 79.0;

Eventing::Event SCD30_CO2_Event(NULL, "SCD30 CO2 ppm", Eventing::TRIGGER_NONE);

void ReadSCD30() {
    if (co2SensorPresent && co2Sensor.dataAvailable()) {
        Eventing::EventData rh;
        Eventing::EventData tempC;
        Eventing::EventData co2;

        rh.fl = co2Sensor.getHumidity();
        tempC.fl = co2Sensor.getTemperature() + co2SensorTempFOffset * 5 / 9;
        co2.uin16 = co2Sensor.getCO2();

        infrastructure::event_hub.Fire(&SCD30_CO2_Event, &co2);
    }
}
DeltaClockEntry SCD30Timer = {
    .action = ReadSCD30,
    .interval = 5000,
    .repeating = true,
};

static AHT20 humiditySensor;
static bool humiditySensorPresent = false;
static constexpr float humiditySensorTempOffset = 0.0;

Eventing::Event AHT20_Humidity_Event(NULL, "AHT20 Relative Humidity %%", Eventing::TRIGGER_NONE);

void ReadAHT20() {
    if (humiditySensorPresent && humiditySensor.isCalibrated()) {
        humiditySensor.triggerMeasurement();

        Eventing::EventData rh;

        rh.fl = humiditySensor.getHumidity();

        infrastructure::event_hub.Fire(&AHT20_Humidity_Event, &rh);
    }
}
static DeltaClockEntry AHT20Timer = {
    .action = ReadAHT20,
    .interval = 1000,
    .repeating = true,
};

/*
bool CalculateAbsHum(Eventing::Event* event) {
    float tempC = -NAN;
    float pressurehPa = -NAN;
    float rh = -NAN;
    for (size_t evt_idx = 0; evt_idx < event->triggers.count; evt_idx++) {
        Eventing::EventTrigger* trigger = event->triggers.list[evt_idx];
        if (trigger->data_ready) {
            if (trigger->source_event == &AHT20_Humidity_Event) {
                rh = trigger->data.fl;
            } else if (trigger->source_event == &LPS25HB_Pressure_Event) {
                pressurehPa = trigger->data.fl;
            } else if (trigger->source_event == &LPS25HB_TempC_Event) {
                tempC = trigger->data.fl;
            } else {
                //Serial.printlnf("Unhandled event \"%s\" %0.2f/%u/%d/%x", trigger->source_event->name, trigger->data.fl, trigger->data.uin16, trigger->data.in16, trigger->data.uin16);
                return false;
            }
        }
    }
    Eventing::EventData absoluteHumidity_g_m3_8_8;
    absoluteHumidity_g_m3_8_8.uin16 = atmospherics::rel_to_abs_humidity(tempC, pressurehPa, rh);
    // This wasn't exactly intended, but it's very much needed
    //Serial.printlnf("Calculated abs hum %u \"%s\"", absoluteHumidity_g_m3_8_8.uin16, event->name);
#warning Absolute Humidity self-firing event still broken
    return true;
    return infrastructure::event_hub.Fire(event, &absoluteHumidity_g_m3_8_8);
}
Eventing::Event AbsoluteHumidity_g_m3_8_8_Event(&CalculateAbsHum, "Calculate absolute humidity from relative humidity", Eventing::TRIGGER_ON_ALL);
*/

void init() {
    lps25hb_pressure_sensor_present = pressureSensor.begin();
    if (lps25hb_pressure_sensor_present) {
        // Application Note AN4672 provides guidelines for balancing power
        // consumption against signal noise. This goes with "Ultra High
        // Resolution" becuase .008 hPa RMS noise sounds nice, and why not?
        // offload averaging to onboard FIFO
        pressureSensor.setFIFOMode(LPS25HB_FIFO_CTRL_MEAN); // FIFO_CTRL, 0b110 00000
        // use decimating function
        pressureSensor.applySetting(LPS25HB_REG_CTRL_REG2, LPS25HB_CTRL_REG2_FIFO_MEAN_DEC);
        // average 32 values together at a time ("FIFO Filter Coefficient" "Running average sample size")
        pressureSensor.setFIFOMeanNum(LPS25HB_FIFO_CTRL_M_32); // FIFO_CTRL, 0b000 11111
        // sample rate of 25 Hz
        pressureSensor.setOutputDataRate(LPS25HB_CTRL_REG1_ODR_25HZ); // CTRL_REG1, 0b0 100 0000
        // oversampling of 64 for temp and 512 for pressure
        pressureSensor.setTemperatureAverages(LPS25HB_RES_CONF_T_64);
        pressureSensor.setPressureAverages(LPS25HB_RES_CONF_P_512);
    }
    co2SensorPresent = co2Sensor.begin();
    if (co2SensorPresent != false) {
        co2Sensor.setAutoSelfCalibration(true);
        co2Sensor.setMeasurementInterval(5);
    }
    humiditySensorPresent = humiditySensor.begin();
}

} // namespace sensors

/*****************************************************************************/
/*****************************************************************************/


/*****************************************************************************\
 * Data
 * Decimators and other global storage for sensor data flow
\*****************************************************************************/

/*****************************************************************************/
/*****************************************************************************/


/*****************************************************************************\
 * UX
 * Serial, OLED, and JSON rendering and emission
\*****************************************************************************/

namespace UX {

bool RenderSerial(Eventing::Event* event) {
    static float press = -NAN, tempF = -NAN, tempC = -NAN, alt = -NAN;
    static uint16_t co2ppm = -1;
    static float rh = -NAN;
    for (size_t evt_idx = 0; evt_idx < event->triggers.count; evt_idx++) {
        Eventing::EventTrigger* trigger = event->triggers.list[evt_idx];
        if (trigger->data_ready) {
            if (trigger->source_event == &sensors::LPS25HB_Pressure_Event) {
                press = trigger->data.fl;
                return true;
            } else if (trigger->source_event == &sensors::LPS25HB_Altitude_Event) {
                alt = trigger->data.fl;
                return true;
            } else if (trigger->source_event == &sensors::LPS25HB_TempC_Event) {
                tempC = trigger->data.fl;
                return true;
            } else if (trigger->source_event == &sensors::LPS25HB_TempF_Event) {
                tempF = trigger->data.fl;
            } else if (trigger->source_event == &sensors::SCD30_CO2_Event) {
                co2ppm = trigger->data.uin16;
            } else if (trigger->source_event == &sensors::AHT20_Humidity_Event) {
                rh = trigger->data.fl;
            } else {
                Serial.printlnf("Unhandled event \"%s\" %0.2f/%u/%d/%x", trigger->source_event->name, trigger->data.fl, trigger->data.uin16, trigger->data.in16, trigger->data.uin16);
            }
        }
    }
    if (sensors::lps25hb_pressure_sensor_present) {
        Serial.printlnf("LPS25HB: %0.2fhPa, %0.1fm, %0.2fF, %0.2fC", press, alt, tempF, tempC);
    } else {
        Serial.println("LPS25HB not present");
    }
    if (sensors::co2SensorPresent) {
        Serial.printlnf("SCD30: %u ppm CO2", co2ppm);
    } else {
        Serial.println("SCD30 not present");
    }
    if (sensors::humiditySensorPresent) {
        Serial.printlnf("AHT20: %0.1f%%", rh);
    } else {
        Serial.println("AHT20 not present");
    }
    //infrastructure::event_hub.DumpStateOnSerial();
    return true;
}
Eventing::Event RenderSerialEvent(&RenderSerial, "RenderSerialEvent", Eventing::EventTriggerType::TRIGGER_ON_ANY);

typedef enum _OledMode {
    HOME,
} OledMode;

bool RenderOled(Eventing::Event* event) {
    static OledMode mode = HOME;

    static float press = -NAN, tempF = -NAN;
    static uint16_t co2ppm = -1;
    static float rh = -NAN;
    for (size_t evt_idx = 0; evt_idx < event->triggers.count; evt_idx++) {
        Eventing::EventTrigger* trigger = event->triggers.list[evt_idx];
        if (trigger->data_ready) {
            if (trigger->source_event == &sensors::LPS25HB_Pressure_Event) {
                press = trigger->data.fl;
            } else if (trigger->source_event == &sensors::LPS25HB_TempF_Event) {
                tempF = trigger->data.fl;
            } else if (trigger->source_event == &sensors::SCD30_CO2_Event) {
                co2ppm = trigger->data.uin16;
            } else if (trigger->source_event == &sensors::AHT20_Humidity_Event) {
                rh = trigger->data.fl;
            } else {
                Serial.printlnf("Unhandled event \"%s\" %0.2f/%u/%d/%x", trigger->source_event->name, trigger->data.fl, trigger->data.uin16, trigger->data.in16, trigger->data.uin16);
            }
        }
    }

    u8g2_ClearBuffer(&peripherals::Display::u8g2);
    if (0) { // debugging reticle
        for (int n = 0; n < 128; n++) {
            if (n % 5 == 0) {
                u8g2_DrawPixel(&peripherals::Display::u8g2, n, 0);
                u8g2_DrawPixel(&peripherals::Display::u8g2, 0, n);
            }
            if (n % 10 == 0) {
                u8g2_DrawPixel(&peripherals::Display::u8g2, n, 1);
                u8g2_DrawPixel(&peripherals::Display::u8g2, 1, n);
            }
        }
        u8g2_DrawPixel(&peripherals::Display::u8g2, 126, 94);
        u8g2_DrawPixel(&peripherals::Display::u8g2, 127, 95);
        u8g2_DrawPixel(&peripherals::Display::u8g2, 128, 96);
        u8g2_DrawPixel(&peripherals::Display::u8g2, 126, 126);
        u8g2_DrawPixel(&peripherals::Display::u8g2, 127, 127);
        u8g2_DrawPixel(&peripherals::Display::u8g2, 128, 128);
    }

    switch (mode) {
    default:
    case HOME: {
        const uint8_t *big_font = u8g2_font_osb29_tf; // 0 is widest at 16px, 100 is 69px, -20 is 58
        constexpr u8g2_uint_t text_big_height = 29;
        const uint8_t *small_font = u8g2_font_osb18_tf; // F is 18 px, C is 17
        constexpr u8g2_uint_t text_small_height = 18;
        u8g2_SetFont(&peripherals::Display::u8g2, small_font);
        char buf[20];
        u8g2_uint_t temp_bottom_left_x = 0, temp_bottom_left_y = peripherals::Display::HEIGHT * 1 / 4 - 10;
        u8g2_uint_t hum_bottom_left_x = 0, hum_bottom_left_y = peripherals::Display::HEIGHT * 2 / 4 - 10;
        u8g2_uint_t co2_bottom_left_x = 0, co2_bottom_left_y = peripherals::Display::HEIGHT * 4 / 4 - 10;
        u8g2_uint_t press_bottom_left_x = 0, press_bottom_left_y = peripherals::Display::HEIGHT * 3 / 4 - 10;
        if (sensors::lps25hb_pressure_sensor_present) {
            snprintf(buf, sizeof(buf), "%0.1fhPa", press);
            u8g2_DrawUTF8(&peripherals::Display::u8g2, press_bottom_left_x, press_bottom_left_y, buf);
            snprintf(buf, sizeof(buf), "%0.1f\u00b0F", tempF);
            u8g2_DrawUTF8(&peripherals::Display::u8g2, temp_bottom_left_x, temp_bottom_left_y, buf);
        } else {
            u8g2_DrawUTF8(&peripherals::Display::u8g2, press_bottom_left_x, press_bottom_left_y, "-hPa");
            u8g2_DrawUTF8(&peripherals::Display::u8g2, temp_bottom_left_x, temp_bottom_left_y, "-\u00b0F");
        }
        if (sensors::co2SensorPresent) {
            snprintf(buf, sizeof(buf), "%uppm CO2", co2ppm);
            u8g2_DrawUTF8(&peripherals::Display::u8g2, co2_bottom_left_x, co2_bottom_left_y, buf);
        } else {
            u8g2_DrawUTF8(&peripherals::Display::u8g2, co2_bottom_left_x, co2_bottom_left_y, "-ppm CO2");
        }
        if (sensors::humiditySensorPresent) {
            snprintf(buf, sizeof(buf), "%0.1f%% rh", rh);
            u8g2_DrawUTF8(&peripherals::Display::u8g2, hum_bottom_left_x, hum_bottom_left_y, buf);
        } else {
            u8g2_DrawUTF8(&peripherals::Display::u8g2, hum_bottom_left_x, hum_bottom_left_y, "-% rh");
        }
    }
    }
    unsigned long drawStart = millis();
    peripherals::Display::u8g2_ssd1327_unlock();
    u8g2_SendBuffer(&peripherals::Display::u8g2);
    peripherals::Display::u8g2_ssd1327_lock();
    unsigned long drawEnd = millis();
    Serial.printlnf("draw latency: %lu ms", drawEnd - drawStart);
    return true;
}
Eventing::Event RenderOledEvent(&RenderOled, "RenderOledEvent", Eventing::EventTriggerType::TRIGGER_ON_ALL);

} // namespace UX

/*****************************************************************************/
/*****************************************************************************/


/*****************************************************************************\
 * Data Flow
 * Event hub members
\*****************************************************************************/

namespace flow {

// Delta clock handlers need to be after events so they can take addresses to fire with

// One Init To Rule Them All, And In The Setup, Bind Them
void init() {
    infrastructure::deltaClock.insert(&sensors::LPS25HBTimer);
    infrastructure::deltaClock.insert(&sensors::SCD30Timer);
    infrastructure::deltaClock.insert(&sensors::AHT20Timer);
    infrastructure::event_hub.Add(&sensors::LPS25HB_Pressure_Event);
    infrastructure::event_hub.Add(&sensors::LPS25HB_Altitude_Event);
    infrastructure::event_hub.Add(&sensors::LPS25HB_TempC_Event);
    infrastructure::event_hub.Add(&sensors::LPS25HB_TempF_Event);
    infrastructure::event_hub.Add(&sensors::SCD30_CO2_Event);
    infrastructure::event_hub.Add(&sensors::AHT20_Humidity_Event);
/*
    infrastructure::event_hub.Add(&sensors::AbsoluteHumidity_g_m3_8_8_Event);
    sensors::AbsoluteHumidity_g_m3_8_8_Event.AddTrigger(&sensors::LPS25HB_TempC_Event);
    sensors::AbsoluteHumidity_g_m3_8_8_Event.AddTrigger(&sensors::LPS25HB_Pressure_Event);
    sensors::AbsoluteHumidity_g_m3_8_8_Event.AddTrigger(&sensors::AHT20_Humidity_Event);
*/
    infrastructure::event_hub.Add(&UX::RenderSerialEvent);
    UX::RenderSerialEvent.AddTrigger(&sensors::LPS25HB_Pressure_Event);
    UX::RenderSerialEvent.AddTrigger(&sensors::LPS25HB_Altitude_Event);
    UX::RenderSerialEvent.AddTrigger(&sensors::LPS25HB_TempC_Event);
    UX::RenderSerialEvent.AddTrigger(&sensors::LPS25HB_TempF_Event);
    UX::RenderSerialEvent.AddTrigger(&sensors::SCD30_CO2_Event);
    UX::RenderSerialEvent.AddTrigger(&sensors::AHT20_Humidity_Event);
    //UX::RenderSerialEvent.AddTrigger(&sensors::AbsoluteHumidity_g_m3_8_8_Event);
    infrastructure::event_hub.Add(&UX::RenderOledEvent);
    UX::RenderOledEvent.AddTrigger(&sensors::LPS25HB_TempF_Event);
    UX::RenderOledEvent.AddTrigger(&sensors::LPS25HB_Pressure_Event);
    UX::RenderOledEvent.AddTrigger(&sensors::SCD30_CO2_Event);
    UX::RenderOledEvent.AddTrigger(&sensors::AHT20_Humidity_Event);
}

} // namespace Flow

/*****************************************************************************/
/*****************************************************************************/


/*****************************************************************************\
 * Arduino
\*****************************************************************************/

void setup() {
    infrastructure::init();
    peripherals::init();
    sensors::init();
    flow::init();
}

void loop() {
    //Serial.printlnf("Loop Start %lu", millis());
    ApplicationWatchdog::checkin();
    infrastructure::deltaClock.update();
    delay(400);
}

/*****************************************************************************/
/*****************************************************************************/
