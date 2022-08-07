
// checked-in dependencies
#include "decimator.h"
#include "Atmospherics.h"
#include "sps30.h"
//#define EVENTHUB_DEBUG
#define EVENTHUB_TEMPORAL
#include "Eventing.h"
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

//SerialLogHandler logHandler(LOG_LEVEL_ALL);

// To use I2C buffers bigger than 32 bytes, we provide a function for allocating the buffers. Particle-specific.
// https://docs.staging.particle.io/cards/firmware/wire-i2c/acquirewirebuffer/
static constexpr size_t I2C_BUFFER_SIZE = 128;

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

    static Eventing::EventHub event_hub;

    static ApplicationWatchdog *wd = NULL;

    void watchdogHandler(void) {
        Serial.printlnf("ApplicationWatchdog triggered!");
        Serial.printlnf("######### FreeRAM: %lu Uptime: %ld", System.freeMemory(), millis());
        Serial.flush();
        System.reset(RESET_NO_WAIT);
    }

    static void init();
    static void init() {
        event_hub.begin();
        wd = new ApplicationWatchdog(30000U, &watchdogHandler, 1536);
    }

    bool DumpOsState(Eventing::PointerList<Eventing::EventTrigger>& triggers, Eventing::EventData& out) {
        Serial.printlnf("######### FreeRAM: %lu Uptime: %ld", System.freeMemory(), millis());
        return false;
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
static constexpr uint8_t PM_MUX_PORT = 4;
static constexpr uint8_t CO2_MUX_PORT = 3;

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

    bool BufferIsDirty = false;
    bool Paint(Eventing::PointerList<Eventing::EventTrigger>& triggers, Eventing::EventData& out) {
        if (BufferIsDirty != false) {
            unsigned long drawStart = millis();
            u8g2_ssd1327_unlock();
            u8g2_SendBuffer(&u8g2);
            u8g2_ssd1327_lock();
            unsigned long drawEnd = millis();
            Serial.printlnf("draw latency: %lu ms", drawEnd - drawStart);
            BufferIsDirty = false;
        }
        return false;
    }

    void init() {
        // Docs say U8G2_SSD1327_EA_W128128_F_HW_I2C, but it chops off the top and bottom 16 rows.
        // u8g2_Setup_ssd1327_i2c_ws_128x128_f seems to work well, at least empirically.
        //u8g2_Setup_ssd1327_i2c_ws_128x128_f(&u8g2, U8G2_R0, u8x8_byte_arduino_hw_i2c, u8x8_gpio_and_delay_arduino);
        u8g2_Setup_ssd1327_i2c_midas_128x128_f(&u8g2, U8G2_R3, u8x8_byte_arduino_hw_i2c, u8x8_gpio_and_delay_arduino);
        //u8g2_Setup_ssd1327_i2c_ea_w128128_f(&u8g2, U8G2_R0, u8x8_byte_arduino_hw_i2c, u8x8_gpio_and_delay_arduino);
        u8g2_InitDisplay(&u8g2);
        u8g2_SetPowerSave(&u8g2, 0);
        u8g2_ssd1327_register_reset();
        u8g2_SetFont(&u8g2, u8g2_font_nerhoe_tf);
        //u8g2_SetDrawColor(&u8g2, 0x8);
        u8g2_ClearBuffer(&u8g2);
        u8g2_SendBuffer(&u8g2);
    }
} // namespace Display

namespace Joystick {
    static JOYSTICK joystick;
    static bool joystickPresent = false;
    typedef enum _JOYSTICK_DIRECTION {
        UP,
        DOWN,
        LEFT,
        RIGHT,
        UP_RIGHT,
        UP_LEFT,
        DOWN_RIGHT,
        DOWN_LEFT,
        CENTER,
    } JOYSTICK_DIRECTION;
    static constexpr uint16_t BOUNDARY_SIZE = 200;
    static constexpr uint16_t MAX = 1023;
    static constexpr uint16_t LEFT_THRESHOLD = BOUNDARY_SIZE;
    static constexpr uint16_t RIGHT_THRESHOLD = MAX - BOUNDARY_SIZE;
    static constexpr uint16_t UP_THRESHOLD = BOUNDARY_SIZE;
    static constexpr uint16_t DOWN_THRESHOLD = MAX - BOUNDARY_SIZE;

    static JOYSTICK_DIRECTION ReadJoystick();
    static JOYSTICK_DIRECTION ReadJoystick() {
        uint16_t horiz = joystick.getHorizontal();
        uint16_t vert = joystick.getVertical();

        JOYSTICK_DIRECTION joyDir;
        if (vert > DOWN_THRESHOLD && horiz > RIGHT_THRESHOLD) {
            joyDir = DOWN_RIGHT;
        } else if (vert > DOWN_THRESHOLD && horiz < LEFT_THRESHOLD) {
            joyDir = DOWN_LEFT;
        } else if (vert < UP_THRESHOLD && horiz > RIGHT_THRESHOLD) {
            joyDir = UP_RIGHT;
        } else if (vert < UP_THRESHOLD && horiz < LEFT_THRESHOLD) {
            joyDir = UP_LEFT;
        } else if (vert > DOWN_THRESHOLD) {
            joyDir = DOWN;
        } else if (vert < UP_THRESHOLD) {
            joyDir = UP;
        } else if (horiz < LEFT_THRESHOLD) {
            joyDir = LEFT;
        } else if (horiz > RIGHT_THRESHOLD) {
            joyDir = RIGHT;
        } else {
            joyDir = CENTER;
        }
        Serial.printlnf("Got joy reading x=%d,y=%d", horiz, vert);
        return joyDir;
    }

    void EmitChangeEvent();
    void EmitChangeEvent() {
        static JOYSTICK_DIRECTION joyDirOld = JOYSTICK_DIRECTION::CENTER;

        if (joystickPresent == false) {
            return;
        }

        JOYSTICK_DIRECTION joyDir1 = ReadJoystick();
        delay(10);
        JOYSTICK_DIRECTION joyDir2 = ReadJoystick();

        Serial.printlnf("Joy dir change check; joyDir1=%d joyDir2=%d old=%d", joyDir1, joyDir2, joyDirOld);
        if (joyDir1 == joyDir2 && joyDir1 != joyDirOld) {
            Eventing::EventData joystickData;
            joystickData.uin16 = joyDir1;
            joyDirOld = joyDir1;

            Serial.printlnf("Joy dir change event sent");
            infrastructure::event_hub.Deliver("Joystick Direction Change", joystickData);
        }
    }
} // namespace Joystick

void init() {
    Serial.begin(115200);

    Wire.setSpeed(I2C_SAFE_SPEED);
    Wire.begin();

    i2cMuxPresent = i2cMux.begin();
    if (i2cMuxPresent) {
        i2cMux.setPortState(0
            //|0x1 // nc
            //|0x2 // nc
            //|0x4 // nc
            | (1 << CO2_MUX_PORT) // CO2
            //|0x10 // nc
            //|0x20 // nc
            //|0x40 // nc
            //|0x80 // SCD30/PM
            );
    }

    // The PM sensor has been muxed off, so speed up
    SpeedUpI2c();
    Display::init();

    Joystick::joystickPresent = Joystick::joystick.begin();
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

static bool LPS25HB_data_is_ready();
static bool LPS25HB_data_is_ready() {
    constexpr uint8_t STATUS_REG_P_DA = 0x2;
    constexpr uint8_t STATUS_REG_T_DA = 0x1;
    uint8_t status = pressureSensor.getStatus();
    return lps25hb_pressure_sensor_present &&
            (status & STATUS_REG_T_DA) &&
            (status & STATUS_REG_P_DA);
}

static bool ReadLPS25HB(Eventing::PointerList<Eventing::EventTrigger>& triggers, Eventing::EventData& out);
static bool ReadLPS25HB(Eventing::PointerList<Eventing::EventTrigger>& triggers, Eventing::EventData& out) {
    if (lps25hb_pressure_sensor_present != false && LPS25HB_data_is_ready()) {
        Eventing::EventData tempC;
        Eventing::EventData tempF;
        Eventing::EventData pressurehPa;
        Eventing::EventData altitudem;

        tempC.fl = pressureSensor.getTemperature_degC() + pressureSensorTempFOffset * 5 / 9;
        tempF.fl = C_TO_F(tempC.fl);
        pressurehPa.fl = pressureSensor.getPressure_hPa();
        altitudem.fl = atmospherics::pressure_to_est_altitude(pressurehPa.fl);

        infrastructure::event_hub.Deliver(String("LPS25HB Pressure hPa"), pressurehPa);
        infrastructure::event_hub.Deliver(String("LPS25HB Altitude m"), altitudem);
        infrastructure::event_hub.Deliver(String("LPS25HB Temp C"), tempC);
        infrastructure::event_hub.Deliver(String("LPS25HB Temp F"), tempF);
    }

    // Data is delivered with Deliver, so out param is unused.
    return false;
}

static SCD30 co2Sensor;
static bool co2SensorPresent = false;
static constexpr uint16_t co2SensorInterval = 10;
// Device returns 79.0 deg F, TemporalScanner returns 80.1
// Offset is added
// 70 deg F in actively cooled airstream was -1 deg F
static constexpr float co2SensorTempFOffset = 80.1 - 79.0;

static bool ReadSCD30(Eventing::PointerList<Eventing::EventTrigger>& triggers, Eventing::EventData& out);
static bool ReadSCD30(Eventing::PointerList<Eventing::EventTrigger>& triggers, Eventing::EventData& out) {
    if (co2SensorPresent && co2Sensor.dataAvailable()) {
        Eventing::EventData rh;
        Eventing::EventData tempC;
        Eventing::EventData co2;

        rh.fl = co2Sensor.getHumidity();
        tempC.fl = co2Sensor.getTemperature() + co2SensorTempFOffset * 5 / 9;
        co2.uin16 = co2Sensor.getCO2();

        infrastructure::event_hub.Deliver(String("SCD30 CO2 ppm"), co2);
    }
    // Data is delivered with Deliver, so out param is unused.
    return false;
}

static AHT20 humiditySensor;
static bool humiditySensorPresent = false;
static constexpr float humiditySensorTempOffset = 0.0;

static bool ReadAHT20(Eventing::PointerList<Eventing::EventTrigger>& triggers, Eventing::EventData& out);
static bool ReadAHT20(Eventing::PointerList<Eventing::EventTrigger>& triggers, Eventing::EventData& out) {
    if (humiditySensorPresent && humiditySensor.isCalibrated()) {
        humiditySensor.triggerMeasurement();
        out.fl = humiditySensor.getHumidity();
        return true;
    }
    return false;
}

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

static SPS30 pmSensor;
static bool pmSensorPresent = false;

static SPS30_DATA_FLOAT sps30_global_datum_struct;
bool ReadSPS30(Eventing::PointerList<Eventing::EventTrigger>& triggers, Eventing::EventData& out) {
    if (pmSensorPresent) {
        bool sendData = false;
        SPS30_ERR readyErr, retrieveErr;
        bool dataIsReady, dataRetrieved;
        peripherals::SlowDownI2c();
        readyErr = pmSensor.is_data_ready();
        // verbose for easy addition of debug statements
        dataIsReady = readyErr == SPS30_OK;
        retrieveErr = pmSensor.read_data_no_wait_float(&sps30_global_datum_struct);
        dataRetrieved = retrieveErr == SPS30_OK;
        sendData = dataIsReady && dataRetrieved;
        peripherals::SpeedUpI2c();
        if (sendData) {
            out.ptr = &sps30_global_datum_struct;
        }
        return sendData;
    }
    return false;
}

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
        co2Sensor.setMeasurementInterval(co2SensorInterval);
    }
    humiditySensorPresent = humiditySensor.begin();
    peripherals::SlowDownI2c();
    pmSensorPresent = false;//pmSensor.begin();
    if (pmSensorPresent) {
        pmSensorPresent = pmSensorPresent && (pmSensor.start_measuring(SPS30_FORMAT_IEEE754) == SPS30_OK);
        sps30_global_datum_struct = {
            .pm_1_0_ug_m3 = -NAN,
            .pm_2_5_ug_m3 = -NAN,
            .pm_4_0_ug_m3 = -NAN,
            .pm_10_ug_m3 = -NAN,
            .pm_0_5_n_cm3 = -NAN,
            .pm_1_0_n_cm3 = -NAN,
            .pm_2_5_n_cm3 = -NAN,
            .pm_4_0_n_cm3 = -NAN,
            .pm_10_n_cm3 = -NAN,
            .typical_size_um = -NAN,
        };
    }
    peripherals::SpeedUpI2c();
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

bool RenderSerial(Eventing::PointerList<Eventing::EventTrigger>& triggers, Eventing::EventData& out) {
    static float press = -NAN, tempF = -NAN, tempC = -NAN, alt = -NAN;
    static uint16_t co2ppm = -1;
    static float rh = -NAN;
    static SPS30_DATA_FLOAT pm = { 0 };
    for (size_t evt_idx = 0; evt_idx < triggers.count; evt_idx++) {
        Eventing::EventTrigger* trigger = triggers.list[evt_idx];
        if (trigger->data_ready) {
            if (trigger->event_id.equalsIgnoreCase(String("LPS25HB Pressure hPa"))) {
                press = trigger->data.fl;
                return false;
            } else if (trigger->event_id.equalsIgnoreCase(String("LPS25HB Altitude m"))) {
                alt = trigger->data.fl;
                return false;
            } else if (trigger->event_id.equalsIgnoreCase(String("LPS25HB Temp C"))) {
                tempC = trigger->data.fl;
                return false;
            } else if (trigger->event_id.equalsIgnoreCase(String("LPS25HB Temp F"))) {
                tempF = trigger->data.fl;
            } else if (trigger->event_id.equalsIgnoreCase(String("SCD30 CO2 ppm"))) {
                co2ppm = trigger->data.uin16;
            } else if (trigger->event_id.equalsIgnoreCase(String("AHT20 Relative Humidity %%"))) {
                rh = trigger->data.fl;
            } else if (trigger->event_id.equalsIgnoreCase(String("SPS30 Raw"))) {
                pm = *((SPS30_DATA_FLOAT*)trigger->data.ptr);
            } else {
                Serial.printlnf("Unhandled event \"%s\" %0.2f/%u/%d/%x/%p", trigger->event_id.c_str(), trigger->data.fl, trigger->data.uin16, trigger->data.in16, trigger->data.uin16, trigger->data.ptr);
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
    if (sensors::pmSensorPresent) {
        String str;
        SPS30::float_append_to_string(pm, str);
        Serial.printlnf("%s", str.c_str());
    } else {
        Serial.println("SPS30 not present");
    }
    //infrastructure::event_hub.DumpStateOnSerial();
    return false;
}

typedef enum _OledMode {
    HOME,
} OledMode;

bool RenderOled(Eventing::PointerList<Eventing::EventTrigger>& triggers, Eventing::EventData& out) {
    static OledMode mode = HOME;

    static float press = -NAN, tempF = -NAN;
    static uint16_t co2ppm = -1;
    static float rh = -NAN;
    for (size_t evt_idx = 0; evt_idx < triggers.count; evt_idx++) {
        Eventing::EventTrigger* trigger = triggers.list[evt_idx];
        if (trigger->data_ready) {
            if (trigger->event_id.equalsIgnoreCase(String("LPS25HB Pressure hPa"))) {
                press = trigger->data.fl;
            } else if (trigger->event_id.equalsIgnoreCase(String("LPS25HB Temp F"))) {
                tempF = trigger->data.fl;
            } else if (trigger->event_id.equalsIgnoreCase(String("SCD30 CO2 ppm"))) {
                co2ppm = trigger->data.uin16;
            } else if (trigger->event_id.equalsIgnoreCase(String("AHT20 Relative Humidity %%"))) {
                rh = trigger->data.fl;
            } else {
                Serial.printlnf("Unhandled event \"%s\" %0.2f/%u/%d/%x", trigger->event_id.c_str(), trigger->data.fl, trigger->data.uin16, trigger->data.in16, trigger->data.uin16);
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
    peripherals::Display::BufferIsDirty = true;
    return false;
}

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
    infrastructure::event_hub.AddHandler(String("LPS25HB Raw"), sensors::ReadLPS25HB, Eventing::TRIGGER_TEMPORAL, 1000); // pre-decimated output is at 1 Hz
    infrastructure::event_hub.AddHandler(String("SCD30 Raw"), sensors::ReadSCD30, Eventing::TRIGGER_TEMPORAL, sensors::co2SensorInterval*1000);
    infrastructure::event_hub.AddHandler(String("AHT20 Relative Humidity %%"), sensors::ReadAHT20, Eventing::TRIGGER_TEMPORAL, 1000);
    infrastructure::event_hub.AddHandler(String("SPS30 Raw"), sensors::ReadSPS30, Eventing::TRIGGER_TEMPORAL, 2500);
/*
    infrastructure::event_hub.Add(&sensors::AbsoluteHumidity_g_m3_8_8_Event);
    sensors::AbsoluteHumidity_g_m3_8_8_Event.AddTrigger(&sensors::LPS25HB_TempC_Event);
    sensors::AbsoluteHumidity_g_m3_8_8_Event.AddTrigger(&sensors::LPS25HB_Pressure_Event);
    sensors::AbsoluteHumidity_g_m3_8_8_Event.AddTrigger(&sensors::AHT20_Humidity_Event);
*/
    infrastructure::event_hub.AddHandler(String("RenderSerialEvent"), UX::RenderSerial, Eventing::TRIGGER_ON_ANY);
    infrastructure::event_hub.AddHandlerTrigger(String("RenderSerialEvent"), String("LPS25HB Pressure hPa"));
    infrastructure::event_hub.AddHandlerTrigger(String("RenderSerialEvent"), String("LPS25HB Altitude m"));
    infrastructure::event_hub.AddHandlerTrigger(String("RenderSerialEvent"), String("LPS25HB Temp C"));
    infrastructure::event_hub.AddHandlerTrigger(String("RenderSerialEvent"), String("LPS25HB Temp F"));
    infrastructure::event_hub.AddHandlerTrigger(String("RenderSerialEvent"), String("SCD30 CO2 ppm"));
    infrastructure::event_hub.AddHandlerTrigger(String("RenderSerialEvent"), String("AHT20 Relative Humidity %%"));
    infrastructure::event_hub.AddHandlerTrigger(String("RenderSerialEvent"), String("SPS30 Raw"));
    //UX::RenderSerialEvent.AddTrigger(&sensors::AbsoluteHumidity_g_m3_8_8_Event);
    infrastructure::event_hub.AddHandler("RenderOledEvent", UX::RenderOled, Eventing::TRIGGER_ON_ANY);
    infrastructure::event_hub.AddHandlerTrigger(String("RenderOledEvent"), String("LPS25HB Temp F"));
    infrastructure::event_hub.AddHandlerTrigger(String("RenderOledEvent"), String("LPS25HB Pressure hPa"));
    infrastructure::event_hub.AddHandlerTrigger(String("RenderOledEvent"), String("SCD30 CO2 ppm"));
    infrastructure::event_hub.AddHandlerTrigger(String("RenderOledEvent"), String("AHT20 Relative Humidity %%"));
    infrastructure::event_hub.AddHandlerTrigger(String("RenderOledEvent"), String("SPS30 Raw"));
    infrastructure::event_hub.AddHandler("PaintOled", peripherals::Display::Paint, Eventing::TRIGGER_TEMPORAL, 500);
    //infrastructure::event_hub.AddHandler("DumpOsState", infrastructure::DumpOsState, Eventing::TRIGGER_TEMPORAL, 5000);
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
    infrastructure::event_hub.update();
    peripherals::Joystick::EmitChangeEvent();
}

/*****************************************************************************/
/*****************************************************************************/
