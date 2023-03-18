
// checked-in dependencies
#include "decimator.h"
#include "sparkline.h"
#include "Atmospherics.h"
#include "sps30.h"
//#define EVENTHUB_DEBUG
#define EVENTHUB_TEMPORAL
#include "Eventing.h"
// checked-in 3p dependencies
#include "SparkFun_SGP30_Arduino_Library.h"
#include "U8g2lib.h"
#include "boxen.h"

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
SYSTEM_MODE(SEMI_AUTOMATIC);

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

typedef struct _KnownNetworkPresenceData {
    bool KnownNetworkPresent;
    WiFiAccessPoint *creds;
    int credsFound;
} KnownNetworkPresenceData;

void KnownNetworkScanCallback(WiFiAccessPoint* ap, void* context) {
    KnownNetworkPresenceData *data = (KnownNetworkPresenceData*)context;
    String apString(ap->ssid, ap->ssidLength);
    for (int idx = 0; idx < data->credsFound; idx++) {
        String credString(data->creds[idx].ssid, data->creds[idx].ssidLength);
        if (credString.equals(apString)) {
            data->KnownNetworkPresent = true;
        }
    }
}

bool IsKnownNetworkPresent() {
    KnownNetworkPresenceData data = { 0 };
    // Particle Photon can hold up to 5 sets of WiFi creds at once.
    data.creds = (WiFiAccessPoint *)malloc(sizeof(WiFiAccessPoint) * 5);
    data.credsFound = WiFi.getCredentials(data.creds, 5);
    data.KnownNetworkPresent = false;
    WiFi.scan(KnownNetworkScanCallback, (void*)&data);
    free(data.creds);
    return data.KnownNetworkPresent;
}

namespace Display {
    static U8G2 *u8g2_oo;
    static u8g2_t u8g2Buf = { 0 };
    static u8g2_t *u8g2 = &u8g2Buf;

    static constexpr u8g2_uint_t WIDTH = 128;
    static constexpr u8g2_uint_t HEIGHT = 128;

    void u8g2_ssd1327_lock() {
        u8g2_SendF(peripherals::Display::u8g2, "ca", 0xFD, 0x12 | (1<<2));
    }

    void u8g2_ssd1327_unlock() {
        u8g2_SendF(peripherals::Display::u8g2, "ca", 0xFD, 0x12 | (0<<2));
    }

    void u8g2_ssd1327_register_reset() {
        // SSD1327 All Register Reset
        // Set Column Address
        u8g2_SendF(peripherals::Display::u8g2, "caa", 0x15, 0x00, 0x3F);
        // Set Row Address
        u8g2_SendF(peripherals::Display::u8g2, "caa", 0x75, 0x00, 0x7f);
        // Set Contrast Control
        u8g2_SendF(peripherals::Display::u8g2, "ca", 0x81, 0x7F);
        // Set Re-map
        //u8g2_SendF(u8g2, "ca", 0xA0, (0<<7)|(0<<6)|(0<<5)|(0<<4)|(0<<3)|(1<<2)|(0<<1)|(0<<0)); // screws it up more
        // Set Display Start Line
        u8g2_SendF(peripherals::Display::u8g2, "ca", 0xA1, 0x00);
        // Set Display Offset
        u8g2_SendF(peripherals::Display::u8g2, "ca", 0xA2, 0x00);
        // Set Display Mode
        u8g2_SendF(peripherals::Display::u8g2, "c", 0xA4);
        // Set MUX Ratio
        u8g2_SendF(peripherals::Display::u8g2, "ca", 0xA8, 127);
        // Function Selection A
        // Needs schematic analysis
        // Set Display ON/OFF
        u8g2_SendF(peripherals::Display::u8g2, "c", 0xAF);
        // Set Phase Length
        // Needs datasheet analysis
        // Set Front Clock Divider/Oscillator Frequency
        // Needs datasheet analysis
        // GPIO
        u8g2_SendF(peripherals::Display::u8g2, "ca", 0xB5, (1<<1) | (1<<0));
        // Set Second pre-charge Period (depends on 0xD5)
        // Needs datasheet analysis
        // Set Gray Scale Table
        //u8g2_SendF(u8g2, "ca", 0xB8, ); // Needs datasheet analysis
        // Linear LUT
        u8g2_SendF(peripherals::Display::u8g2, "caaaaaaaaaaaaaaaa", 0xB9,
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
        //u8g2_SendF(u8g2, "ca", 0xFD, 0x12 | (1<<2)); // lock isn't the issue
        // Continuous Horizontal Scroll Setup
        u8g2_SendF(peripherals::Display::u8g2, "caaaaaaa", 0xB9,
            0, // dummy
            0, // start row
            0, // step freq
            0x7F, // end row
            0, // start column
            0x3F, // end column
            0); // dummy
        // Deactivate scroll
        u8g2_SendF(peripherals::Display::u8g2, "c", 0x2E);
        //u8g2_SendF(u8g2, "c", 0x2F); // yup, that was it

        // blink inverted so I know things are working
        u8g2_SendF(peripherals::Display::u8g2, "c", 0xA7);
        delay(500);
        u8g2_SendF(peripherals::Display::u8g2, "c", 0xA4);
    }

    void u8g2_ssd1327_errata_workaround() {
        // Somehow some configuration bits sometimes get flipped; fix just the ones I know are busted:
        // Deactivate scroll
        u8g2_SendF(peripherals::Display::u8g2, "c", 0x2E);
    }

    bool BufferIsDirty = false;
    bool Paint(Eventing::PointerList<Eventing::EventTrigger>& triggers, Eventing::EventData& out) {
        if (BufferIsDirty != false) {
            //unsigned long drawStart = millis();
            u8g2_ssd1327_unlock();
            u8g2_ssd1327_errata_workaround();
            u8g2_SendBuffer(u8g2);
            u8g2_ssd1327_lock();
            //unsigned long drawEnd = millis();
            //Serial.printlnf("draw latency: %lu ms", drawEnd - drawStart);
            BufferIsDirty = false;
        }
        return false;
    }

    void init() {
        // FUll U8G2, SSD1327 controller, Midas 128x128 display, full framebuffer, First Arduino Hardware I2C, rotated so USB is on left
        // Docs say U8G2_SSD1327_EA_W128128_F_HW_I2C, but it chops off the top and bottom 16 rows.
        // u8g2_Setup_ssd1327_i2c_ws_128x128_f seems to work well, at least empirically.
        // Something about the C++ process causes lockups, at least on toolchain 3.3.0.
        // I thought something about u8x8_gpio_and_delay_arduino caused lockups too;
        // replacing it with a do-nothing 'return 0;' worked when I encountered that.

        u8g2_Setup_ssd1327_i2c_midas_128x128_f(u8g2, U8G2_R3, u8x8_byte_arduino_hw_i2c, u8x8_gpio_and_delay_arduino);
        //u8g2_oo = new U8G2_SSD1327_MIDAS_128X128_F_HW_I2C(U8G2_R1);
        //u8g2_oo->beginSimple();
        //u8g2 = u8g2_oo->getU8g2();
        u8g2_InitDisplay(u8g2);
        u8g2_SetPowerSave(u8g2, 0);
        u8g2_ssd1327_register_reset();
        u8g2_SetFont(u8g2, u8g2_font_nerhoe_tf);
        //u8g2_SetDrawColor(u8g2, 1); // GetDrawColor returns 0 anyways
        u8g2_ClearBuffer(u8g2);
        u8g2_SendBuffer(u8g2);
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
        //Serial.printlnf("Got joy reading x=%d,y=%d", horiz, vert);
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

        //Serial.printlnf("Joy dir change check; joyDir1=%d joyDir2=%d old=%d", joyDir1, joyDir2, joyDirOld);
        if (joyDir1 == joyDir2 && joyDir1 != joyDirOld) {
            Eventing::EventData joystickData;
            joystickData.uin16 = joyDir1;
            joyDirOld = joyDir1;

            //Serial.printlnf("Joy dir change event sent");
            infrastructure::event_hub.Deliver("Joystick Direction Change", joystickData);
        }
    }
} // namespace Joystick

namespace NvStorage {
    struct {
        uint8_t version;
        uint16_t vocBaselineCo2;
        uint16_t vocBaselineTvoc;
    } NvSettings;
    int NvSettingsAddress = 0;

    constexpr uint8_t currentVersion = 1;
    static_assert(currentVersion != 0x00, "The version must be distinguishable from the new-firmware value 0x00.");
    static_assert(currentVersion != 0xFF, "The version must be distinguishable from the erased-flash value 0xFF.");

    void begin() {
        // The Photon has 2047 bytes of emulated (flash-backed, page-erase-worn) EEPROM
        // It appears that when the firmware is updated, the EEPROM area is zero'd out,
        // so the baseline isn't preserved across flashes and the version can't start
        // at zero.
        EEPROM.get(NvSettingsAddress, NvSettings);
        if (NvSettings.version != currentVersion) {
            EEPROM.clear();
            EEPROM.get(NvSettingsAddress, NvSettings);
            NvSettings.version = currentVersion;
            NvSettings.vocBaselineCo2 = 40014;  // eco2 39917,40076,40014
            NvSettings.vocBaselineTvoc = 41427; // tvoc 41061,41336,41427
            EEPROM.put(NvSettingsAddress, NvSettings);
        }
    }

    void commit() {
        EEPROM.put(NvSettingsAddress, NvSettings);
    }

    void Print() {
        Serial.printlnf("v%d bco2:%d btvoc:%d", NvSettings.version, NvSettings.vocBaselineCo2, NvSettings.vocBaselineTvoc);
    }
} // namespace NvStorage

void init() {
    Serial.begin(115200);

    NvStorage::begin();

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

//
// The SCD30's Autmatic Self-Calibration (ASC) feature requires regular reocnditioning:
// * A 7-day consecutive run period with 1h/d of fresh air will set the internal
//   non-volatile calibration values
// * The sensor must be exposed to >=400ppm concentrations "regularly"
// * ASC has to be actively enabled, but enablement is non-volatile
// * The sensor must be exposed to 1h/d fresh air
// * Continuous Measurement must be used
//
// External (forced) recalibration requires a 2-minute steady environment with CO2
// concentration in the range 400ppm - 2000ppm. After reset this cannot be read back.
//

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
        infrastructure::event_hub.Deliver(String("SCD30 temp C"), tempC);
        infrastructure::event_hub.Deliver(String("SCD30 rh %"), rh);
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

static bool CalculateAbsoluteHumidity_8_8_g_m3(Eventing::PointerList<Eventing::EventTrigger>& triggers, Eventing::EventData& out);
static bool CalculateAbsoluteHumidity_8_8_g_m3(Eventing::PointerList<Eventing::EventTrigger>& triggers, Eventing::EventData& out) {
    float tempC = -NAN;
    float pressurehPa = -NAN;
    float rh = -NAN;
    for (size_t evt_idx = 0; evt_idx < triggers.count; evt_idx++) {
        Eventing::EventTrigger* trigger = triggers.list[evt_idx];
        if (trigger->data_ready) {
            if (trigger->event_id.equalsIgnoreCase(String("LPS25HB Pressure hPa"))) {
                pressurehPa = trigger->data.fl;
            } else if (trigger->event_id.equalsIgnoreCase(String("LPS25HB Temp C"))) {
                tempC = trigger->data.fl;
            } else if (trigger->event_id.equalsIgnoreCase(String("AHT20 Relative Humidity %%"))) {
                rh = trigger->data.fl;
            }
        }
    }
    if (tempC != -NAN && pressurehPa != -NAN && rh != -NAN) {
        out.uin16 = atmospherics::rel_to_abs_humidity(tempC, pressurehPa, rh);
        return true;
    } else {
        return false;
    }
}

//
// The SGP30 can return raw H2 and Ethanol concentrations, but they are not absolute measures and are
// related to each other with a formula in the datasheets.
//
// It may be possible to approximate calibration by relating values measured in open atmosphere to
// average atmospheric composition values.
//
// https://en.wikipedia.org/wiki/Atmosphere_of_Earth
//
// One set of such values can be found in the Earth section of Allen's Astrophysical Quantities, but
// Google Books omits the two pages with the table. Springer Link offers it as an eBook for a
// considerable sum.
//
// CRC Handbook of Chemistry and Physics also has the same problem, and just cites Allen's anynow.
//
// A paper about atmospheric ethanol concentrations, "Wet deposition ethanol concentration at US
// atmospheric integrated research monitoring network (AIRMoN) sites" (Journal of Atmospheric
// Chemistry, 2021), may contain useful numbers and background--for ethanol, at least.
//
// The SGP30 needs to run for at least 12 hours to establish its baseline calibration, which can then
// be restored after a reset to avoid danother 12-hour baselining period.
//

static SGP30 vocSensor;
static bool vocSensorPresent = false;
constexpr uint32_t vocReadInterval = 1000;

static bool ReadSGP30(Eventing::PointerList<Eventing::EventTrigger>& triggers, Eventing::EventData& out);
static bool ReadSGP30(Eventing::PointerList<Eventing::EventTrigger>& triggers, Eventing::EventData& out) {
    if (vocSensorPresent) {
        Eventing::EventData datum;
        vocSensor.measureAirQuality();
        if (vocSensor.CO2 == 400 && vocSensor.TVOC == 0) {
            // Sensor is still initializing (first 15s after init)
            return false;
        }
        datum.uin16 = vocSensor.TVOC;
        infrastructure::event_hub.Deliver(String("SGP30 tVOC ppb"), datum);
        datum.uin16 = vocSensor.CO2;
        infrastructure::event_hub.Deliver(String("SGP30 eCO2 ppm"), datum);
        vocSensor.measureRawSignals();
        datum.uin16 = vocSensor.H2;
        infrastructure::event_hub.Deliver(String("SGP30 H2"), datum);
        datum.uin16 = vocSensor.ethanol;
        infrastructure::event_hub.Deliver(String("SGP30 ethanol"), datum);
    }
    return false;
}

static bool SaveSGP30Baselines(Eventing::PointerList<Eventing::EventTrigger>& triggers, Eventing::EventData& out);
static bool SaveSGP30Baselines(Eventing::PointerList<Eventing::EventTrigger>& triggers, Eventing::EventData& out) {
    if (vocSensorPresent) {
        vocSensor.getBaseline();
        peripherals::NvStorage::NvSettings.vocBaselineCo2 = vocSensor.baselineCO2;
        peripherals::NvStorage::NvSettings.vocBaselineTvoc = vocSensor.baselineTVOC;
        peripherals::NvStorage::commit();
    }
    return false;
}

static bool SetSGP30AbsoluteHumidity(Eventing::PointerList<Eventing::EventTrigger>& triggers, Eventing::EventData& out);
static bool SetSGP30AbsoluteHumidity(Eventing::PointerList<Eventing::EventTrigger>& triggers, Eventing::EventData& out) {
    if (vocSensorPresent && triggers.count >= 1 && triggers.list[0]->data_ready) {
        vocSensor.setHumidity(triggers.list[0]->data.uin16);
    }
    return false;
}

static SPS30 pmSensor;
static bool pmSensorPresent = false;
static constexpr uint32_t cleanIntervalGoal = 60 /* sec/min */ * 60 /* min/hr */ * 24 /* hr/day */ * 7 /* day/wk */ * 1;
static uint8_t pmTickCounter = 0;
static constexpr uint8_t pmMeasurementInterval = 60; // seconds

static SPS30_DATA_FLOAT sps30_global_datum_struct;
bool ReadSPS30(Eventing::PointerList<Eventing::EventTrigger>& triggers, Eventing::EventData& out) {
    if (pmSensorPresent) {
        bool sendData = false;
        SPS30_ERR readyErr, retrieveErr;
        bool dataIsReady, dataRetrieved;

        switch (pmTickCounter) {
        case 0:
            // start
            peripherals::SlowDownI2c();
            pmSensor.start_measuring(SPS30_FORMAT_IEEE754);
            peripherals::SpeedUpI2c();
            break;
        case 1:
            // wait; 2s will always produce data (1hz)
            break;
        case 2:
            // read and stop
            peripherals::SlowDownI2c();
            readyErr = pmSensor.is_data_ready();
            // verbose for easy addition of debug statements
            dataIsReady = readyErr == SPS30_OK;
            retrieveErr = pmSensor.read_data_no_wait_float(&sps30_global_datum_struct);
            dataRetrieved = retrieveErr == SPS30_OK;
            sendData = dataIsReady && dataRetrieved;
            pmSensor.stop_measuring();
            peripherals::SpeedUpI2c();
            break;
        default:
            // wait
            break;
        }

        if (sendData) {
            out.ptr = &sps30_global_datum_struct;
        }
        pmTickCounter = (pmTickCounter + 1) % pmMeasurementInterval;
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
    vocSensorPresent = vocSensor.begin();
    if (vocSensorPresent) {
        vocSensor.initAirQuality();
        // 0xFF is the unwritten-flash value
        if (peripherals::NvStorage::NvSettings.vocBaselineCo2 != 0xFFFF &&
            peripherals::NvStorage::NvSettings.vocBaselineTvoc != 0xFFFF) {
            vocSensor.setBaseline(peripherals::NvStorage::NvSettings.vocBaselineCo2,
                                  peripherals::NvStorage::NvSettings.vocBaselineTvoc);
        }
    }
    peripherals::SlowDownI2c();
    pmSensorPresent = pmSensor.begin();
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
        uint32_t fanCleanInterval;
        if (pmSensor.read_fan_cleaning_interval(&fanCleanInterval) == SPS30_OK) {
            if (fanCleanInterval != cleanIntervalGoal) {
                pmSensor.set_fan_cleaning_interval(cleanIntervalGoal);
            }
        }
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

namespace Data {

// TODO: statically allocated buffers
// fine: every 10 seconds
float pressureInst = -NAN;
// Instead of doing the EPA careful route right now, go for pretty/useful displays.
// data is 1 Hz
// fine is last minute
// medium is last hour
// coarse is last day
// TODO: predecimate to every 10s
Decimator pressureDecimator(60, 60, 24);
float tempFInst = -NAN;
Decimator tempFDecimator(60, 60, 24);
float tempCInst = -NAN;
Decimator tempCDecimator(60, 60, 24);
float altitudeInst = -NAN;
//Decimator altitudeDecimator(60, 60, 24);
uint16_t co2ppmInst = -1;
Decimator co2ppmDecimator(12, 60, 24); // every 5s
float rhInst = -NAN;
Decimator rhDecimator(60, 60, 24);
// pm is too complicated to decimate for now
SPS30_DATA_FLOAT pmInst = { 0 };
uint16_t eco2Inst = -1;
//Decimator eco2Decimator(12, 60, 24); // every 5s
uint16_t tvocInst = -1;
//Decimator tvocDecimator(12, 60, 24); // every 5s
uint16_t abshumInst = -1;
//Decimator abshumDecimator(12, 60, 24); // every 5s

bool GatherData(Eventing::PointerList<Eventing::EventTrigger>& triggers, Eventing::EventData& out) {
    static system_tick_t lastUpdate = 0;
    system_tick_t currentUpdate = millis();
    for (size_t evt_idx = 0; evt_idx < triggers.count; evt_idx++) {
        Eventing::EventTrigger* trigger = triggers.list[evt_idx];
        if (trigger->data_ready) {
            if (trigger->event_id.equalsIgnoreCase(String("LPS25HB Pressure hPa"))) {
                pressureInst = trigger->data.fl;
                pressureDecimator.push(pressureInst);
                return false;
            } else if (trigger->event_id.equalsIgnoreCase(String("LPS25HB Altitude m"))) {
                altitudeInst = trigger->data.fl;
                return false;
            } else if (trigger->event_id.equalsIgnoreCase(String("LPS25HB Temp C"))) {
                tempCInst = trigger->data.fl;
                tempCDecimator.push(tempCInst);
                return false;
            } else if (trigger->event_id.equalsIgnoreCase(String("LPS25HB Temp F"))) {
                tempFInst = trigger->data.fl;
                tempFDecimator.push(tempFInst);
            } else if (trigger->event_id.equalsIgnoreCase(String("SCD30 CO2 ppm"))) {
                co2ppmInst = trigger->data.uin16;
                co2ppmDecimator.push(co2ppmInst);
            } else if (trigger->event_id.equalsIgnoreCase(String("AHT20 Relative Humidity %%"))) {
                rhInst = trigger->data.fl;
                rhDecimator.push(rhInst);
            } else if (trigger->event_id.equalsIgnoreCase(String("SPS30 Raw"))) {
                pmInst = *((SPS30_DATA_FLOAT*)trigger->data.ptr);
            } else if (trigger->event_id.equalsIgnoreCase(String("SGP30 tVOC ppb"))) {
                tvocInst = trigger->data.uin16;
            } else if (trigger->event_id.equalsIgnoreCase(String("SGP30 eCO2 ppm"))) {
                eco2Inst = trigger->data.uin16;
            } else if (trigger->event_id.equalsIgnoreCase(String("Absolute Humidity 8.8 g/m^3"))) {
                abshumInst = trigger->data.uin16;
            } else {
                Serial.printlnf("Unhandled event \"%s\" %0.2f/%u/%d/%x/%p", trigger->event_id.c_str(), trigger->data.fl, trigger->data.uin16, trigger->data.in16, trigger->data.uin16, trigger->data.ptr);
            }
        }
    }
    // Fire at most every 1000ms
    if (currentUpdate - lastUpdate < 1000) {
        return false;
    }
    out.in16 = 32;
    return true;
}

} // namespace Data

/*****************************************************************************/
/*****************************************************************************/


/*****************************************************************************\
 * UX
 * Serial, OLED, and JSON rendering and emission
\*****************************************************************************/

namespace UX {

using namespace Data;

bool RenderSerial(Eventing::PointerList<Eventing::EventTrigger>& triggers, Eventing::EventData& out) {
    static system_tick_t lastFire = 0;
    system_tick_t currentFire = millis();
    // Fire at most every 5000ms
    if (currentFire - lastFire < 5000) {
        return false;
    }
    lastFire = currentFire;
    if (sensors::lps25hb_pressure_sensor_present) {
        Serial.printlnf("LPS25HB: %0.2fhPa, %0.1fm, %0.2fF, %0.2fC", pressureInst, altitudeInst, tempFInst, tempCInst);
    } else {
        Serial.println("LPS25HB not present");
    }
    if (sensors::co2SensorPresent) {
        Serial.printlnf("SCD30: %u ppm CO2", co2ppmInst);
    } else {
        Serial.println("SCD30 not present");
    }
    if (sensors::humiditySensorPresent) {
        Serial.printlnf("AHT20: %0.1f%%", rhInst);
    } else {
        Serial.println("AHT20 not present");
    }
    if (sensors::vocSensorPresent) {
        Serial.printlnf("SGP30 tVOC:%uppb eCO2:%uppm abshum:0x%x.%02xg/m^3", tvocInst, eco2Inst, abshumInst >> 8, abshumInst & 0xFF);
    } else {
        Serial.println("SGP30 not present");
    }
    if (sensors::pmSensorPresent) {
        String str;
        float totalugcm3 = pmInst.pm_1_0_ug_m3 + pmInst.pm_2_5_ug_m3 + pmInst.pm_4_0_ug_m3 + pmInst.pm_10_ug_m3;
        float totalcm3 = pmInst.pm_0_5_n_cm3 + pmInst.pm_1_0_n_cm3 + pmInst.pm_2_5_n_cm3 + pmInst.pm_4_0_n_cm3 + pmInst.pm_10_n_cm3;
        str += "SPS30: ";
        str += String(totalugcm3, 1);
        str += "ug/cm3 ";
        str += String(totalcm3, 1);
        str += "n/cm3 ";
        str += String(pmInst.typical_size_um, 1);
        str += "um typical ";
        Serial.printlnf("%s", str.c_str());
        Serial.printlnf("  n .5um:%0.0f,1:%0.0f,2.5:%0.0f,4.0:%0.0f,10:%0.0f", pmInst.pm_0_5_n_cm3, pmInst.pm_1_0_n_cm3, pmInst.pm_2_5_n_cm3, pmInst.pm_4_0_n_cm3, pmInst.pm_10_n_cm3);
        Serial.printlnf("  ug 1um:%0.0f,2.5:%0.0f,4.0:%0.0f,10:%0.0f", pmInst.pm_1_0_ug_m3, pmInst.pm_2_5_ug_m3, pmInst.pm_4_0_ug_m3, pmInst.pm_10_ug_m3);
    } else {
        Serial.println("SPS30 not present");
    }
    Serial.println("### End Serial Update");
    return false;
}

typedef enum _OledMode {
    HOME,
    PM_DISPLAY,
    BLANK,
    DEBUG_RETICLE,
} OledMode;

Box *PressureBox;
Box *TempBox;
Box *Co2Box;
Box *RhBox;
Box *TvocBox;
Box *Eco2Box;

Box *pmMassBox;
Box *pmCountBox;
Box *pmTypicalBox;

bool RenderOled(Eventing::PointerList<Eventing::EventTrigger>& triggers, Eventing::EventData& out) {
    static OledMode mode = HOME;
    for (size_t evt_idx = 0; evt_idx < triggers.count; evt_idx++) {
        Eventing::EventTrigger* trigger = triggers.list[evt_idx];
        if (trigger->data_ready) {
            if (trigger->event_id.equalsIgnoreCase(String("Joystick Direction Change"))) {
                peripherals::Joystick::JOYSTICK_DIRECTION joyDir = (peripherals::Joystick::JOYSTICK_DIRECTION)trigger->data.uin16;
                switch (joyDir) {
                default:
                case peripherals::Joystick::JOYSTICK_DIRECTION::CENTER:
                    break;
                case peripherals::Joystick::JOYSTICK_DIRECTION::LEFT:
                    mode = PM_DISPLAY;
                    break;
                case peripherals::Joystick::JOYSTICK_DIRECTION::DOWN:
                    mode = HOME;
                    break;
                case peripherals::Joystick::JOYSTICK_DIRECTION::RIGHT:
                    mode = BLANK;
                    break;
                case peripherals::Joystick::JOYSTICK_DIRECTION::UP:
                    mode = DEBUG_RETICLE;
                    break;
                }
            } else {
                Serial.printlnf("Unhandled event \"%s\" %0.2f/%u/%d/%x", trigger->event_id.c_str(), trigger->data.fl, trigger->data.uin16, trigger->data.in16, trigger->data.uin16);
            }
        }
    }

    u8g2_ClearBuffer(peripherals::Display::u8g2);

    switch (mode) {
    default:
    case HOME: {
            if (sensors::lps25hb_pressure_sensor_present) {
                PressureBox->UpdateValue(pressureInst);
                RenderSparkline(peripherals::Display::u8g2, pressureDecimator.coarse, 0, 0*23, 32, 23, true, 1.0f);
                TempBox->UpdateValue(tempFInst);
                RenderSparkline(peripherals::Display::u8g2, tempFDecimator.mid, 0, 1*23, 32, 23, true, 0.1f);
            } else {
                PressureBox->UpdateValue(-NAN);
                TempBox->UpdateValue(-NAN);
            }
            if (sensors::co2SensorPresent) {
                Co2Box->UpdateValue((uint32_t)co2ppmInst);
                RenderSparkline(peripherals::Display::u8g2, co2ppmDecimator.mid, 0, 2*23, 32, 23, true, 10.0f);
            } else {
                Co2Box->UpdateValue(9999UL);
            }
            if (sensors::humiditySensorPresent) {
                RhBox->UpdateValue(rhInst);
                RenderSparkline(peripherals::Display::u8g2, rhDecimator.mid, 0, 3*23, 32, 23, true, 0.5f);
            } else {
                RhBox->UpdateValue(-NAN);
            }
            if (sensors::vocSensorPresent) {
                TvocBox->UpdateValue((uint32_t)tvocInst);
                Eco2Box->UpdateValue((uint32_t)eco2Inst);
            } else {
                TvocBox->UpdateValue(9999UL);
                Eco2Box->UpdateValue(9999UL);
            }
        }
        break;
    case PM_DISPLAY: {
            float totalugm3 = pmInst.pm_1_0_ug_m3 + pmInst.pm_2_5_ug_m3 + pmInst.pm_4_0_ug_m3 + pmInst.pm_10_ug_m3;
            pmMassBox->UpdateValue(totalugm3);

            float totalcm3 = pmInst.pm_0_5_n_cm3 + pmInst.pm_1_0_n_cm3 + pmInst.pm_2_5_n_cm3 + pmInst.pm_4_0_n_cm3 + pmInst.pm_10_n_cm3;
            pmCountBox->UpdateValue(totalcm3);

            pmTypicalBox->UpdateValue(pmInst.typical_size_um);
        }
        break;
    case BLANK:
        // Buffer is already cleared by default
        break;
    case DEBUG_RETICLE:
        for (int n = 0; n < 128; n++) {
            if (n % 5 == 0) {
                u8g2_DrawPixel(peripherals::Display::u8g2, n, 0);
                u8g2_DrawPixel(peripherals::Display::u8g2, 0, n);
            }
            if (n % 10 == 0) {
                u8g2_DrawPixel(peripherals::Display::u8g2, n, 1);
                u8g2_DrawPixel(peripherals::Display::u8g2, 1, n);
            }
        }
        u8g2_DrawPixel(peripherals::Display::u8g2, 126, 94);
        u8g2_DrawPixel(peripherals::Display::u8g2, 127, 95);
        u8g2_DrawPixel(peripherals::Display::u8g2, 128, 96);
        u8g2_DrawPixel(peripherals::Display::u8g2, 126, 126);
        u8g2_DrawPixel(peripherals::Display::u8g2, 127, 127);
        u8g2_DrawPixel(peripherals::Display::u8g2, 128, 128);
        break;
    }
    peripherals::Display::BufferIsDirty = true;
    return false;
}

int ManualSerial(String s) {
    Eventing::PointerList<Eventing::EventTrigger> triggers;
    Eventing::EventData data;
    RenderSerial(triggers, data);
    peripherals::NvStorage::Print();
    return 33;
}

// Each RenderCloud is one Data Operation.
// Current free tier is 100,000 Data Operations per month.
// Render every 10 minutes for ~4320/mo
constexpr time_t RenderCloudInterval_ms = 10 * 60 * 1000;

bool RenderCloud(Eventing::PointerList<Eventing::EventTrigger>& triggers, Eventing::EventData& out) {
    if (!Particle.connected()) {
        return false;
    }
    char *buf;
    constexpr size_t bufLen = 622; // limit on Particle Photon running OS 2.3.0
    buf = (char*)malloc(bufLen);
    memset(buf, 0, bufLen);
    float data_buf = 0.0f;
    JSONBufferWriter writer(buf, bufLen-1); // always null-terminated
    writer.beginObject();
        writer.name("ver").value(1); // versioning allows for easier mass-parsing later
        writer.name("inst").beginObject();
            Data::pressureDecimator.fine.peek(0, &data_buf);
            writer.name("press_hPa").value(data_buf);
            Data::tempCDecimator.fine.peek(0, &data_buf);
            writer.name("T_C").value(data_buf);
            Data::rhDecimator.fine.peek(0, &data_buf);
            writer.name("rh_%").value(data_buf);
            Data::co2ppmDecimator.fine.peek(0, &data_buf);
            writer.name("co2_ppm").value(data_buf);
        writer.endObject();
    writer.endObject();
    Particle.publish("data/10min", buf);
    free(buf);
    return false;
}

int Report(String s) {
    char *buf;
    constexpr size_t bufLen = 622; // limit on Particle Photon running OS 2.3.0
    buf = (char*)malloc(bufLen);
    memset(buf, 0, bufLen);
    JSONBufferWriter writer(buf, bufLen-1); // always null-terminated
    writer.beginObject();
        writer.name("nv").beginObject();
            writer.name("ver").value(peripherals::NvStorage::NvSettings.version);
            writer.name("tvocBase").value(peripherals::NvStorage::NvSettings.vocBaselineTvoc);
            writer.name("co2Base").value(peripherals::NvStorage::NvSettings.vocBaselineCo2);
        writer.endObject();
        writer.name("uptime").value(millis());
    writer.endObject();
    Particle.publish("status", buf);
    free(buf);

    {
        Eventing::PointerList<Eventing::EventTrigger> triggers;
        Eventing::EventData out;
        RenderCloud(triggers, out);
    }
    return 0;
}

void init() {
    //FontData *candidates[] = {u8g2_font_6x10_tf, u8g2_font_profont11_tf, u8g2_font_simple1_tf, u8g2_font_NokiaSmallPlain_tf };
    PressureBox = new Box(peripherals::Display::u8g2, 32, 0 * 23, 128-32, 24, u8g2_font_bitcasual_tf, "hPa", "", u8g2_font_osb18_tf, 0);
    TempBox = new Box(peripherals::Display::u8g2, 32, 1 * 23, 128-32, 24, u8g2_font_bitcasual_tf, /*"\u00b0" actual degree symbol */"deg", "F", u8g2_font_osb18_tf, 1);
    Co2Box = new Box(peripherals::Display::u8g2, 32, 2 * 23, 128-32, 24, u8g2_font_bitcasual_tf, "ppm", "CO2", u8g2_font_osb18_tf, 0);
    RhBox = new Box(peripherals::Display::u8g2, 32, 3 * 23, 128-32, 24, u8g2_font_bitcasual_tf, "%", "rh", u8g2_font_osb18_tf, 1);
    TvocBox = new Box(peripherals::Display::u8g2, 64, 4 * 23 + 11, 64, 12, u8g2_font_nerhoe_tf, "ppb tVOC", "", u8g2_font_nerhoe_tf, 1);
    Eco2Box = new Box(peripherals::Display::u8g2, 64, 4 * 23, 64, 12, u8g2_font_nerhoe_tf, "ppm eCO2", "", u8g2_font_nerhoe_tf, 1);

    pmMassBox = new Box(peripherals::Display::u8g2, 0, 1 * 23, 128, 24, u8g2_font_nerhoe_tf, "ug/", "m^3", u8g2_font_osb18_tf, 1);
    pmCountBox = new Box(peripherals::Display::u8g2, 0, 2 * 23, 128, 24, u8g2_font_nerhoe_tf, "part/", "cm^3", u8g2_font_osb18_tf, 1);
    pmTypicalBox = new Box(peripherals::Display::u8g2, 0, 3 * 23, 128, 24, u8g2_font_nerhoe_tf, "um", "typ", u8g2_font_osb18_tf, 1);

    // Apparently .connect() will call .on() for me, but if I want to call .scan() first
    // then I need to call it myself.
    WiFi.on();
    delay(1000); // allow the module to finish its initial scan
    // This can be anywhere as long as we're not in AUTOMATIC mode since the OS is >= 1.5.0.
    if (peripherals::IsKnownNetworkPresent()) {
        Particle.function("ManualSerial", ManualSerial);
        Particle.function("Report", Report);
        Particle.connect();
        Particle.publishVitals(30min);
    } else {
        WiFi.off();
    }
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
    infrastructure::event_hub.AddHandler("LPS25HB Raw", sensors::ReadLPS25HB, Eventing::TRIGGER_TEMPORAL, 1000); // pre-decimated output is at 1 Hz
    infrastructure::event_hub.AddHandler("SCD30 Raw", sensors::ReadSCD30, Eventing::TRIGGER_TEMPORAL, sensors::co2SensorInterval*1000);
    infrastructure::event_hub.AddHandler("AHT20 Relative Humidity %%", sensors::ReadAHT20, Eventing::TRIGGER_TEMPORAL, 1000);
    infrastructure::event_hub.AddHandler("SPS30 Raw", sensors::ReadSPS30, Eventing::TRIGGER_TEMPORAL, 1000);
    infrastructure::event_hub.AddHandler("SGP30 Raw", sensors::ReadSGP30, Eventing::TRIGGER_TEMPORAL, sensors::vocReadInterval);
    infrastructure::event_hub.AddHandler("SGP30 Save Baselines", sensors::SaveSGP30Baselines, Eventing::TRIGGER_TEMPORAL, 12*60*60*1000);
    infrastructure::event_hub.AddHandler("Absolute Humidity 8.8 g/m^3", sensors::CalculateAbsoluteHumidity_8_8_g_m3, Eventing::TRIGGER_ON_ALL);
    infrastructure::event_hub.AddHandlerTrigger("Absolute Humidity 8.8 g/m^3", "LPS25HB Temp C");
    infrastructure::event_hub.AddHandlerTrigger("Absolute Humidity 8.8 g/m^3", "LPS25HB Pressure hPa");
    infrastructure::event_hub.AddHandlerTrigger("Absolute Humidity 8.8 g/m^3", "AHT20 Relative Humidity %%");
    infrastructure::event_hub.AddHandler("SGP30 Update Absolute Humidity", sensors::SetSGP30AbsoluteHumidity, Eventing::TRIGGER_ON_ANY);
    infrastructure::event_hub.AddHandlerTrigger("SGP30 Update Absolute Humidity", "Absolute Humidity 8.8 g/m^3");
    infrastructure::event_hub.AddHandler("GatherDataFired", Data::GatherData, Eventing::TRIGGER_ON_ANY);
    infrastructure::event_hub.AddHandlerTrigger("GatherDataFired", "LPS25HB Pressure hPa");
    infrastructure::event_hub.AddHandlerTrigger("GatherDataFired", "LPS25HB Altitude m");
    infrastructure::event_hub.AddHandlerTrigger("GatherDataFired", "LPS25HB Temp C");
    infrastructure::event_hub.AddHandlerTrigger("GatherDataFired", "LPS25HB Temp F");
    infrastructure::event_hub.AddHandlerTrigger("GatherDataFired", "SCD30 CO2 ppm");
    infrastructure::event_hub.AddHandlerTrigger("GatherDataFired", "AHT20 Relative Humidity %%");
    infrastructure::event_hub.AddHandlerTrigger("GatherDataFired", "SPS30 Raw");
    infrastructure::event_hub.AddHandlerTrigger("GatherDataFired", "Absolute Humidity 8.8 g/m^3");
    infrastructure::event_hub.AddHandlerTrigger("GatherDataFired", "SGP30 tVOC ppb");
    infrastructure::event_hub.AddHandlerTrigger("GatherDataFired", "SGP30 eCO2 ppm");
    infrastructure::event_hub.AddHandler("RenderSerialEvent", UX::RenderSerial, Eventing::TRIGGER_ON_ANY);
    infrastructure::event_hub.AddHandlerTrigger("RenderSerialEvent", "GatherDataFired");
    infrastructure::event_hub.AddHandler("RenderOledEvent", UX::RenderOled, Eventing::TRIGGER_ON_ANY);
    infrastructure::event_hub.AddHandlerTrigger("RenderOledEvent", "GatherDataFired");
    infrastructure::event_hub.AddHandlerTrigger("RenderOledEvent", "Joystick Direction Change");
    infrastructure::event_hub.AddHandler("PaintOled", peripherals::Display::Paint, Eventing::TRIGGER_TEMPORAL, 1200); // long enough for the longest loop to prevent delta clock recursion
    infrastructure::event_hub.AddHandler("RenderCloud", UX::RenderCloud, Eventing::TRIGGER_TEMPORAL, UX::RenderCloudInterval_ms);
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
    UX::init();
    flow::init();
}

void loop() {
    system_tick_t start = millis();
    ApplicationWatchdog::checkin();
    infrastructure::event_hub.update();
    peripherals::Joystick::EmitChangeEvent();
    system_tick_t end = millis();
    if (end - start > 100) {
        Serial.printlnf("###### Loop End; duration %lu", millis() - start);
    }
}

/*****************************************************************************/
/*****************************************************************************/
