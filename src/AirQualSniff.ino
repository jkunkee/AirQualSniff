
// checked-in dependencies
#include "decimator.h"
#include "sparkline.h"
#include "Atmospherics.h"
#include "sps30.h"
#define JET_EVT_HUB_TEMPORAL
uint32_t jet_evt_measure_timing_func() { return (uint32_t)millis(); }
#define JET_EVT_MEASURE_TIMING_FUNC jet_evt_measure_timing_func
#include "jet.h"
using jet::evt::jet_time_t;
// checked-in 3p dependencies
#include "SparkFun_SGP30_Arduino_Library.h"
#include "U8g2lib.h"
#include "boxen.h"

// Particle-style dependencies
#include <Base64RK.h>
#include <photon-vbat.h>
#include <SparkFun_SCD30_Arduino_Library.h>
#include <SparkFun_LPS25HB_Arduino_Library.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h>
#include <SparkFun_Qwiic_Humidity_AHT20.h>
#include <SparkFun_Qwiic_Joystick_Arduino_Library.h>
#include <SparkFunMicroOLED.h>
#include <SparkFunMAX17043.h>
#include <SparkFun_BMA400_Arduino_Library.h>
#include <SparkFun_External_EEPROM.h>
#include <MQTT5.h>
#include <mDNSResolver.h>

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

    static jet::evt::Hub event_hub;
    static jet_time_t hub_time_offset((uint32_t)0);

    static ApplicationWatchdog *wd = NULL;
    static int wd_count = 0;

    void watchdogHandler(void) {
        Serial.printlnf("ApplicationWatchdog triggered!");
        Serial.printlnf("######### FreeRAM: %lu Uptime: %ld", System.freeMemory(), millis());
        String hubSummary;
        event_hub.debug_string(hubSummary);
        Serial.print(hubSummary);
        Serial.flush();
        if (wd_count > 3) {
            if (Particle.connected()) {
                System.reset(RESET_REASON_WATCHDOG/*, not RESET_NO_WAIT*/);
            } else {
                System.reset(RESET_REASON_WATCHDOG, RESET_NO_WAIT);
            }
        }
        wd_count++;
    }

    static void init();
    static void init() {
        // first call to update() sets baseline time
        event_hub.update(hub_time_offset + millis());
        wd = new ApplicationWatchdog(30000U, &watchdogHandler, 1536);
    }

    bool DumpOsState(jet::evt::TriggerList& triggers, jet::evt::Datum& out) {
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
static constexpr uint8_t AHT20_MUX_PORT = 0;
static constexpr uint8_t SCREEN_MUX_PORT = 1;
static constexpr uint8_t LPS25HB_MUX_PORT = 2;
static constexpr uint8_t CO2_MUX_PORT = 3;
static constexpr uint8_t PM_MUX_PORT = 4;
static constexpr uint8_t UNUSED1_MUX_PORT = 5;
static constexpr uint8_t BMA400_MUX_PORT = 6;
static constexpr uint8_t UNUSED2_MUX_PORT = 7;

static constexpr uint8_t SGP30_MUX_PORT = UNUSED1_MUX_PORT;

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
    //static U8G2 *u8g2_oo;
    static u8g2_t u8g2Buf = { 0 };
    static u8g2_t *u8g2 = &u8g2Buf;

    static constexpr u8g2_uint_t WIDTH = 128;
    static constexpr u8g2_uint_t HEIGHT = 128;

    void u8g2_ssd1327_lock() {
        u8g2_SendF(peripherals::Display::u8g2, "ca", 0xFD, 0x12 | (1<<2));
        i2cMux.disablePort(SCREEN_MUX_PORT);
    }

    void u8g2_ssd1327_unlock() {
        i2cMux.enablePort(SCREEN_MUX_PORT);
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
    bool Paint(jet::evt::TriggerList& triggers, jet::evt::Datum& out) {
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

    MicroOLED uoled;
    bool uoledPresent = false;

    void init() {
        // FUll U8G2, SSD1327 controller, Midas 128x128 display, full framebuffer, First Arduino Hardware I2C, rotated so USB is on left
        // Docs say U8G2_SSD1327_EA_W128128_F_HW_I2C, but it chops off the top and bottom 16 rows.
        // u8g2_Setup_ssd1327_i2c_ws_128x128_f seems to work well, at least empirically.
        // Something about the C++ process causes lockups, at least on toolchain 3.3.0.
        // I thought something about u8x8_gpio_and_delay_arduino caused lockups too;
        // replacing it with a do-nothing 'return 0;' worked when I encountered that.

        i2cMux.enablePort(SCREEN_MUX_PORT);
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
        u8g2_ssd1327_lock();

        // shield defaults to SPI config
        Serial.println("init'in' display");
        uoled.begin();
        // ...well, it would appear there's no other way
        uoledPresent = true;
        uoled.clear(ALL);
        uoled.clear(PAGE);
        uoled.invert(false);
        uoled.contrast(3 * 32);
        uoled.display();
        uoled.setFontType(0);
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
        i2cMux.enablePort(SCREEN_MUX_PORT); // Rewiring this one to its own port was just too much effort for me.
        uint16_t horiz = joystick.getHorizontal();
        uint16_t vert = joystick.getVertical();
        i2cMux.disablePort(SCREEN_MUX_PORT);

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
            jet::evt::Datum joystickData;
            joystickData.uin16 = joyDir1;
            joyDirOld = joyDir1;

            //Serial.printlnf("Joy dir change event sent");
            infrastructure::event_hub.deliver("Joystick Direction Change", joystickData);
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

    ExternalEEPROM externalEeprom;
    bool externalEepromPresent = false;

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

        externalEeprom.setMemoryType(512); // CAT24C512
        //externalEepromPresent = externalEeprom.begin(); // causes SOS
    }

    void commit() {
        EEPROM.put(NvSettingsAddress, NvSettings);
    }

    void Print() {
        Serial.printlnf("v%d bco2:%d btvoc:%d", NvSettings.version, NvSettings.vocBaselineCo2, NvSettings.vocBaselineTvoc);
    }
} // namespace NvStorage

MAX17043 & lipo = lipo;
bool lipoShieldPresent = false;

void init() {
    Serial.begin(115200);

    // Enable reset reason storage
    if (!System.featureEnabled(FEATURE_RESET_INFO)) {
        System.enableFeature(FEATURE_RESET_INFO);
    }

    NvStorage::begin();

    Wire.setSpeed(I2C_SAFE_SPEED);
    Wire.begin();

    i2cMuxPresent = i2cMux.begin();
    if (i2cMuxPresent) {
        i2cMux.setPortState(0);
    }

    // The PM sensor has been muxed off, so speed up
    SpeedUpI2c();
    Display::init();

    i2cMux.enablePort(SCREEN_MUX_PORT);
    Joystick::joystickPresent = Joystick::joystick.begin();
    i2cMux.disablePort(SCREEN_MUX_PORT);

    // Since it's a Particle shield, the library even declares the object.
    if (lipo.getVersion() == 3) {
        lipoShieldPresent = true;
        lipo.begin();
        lipo.quickStart();
    }
}

} // namespace peripherals

/*****************************************************************************/
/*****************************************************************************/


/*****************************************************************************\
 * Sensors
 * Sensor device structures and functions
\*****************************************************************************/

namespace sensors {

BMA400 accel;
bool accelPresent = false;
BMA400_SensorData accelDatum = { 0 };
const uint32_t accelReadInterval = 300;

static bool ReadBMA400(jet::evt::TriggerList& triggers, jet::evt::Datum& out);
static bool ReadBMA400(jet::evt::TriggerList& triggers, jet::evt::Datum& out) {
    if (accelPresent != false) {
        peripherals::i2cMux.enablePort(peripherals::BMA400_MUX_PORT);
        accel.getSensorData();
        peripherals::i2cMux.disablePort(peripherals::BMA400_MUX_PORT);
        accelDatum = accel.data;
        out.ptr = &accelDatum;
        return true;
    }
    return false;
}

enum Orientation {
    FACE_UP,
    FACE_DOWN,
    ON_LEFT_SIDE,
    ON_RIGHT_SIDE,
    ON_TOP,
    ON_BOTTOM,
};

Orientation currentOrientation = ON_BOTTOM;

static bool ReduceBMA400ToOrientation(jet::evt::TriggerList& triggers, jet::evt::Datum& out);
static bool ReduceBMA400ToOrientation(jet::evt::TriggerList& triggers, jet::evt::Datum& out) {
    Orientation newOrientation = currentOrientation;
    bool hasChanged = false;
    if (triggers.size() == 1) {
        jet::evt::Trigger* trigger = triggers.get(0);
        BMA400_SensorData datum = *(BMA400_SensorData*)trigger->data.ptr;
        // decide the new orientation
        // the "bottom" is "down" is with the joystick on the left and the screen on the right, sitting flat on a desk facing the user
        // +X is towards left side
        // -X is towards right side
        // +Y is towards bottom
        // -Y is towards top
        // +Z is towards face
        // -Z is towards back
        // Sure, I could do dot products or Euler angles
        // but what will work NOW?
        // magnitudes!
        if (abs(datum.accelX) > abs(datum.accelY) && abs(datum.accelX) > abs(datum.accelZ)) {
            if (datum.accelX > 0.0f) {
                newOrientation = ON_RIGHT_SIDE;
            } else {
                newOrientation = ON_LEFT_SIDE;
            }
        } else if (abs(datum.accelY) > abs(datum.accelX) && abs(datum.accelY) > abs(datum.accelZ)) {
            if (datum.accelY > 0.0f) {
                newOrientation = ON_TOP;
            } else {
                newOrientation = ON_BOTTOM;
            }
        } else if (abs(datum.accelZ) > abs(datum.accelX) && abs(datum.accelZ) > abs(datum.accelY)) {
            if (datum.accelZ > 0.0f) {
                newOrientation = FACE_UP;
            } else {
                newOrientation = FACE_DOWN;
            }
        } else {
            newOrientation = FACE_UP;
        }
    }
    hasChanged = newOrientation != currentOrientation;
    currentOrientation = newOrientation;
    out.uin16 = currentOrientation;
    return hasChanged;
}

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
    peripherals::i2cMux.enablePort(peripherals::LPS25HB_MUX_PORT);
    uint8_t status = pressureSensor.getStatus();
    peripherals::i2cMux.disablePort(peripherals::LPS25HB_MUX_PORT);
    return lps25hb_pressure_sensor_present &&
            (status & STATUS_REG_T_DA) &&
            (status & STATUS_REG_P_DA);
}

static bool ReadLPS25HB(jet::evt::TriggerList& triggers, jet::evt::Datum& out);
static bool ReadLPS25HB(jet::evt::TriggerList& triggers, jet::evt::Datum& out) {
    if (lps25hb_pressure_sensor_present != false && LPS25HB_data_is_ready()) {
        jet::evt::Datum tempC;
        jet::evt::Datum tempF;
        jet::evt::Datum pressurehPa;
        jet::evt::Datum altitudem;

        peripherals::i2cMux.enablePort(peripherals::LPS25HB_MUX_PORT);
        tempC.fl = pressureSensor.getTemperature_degC() + pressureSensorTempFOffset * 5 / 9;
        pressurehPa.fl = pressureSensor.getPressure_hPa();
        peripherals::i2cMux.disablePort(peripherals::LPS25HB_MUX_PORT);
        tempF.fl = C_TO_F(tempC.fl);
        altitudem.fl = atmospherics::pressure_to_est_altitude(pressurehPa.fl);

        infrastructure::event_hub.deliver(String("LPS25HB Pressure hPa"), pressurehPa);
        infrastructure::event_hub.deliver(String("LPS25HB Altitude m"), altitudem);
        infrastructure::event_hub.deliver(String("LPS25HB Temp C"), tempC);
        infrastructure::event_hub.deliver(String("LPS25HB Temp F"), tempF);
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

static bool ReadSCD30(jet::evt::TriggerList& triggers, jet::evt::Datum& out);
static bool ReadSCD30(jet::evt::TriggerList& triggers, jet::evt::Datum& out) {
    peripherals::i2cMux.enablePort(peripherals::CO2_MUX_PORT);
    if (co2SensorPresent && co2Sensor.dataAvailable()) {
        jet::evt::Datum rh;
        jet::evt::Datum tempC;
        jet::evt::Datum co2;

        rh.fl = co2Sensor.getHumidity();
        tempC.fl = co2Sensor.getTemperature() + co2SensorTempFOffset * 5 / 9;
        co2.uin16 = co2Sensor.getCO2();

        infrastructure::event_hub.deliver(String("SCD30 CO2 ppm"), co2);
        infrastructure::event_hub.deliver(String("SCD30 temp C"), tempC);
        infrastructure::event_hub.deliver(String("SCD30 rh %"), rh);
    }
    peripherals::i2cMux.disablePort(peripherals::CO2_MUX_PORT);
    // Data is delivered with Deliver, so out param is unused.
    return false;
}

static AHT20 humiditySensor;
static bool humiditySensorPresent = false;
static constexpr float humiditySensorTempOffset = 0.0;

static bool ReadAHT20(jet::evt::TriggerList& triggers, jet::evt::Datum& out);
static bool ReadAHT20(jet::evt::TriggerList& triggers, jet::evt::Datum& out) {
    peripherals::i2cMux.enablePort(peripherals::AHT20_MUX_PORT);
    if (humiditySensorPresent && humiditySensor.isCalibrated()) {
        humiditySensor.triggerMeasurement();
        out.fl = humiditySensor.getHumidity();
        peripherals::i2cMux.disablePort(peripherals::AHT20_MUX_PORT);
        return true;
    }
    peripherals::i2cMux.disablePort(peripherals::AHT20_MUX_PORT);
    return false;
}

static bool CalculateAbsoluteHumidity_8_8_g_m3(jet::evt::TriggerList& triggers, jet::evt::Datum& out);
static bool CalculateAbsoluteHumidity_8_8_g_m3(jet::evt::TriggerList& triggers, jet::evt::Datum& out) {
    float tempC = -NAN;
    float pressurehPa = -NAN;
    float rh = -NAN;
    for (size_t evt_idx = 0; evt_idx < triggers.size(); evt_idx++) {
        jet::evt::Trigger* trigger = triggers.get(evt_idx);
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

static bool ReadSGP30(jet::evt::TriggerList& triggers, jet::evt::Datum& out);
static bool ReadSGP30(jet::evt::TriggerList& triggers, jet::evt::Datum& out) {
    if (vocSensorPresent) {
        jet::evt::Datum datum;
        peripherals::i2cMux.enablePort(peripherals::SGP30_MUX_PORT);
        vocSensor.measureAirQuality();
        peripherals::i2cMux.disablePort(peripherals::SGP30_MUX_PORT);
        if (vocSensor.CO2 == 400 && vocSensor.TVOC == 0) {
            // Sensor is still initializing (first 15s after init)
            return false;
        }
        // Silicon may be new enough that this is disabled
        peripherals::i2cMux.enablePort(peripherals::SGP30_MUX_PORT);
        SGP30ERR rawReadStatus = vocSensor.measureRawSignals();
        peripherals::i2cMux.disablePort(peripherals::SGP30_MUX_PORT);
        if (rawReadStatus ==  SGP30_SUCCESS) {
            datum.uin16 = vocSensor.H2;
            infrastructure::event_hub.deliver(String("SGP30 H2"), datum);
            datum.uin16 = vocSensor.ethanol;
            infrastructure::event_hub.deliver(String("SGP30 ethanol"), datum);
        } else {
            Serial.printlnf("SGP30 raw read failed with %d", rawReadStatus);
        }
        datum.uin16 = vocSensor.TVOC;
        infrastructure::event_hub.deliver(String("SGP30 tVOC ppb"), datum);
        datum.uin16 = vocSensor.CO2;
        infrastructure::event_hub.deliver(String("SGP30 eCO2 ppm"), datum);
    }
    return false;
}

static bool SaveSGP30Baselines(jet::evt::TriggerList& triggers, jet::evt::Datum& out);
static bool SaveSGP30Baselines(jet::evt::TriggerList& triggers, jet::evt::Datum& out) {
    if (vocSensorPresent) {
        peripherals::i2cMux.enablePort(peripherals::SGP30_MUX_PORT);
        vocSensor.getBaseline();
        peripherals::i2cMux.disablePort(peripherals::SGP30_MUX_PORT);
        peripherals::NvStorage::NvSettings.vocBaselineCo2 = vocSensor.baselineCO2;
        peripherals::NvStorage::NvSettings.vocBaselineTvoc = vocSensor.baselineTVOC;
        peripherals::NvStorage::commit();
    }
    return false;
}

static bool SetSGP30AbsoluteHumidity(jet::evt::TriggerList& triggers, jet::evt::Datum& out);
static bool SetSGP30AbsoluteHumidity(jet::evt::TriggerList& triggers, jet::evt::Datum& out) {
    if (vocSensorPresent && triggers.size() >= 1 && triggers.get(0)->data_ready) {
        peripherals::i2cMux.enablePort(peripherals::SGP30_MUX_PORT);
        vocSensor.setHumidity(triggers.get(0)->data.uin16);
        peripherals::i2cMux.disablePort(peripherals::SGP30_MUX_PORT);
    }
    return false;
}

static SPS30 pmSensor;
static bool pmSensorPresent = false;
static constexpr uint32_t cleanIntervalGoal = 60 /* sec/min */ * 60 /* min/hr */ * 24 /* hr/day */ * 7 /* day/wk */ * 1;
static uint8_t pmTickCounter = 0;
static constexpr uint8_t pmMeasurementInterval = 60; // seconds between on/measure/off cycles; ReadSPS30 is called at 1Hz flat

static SPS30_DATA_FLOAT sps30_global_datum_struct;
bool ReadSPS30(jet::evt::TriggerList& triggers, jet::evt::Datum& out) {
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
            // Something goes wrong often and this returns zero'd-out data.
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
    // sensor not present
    return false;
}

void init() {
    peripherals::i2cMux.enablePort(peripherals::BMA400_MUX_PORT);
    if (accel.beginI2C() == BMA400_OK) {
        accelPresent = true;
        accel.setODR(BMA400_ODR_12_5HZ);
        accel.setOSR(BMA400_ACCEL_OSR_SETTING_1); // just because I like oversampling
    }
    peripherals::i2cMux.disablePort(peripherals::BMA400_MUX_PORT);

    peripherals::i2cMux.enablePort(peripherals::LPS25HB_MUX_PORT);
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
    peripherals::i2cMux.disablePort(peripherals::LPS25HB_MUX_PORT);

    peripherals::i2cMux.enablePort(peripherals::CO2_MUX_PORT);
    co2SensorPresent = co2Sensor.begin();
    if (co2SensorPresent != false) {
        co2Sensor.setAutoSelfCalibration(true);
        co2Sensor.setMeasurementInterval(co2SensorInterval);
    }
    peripherals::i2cMux.disablePort(peripherals::CO2_MUX_PORT);

    peripherals::i2cMux.enablePort(peripherals::AHT20_MUX_PORT);
    humiditySensorPresent = humiditySensor.begin();
    peripherals::i2cMux.disablePort(peripherals::AHT20_MUX_PORT);

    peripherals::i2cMux.enablePort(peripherals::SGP30_MUX_PORT);
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
    peripherals::i2cMux.disablePort(peripherals::SGP30_MUX_PORT);

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
BMA400_SensorData accelDatum = { 0 };
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
uint16_t h2Rel = -1;
uint16_t ethanolRel = -1;
uint16_t abshumInst = -1;
//Decimator abshumDecimator(12, 60, 24); // every 5s

bool GatherData(jet::evt::TriggerList& triggers, jet::evt::Datum& out) {
    static system_tick_t lastUpdate = 0;
    system_tick_t currentUpdate = millis();
    for (size_t evt_idx = 0; evt_idx < triggers.size(); evt_idx++) {
        jet::evt::Trigger* trigger = triggers.get(evt_idx);
        if (trigger->data_ready) {
            if (trigger->event_id.equalsIgnoreCase(String("BMA400 Raw"))) {
                accelDatum = *(BMA400_SensorData*)trigger->data.ptr;
            } else if (trigger->event_id.equalsIgnoreCase(String("LPS25HB Pressure hPa"))) {
                pressureInst = trigger->data.fl;
                pressureDecimator.push(pressureInst);
                // Each of these is delivered singly from ReadLPS25HB, so only generate
                // a big update when the last one lands.
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
            } else if (trigger->event_id.equalsIgnoreCase(String("SGP30 H2"))) {
                h2Rel = trigger->data.uin16;
                return false;
            } else if (trigger->event_id.equalsIgnoreCase(String("SGP30 ethanol"))) {
                ethanolRel = trigger->data.uin16;
                return false;
            } else if (trigger->event_id.equalsIgnoreCase(String("SGP30 tVOC ppb"))) {
                tvocInst = trigger->data.uin16;
                return false;
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

bool RenderSerial(jet::evt::TriggerList& triggers, jet::evt::Datum& out) {
    static system_tick_t lastFire = 0;
    system_tick_t currentFire = millis();
    // Fire at most every 5000ms
    if (currentFire - lastFire < 10*60*1000) {
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
    PM_SUMMARY,
    PM_DETAIL,
    BLANK,
    DEBUG_RETICLE,
} OledMode;

Box *PressureBox;
Box *TempBox;
Box *Co2Box;
Box *RhBox;
Box *TvocBox;
Box *Eco2Box;
Box *DebugBoxA;
Box *DebugBoxB;

Box *pmMassBox;
Box *pmCountBox;
Box *pmTypicalBox;

Box *pm1_0MassBox;
Box *pm2_5MassBox;
Box *pm4_0MassBox;
Box *pm10_MassBox;
Box *pm0_5CountBox;
Box *pm1_0CountBox;
Box *pm2_5CountBox;
Box *pm4_0CountBox;
Box *pm10_CountBox;

bool RenderOled(jet::evt::TriggerList& triggers, jet::evt::Datum& out) {
    static OledMode mode = BLANK;
    for (size_t evt_idx = 0; evt_idx < triggers.size(); evt_idx++) {
        jet::evt::Trigger* trigger = triggers.get(evt_idx);
        if (trigger->data_ready) {
            if (trigger->event_id.equalsIgnoreCase(String("Joystick Direction Change"))) {
                peripherals::Joystick::JOYSTICK_DIRECTION joyDir = (peripherals::Joystick::JOYSTICK_DIRECTION)trigger->data.uin16;
                switch (joyDir) {
                default:
                case peripherals::Joystick::JOYSTICK_DIRECTION::CENTER:
                    break;
                case peripherals::Joystick::JOYSTICK_DIRECTION::LEFT:
                    if (mode != PM_SUMMARY && mode != PM_DETAIL) {
                        mode = PM_SUMMARY;
                    } else if (mode == PM_SUMMARY) {
                        mode = PM_DETAIL;
                    } else if (mode == PM_DETAIL) {
                        mode = PM_SUMMARY;
                    }
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
            } else if (trigger->event_id.equalsIgnoreCase(String("Accel Orientation"))) {
                Serial.println("using accel");
                switch ((sensors::Orientation)trigger->data.uin16) {
                    case sensors::Orientation::ON_TOP:
                        u8g2_SetDisplayRotation(peripherals::Display::u8g2, U8G2_R1);
                        break;
                    case sensors::Orientation::ON_BOTTOM:
                        u8g2_SetDisplayRotation(peripherals::Display::u8g2, U8G2_R3);
                        break;
                    case sensors::Orientation::ON_LEFT_SIDE:
                        u8g2_SetDisplayRotation(peripherals::Display::u8g2, U8G2_R0);
                        break;
                    case sensors::Orientation::ON_RIGHT_SIDE:
                        u8g2_SetDisplayRotation(peripherals::Display::u8g2, U8G2_R2);
                        break;
                    case sensors::Orientation::FACE_UP:
                    case sensors::Orientation::FACE_DOWN:
                        break;
                }
            } else if (trigger->event_id.equalsIgnoreCase(String("GatherDataFired"))) {
                // TODO: only update screen when there's a change?
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
            DebugBoxA->UpdateValue(0.0f);
            DebugBoxB->UpdateValue(0.0f);
        }
        break;
    case PM_DETAIL:
        pm1_0MassBox->UpdateValue(pmInst.pm_1_0_ug_m3);
        pm2_5MassBox->UpdateValue(pmInst.pm_2_5_ug_m3);
        pm4_0MassBox->UpdateValue(pmInst.pm_4_0_ug_m3);
        pm10_MassBox->UpdateValue(pmInst.pm_10_ug_m3);
        pm0_5CountBox->UpdateValue(pmInst.pm_0_5_n_cm3);
        pm1_0CountBox->UpdateValue(pmInst.pm_1_0_n_cm3);
        pm2_5CountBox->UpdateValue(pmInst.pm_2_5_n_cm3);
        pm4_0CountBox->UpdateValue(pmInst.pm_4_0_n_cm3);
        pm10_CountBox->UpdateValue(pmInst.pm_10_n_cm3);
        break;
    case PM_SUMMARY: {
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
    jet::evt::TriggerList triggers;
    jet::evt::Datum data;
    RenderSerial(triggers, data);
    peripherals::NvStorage::Print();
    return 33;
}

// Each RenderCloud is one Data Operation.
// Current free tier is 100,000 Data Operations per month.
// Render every 10 minutes for ~4320/mo
jet_time_t RenderCloudInterval_ms((uint32_t)10 * 60 * 1000);

bool RenderCloud(jet::evt::TriggerList& triggers, jet::evt::Datum& out) {
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

bool RenderMqtt(jet::evt::TriggerList& triggers, jet::evt::Datum& out);

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
        writer.name("watchdogTimeouts").value(infrastructure::wd_count);
        writer.name("resetData").beginObject();
            writer.name("reason").value(System.resetReason());
            writer.name("data").value(System.resetReasonData());
        writer.endObject();
    writer.endObject();
    Particle.publish("status", buf);

    memset(buf, 0, bufLen);
    String eventState;
    infrastructure::event_hub.debug_string(eventState);
    memcpy(buf, eventState.c_str(), min(bufLen-1, eventState.length()));
    Particle.publish("HubState", buf);

    free(buf);

    {
        jet::evt::TriggerList triggers;
        jet::evt::Datum out;
        RenderCloud(triggers, out);
        RenderMqtt(triggers, out);
    }
    return 0;
}

int Framebuffer(String s) {
    constexpr size_t bufLen = 622; // max message length for OS 2.3.x
    // See u8g2_m_16_16_f for buffer info. Called from u8g2 constructor.
    uint8_t page_cnt;
    uint8_t *framebuffer = u8g2_m_16_16_f(&page_cnt);
    constexpr uint16_t framebufferSize = 2048;

    // Dump to the event hub in a compact manner
    uint16_t encodedTotal = Base64::getEncodedSize(framebufferSize, false);
    uint16_t encodedChunkCount = (encodedTotal / (bufLen - 1))+1; // TODO: round up correctly
    String encoded = Base64::encodeToString(framebuffer, framebufferSize);
    for (uint16_t chunkIdx = 0; chunkIdx < encodedChunkCount; chunkIdx++) {
        String chunk = encoded.substring(chunkIdx * (bufLen-1), (chunkIdx+1) * (bufLen-1));
        String chunkName = "fb_";
        chunkName += chunkIdx;
        if (chunkIdx != 0) { delay(500); } // avoid throttling
        Particle.publish(chunkName, chunk);
    }

    // Dump to serial in a less-compact manner
    constexpr size_t WIDTH = peripherals::Display::WIDTH;
    constexpr size_t HEIGHT = peripherals::Display::HEIGHT;

    for (uint16_t row = 0; row < HEIGHT; row += 2) {
        for (uint16_t col = 0; col < WIDTH; col++) {
            // The screen is 128x128
            // SSD1327 is 4bpp, u8g2 uses 1bpp (see u8g2_m_16_16_f)
            // u8g2 also tiles the bytes vertically: each byte is a column of 8 bits next to the previous column of 8 (I think)
            int byteIndex = col + (row / 8) * WIDTH;
            int shift = row % 8;
            int val = (framebuffer[byteIndex] >> shift) & 0x3;
            const char *glyph;
            switch (val) {
                case 0:
                    glyph = " ";
                    break;
                case 1:
                    glyph = "'";
                    break;
                case 2:
                    glyph = ",";
                    break;
                case 3:
                    glyph = "8";
                    break;
                default:
                    glyph = "?";
            }
            Serial.print(glyph);
        }
        Serial.println();
    }

    return framebufferSize;
}

int RemoteJoystick(String s) {
    jet::evt::Datum joystickData;
    joystickData.uin16 = s.toInt();
    if (joystickData.uin16 > peripherals::Joystick::JOYSTICK_DIRECTION::CENTER) {
        return -1;
    }
    infrastructure::event_hub.deliver("Joystick Direction Change", joystickData);
    return 0;
}

namespace networking {
    bool IsNetworkConnected() {
        return WiFi.ready() && Particle.connected();
    }

    // mDNS facility
    UDP udp;
    mDNSResolver::Resolver resolver(udp);
    bool LookUpMdnsName(String name, IPAddress &address); // because Wiring preprocessing
    bool LookUpMdnsName(String name, IPAddress &address) {
        address = resolver.search(name.c_str());
        return !address.toString().equalsIgnoreCase(INADDR_NONE.toString());
    }

    namespace mqtt {
        constexpr size_t BUFFER_LENGTH = 3*1024;
        IPAddress mqttServerAddress = INADDR_NONE;
        MQTT5 client(BUFFER_LENGTH + 64); // allow for MQTT packet header and topic data
        MQTT5_REASON_CODE connectReason = MQTT5_REASON_CODE::UNSPECIFIED_ERROR;
        MQTT5_REASON_CODE publishReason = MQTT5_REASON_CODE::UNSPECIFIED_ERROR;
        bool Connect() {
            // mDNS from the MQTT server is flaky, so be a little resilient.
            // do a fresh lookup
            IPAddress newAddress;
            bool resolveSucceeded = LookUpMdnsName("pi.local", newAddress);
            if (resolveSucceeded) {
                // if that worked, keep it
                mqttServerAddress = newAddress;
            }
            // if that failed, use the cached value
            if (mqttServerAddress == INADDR_NONE) {
                Serial.println("MQTT Connect did not find an address");
                return false;
            }

            // Fill out connection options
            MQTT5ConnectOptions mqttOpts;
            mqttOpts = MQTT5_DEFAULT_CONNECT_OPTIONS;
            mqttOpts.username = "AirQualSniff";
            String password = System.deviceID(); // remember, System.deviceID().c_str() destroys the c_str before it's used
            mqttOpts.password = password.c_str(); // secure as tissue paper, just like the non-TLS connection

            // marshall the stashed address to pass through the MQTT5 API
            uint8_t addrAsBytes[4];
            // This array gets handed to IPAddress(uint8_t*), and this is the correct order.
            addrAsBytes[0] = (mqttServerAddress.raw().ipv4 >> 3*8) & 0xFF;
            addrAsBytes[1] = (mqttServerAddress.raw().ipv4 >> 2*8) & 0xFF;
            addrAsBytes[2] = (mqttServerAddress.raw().ipv4 >> 1*8) & 0xFF;
            addrAsBytes[3] = (mqttServerAddress.raw().ipv4 >> 0*8) & 0xFF;

            // Initiate the connection; this is only the start of the connection
            if (!client.connect(&addrAsBytes[0], 1883, nullptr, mqttOpts)) {
                Serial.printlnf("MQTT5 socket setup failed (getting a reason (%d) is like pulling teeth)", connectReason);
                return false;
            }
            return true;
        }
        bool Publish(String subtopic, String data) {
            // If there's no connection yet, start one.
            if (!client.connected() && !client.connecting()) {
                // Bail out if setup failed (like if the server isn't present)
                if (!Connect()) {
                    return false;
                }
                // if packets have already arrived, process them.
                client.loop();
            }
            // If the connection is still in progress, wait a moment for it to complete.
            if (client.connecting()) {
                delay(300);
                // process packets again
                client.loop();
                // If it is *still* connecting, bail out.
                if (client.connecting()) {
                    Serial.println("MQTT5 connection still pending");
                    return false;
                }
            }
            if (!client.connected()) {
                Serial.println("MQTT5 connection failed");
                return false;
            }
            publishReason = MQTT5_REASON_CODE::SUCCESS;
            return client.publish(subtopic.c_str(), data.c_str());
        }
        void ConnectSuccessCallback(bool sessionPresent);
        void ConnectSuccessCallback(bool sessionPresent) {
            connectReason = MQTT5_REASON_CODE::SUCCESS;
        }
        void ConnectFailureCallback(MQTT5_REASON_CODE Code);
        void ConnectFailureCallback(MQTT5_REASON_CODE Code) {
            connectReason = Code;
            Serial.printlnf("MQTT5 connect failure w/code %d", (int)Code);
        }
        void PublishFailureCallback(MQTT5_REASON_CODE Code);
        void PublishFailureCallback(MQTT5_REASON_CODE Code) {
            publishReason = Code;
            Serial.printlnf("MQTT5 publish failure w/code %d", (int)Code);
        }
        void init() {
            client.onConnectSuccess(ConnectSuccessCallback);
            client.onConnectFailed(ConnectFailureCallback);
            client.onPublishFailed(PublishFailureCallback);
        }
    }
    bool TryConnect() {
        if (IsNetworkConnected()) {
            return true;
        }
        // Apparently .connect() will call .on() for me, but if I want to call .scan() first
        // then I need to call it myself.
        WiFi.on();
        delay(1000); // allow the module to finish its initial scan
        // This can be anywhere as long as we're not in AUTOMATIC mode since the OS is >= 1.5.0.
        if (peripherals::IsKnownNetworkPresent()) {
            Particle.connect();
            mqtt::Connect();
        } else {
            WiFi.off();
        }
        return IsNetworkConnected();
    }
    void init() {
        Particle.function("ManualSerial", ManualSerial);
        Particle.function("Report", Report);
        Particle.function("Framebuffer", Framebuffer);
        Particle.function("RemoteJoystick", RemoteJoystick);
        //Particle.function("MdnsLookup", MdnsLookupFunction);
        //Particle.function("MqttPublish", mqtt::PublishFunction);
        Particle.publishVitals(30min);
        mqtt::init();
        TryConnect();
    }
    void update() {
        resolver.loop();
        mqtt::client.loop();
    }
} // namespace UX::networking

bool RenderMqtt(jet::evt::TriggerList& triggers, jet::evt::Datum& out) {
    constexpr size_t bufLen = networking::mqtt::BUFFER_LENGTH;
    char *buf = (char*)malloc(bufLen);
    memset(buf, 0, bufLen);
    JSONBufferWriter writer(buf, bufLen-1); // always null-terminated
    writer.beginObject();
        writer.name("ver").value(1);
        writer.name("inst").beginObject();
            writer.name("temp_F").value(Data::tempFInst);
            writer.name("temp_C").value(Data::tempCInst);
            writer.name("rh_percent").value(Data::rhInst);
            writer.name("co2_ppm").value(Data::co2ppmInst);
            writer.name("pm").beginObject();
                writer.name("typical_size_um").value(Data::pmInst.typical_size_um);
            if (Data::pmInst.pm_0_5_n_cm3 != 0.0f ||
                Data::pmInst.pm_1_0_n_cm3 != 0.0f ||
                Data::pmInst.pm_2_5_n_cm3 != 0.0f ||
                Data::pmInst.pm_4_0_n_cm3 != 0.0f ||
                Data::pmInst.pm_10_n_cm3 != 0.0f ||
                Data::pmInst.pm_1_0_ug_m3 != 0.0f ||
                Data::pmInst.pm_2_5_ug_m3 != 0.0f ||
                Data::pmInst.pm_4_0_ug_m3 != 0.0f ||
                Data::pmInst.pm_10_ug_m3 != 0.0f)
            {
                writer.name("0_5_n_cm3").value(Data::pmInst.pm_0_5_n_cm3);
                writer.name("1_0_n_cm3").value(Data::pmInst.pm_1_0_n_cm3);
                writer.name("2_5_n_cm3").value(Data::pmInst.pm_2_5_n_cm3);
                writer.name("4_0_n_cm3").value(Data::pmInst.pm_4_0_n_cm3);
                writer.name("10_n_cm3").value(Data::pmInst.pm_10_n_cm3);
                writer.name("1_0_ug_m3").value(Data::pmInst.pm_1_0_ug_m3);
                writer.name("2_5_ug_m3").value(Data::pmInst.pm_2_5_ug_m3);
                writer.name("4_0_ug_m3").value(Data::pmInst.pm_4_0_ug_m3);
                writer.name("10_ug_m3").value(Data::pmInst.pm_10_ug_m3);
            }
            writer.endObject();
            writer.name("pressure_hPa").value(Data::pressureInst);
            writer.name("tvoc_ppb").value(Data::tvocInst);
            writer.name("eco2_ppm").value(Data::eco2Inst);
            // Another driver that sometimes doesn't work right
            if (Data::h2Rel != 0xffff || Data::ethanolRel != 0xffff) {
            writer.name("h2_rel").value(Data::h2Rel);
            writer.name("ethanol_rel").value(Data::ethanolRel);
            }
        writer.endObject();
    writer.endObject();
    if (writer.dataSize() >= writer.bufferSize()) {
        networking::mqtt::Publish("AirQualSniff/status/error", "JSON too big for buffer");
    }
    if (!networking::mqtt::Publish("AirQualSniff/data/10min", buf)) {
        Serial.println("RenderMqtt failed to publish");
    }

    // Also dump HubState to try and find the cause(s) for the data gaps/device hangs
    memset(buf, 0, bufLen);
    String eventState;
    infrastructure::event_hub.debug_string(eventState);
    memcpy(buf, eventState.c_str(), min(bufLen-1, eventState.length()));
    networking::mqtt::Publish("AirQualSniff/status/error", buf);

    free(buf);
    return false;
}

bool RenderTestToSerial(jet::evt::TriggerList& triggers, jet::evt::Datum& out) {
    Serial.println("RenderTestToSerial");
    peripherals::lipo.quickStart();
    float lipoSoc = peripherals::lipo.getSOC();
    float lipoV = peripherals::lipo.getVoltage();
    Serial.printlnf("lipo: %d %f%% %fV", peripherals::lipoShieldPresent, lipoSoc, peripherals::lipo.getVoltage());
    static uint8_t count = 0;
    bool invert;
    bool screenOn;

    count++;
    uint8_t tick = count % 20;

    screenOn = tick <= 7;
    invert = 6 <= tick && tick <= 7;

    if (screenOn != false) {
        peripherals::Display::uoled.setCursor(0, 0);
        peripherals::Display::uoled.clear(PAGE);
        peripherals::Display::uoled.invert(invert);
        peripherals::Display::uoled.printlnf("BAT%% %3.0f", lipoSoc);
        peripherals::Display::uoled.printlnf("BATV %0.2f", lipoV);
        if (WiFi.ready()) {
            peripherals::Display::uoled.printlnf("WiFi OK");
        } else if (WiFi.connecting()) {
            peripherals::Display::uoled.printlnf("WiFi try"); // 'tries' fits but triggers newline
        } else {
            peripherals::Display::uoled.printlnf("WiFi ded");
        }
        if (Particle.connected()) {
            peripherals::Display::uoled.printlnf("PartCl OK");
        } else {
            peripherals::Display::uoled.printlnf("PartCl no"); // 'ded' fits but triggers newline
        }
        if (networking::resolver.lastResult == E_MDNS_OK) {
            peripherals::Display::uoled.printlnf("mDNS OK");
        } else {
            peripherals::Display::uoled.printlnf("mDNS %X", networking::resolver.lastResult);
        }
        if (networking::mqtt::connectReason != MQTT5_REASON_CODE::SUCCESS) {
            peripherals::Display::uoled.printlnf("mqttc %d", (uint8_t)networking::mqtt::connectReason);
        } else if (networking::mqtt::publishReason != MQTT5_REASON_CODE::SUCCESS) {
            peripherals::Display::uoled.printlnf("mqttp %d", (uint8_t)networking::mqtt::connectReason);
        } else {
            peripherals::Display::uoled.printlnf("mqtt OK");
        }
        const uint8_t height = peripherals::Display::uoled.getLCDHeight(); // 48
        const uint8_t width = peripherals::Display::uoled.getLCDWidth(); // 64
        for (int idx = 0; idx < max(height, width); idx++) {
            if ((idx % 2) == 1) {
                peripherals::Display::uoled.pixel(idx, height-1);
                peripherals::Display::uoled.pixel(width-1, idx);
            }
            if ((idx % 5) == 4) {
                peripherals::Display::uoled.pixel(idx, height-2);
                peripherals::Display::uoled.pixel(width-2, idx);
            }
            if ((idx % 10) == 9) {
                peripherals::Display::uoled.pixel(idx, height-3);
                peripherals::Display::uoled.pixel(width-3, idx);
            }
        }
        switch (sensors::currentOrientation) {
            case sensors::Orientation::FACE_UP:
            case sensors::Orientation::FACE_DOWN:
            case sensors::Orientation::ON_TOP:
            case sensors::Orientation::ON_BOTTOM:
            case sensors::Orientation::ON_RIGHT_SIDE:
                peripherals::Display::uoled.flipHorizontal(false);
                peripherals::Display::uoled.flipVertical(false);
                break;
            case sensors::Orientation::ON_LEFT_SIDE:
                peripherals::Display::uoled.flipHorizontal(true);
                peripherals::Display::uoled.flipVertical(true);
                break;
        }
        peripherals::Display::uoled.display();
    } else {
        peripherals::Display::uoled.clear(PAGE);
        peripherals::Display::uoled.invert(false);
        peripherals::Display::uoled.display();
    }
    return false;
}

void init() {
    //FontData *candidates[] = {u8g2_font_6x10_tf, u8g2_font_profont11_tf, u8g2_font_simple1_tf, u8g2_font_NokiaSmallPlain_tf };
    PressureBox = new Box(peripherals::Display::u8g2, 32, 0 * 23, 128-32, 24, u8g2_font_bitcasual_tf, "hPa", "", u8g2_font_osb18_tf, 0);
    TempBox = new Box(peripherals::Display::u8g2, 32, 1 * 23, 128-32, 24, u8g2_font_bitcasual_tf, /*"\u00b0" actual degree symbol */"deg", "F", u8g2_font_osb18_tf, 1);
    Co2Box = new Box(peripherals::Display::u8g2, 32, 2 * 23, 128-32, 24, u8g2_font_bitcasual_tf, "ppm", "CO2", u8g2_font_osb18_tf, 0);
    RhBox = new Box(peripherals::Display::u8g2, 32, 3 * 23, 128-32, 24, u8g2_font_bitcasual_tf, "%", "rh", u8g2_font_osb18_tf, 1);
    TvocBox = new Box(peripherals::Display::u8g2, 64, 4 * 23 + 11, 64, 12, u8g2_font_nerhoe_tf, "ppb tVOC", "", u8g2_font_nerhoe_tf, 1);
    Eco2Box = new Box(peripherals::Display::u8g2, 64, 4 * 23, 64, 12, u8g2_font_nerhoe_tf, "ppm eCO2", "", u8g2_font_nerhoe_tf, 1);
    DebugBoxA = new Box(peripherals::Display::u8g2, 0, 5 * 23 + 11, 128, 12, u8g2_font_nerhoe_tf, "A", "", u8g2_font_nerhoe_tf, 0);
    DebugBoxB = new Box(peripherals::Display::u8g2, 0, 5 * 23, 128, 12, u8g2_font_nerhoe_tf, "B", "", u8g2_font_nerhoe_tf, 0);

    pmMassBox = new Box(peripherals::Display::u8g2, 0, 1 * 23, 128, 24, u8g2_font_nerhoe_tf, "ug/", "m^3", u8g2_font_osb18_tf, 1);
    pmCountBox = new Box(peripherals::Display::u8g2, 0, 2 * 23, 128, 24, u8g2_font_nerhoe_tf, "part/", "cm^3", u8g2_font_osb18_tf, 1);
    pmTypicalBox = new Box(peripherals::Display::u8g2, 0, 3 * 23, 128, 24, u8g2_font_nerhoe_tf, "um", "typ", u8g2_font_osb18_tf, 1);

    // particle bins w/sparklines
    pm1_0MassBox = new Box(peripherals::Display::u8g2, 64, 0 * 12, 64, 13, u8g2_font_nerhoe_tf, " ug/m^3", "", u8g2_font_nerhoe_tf, 2);
    pm2_5MassBox = new Box(peripherals::Display::u8g2, 64, 1 * 12, 64, 13, u8g2_font_nerhoe_tf, " ug/m^3", "", u8g2_font_nerhoe_tf, 2);
    pm4_0MassBox = new Box(peripherals::Display::u8g2, 64, 2 * 12, 64, 13, u8g2_font_nerhoe_tf, " ug/m^3", "", u8g2_font_nerhoe_tf, 2);
    pm10_MassBox = new Box(peripherals::Display::u8g2, 64, 3 * 12, 64, 13, u8g2_font_nerhoe_tf, " ug/m^3", "", u8g2_font_nerhoe_tf, 2);
    pm0_5CountBox = new Box(peripherals::Display::u8g2, 64, 4 * 12, 64, 13, u8g2_font_nerhoe_tf, " n/cm^3", "", u8g2_font_nerhoe_tf, 2);
    pm1_0CountBox = new Box(peripherals::Display::u8g2, 64, 5 * 12, 64, 13, u8g2_font_nerhoe_tf, " n/cm^3", "", u8g2_font_nerhoe_tf, 2);
    pm2_5CountBox = new Box(peripherals::Display::u8g2, 64, 6 * 12, 64, 13, u8g2_font_nerhoe_tf, " n/cm^3", "", u8g2_font_nerhoe_tf, 2);
    pm4_0CountBox = new Box(peripherals::Display::u8g2, 64, 7 * 12, 64, 13, u8g2_font_nerhoe_tf, " n/cm^3", "", u8g2_font_nerhoe_tf, 2);
    pm10_CountBox = new Box(peripherals::Display::u8g2, 64, 8 * 12, 64, 13, u8g2_font_nerhoe_tf, " n/cm^3", "", u8g2_font_nerhoe_tf, 2);

    networking::init();
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
    infrastructure::event_hub.add_event("BMA400 Raw", sensors::ReadBMA400, jet::evt::TRIGGER_TEMPORAL, sensors::accelReadInterval);
    infrastructure::event_hub.add_event("Accel Orientation", sensors::ReduceBMA400ToOrientation, jet::evt::TRIGGER_ON_ANY);
    infrastructure::event_hub.add_event_trigger("Accel Orientation", "BMA400 Raw");
    infrastructure::event_hub.add_event("LPS25HB Raw", sensors::ReadLPS25HB, jet::evt::TRIGGER_TEMPORAL, (uint32_t)1000); // pre-decimated output is at 1 Hz
    infrastructure::event_hub.add_event("SCD30 Raw", sensors::ReadSCD30, jet::evt::TRIGGER_TEMPORAL, sensors::co2SensorInterval*(uint32_t)1000);
    infrastructure::event_hub.add_event("AHT20 Relative Humidity %%", sensors::ReadAHT20, jet::evt::TRIGGER_TEMPORAL, (uint32_t)1000);
    infrastructure::event_hub.add_event("SPS30 Raw", sensors::ReadSPS30, jet::evt::TRIGGER_TEMPORAL, (uint32_t)1000);
    infrastructure::event_hub.add_event("SGP30 Raw", sensors::ReadSGP30, jet::evt::TRIGGER_TEMPORAL, sensors::vocReadInterval);
    infrastructure::event_hub.add_event("SGP30 Save Baselines", sensors::SaveSGP30Baselines, jet::evt::TRIGGER_TEMPORAL, (uint32_t)12*60*60*1000);
    infrastructure::event_hub.add_event("Absolute Humidity 8.8 g/m^3", sensors::CalculateAbsoluteHumidity_8_8_g_m3, jet::evt::TRIGGER_ON_ALL);
    infrastructure::event_hub.add_event_trigger("Absolute Humidity 8.8 g/m^3", "LPS25HB Temp C");
    infrastructure::event_hub.add_event_trigger("Absolute Humidity 8.8 g/m^3", "LPS25HB Pressure hPa");
    infrastructure::event_hub.add_event_trigger("Absolute Humidity 8.8 g/m^3", "AHT20 Relative Humidity %%");
    infrastructure::event_hub.add_event("SGP30 Update Absolute Humidity", sensors::SetSGP30AbsoluteHumidity, jet::evt::TRIGGER_ON_ANY);
    infrastructure::event_hub.add_event_trigger("SGP30 Update Absolute Humidity", "Absolute Humidity 8.8 g/m^3");
    infrastructure::event_hub.add_event("GatherDataFired", Data::GatherData, jet::evt::TRIGGER_ON_ANY);
    infrastructure::event_hub.add_event_trigger("GatherDataFired", "BMA400 Raw");
    infrastructure::event_hub.add_event_trigger("GatherDataFired", "LPS25HB Pressure hPa");
    infrastructure::event_hub.add_event_trigger("GatherDataFired", "LPS25HB Altitude m");
    infrastructure::event_hub.add_event_trigger("GatherDataFired", "LPS25HB Temp C");
    infrastructure::event_hub.add_event_trigger("GatherDataFired", "LPS25HB Temp F");
    infrastructure::event_hub.add_event_trigger("GatherDataFired", "SCD30 CO2 ppm");
    infrastructure::event_hub.add_event_trigger("GatherDataFired", "AHT20 Relative Humidity %%");
    infrastructure::event_hub.add_event_trigger("GatherDataFired", "SPS30 Raw");
    infrastructure::event_hub.add_event_trigger("GatherDataFired", "Absolute Humidity 8.8 g/m^3");
    infrastructure::event_hub.add_event_trigger("GatherDataFired", "SGP30 tVOC ppb");
    infrastructure::event_hub.add_event_trigger("GatherDataFired", "SGP30 eCO2 ppm");
    infrastructure::event_hub.add_event("RenderTestToSerial", UX::RenderTestToSerial, jet::evt::TRIGGER_TEMPORAL, (uint32_t)500);
    //infrastructure::event_hub.add_event("RenderSerialEvent", UX::RenderSerial, jet::evt::TRIGGER_ON_ANY);
    //infrastructure::event_hub.add_event_trigger("RenderSerialEvent", "GatherDataFired");
    infrastructure::event_hub.add_event("RenderOledEvent", UX::RenderOled, jet::evt::TRIGGER_ON_ANY);
    infrastructure::event_hub.add_event_trigger("RenderOledEvent", "GatherDataFired");
    infrastructure::event_hub.add_event_trigger("RenderOledEvent", "Joystick Direction Change");
    infrastructure::event_hub.add_event_trigger("RenderOledEvent", "Accel Orientation");
    infrastructure::event_hub.add_event("PaintOled", peripherals::Display::Paint, jet::evt::TRIGGER_TEMPORAL, (uint32_t)1200); // long enough for the longest loop to prevent delta clock recursion
    infrastructure::event_hub.add_event("RenderCloud", UX::RenderCloud, jet::evt::TRIGGER_TEMPORAL, UX::RenderCloudInterval_ms);
    infrastructure::event_hub.add_event("RenderMqtt", UX::RenderMqtt, jet::evt::TRIGGER_TEMPORAL, UX::RenderCloudInterval_ms);
    //infrastructure::event_hub.add_event("DumpOsState", infrastructure::DumpOsState, jet::evt::TRIGGER_TEMPORAL, 5000);
    if (!infrastructure::event_hub.is_dag()) {
        Serial.println("Hub graph is not a DAG!!");
    }
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
    system_tick_t start = millis() + infrastructure::hub_time_offset.to_uint32_t();
    ApplicationWatchdog::checkin();
    infrastructure::event_hub.update((jet_time_t)start);
    peripherals::Joystick::EmitChangeEvent();
    UX::networking::update();
    system_tick_t end = millis() + infrastructure::hub_time_offset.to_uint32_t();
    if (end - start > (system_tick_t)10000ULL) {
        Serial.printlnf("###### Loop End; duration %lu-%lu=%lu", end, start, end - start);
        String mqttMsg = String::format("long loop, duration %lu-%lu=%lu", end, start, end - start);
        UX::networking::mqtt::Publish("AirQualSniff/status/error", mqttMsg);
    }
}

/*****************************************************************************/
/*****************************************************************************/
