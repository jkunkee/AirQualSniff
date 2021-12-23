
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
    if (i2cMuxPresent) {
        peripherals::i2cMux.disablePort(PM_MUX_PORT);
        Wire.end();
        Wire.setSpeed(I2C_DEFAULT_SPEED); // SPS30 only supports 100KHz
        Wire.begin();
    }
}
void SlowDownI2c() {
    if (i2cMuxPresent) {
        Wire.end();
        Wire.setSpeed(I2C_SAFE_SPEED); // SPS30 only supports 100KHz
        Wire.begin();
        peripherals::i2cMux.enablePort(PM_MUX_PORT);
    }
}

void init() {
    peripherals::i2cMuxPresent = peripherals::i2cMux.begin();
    if (peripherals::i2cMuxPresent) {
        peripherals::i2cMux.setPortState(0
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
    /*
    //UX::RenderSerialEvent.AddTrigger(&sensors::AbsoluteHumidity_g_m3_8_8_Event);
    */
}

} // namespace Flow

/*****************************************************************************/
/*****************************************************************************/


/*****************************************************************************\
 * Arduino
\*****************************************************************************/

void setup() {
    Serial.begin(115200);
    Serial.println("Hello World");

    Wire.setSpeed(peripherals::I2C_SAFE_SPEED);
    Wire.begin();

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
