#define TEXT_FOREGROUND 0x4
#define TEXT_BACKGROUND 0x0
#define TEXT_HEIGHT 8
#define TEXT_LINE_SPACING 1

    void BlinkActionFunc(void) {
        static bool ledState = false;
        ledState = !ledState;
        digitalWrite(LED, ledState ? HIGH : LOW);
    }
    static DeltaClockEntry BlinkAction = {
        .action = &BlinkActionFunc,
        .interval = 1000,
        .repeating = true,
    };

namespace peripherals {

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
    } // namespace Joystick

    namespace Display {
        // FUll U8G2, SSD1327 controller, EA_128128 display, full framebuffer, First Arduino Hardware I2C, not rotated
        // Something about the C++ process causes lockups
        //U8G2_SSD1327_EA_W128128_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, U8X8_PIN_NONE, U8X8_PIN_NONE);
        static u8g2_t u8g2 = { 0 };

        // I thought something about u8x8_gpio_and_delay_arduino caused lockups too;
        // replacing it with a do-nothing return 0; works too.

        static constexpr u8g2_uint_t WIDTH = 128;
        static constexpr u8g2_uint_t HEIGHT = 128;

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

    static QWIICMUX i2cMux;
    static bool i2cMuxPresent = false;


} // namespace peripherals

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

namespace sensors {

    // https://github.com/sparkfun/SparkFun_SGP30_Arduino_Library/tree/main/src
    static SGP30 vocSensor;
    static bool vocSensorPresent = false;
    static constexpr int VOC_START_DELAY_MILLIS = 15*1000;
    static unsigned long vocSensorInitStart = 0;
    static bool vocSensorInitDone = false;

    static SPS30 pmSensor;
    static bool pmSensorPresent = false;
    static constexpr uint8_t PM_MUX_PORT = 4;

    static PhotonVBAT vbat(A0);

} // namespace sensors

namespace data {

    static Decimator co2Decimator(3, 3, 3);

    // Measured values from LPS25HB (pressure sensor)
    static float pressurehPa = 0;
    static float tempC_LPS25HB = 0;
    // Calculated values from LPS25HB
    // using pressure to estimate altitude
    //https://en.wikipedia.org/wiki/Barometric_formula
    static float tempC = 0;
    static float tempF = 0;
    static float altitudem = 0;

    // Measured values from SCD30 (CO2 sensor)
    // requires pressure, (if available) altitude, and (need to read docs) temperature offset
    static float relativeHumidityPercent = 0;
    static float tempC_SCD30 = 0;
    static uint16_t co2ppm = 0;
    // Calculated values from SCD30
    //static float tempOffsetC_SCD30 = 0; // TODO: figure out difference between input air and sensor temps as input for SCD30 correction

    // Composite calculated value
    static uint16_t absoluteHumidity_g_m3_8_8 = 0xF80; // default value in SGP30

    // Measured values from SPS30 (particulate sensor)
    static SPS30_DATA_INT pmData;

    // Measured values from SGP30 (VOC sensor)
    //static uint16_t TVOCppb = 0; // currently only read into locals
    //static uint16_t eCO2ppm = 0; // currently only read into locals

    // Curated (and eventually averaged) values for display
    namespace Display {
        static float temperature_C = 0;
        static float temperature_F = 0;
        static bool temperature_set = false;
        //static bool temperature_fresh = false;
    }

} // namespace data

// Event Structure
namespace Flow {

static bool foo(EventHub::Event* event);
static bool foo(EventHub::Event* event) {
    Serial.printlnf("fired %s", event->name);
    for (size_t input_idx = 0; input_idx < event->triggers_count; input_idx++) {
        if (event->triggers[input_idx].data_ready) {
            Serial.printlnf("  Rx'd #%d %f", input_idx, event->triggers[input_idx].data.fl);
        }
    }
    return true;
}

static EventHub::EventHub event_hub_actual = {
    .events_count = 5,
    .events = {
        {
            .action = NULL,
            .name = "LPS25HB Pressure Sensor Temp C",
            .type = EventHub::TRIGGER_NONE,
            .triggers_count = 0,
            .triggers = {},
        },
        {
            .action = NULL,
            .name = "LPS25HB Pressure Sensor Temp F",
            .type = EventHub::TRIGGER_NONE,
            .triggers_count = 0,
            .triggers = {},
        },
        {
            .action = NULL,
            .name = "LPS25HB Pressure Sensor Pressure hPa",
            .type = EventHub::TRIGGER_NONE,
            .triggers_count = 0,
            .triggers = {},
        },
        {
            .action = &foo,
            .name = "foo",
            .type = EventHub::TRIGGER_ON_ANY,
            .triggers_count = 2,
            .triggers = {
                {
                    .source_event = &event_hub_actual.events[0],
                },
                {
                    .source_event = &event_hub_actual.events[1],
                },
            },
        },
        {
            .action = &foo,
            .name = "Render ",
            .type = EventHub::TRIGGER_ON_ANY,
            .triggers_count = 1,
            .triggers = {
                {
                    .source_event = &event_hub_actual.events[2],
                },
            },
        },
    },
};

static void ReadLPS25HB();
static void ReadLPS25HB() {
    if (sensors::pressureSensorPresent != false) {
        data::Display::temperature_C = data::tempC = data::tempC_LPS25HB = sensors::pressureSensor.getTemperature_degC() + sensors::pressureSensorTempFOffset * 5 / 9;
        data::pressurehPa = sensors::pressureSensor.getPressure_hPa();

        data::Display::temperature_F = data::tempF = C_TO_F(data::tempC_LPS25HB);
        data::Display::temperature_set = true;
        data::altitudem = atmospherics::pressure_to_est_altitude(data::pressurehPa);

        EventHub::EventData data;
        data.fl = data::tempC;
        EventHub::FireEvent(&event_hub_actual, &event_hub_actual.events[0], &data);
        data.fl = data::tempF;
        EventHub::FireEvent(&event_hub_actual, &event_hub_actual.events[1], &data);
        data.fl = data::pressurehPa;
        EventHub::FireEvent(&event_hub_actual, &event_hub_actual.events[2], &data);
    }
}
static DeltaClockEntry LPS25HBTimer = {
    .action = ReadLPS25HB,
    .interval = 1000,
    .repeating = true,
};

} // namespace Flow

#define PRINTLN(cstr) u8g2_DrawUTF8(&peripherals::Display::u8g2, 0, 0+(lineNo++)*(TEXT_HEIGHT+TEXT_LINE_SPACING), cstr);

static void RenderTextRowDisplay();
static void RenderTextRowDisplay() {
    int lineNo = 0;
lineNo = 2;
    if (sensors::pressureSensorPresent != false) {
        String val;
        val += String(data::tempC, 3);
        val += " C ";
        val += String(data::pressurehPa, 3);
        val += " hPa";
        PRINTLN(val.c_str()); // 3
        val = String(data::tempF, 3);
        val += " F ";
        val += String(data::altitudem, 0);
        val += " m";
        PRINTLN(val.c_str()); // 4
            data::co2Decimator.push(data::tempF);
            if (data::co2Decimator.is_full()) {
                float final_val;
                data::co2Decimator.decimate_and_clear(&final_val);
                Serial.printlnf("final_val: %0.1f, n=%d", final_val, data::co2Decimator.capacity());
            }
    } else {
        PRINTLN("LPS25HB not present");
    }
}

void setup() {
    Serial.begin(115200);
Serial.println("Hi there!\n");
return;
    infrastructure::deltaClock.begin();
    infrastructure::deltaClock.insert(&infrastructure::BlinkAction);
    infrastructure::deltaClock.insert(&Flow::LPS25HBTimer);

    // setSpeed must be before begin()
    // https://docs.staging.particle.io/cards/firmware/wire-i2c/setspeed/
    Wire.setSpeed(peripherals::I2C_SAFE_SPEED);
    Wire.begin();

    Serial.begin(115200);

    peripherals::Joystick::joystickPresent = peripherals::Joystick::joystick.begin();

    sensors::pressureSensorPresent = sensors::pressureSensor.begin();

    sensors::co2SensorPresent = sensors::co2Sensor.begin();
    if (sensors::co2SensorPresent != false) {
        sensors::co2Sensor.setAutoSelfCalibration(true);
        sensors::co2Sensor.setMeasurementInterval(5);
    }

    sensors::humiditySensorPresent = sensors::humiditySensor.begin();

    sensors::vocSensorPresent = sensors::vocSensor.begin();
    if (sensors::vocSensorPresent != false)
    {
        sensors::vocSensor.setHumidity(0x0F80);
        sensors::vocSensor.initAirQuality();
        sensors::vocSensorInitDone = false;
        sensors::vocSensorInitStart = millis();
    }

    if (peripherals::i2cMuxPresent) {
        // Slow down I2C
        Wire.end();
        Wire.setSpeed(peripherals::I2C_SAFE_SPEED); // SPS30 only supports 100KHz
        Wire.begin();
        // Enable I2C mux
        peripherals::i2cMux.enablePort(sensors::PM_MUX_PORT);
    }
    sensors::pmSensorPresent = sensors::pmSensor.begin();
    if (sensors::pmSensorPresent != false) {
        sensors::pmSensor.wake();
        sensors::pmSensor.reset();
        uint32_t interval;
        sensors::pmSensor.read_fan_cleaning_interval(&interval);
        if (interval != 7 * 24 * 60 * 60) {
            interval = 7 * 24 * 60 * 60;
            sensors::pmSensor.set_fan_cleaning_interval(interval);
        }
        sensors::pmSensor.start_measuring(SPS30_FORMAT_UINT16);
    }
    if (peripherals::i2cMuxPresent) {
        // Disable I2C mux
        peripherals::i2cMux.disablePort(sensors::PM_MUX_PORT);
        // Speed up I2C
        Wire.end();
        Wire.setSpeed(peripherals::I2C_DEFAULT_SPEED); // Display really needs 400KHz
        Wire.begin();
    }

    peripherals::Display::init();

    Time.zone(-8.0);
    Time.setDSTOffset(+1.0);

    pinMode(LED, OUTPUT);

    infrastructure::wd = new ApplicationWatchdog(3000U, &infrastructure::watchdogHandler);
}

void loop() {
Serial.println("goin'!\n");
return;
    ApplicationWatchdog::checkin();
    infrastructure::deltaClock.update();

    {
        EventHub::DumpStateOnSerial(&Flow::event_hub_actual);
    }

    u8g2_ClearBuffer(&peripherals::Display::u8g2);

    int lineNo = 0;

    PRINTLN("Air Quality Sniffer"); // 1

    if (peripherals::Joystick::joystickPresent) {
        uint16_t x = peripherals::Joystick::joystick.getHorizontal();
        uint16_t y = peripherals::Joystick::joystick.getVertical();
        byte button = peripherals::Joystick::joystick.checkButton();
        String val;
        val += "joy:";
        val += "x=";
        val += x;
        val += ",y=";
        val += y;
        val += ",b=";
        val += button;
        val += "         ";
        PRINTLN(val.c_str()); // 2
    } else {
        PRINTLN("Joystick not found"); // 2
    }

    RenderTextRowDisplay();

    if (sensors::co2SensorPresent != false) {
        sensors::co2Sensor.setAmbientPressure(data::pressurehPa /* 1.0 hPa/mb */);
        // altitude ignored when ambient pressure set

        if (sensors::co2Sensor.dataAvailable() != false) {
            data::relativeHumidityPercent = sensors::co2Sensor.getHumidity();
            data::tempC_SCD30 = sensors::co2Sensor.getTemperature() + sensors::co2SensorTempFOffset * 5 / 9;
            data::co2ppm = sensors::co2Sensor.getCO2();
        }

        String val;
        val += "co2:";
        val += String(data::co2ppm);
        val += "ppm rh:";
        val += String(data::relativeHumidityPercent, 0);
        val += "%       ";
        PRINTLN(val.c_str()); // 5
    } else {
        PRINTLN("SCD30 not present");
    }

    if (sensors::pressureSensorPresent != false && sensors::co2SensorPresent != false) {
        data::absoluteHumidity_g_m3_8_8 = atmospherics::rel_to_abs_humidity(data::tempC_LPS25HB, data::pressurehPa, data::relativeHumidityPercent);
    }

    if (sensors::humiditySensorPresent != false) {
        sensors::humiditySensor.triggerMeasurement();
        String val;
        val += "hum:h=";
        val += String(sensors::humiditySensor.getHumidity(), 0);
        val += ",T=";
        val += String(sensors::humiditySensor.getTemperature(), 0);
        val += ",s=";
        val += sensors::humiditySensor.getStatus();
        val += sensors::humiditySensor.isCalibrated() ? ",c" : ",nc";
        PRINTLN(val.c_str()); // ?
    } else {
        PRINTLN("AHT20 not found"); // ?
    }

    if (sensors::vocSensorPresent != false) {
        String val = "";

        sensors::vocSensor.setHumidity(data::absoluteHumidity_g_m3_8_8);

        SGP30ERR sgperr = sensors::vocSensor.measureAirQuality();
        if (sgperr != SGP30_SUCCESS)
        {
            sensors::vocSensorPresent = false;
            val = "SGP30 measurement failed: ";
            val += String(sgperr);
        } else if (sensors::vocSensorInitDone == false) {
            sensors::vocSensorInitDone = millis() - sensors::vocSensorInitStart > sensors::VOC_START_DELAY_MILLIS;
            val = "SGP30 initializing";
        } else {
            uint16_t tvoc_ppb = sensors::vocSensor.TVOC;
            uint16_t eCO2_ppm = sensors::vocSensor.CO2;
            //Particle.publish("TVOC", String(tvoc_ppb));
            
            val = "TVOC:";
            val += String(tvoc_ppb);
            val += "ppb(";
            val += String(eCO2_ppm);
            val += "ppmCO2)   ";
        }
        PRINTLN(val.c_str()); // 6

        sgperr = sensors::vocSensor.measureRawSignals();
        if (sgperr == SGP30_SUCCESS) {
            val = "H2:";
            val += String(sensors::vocSensor.H2, HEX);
            val += " C2H6O:";
            val += String(sensors::vocSensor.ethanol, HEX);
            PRINTLN(val.c_str()); // 7
        } else {
            PRINTLN("SGP30 raw read failed");
        }
    } else {
        PRINTLN("SGP30 not present");
    }

    String decVal = "08";
    for (int idx = 0; idx < data::co2Decimator.fine.size(); idx++) {
        float val;
        decVal += " ";
        data::co2Decimator.fine.peek(idx, &val);
        decVal += String(val, 0);
    }
    decVal += ",";
    for (int idx = 0; idx < data::co2Decimator.mid.size(); idx++) {
        float val;
        decVal += " ";
        data::co2Decimator.mid.peek(idx, &val);
        decVal += String(val, 0);
    }
    decVal += ",";
    for (int idx = 0; idx < data::co2Decimator.coarse.size(); idx++) {
        float val;
        decVal += " ";
        data::co2Decimator.coarse.peek(idx, &val);
        decVal += String(val, 0);
    }
    decVal += "                    "; // ensure line gets blanked
    PRINTLN(decVal.c_str());
    Serial.println(decVal.c_str());

    if (peripherals::i2cMuxPresent) {
        // Slow down I2C
        Wire.end();
        Wire.setSpeed(peripherals::I2C_SAFE_SPEED);
        Wire.begin();
        // Enable I2C mux
        peripherals::i2cMux.enablePort(sensors::PM_MUX_PORT);
    }
    if (sensors::pmSensorPresent != false && sensors::pmSensor.is_data_ready() == SPS30_OK) {
        sensors::pmSensor.read_data_no_wait_int(&data::pmData);
        String val;
        val += String(data::pmData.pm_1_0_ug_m3);
        val += ",";
        val += String(data::pmData.pm_2_5_ug_m3);
        val += ",";
        val += String(data::pmData.pm_4_0_ug_m3);
        val += ",";
        val += String(data::pmData.pm_10_ug_m3);
        val += ",";
        val += String(data::pmData.typical_size_nm);
        val += "            ";
        PRINTLN(val.c_str()); // 9
        val = "";
        val += String(data::pmData.pm_0_5_n_cm3);
        val += ",";
        val += String(data::pmData.pm_1_0_n_cm3);
        val += ",";
        val += String(data::pmData.pm_2_5_n_cm3);
        val += ",";
        val += String(data::pmData.pm_4_0_n_cm3);
        val += ",";
        val += String(data::pmData.pm_10_n_cm3);
        val += "            ";
        PRINTLN(val.c_str()); // 10
    } else {
        PRINTLN("pm not present"); // 9
        String val;
        val += "10 ";
        val += sensors::pmSensor.device_status;
        val += " ";
        val += sensors::pmSensor.firmware_version[0];
        val += ".";
        val += sensors::pmSensor.firmware_version[1];
        val += " ";
        val += sensors::pmSensor.is_data_ready();
        PRINTLN(val.c_str()); // 10
    }
    if (peripherals::i2cMuxPresent) {
        // Disable I2C mux
        peripherals::i2cMux.disablePort(sensors::PM_MUX_PORT);
        // Speed up I2C
        Wire.end();
        Wire.setSpeed(peripherals::I2C_DEFAULT_SPEED);
        Wire.begin();
    }

    if (Time.isValid()) {
        PRINTLN(Time.format("%F %T PST").c_str()); // 11
    } else {
        PRINTLN("Time not ready");
    }

    if (sensors::co2SensorPresent && sensors::pressureSensorPresent) {
        String str;
        str += "s:";
        str += String(C_TO_F(data::tempC_SCD30), 2);
        str += ",l:";
        str += String(C_TO_F(data::tempC_LPS25HB), 2);
        str += ",c:";
        str += String(C_TO_F(sensors::vbat.readTempC()), 0);
        str += ",v:";
        str += String(sensors::vbat.readVBAT(), 2);
        PRINTLN(str.c_str()); // 12
    } else {
        PRINTLN("no summary avail"); // 12
    }
    Serial.printlnf("scd: %0.3fF lps: %0.3fF CPU: %0.3fF", C_TO_F(data::tempC_SCD30), C_TO_F(data::tempC_LPS25HB), C_TO_F(sensors::vbat.readTempC()));
    Serial.printlnf("rH: %0.1f Pressure: %0.3fhPa", data::relativeHumidityPercent, data::pressurehPa);

    PRINTLN("Bld " __DATE__); // 13
    PRINTLN("Bld " __TIME__); // 14

    //Particle.publish("operating", NULL, 60, PRIVATE);

    peripherals::Joystick::JOYSTICK_DIRECTION joyDir;
    uint16_t vert = peripherals::Joystick::joystick.getVertical();
    uint16_t horiz = peripherals::Joystick::joystick.getHorizontal();
    if (vert > peripherals::Joystick::DOWN_THRESHOLD && horiz > peripherals::Joystick::RIGHT_THRESHOLD) {
        joyDir = peripherals::Joystick::DOWN_RIGHT;
    } else if (vert > peripherals::Joystick::DOWN_THRESHOLD && horiz < peripherals::Joystick::LEFT_THRESHOLD) {
        joyDir = peripherals::Joystick::DOWN_LEFT;
    } else if (vert < peripherals::Joystick::UP_THRESHOLD && horiz > peripherals::Joystick::RIGHT_THRESHOLD) {
        joyDir = peripherals::Joystick::UP_RIGHT;
    } else if (vert < peripherals::Joystick::UP_THRESHOLD && horiz < peripherals::Joystick::LEFT_THRESHOLD) {
        joyDir = peripherals::Joystick::UP_LEFT;
    } else if (vert > peripherals::Joystick::DOWN_THRESHOLD) {
        joyDir = peripherals::Joystick::DOWN;
    } else if (vert < peripherals::Joystick::UP_THRESHOLD) {
        joyDir = peripherals::Joystick::UP;
    } else if (horiz < peripherals::Joystick::LEFT_THRESHOLD) {
        joyDir = peripherals::Joystick::LEFT;
    } else if (horiz > peripherals::Joystick::RIGHT_THRESHOLD) {
        joyDir = peripherals::Joystick::RIGHT;
    } else {
        joyDir = peripherals::Joystick::CENTER;
    }

    unsigned long drawStart = millis();

    // For convenience: https://github.com/olikraus/u8g2/wiki/u8g2reference

    // Unlock the SSD1327
    switch (joyDir) {
    case peripherals::Joystick::UP:
        u8g2_SendF(&peripherals::Display::u8g2, "ca", 0xFD, 0x12 | (0<<2));
        peripherals::Display::init();
        u8g2_SendF(&peripherals::Display::u8g2, "ca", 0xFD, 0x12 | (1<<2));
        break;
    case peripherals::Joystick::DOWN:
        u8g2_SendF(&peripherals::Display::u8g2, "ca", 0xFD, 0x12 | (0<<2));
        //u8g2_SetContrast(&peripherals::Display::u8g2, 0);
        //u8g2_SendF(&peripherals::Display::u8g2, "ca", 0x81, 0xFF);
        // These are the cause of the bad screen offsets.
        // See u8x8_d_ssd1327_128x128_init_seq and friends for comparison.
        // The Zio docs are just flat wrong.
        u8g2_SendF(&peripherals::Display::u8g2, "ca", 0xa2, 0x00);
        u8g2_SendF(&peripherals::Display::u8g2, "ca", 0xA8, 127);
        u8g2_SendF(&peripherals::Display::u8g2, "ca", 0xFD, 0x12 | (1<<2));
        break;
    case peripherals::Joystick::LEFT:
        u8g2_SendF(&peripherals::Display::u8g2, "ca", 0xFD, 0x12 | (0<<2));
        peripherals::Display::u8g2_ssd1327_register_reset();
        u8g2_SendF(&peripherals::Display::u8g2, "ca", 0xFD, 0x12 | (1<<2));
        break;
    case peripherals::Joystick::RIGHT: {
            u8g2_ClearBuffer(&peripherals::Display::u8g2);

            const uint8_t *big_font = u8g2_font_osb29_tf; // 0 is widest at 16px, 100 is 69px, -20 is 58
            constexpr u8g2_uint_t text_big_height = 29;
            const uint8_t *small_font = u8g2_font_osb18_tf; // F is 18 px, C is 17
            constexpr u8g2_uint_t text_small_height = 18;

            String tempStr = data::Display::temperature_set ? String(data::Display::temperature_F, 0) : "NaN";
            String degStr = "\u00b0";
            String unitStr = "F";

            // everything works from bottom-left coordinates (for English UTF8, at least...)

            constexpr u8g2_uint_t temp_y = text_big_height;
            constexpr u8g2_uint_t unit_y = temp_y;
            constexpr u8g2_uint_t deg_y = text_small_height;

            u8g2_SetFont(&peripherals::Display::u8g2, small_font);
            const u8g2_uint_t unit_width = u8g2_GetUTF8Width(&peripherals::Display::u8g2, unitStr.c_str());
            const u8g2_uint_t unit_x = peripherals::Display::WIDTH - unit_width;
            const u8g2_uint_t deg_x = unit_x;
            u8g2_SetFont(&peripherals::Display::u8g2, big_font);
            const u8g2_uint_t temp_width = u8g2_GetUTF8Width(&peripherals::Display::u8g2, tempStr.c_str());
            const u8g2_uint_t temp_x = peripherals::Display::WIDTH - unit_width - temp_width;

            // Write the numeric value
            u8g2_SetFont(&peripherals::Display::u8g2, big_font);
            u8g2_DrawUTF8(&peripherals::Display::u8g2, temp_x, temp_y, tempStr.c_str());
            // Write the symbol labels
            u8g2_SetFont(&peripherals::Display::u8g2, small_font);
            u8g2_DrawUTF8(&peripherals::Display::u8g2, deg_x, deg_y, degStr.c_str());
            u8g2_DrawUTF8(&peripherals::Display::u8g2, unit_x, unit_y, unitStr.c_str());

            // Reset to global default
            u8g2_SetFont(&peripherals::Display::u8g2, u8g2_font_nerhoe_tf);
            //Serial.printlnf("%u %u", u8g2_GetAscent(&peripherals::Display::u8g2), u8g2_GetDescent(&peripherals::Display::u8g2));
            u8g2_DrawFrame(&peripherals::Display::u8g2, temp_x-2, 0, peripherals::Display::WIDTH - temp_x + 2, temp_y + 2);
        }
        break;
    case peripherals::Joystick::UP_LEFT:
        break;
    case peripherals::Joystick::UP_RIGHT:
        break;
    case peripherals::Joystick::DOWN_LEFT:
        break;
    case peripherals::Joystick::DOWN_RIGHT:
        break;
    case peripherals::Joystick::CENTER:
        break;
    default:
        break;
    }
    {
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
    // Unlock the SSD1327
    u8g2_SendF(&peripherals::Display::u8g2, "ca", 0xFD, 0x12 | (0<<2));
    u8g2_SendBuffer(&peripherals::Display::u8g2);
    // Lock the SSD1327
    // At some point scrolling got enabled and it started overwriting the screen with gibberish
    // after the first render. It was weird and actually suggests there's still something very
    // broken somewhere else.
    u8g2_SendF(&peripherals::Display::u8g2, "ca", 0xFD, 0x12 | (1<<2));

    unsigned long drawTime = millis() - drawStart;
    if (drawTime < 1000) {
        delay(1000 - drawTime);
    }
    Serial.printlnf("draw latency: %lu ms", drawTime);
}
