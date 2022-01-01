
// my own composition, probably based a bit on the Sensirion official driver and definitely styled after other Arduino libraries
// MIT license

#ifndef SPS30_H
#define SPS30_H

#include "Arduino.h"
#include <Wire.h>

// Reading floating-point values from the SPS30 requires a ~40-byte contiguous
// I2C transaction. This is impossible on standard Arduino, which has a hard-
// coded I2C buffer size of 32, but the underlying HAL may offer platform-
// specific techniques for overriding this value.
#if defined(PARTICLE_WIRING_ARDUINO_COMPATIBILTY)
// Set this to the value of hal_i2c_config_t.rx_buffer and .tx_buffer that
// you provide in acquireWireBuffer().
#define SPS30_I2C_BUFFER_LEN 512
#elif defined(ARDUINO)
#define SPS30_I2C_BUFFER_LEN 32
#else
#warning SPS30 driver does not have an I2C buffer length for this platform; defaulting to Arduino 32-byte limit.
#endif

#ifndef SPS30_I2C_BUFFER_LEN
#define SPS30_I2C_BUFFER_LEN 32
#endif

// Each I2C RX transaction is
// TX: i2c addr, reg addr MSB, reg addr LSB, RX: data 0, data 1, data 0-1 CRC[, data n, data n+1, data n-n+1 CRC]
// The I2C addr byte is usually handled separately by the HAL, so it does not need to be acccounted for.
// This means that the I2C buffer length for a given message, as a function of the message, is:
#define SPS30_I2C_BUFFER_LEN_FROM_DATA_SIZE(data_byte_count) (data_byte_count * 3 / 2)
#define SPS30_I2C_DATA_SIZE_FROM_BUFFER_LEN(len) (len * 2 / 3)

// Used when calling read_data to validate arguments and allocate arrays
#define SPS30_I2C_FLOAT_DATA_SIZE (10*4)
#define SPS30_I2C_INT_DATA_SIZE (10*2)
#define SPS30_I2C_SERIAL_BUFFER_LEN (48)
#define SPS30_I2C_SERIAL_DATA_SIZE SPS30_I2C_DATA_SIZE_FROM_BUFFER_LEN(SPS30_I2C_SERIAL_BUFFER_LEN)
#define SPS30_I2C_MAX_DATA_SIZE SPS30_I2C_FLOAT_DATA_SIZE
#define SPS30_I2C_MAX_RX_BUFFER_LEN SPS30_I2C_BUFFER_LEN_FROM_DATA_SIZE(SPS30_I2C_MAX_DATA_SIZE)
// Used in ifdefs to remove unsupportable functionality
#define SPS30_I2C_BUFFER_LEN_FOR_FLOAT SPS30_I2C_BUFFER_LEN_FROM_DATA_SIZE(SPS30_I2C_FLOAT_DATA_SIZE)
#define SPS30_I2C_BUFFER_LEN_FOR_SERIAL SPS30_I2C_BUFFER_LEN_FROM_DATA_SIZE(SPS30_I2C_SERIAL_DATA_SIZE)

typedef enum _SPS30_ERR {
    SPS30_OK = 0,
    SPS30_I2C_TOO_LONG = 1,
    SPS30_I2C_NACK_ON_ADDR = 2,
    SPS30_I2C_NACK_ON_DATA = 3,
    SPS30_I2C_OTHER = 4,
    SPS30_CRC_FAIL = 16,
    SPS30_READ_INCOMPLETE,
    SPS30_BAD_ARGUMENTS,
    SPS30_NO_DATA,
    SPS30_NOT_PRESENT,
    SPS30_FW_VERSION_TOO_LOW,
    SPS30_WRONG_FORMAT,
    SPS30_ERR_MAX
} SPS30_ERR;

typedef enum _SPS30_DATA_FORMAT {
    SPS30_FORMAT_IEEE754 = 0x03,
    SPS30_FORMAT_UINT16 = 0x05,
    SPS30_FORMAT_MAX
} SPS30_DATA_FORMAT;

typedef struct _SPS30_DATA_FLOAT {
    float pm_1_0_ug_m3;
    float pm_2_5_ug_m3;
    float pm_4_0_ug_m3;
    float pm_10_ug_m3;
    float pm_0_5_n_cm3;
    float pm_1_0_n_cm3;
    float pm_2_5_n_cm3;
    float pm_4_0_n_cm3;
    float pm_10_n_cm3;
    float typical_size_um;
} SPS30_DATA_FLOAT;

typedef struct _SPS30_DATA_INT {
    uint16_t pm_1_0_ug_m3;
    uint16_t pm_2_5_ug_m3;
    uint16_t pm_4_0_ug_m3;
    uint16_t pm_10_ug_m3;
    uint16_t pm_0_5_n_cm3;
    uint16_t pm_1_0_n_cm3;
    uint16_t pm_2_5_n_cm3;
    uint16_t pm_4_0_n_cm3;
    uint16_t pm_10_n_cm3;
    uint16_t typical_size_nm;
} SPS30_DATA_INT;

class SPS30 {
public:
    SPS30();
    bool begin(TwoWire &wire_port = Wire, uint8_t i2c_addr = 0x69);
    SPS30_ERR start_measuring(SPS30_DATA_FORMAT data_format);
    SPS30_ERR stop_measuring();
    SPS30_ERR is_data_ready();
#if SPS30_I2C_BUFFER_LEN >= SPS30_I2C_BUFFER_LEN_FOR_FLOAT
    SPS30_ERR read_data_no_wait_float(SPS30_DATA_FLOAT *data_struct);
#endif
    SPS30_ERR read_data_no_wait_int(SPS30_DATA_INT *data_struct);

    SPS30_ERR sleep();
    SPS30_ERR wake();

    SPS30_ERR reset();

    SPS30_ERR start_fan_cleaning_cycle();
    SPS30_ERR read_fan_cleaning_interval(uint32_t *interval_seconds);
    SPS30_ERR set_fan_cleaning_interval(uint32_t interval_seconds);

    SPS30_ERR read_status_register(bool *fan_speed_warning, bool *laser_current_error, bool *fan_rpm_error);
    SPS30_ERR clear_status_register();

#if SPS30_I2C_BUFFER_LEN >= SPS30_I2C_BUFFER_LEN_FOR_SERIAL
    SPS30_ERR read_serial(String &str);
#endif

    static SPS30_ERR float_append_to_string(SPS30_DATA_FLOAT &datum, String &str);
    static SPS30_ERR int_append_to_string(SPS30_DATA_INT &datum, String &str);

    uint8_t product_type[8];
    uint8_t firmware_version[2];
    SPS30_ERR device_status;

private:
    uint8_t sensirion_CalcCrc(uint8_t data[2]);

    SPS30_ERR set_pointer(uint16_t ptr_val);
    SPS30_ERR read_data(uint8_t desired_bytes, uint8_t *buf, size_t buf_len, bool stop = true);
    SPS30_ERR set_pointer_and_write(uint16_t addr, uint8_t *buf, uint8_t buf_count);

    TwoWire *m_wirePort;
    uint8_t m_i2c_addr;
    SPS30_DATA_FORMAT m_data_format;
};

#endif // SPS30_H
