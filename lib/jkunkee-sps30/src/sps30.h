
// my own composition, probably based a bit on the Sensirion official driver and definitely styled after other Arduino libraries
// MIT license

#ifndef SPS30_H
#define SPS30_H

#include "Arduino.h"
#include <Wire.h>

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
    SPS30_ERR read_data_no_wait_float(SPS30_DATA_FLOAT *data_struct);
    SPS30_ERR read_data_no_wait_int(SPS30_DATA_INT *data_struct);

    SPS30_ERR sleep();
    SPS30_ERR wake();

    SPS30_ERR reset();

    SPS30_ERR start_fan_cleaning_cycle();
    SPS30_ERR read_fan_cleaning_interval(uint32_t *interval_seconds);
    SPS30_ERR set_fan_cleaning_interval(uint32_t interval_seconds);

    SPS30_ERR read_status_register(bool *fan_speed_warning, bool *laser_current_error, bool *fan_rpm_error);
    SPS30_ERR clear_status_register();

    SPS30_ERR read_serial(String &str);

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
