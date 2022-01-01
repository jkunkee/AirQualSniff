
#include "sps30.h"

//#define SERIAL_DEBUG

static const uint16_t ADDR_START_MEASURING = 0x0010;
static const uint16_t ADDR_STOP_MEASURING = 0x0104;
static const uint16_t ADDR_DATA_READY = 0x0202;
static const uint16_t ADDR_DATA = 0x0300;
static const uint16_t ADDR_SLEEP = 0x1001;
static const uint16_t ADDR_WAKE = 0x1103;
static const uint16_t ADDR_RUN_FAN_CLEAN = 0x5607;
static const uint16_t ADDR_AUTO_CLEAN_INTERVAL = 0x8004;
static const uint16_t ADDR_PRODUCT_TYPE = 0xD002;
static const uint16_t ADDR_SERIAL = 0xD033;
static const uint16_t ADDR_FW_VERSION = 0xD100;
static const uint16_t ADDR_STATUS_REG_READ = 0xD206;
static const uint16_t ADDR_STATUS_REG_CLEAR = 0xD210;
static const uint16_t ADDR_RESET = 0xD304;

static const char *DEVICE_INFO_SHIBBOLETH = "00080000";

SPS30::SPS30() {
    m_wirePort = NULL;
    m_i2c_addr = 0x00;
    device_status = SPS30_NOT_PRESENT;
    m_data_format = SPS30_FORMAT_MAX;
}

bool SPS30::begin(TwoWire &wirePort, uint8_t i2c_addr) {
    m_wirePort = &wirePort;
    m_wirePort->begin();
    m_i2c_addr = i2c_addr;

    set_pointer(ADDR_PRODUCT_TYPE);
    SPS30_ERR type_rd = read_data(8, product_type, sizeof(product_type));

    set_pointer(ADDR_FW_VERSION);
    SPS30_ERR ver_rd = read_data(2, firmware_version, sizeof(firmware_version));

    bool shibboleth_ok = type_rd == SPS30_OK &&
        product_type[0] == DEVICE_INFO_SHIBBOLETH[0] &&
        product_type[1] == DEVICE_INFO_SHIBBOLETH[1] &&
        product_type[2] == DEVICE_INFO_SHIBBOLETH[2] &&
        product_type[3] == DEVICE_INFO_SHIBBOLETH[3] &&
        product_type[4] == DEVICE_INFO_SHIBBOLETH[4] &&
        product_type[5] == DEVICE_INFO_SHIBBOLETH[5] &&
        product_type[6] == DEVICE_INFO_SHIBBOLETH[6] &&
        product_type[7] == DEVICE_INFO_SHIBBOLETH[7];

    bool version_ok = ver_rd == SPS30_OK &&
        firmware_version[0] >= 2 &&
        firmware_version[1] >= 2;

    if (!shibboleth_ok) {
        device_status = SPS30_NOT_PRESENT;
    } else if (!version_ok) {
        device_status = SPS30_FW_VERSION_TOO_LOW;
    } else {
        device_status = SPS30_OK;
    }

    stop_measuring();

    return shibboleth_ok && version_ok;
}

SPS30_ERR SPS30::start_measuring(SPS30_DATA_FORMAT data_format) {
    if (data_format != SPS30_FORMAT_IEEE754 && data_format != SPS30_FORMAT_UINT16) {
        return SPS30_BAD_ARGUMENTS;
    }
    uint8_t buf[2];
    buf[0] = (uint8_t)data_format;
    buf[1] = 0;
    SPS30_ERR result = set_pointer_and_write(ADDR_START_MEASURING, buf, sizeof(buf));
    delay(20);
    m_data_format = data_format;
    return result;
}

SPS30_ERR SPS30::stop_measuring() {
    SPS30_ERR result = set_pointer(ADDR_STOP_MEASURING);
    delay(20);
    return result;
}

SPS30_ERR SPS30::is_data_ready() {
    uint8_t buf[2];
    set_pointer(ADDR_DATA_READY);
    SPS30_ERR result = read_data(2, buf, sizeof(buf));
    if (result != SPS30_OK) {
        return result;
    } else {
        return buf[1] == 0x1 ? SPS30_OK : SPS30_NO_DATA;
    }
}

#if SPS30_I2C_BUFFER_LEN >= SPS30_I2C_BUFFER_LEN_FOR_FLOAT
typedef union {
    uint32_t int32; // using a byte array would require knowing the endianness of the system
    float fl;
} int_float;
SPS30_ERR SPS30::read_data_no_wait_float(SPS30_DATA_FLOAT *data_struct) {
    if (data_struct == NULL) {
        return SPS30_BAD_ARGUMENTS;
    }
    if (m_data_format != SPS30_FORMAT_IEEE754) {
        return SPS30_WRONG_FORMAT;
    }
    uint8_t buf[SPS30_I2C_FLOAT_DATA_SIZE] = { 0 };
    set_pointer(ADDR_DATA);
    SPS30_ERR result = read_data(sizeof(buf), buf, sizeof(buf));
    if (result != SPS30_OK) {
        return result;
    }
    int idx = 0;
    int_float mixer;
    mixer.int32 = (((uint32_t)buf[idx]) << 24) | (((uint32_t)buf[idx+1]) << 16) | (((uint32_t)buf[idx+2]) << 8) | (((uint32_t)buf[idx+3]) << 0); idx += 4; data_struct->pm_1_0_ug_m3 = mixer.fl;
    mixer.int32 = (((uint32_t)buf[idx]) << 24) | (((uint32_t)buf[idx+1]) << 16) | (((uint32_t)buf[idx+2]) << 8) | (((uint32_t)buf[idx+3]) << 0); idx += 4; data_struct->pm_2_5_ug_m3 = mixer.fl;
    mixer.int32 = (((uint32_t)buf[idx]) << 24) | (((uint32_t)buf[idx+1]) << 16) | (((uint32_t)buf[idx+2]) << 8) | (((uint32_t)buf[idx+3]) << 0); idx += 4; data_struct->pm_4_0_ug_m3 = mixer.fl;
    mixer.int32 = (((uint32_t)buf[idx]) << 24) | (((uint32_t)buf[idx+1]) << 16) | (((uint32_t)buf[idx+2]) << 8) | (((uint32_t)buf[idx+3]) << 0); idx += 4; data_struct->pm_10_ug_m3 = mixer.fl;
    mixer.int32 = (((uint32_t)buf[idx]) << 24) | (((uint32_t)buf[idx+1]) << 16) | (((uint32_t)buf[idx+2]) << 8) | (((uint32_t)buf[idx+3]) << 0); idx += 4; data_struct->pm_0_5_n_cm3 = mixer.fl;
    mixer.int32 = (((uint32_t)buf[idx]) << 24) | (((uint32_t)buf[idx+1]) << 16) | (((uint32_t)buf[idx+2]) << 8) | (((uint32_t)buf[idx+3]) << 0); idx += 4; data_struct->pm_1_0_n_cm3 = mixer.fl;
    mixer.int32 = (((uint32_t)buf[idx]) << 24) | (((uint32_t)buf[idx+1]) << 16) | (((uint32_t)buf[idx+2]) << 8) | (((uint32_t)buf[idx+3]) << 0); idx += 4; data_struct->pm_2_5_n_cm3 = mixer.fl;
    mixer.int32 = (((uint32_t)buf[idx]) << 24) | (((uint32_t)buf[idx+1]) << 16) | (((uint32_t)buf[idx+2]) << 8) | (((uint32_t)buf[idx+3]) << 0); idx += 4; data_struct->pm_4_0_n_cm3 = mixer.fl;
    mixer.int32 = (((uint32_t)buf[idx]) << 24) | (((uint32_t)buf[idx+1]) << 16) | (((uint32_t)buf[idx+2]) << 8) | (((uint32_t)buf[idx+3]) << 0); idx += 4; data_struct->pm_10_n_cm3 = mixer.fl;
    mixer.int32 = (((uint32_t)buf[idx]) << 24) | (((uint32_t)buf[idx+1]) << 16) | (((uint32_t)buf[idx+2]) << 8) | (((uint32_t)buf[idx+3]) << 0); idx += 4; data_struct->typical_size_um = mixer.fl;
    return SPS30_OK;
}
#endif

SPS30_ERR SPS30::read_data_no_wait_int(SPS30_DATA_INT *data_struct) {
    uint8_t buf[SPS30_I2C_INT_DATA_SIZE];
    SPS30_ERR result;

    if (data_struct == NULL) {
        return SPS30_BAD_ARGUMENTS;
    }
    if (m_data_format != SPS30_FORMAT_UINT16) {
        return SPS30_WRONG_FORMAT;
    }
    result = set_pointer(ADDR_DATA);
    if (result != SPS30_OK) {
        return result;
    }
    result = read_data(sizeof(buf), buf, sizeof(buf));
    if (result != SPS30_OK) {
        return result;
    }
    int idx = 0;
    data_struct->pm_1_0_ug_m3 = ((((uint16_t)buf[idx]) << 8) | (((uint16_t)buf[idx+1]) << 0)); idx += 2;
    data_struct->pm_2_5_ug_m3 = ((((uint16_t)buf[idx]) << 8) | (((uint16_t)buf[idx+1]) << 0)); idx += 2;
    data_struct->pm_4_0_ug_m3 = ((((uint16_t)buf[idx]) << 8) | (((uint16_t)buf[idx+1]) << 0)); idx += 2;
    data_struct->pm_10_ug_m3  = ((((uint16_t)buf[idx]) << 8) | (((uint16_t)buf[idx+1]) << 0)); idx += 2;
    data_struct->pm_0_5_n_cm3 = ((((uint16_t)buf[idx]) << 8) | (((uint16_t)buf[idx+1]) << 0)); idx += 2;
    data_struct->pm_1_0_n_cm3 = ((((uint16_t)buf[idx]) << 8) | (((uint16_t)buf[idx+1]) << 0)); idx += 2;
    data_struct->pm_2_5_n_cm3 = ((((uint16_t)buf[idx]) << 8) | (((uint16_t)buf[idx+1]) << 0)); idx += 2;
    data_struct->pm_4_0_n_cm3 = ((((uint16_t)buf[idx]) << 8) | (((uint16_t)buf[idx+1]) << 0)); idx += 2;
    data_struct->pm_10_n_cm3  = ((((uint16_t)buf[idx]) << 8) | (((uint16_t)buf[idx+1]) << 0)); idx += 2;
    data_struct->typical_size_nm = ((((uint16_t)buf[idx]) << 8) | (((uint16_t)buf[idx+1]) << 0)); idx += 2;
    return SPS30_OK;
}

SPS30_ERR SPS30::sleep() {
    SPS30_ERR result = set_pointer(ADDR_SLEEP);
    delay(5);
    return result;
}

SPS30_ERR SPS30::wake() {
    // Use fallback wake technique
    // Wake I2C interface. The send fails (SPS30_I2C_NACK_ON_DATA) because the command is ignored.
    SPS30_ERR result;
    set_pointer(ADDR_WAKE);
    // Send a second time now that the interface is awake.
    result = set_pointer(ADDR_WAKE);
    delay(5);
    return result;
}

SPS30_ERR SPS30::reset() {
    SPS30_ERR result = set_pointer(ADDR_RESET);
    delay(100);
    return result;
}

SPS30_ERR SPS30::start_fan_cleaning_cycle() {
    SPS30_ERR result = set_pointer(ADDR_RUN_FAN_CLEAN);
    delay(5);
    return result;
}

SPS30_ERR SPS30::read_fan_cleaning_interval(uint32_t *interval_seconds) {
    uint8_t buf[4];
    SPS30_ERR result;

    if (interval_seconds == NULL) {
        return SPS30_BAD_ARGUMENTS;
    }

    result = set_pointer(ADDR_AUTO_CLEAN_INTERVAL);
    if (result != SPS30_OK) {
        return result;
    }
    delay(5);
    result = read_data(4, buf, sizeof(buf));
    if (result != SPS30_OK) {
        return result;
    }

    *interval_seconds = 0;
    *interval_seconds |= (buf[0] << 24) & 0xff;
    *interval_seconds |= (buf[1] << 16) & 0xff;
    *interval_seconds |= (buf[2] <<  8) & 0xff;
    *interval_seconds |= (buf[3] <<  0) & 0xff;

    return SPS30_OK;
}

SPS30_ERR SPS30::set_fan_cleaning_interval(uint32_t interval_seconds) {
    uint8_t buf[4];
    SPS30_ERR result;

    buf[0] = (interval_seconds >> 24) & 0xFF;
    buf[1] = (interval_seconds >> 16) & 0xFF;
    buf[2] = (interval_seconds >>  8) & 0xFF;
    buf[3] = (interval_seconds >>  0) & 0xFF;

    result = set_pointer_and_write(ADDR_AUTO_CLEAN_INTERVAL, buf, 4);
    delay(20);
    return result;
}

SPS30_ERR SPS30::read_status_register(bool *fan_speed_warning, bool *laser_current_error, bool *fan_speed_error) {
    uint8_t buf[4];
    SPS30_ERR result;

    if (fan_speed_warning == NULL && laser_current_error == NULL && fan_speed_error == NULL) {
        return SPS30_BAD_ARGUMENTS;
    }

    result = set_pointer(ADDR_STATUS_REG_READ);
    if (result != SPS30_OK) {
        return result;
    }
    result = read_data(4, buf, sizeof(buf));
    if (result != SPS30_OK) {
        return result;
    }

    if (fan_speed_warning != NULL) {
        *fan_speed_warning = (buf[1] & 0x20) != 0;
    }
    if (laser_current_error != NULL) {
        *laser_current_error = (buf[3] & 0x20) != 0;
    }
    if (fan_speed_warning != NULL) {
        *fan_speed_warning = (buf[3] & 0x10) != 0;
    }
    return SPS30_OK;
}

SPS30_ERR SPS30::clear_status_register() {
    SPS30_ERR result = set_pointer(ADDR_STATUS_REG_CLEAR);
    delay(5);
    return result;
}

#if SPS30_I2C_BUFFER_LEN >= SPS30_I2C_BUFFER_LEN_FOR_SERIAL
// Warning, this has not been tested
SPS30_ERR SPS30::read_serial(String &str) {
    uint8_t buf[48];
    SPS30_ERR result;

    result = set_pointer(ADDR_SERIAL);
    if (result != SPS30_OK) {
        return result;
    }
    result = read_data(20, buf, sizeof(buf));
    if (result != SPS30_OK) {
        return result;
    }
    result = set_pointer(ADDR_SERIAL+20);
    if (result != SPS30_OK) {
        return result;
    }
    result = read_data(20, buf+20, sizeof(buf)-20);
    if (result != SPS30_OK) {
        return result;
    }
    result = set_pointer(ADDR_SERIAL+40);
    if (result != SPS30_OK) {
        return result;
    }
    result = read_data(8, buf+40, sizeof(buf)-40);
    if (result != SPS30_OK) {
        return result;
    }
    str += (char*)buf;
    return SPS30_OK;
}
#endif

/*
SPS30 fp datum: ug1.0um/m3:0.693 ug2.5um/m3:0.733 ug4.0um/m3:0.733 ug10um/m3:0.733 n0.5um/cm3:4.735 n1.0um/cm3:5.517 n2.5um/cm3:5.533 n4.0um/cm3:5.534 n10um/cm3:5.535 typical um:0.435
*/
SPS30_ERR SPS30::float_append_to_string(SPS30_DATA_FLOAT &datum, String &str) {
    constexpr int decimal_places = 3;
    str += "SPS30 fp datum:";
    str += " ug1.0um/m3:" + String(datum.pm_1_0_ug_m3, decimal_places);
    str += " ug2.5um/m3:" + String(datum.pm_2_5_ug_m3, decimal_places);
    str += " ug4.0um/m3:" + String(datum.pm_4_0_ug_m3, decimal_places);
    str += " ug10um/m3:" + String(datum.pm_10_ug_m3, decimal_places);
    str += " n0.5um/cm3:" + String(datum.pm_0_5_n_cm3, decimal_places);
    str += " n1.0um/cm3:" + String(datum.pm_1_0_n_cm3, decimal_places);
    str += " n2.5um/cm3:" + String(datum.pm_2_5_n_cm3, decimal_places);
    str += " n4.0um/cm3:" + String(datum.pm_4_0_n_cm3, decimal_places);
    str += " n10um/cm3:" + String(datum.pm_10_n_cm3, decimal_places);
    str += " typical um:" + String(datum.typical_size_um, decimal_places);
    return SPS30_OK;
}
/*
SPS30 int datum: ug1.0um/m3:1 ug2.5um/m3:1 ug4.0um/m3:1 ug10um/m3:1 n0.5um/cm3:8 n1.0um/cm3:9 n2.5um/cm3:9 n4.0um/cm3:9 n10um/cm3:9 typical nm:398
*/
SPS30_ERR SPS30::int_append_to_string(SPS30_DATA_INT &datum, String &str) {
    str += "SPS30 int datum:";
    str += " ug1.0um/m3:" + String(datum.pm_1_0_ug_m3);
    str += " ug2.5um/m3:" + String(datum.pm_2_5_ug_m3);
    str += " ug4.0um/m3:" + String(datum.pm_4_0_ug_m3);
    str += " ug10um/m3:" + String(datum.pm_10_ug_m3);
    str += " n0.5um/cm3:" + String(datum.pm_0_5_n_cm3);
    str += " n1.0um/cm3:" + String(datum.pm_1_0_n_cm3);
    str += " n2.5um/cm3:" + String(datum.pm_2_5_n_cm3);
    str += " n4.0um/cm3:" + String(datum.pm_4_0_n_cm3);
    str += " n10um/cm3:" + String(datum.pm_10_n_cm3);
    str += " typical nm:" + String(datum.typical_size_nm);
    return SPS30_OK;
}

// function pulled from datasheet
uint8_t SPS30::sensirion_CalcCrc(uint8_t data[2]) {
    uint8_t crc = 0xFF;
    for(int i = 0; i < 2; i++) {
        crc ^= data[i];
        for(uint8_t bit = 8; bit > 0; --bit) {
            if(crc & 0x80) {
                crc = (crc << 1) ^ 0x31u;
            } else {
                crc = (crc << 1);
            }
        }
    }
    return crc;
}

SPS30_ERR SPS30::set_pointer(uint16_t ptr_val) {
    m_wirePort->beginTransmission(m_i2c_addr);
    m_wirePort->write(ptr_val >> 8);
    m_wirePort->write(ptr_val & 0xff);
    return (SPS30_ERR)m_wirePort->endTransmission();
}

// Attempt to read desired_bytes bytes from the device into buf after verifying checksum, without exceeding buf_len
// Return number of bytes read and checksummed
// Data is read in pairs of bytes, so desired_bytes must be even
SPS30_ERR SPS30::read_data(uint8_t desired_bytes, uint8_t *buf, size_t buf_len, bool stop) {
    if (desired_bytes > buf_len || buf == NULL || (desired_bytes % 2) != 0) {
        return SPS30_BAD_ARGUMENTS;
    }

    // The SPS30 doesn't seem to handle separate 30-byte reads; instead of continuing where it left off (persistent pointer theory),
    // it just spits out junk bytes.

//    // Break read into 20-byte segments: every two bytes read has one byte of checksum and the read has to fit within the Arduino Wire 32-byte buffer
//    while (desired_bytes > 10) {
//        SPS30_ERR stat = read_data(10, buf, buf_len, false);
//        if (stat != SPS30_OK) {
//            return stat;
//        }
//        bytes_read += 10;
//        desired_bytes -= 10;
//        buf += 10;
//        buf_len -= 10;
//    }

    // prep CRC'd buffer
    uint8_t raw_buf[SPS30_I2C_MAX_RX_BUFFER_LEN];
    uint8_t bytes_read = 0;
    if (desired_bytes > sizeof(raw_buf) || desired_bytes > SPS30_I2C_MAX_DATA_SIZE) {
        return SPS30_I2C_TOO_LONG;
    }
    // prep bounds
    uint8_t raw_expected = desired_bytes + desired_bytes / 2; // one CRC byte for every two data bytes
    // run i2c transaction
    uint8_t raw_received = m_wirePort->requestFrom(m_i2c_addr, raw_expected, stop);
    // break if not all was received
    if (raw_received != raw_expected) {
#ifdef SERIAL_DEBUG
        Serial.printlnf("requested %d, got %d bytes", raw_expected, raw_received);
#endif
        return SPS30_READ_INCOMPLETE;
    }
    // fill CRC'd buffer
    for (int raw_buf_idx = 0; raw_buf_idx < raw_received; raw_buf_idx++) {
        raw_buf[raw_buf_idx] = m_wirePort->read();
    }
    // only triplets of bytes can pass CRC, but the checks earlier already guarantee triplets
    // convert CRC'd data into confirmed data
    for (int pair_idx = 0; pair_idx*3 < raw_received; pair_idx++) {
        int raw_idx = pair_idx * 3;
        uint8_t checksum = sensirion_CalcCrc(&raw_buf[raw_idx]);
        if (raw_buf[raw_idx+2] != checksum) {
#ifdef SERIAL_DEBUG
            Serial.printlnf("tuple %x %x %x (%x) failed checksum; breaking", raw_buf[raw_idx], raw_buf[raw_idx+1], raw_buf[raw_idx+2], checksum);
#endif
            return SPS30_CRC_FAIL;
        }
        buf[0] = raw_buf[raw_idx];
        buf[1] = raw_buf[raw_idx+1];
        buf += 2;
        buf_len -= 2;
        bytes_read += 2;
#ifdef SERIAL_DEBUG
        Serial.printlnf("read '%x' '%x' %x (%x)", raw_buf[raw_idx], raw_buf[raw_idx+1], raw_buf[raw_idx+2], checksum);
#endif
    }
    return SPS30_OK;
}

SPS30_ERR SPS30::set_pointer_and_write(uint16_t addr, uint8_t *buf, uint8_t buf_count) {
    // Luckily, no write sequence is longer than 20 bytes!
    if (buf_count > 20 || buf == NULL) {
        return SPS30_BAD_ARGUMENTS;
    }

    m_wirePort->beginTransmission(m_i2c_addr);
    uint8_t cmd[2];
    cmd[0] = addr >> 8;
    cmd[1] = addr & 0xff;
    m_wirePort->write(cmd, 2);
    for (int pairIdx = 0; pairIdx < buf_count/2; pairIdx += 1) {
        uint8_t *buf_addr = &buf[pairIdx * 2];
        m_wirePort->write(buf_addr, 2);
        uint8_t crc = sensirion_CalcCrc(buf_addr);
        m_wirePort->write(&crc, 1);
    }
    return (SPS30_ERR)m_wirePort->endTransmission();
}
