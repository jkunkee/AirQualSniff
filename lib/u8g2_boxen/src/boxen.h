
#include "U8g2lib.h"

typedef const uint8_t FontData;

class Box {
public:
    enum BoxValueType {BoxTypeFloat, BoxTypeIntSigned, BoxTypeIntUnsigned};
private:
    Box();
    u8g2_t *m_u8g2;
    BoxValueType m_boxType;
    u8g2_int_t m_x;
    u8g2_int_t m_y;
    u8g2_int_t m_width;
    u8g2_int_t m_height;
    FontData *m_labelFont;
    char *m_labelTop;
    char *m_labelBottom;
    FontData *m_valueFont;
    uint8_t m_digits;
    union {
        uint32_t u;
        int32_t i;
        float f;
    } m_value;
    void Render();
public:
    // x,y is the top left corner of the Box, width and height count down and do the right.
    Box(u8g2_t *u8g2, int32_t value,        u8g2_int_t x, u8g2_int_t y, u8g2_int_t width, u8g2_int_t height, const uint8_t *labelFont, char *labelTop, char *labelBottom, const uint8_t *valueFont, uint8_t digits);
    Box(u8g2_t *u8g2, uint32_t value,       u8g2_int_t x, u8g2_int_t y, u8g2_int_t width, u8g2_int_t height, const uint8_t *labelFont, char *labelTop, char *labelBottom, const uint8_t *valueFont, uint8_t digits);
    Box(u8g2_t *u8g2, float value,          u8g2_int_t x, u8g2_int_t y, u8g2_int_t width, u8g2_int_t height, const uint8_t *labelFont, char *labelTop, char *labelBottom, const uint8_t *valueFont, uint8_t digits);
    Box(u8g2_t *u8g2, BoxValueType boxType, u8g2_int_t x, u8g2_int_t y, u8g2_int_t width, u8g2_int_t height, const uint8_t *labelFont, char *labelTop, char *labelBottom, const uint8_t *valueFont, uint8_t digits);

    void UpdateValue(int32_t val);
    void UpdateValue(uint32_t val);
    void UpdateValue(float val);
};
