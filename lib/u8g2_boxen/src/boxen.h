
#include "U8g2lib.h"

typedef const uint8_t FontData;

class Box {
public:
    enum BoxValueType {BoxTypeUnset, BoxTypeFloat, BoxTypeIntSigned, BoxTypeIntUnsigned};
private:
    Box();
    u8g2_t *m_u8g2;
    u8g2_int_t m_x;
    u8g2_int_t m_y;
    u8g2_int_t m_width;
    u8g2_int_t m_height;
    FontData *m_labelFont;
    const char *m_labelTop;
    const char *m_labelBottom;
    FontData *m_valueFont;
    uint8_t m_digits;
    union {
        uint32_t u;
        int32_t i;
        float f;
    } m_value;
    void Render(BoxValueType boxType);
public:
    // x,y is the top left corner of the Box, height and width count down and do the right.
    Box(u8g2_t *u8g2, u8g2_int_t x, u8g2_int_t y, u8g2_int_t width, u8g2_int_t height, const uint8_t *labelFont, const char *labelTop, const char *labelBottom, const uint8_t *valueFont, uint8_t digits);

    void UpdateValue(int32_t val);
    void UpdateValue(uint32_t val);
    void UpdateValue(float val);
    void UpdateValue();
};
