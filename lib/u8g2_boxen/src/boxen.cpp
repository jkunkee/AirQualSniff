
#include "boxen.h"

Box::Box(u8g2_t *u8g2, int32_t value,        u8g2_int_t x, u8g2_int_t y, u8g2_int_t width, u8g2_int_t height, const uint8_t *labelFont, char *labelTop, char *labelBottom, const uint8_t *valueFont, uint8_t digits)
:
m_u8g2(u8g2), m_boxType(BoxTypeIntSigned), m_x(x), m_y(y), m_width(width), m_height(height), m_labelFont(labelFont), m_labelTop(labelTop), m_labelBottom(labelBottom), m_valueFont(valueFont), m_digits(digits)
{
    m_value.i = value;
}
Box::Box(u8g2_t *u8g2, uint32_t value,       u8g2_int_t x, u8g2_int_t y, u8g2_int_t width, u8g2_int_t height, const uint8_t *labelFont, char *labelTop, char *labelBottom, const uint8_t *valueFont, uint8_t digits)
:
m_u8g2(u8g2), m_boxType(BoxTypeIntUnsigned), m_x(x), m_y(y), m_width(width), m_height(height), m_labelFont(labelFont), m_labelTop(labelTop), m_labelBottom(labelBottom), m_valueFont(valueFont), m_digits(digits)
{
    m_value.u = value;
}
Box::Box(u8g2_t *u8g2, float value,          u8g2_int_t x, u8g2_int_t y, u8g2_int_t width, u8g2_int_t height, const uint8_t *labelFont, char *labelTop, char *labelBottom, const uint8_t *valueFont, uint8_t digits)
:
m_u8g2(u8g2), m_boxType(BoxTypeFloat), m_x(x), m_y(y), m_width(width), m_height(height), m_labelFont(labelFont), m_labelTop(labelTop), m_labelBottom(labelBottom), m_valueFont(valueFont), m_digits(digits)
{
    m_value.f = value;
}
Box::Box(u8g2_t *u8g2, BoxValueType boxType, u8g2_int_t x, u8g2_int_t y, u8g2_int_t width, u8g2_int_t height, const uint8_t *labelFont, char *labelTop, char *labelBottom, const uint8_t *valueFont, uint8_t digits)
:
m_u8g2(u8g2), m_boxType(boxType), m_x(x), m_y(y), m_width(width), m_height(height), m_labelFont(labelFont), m_labelTop(labelTop), m_labelBottom(labelBottom), m_valueFont(valueFont), m_digits(digits)
{
    m_value.i = 0;
}

void setFontWrapper(u8g2_t *u, FontData *f) {
    u8g2_SetFont(u, f);
    // I can't tell why, but these lines buried in u8g2_SetFont don't run on my Particle Photon
    u->font_info.ascent_A = f[13];
    u->font_info.descent_g = f[14];
    u->font_ref_ascent = u->font_info.ascent_A;
    u->font_ref_descent = u->font_info.descent_g;
}

void Box::Render() {
    constexpr size_t bufLen = 20;
    char buf[bufLen];
    switch (m_boxType) {
    case BoxTypeFloat:
        switch (m_digits) {
        case 0:
            snprintf(buf, bufLen, "%0.0f", m_value.f);
            break;
        default:
        case 1:
            snprintf(buf, bufLen, "%0.1f", m_value.f);
            break;
        case 2:
            snprintf(buf, bufLen, "%0.2f", m_value.f);
            break;
        case 3:
            snprintf(buf, bufLen, "%0.3f", m_value.f);
            break;
        }
        break;
    case BoxTypeIntSigned:
        snprintf(buf, bufLen, "%ld", m_value.i);
        break;
    default:
    case BoxTypeIntUnsigned:
        snprintf(buf, bufLen, "%lu", m_value.u);
        break;
    }

    // 'push' existing font
    FontData *stashedFont = m_u8g2->font;

    // Don't draw outside the specified box
    u8g2_SetClipWindow(m_u8g2, m_x, m_y, m_x + m_width, m_y + m_height);

    // Don't overwrite what's underneath
    u8g2_SetFontMode(m_u8g2, 1);

    // Draw the value
    setFontWrapper(m_u8g2, m_valueFont);
    u8g2_int_t new_x = u8g2_DrawUTF8(m_u8g2, m_x, m_y + u8g2_GetFontAscent(m_u8g2), buf);

    // Draw the label(s)
    setFontWrapper(m_u8g2, m_labelFont);
    if (strlen(m_labelBottom) == 0) {
        u8g2_DrawUTF8(m_u8g2, new_x, m_y + m_height + u8g2_GetFontDescent(m_u8g2), m_labelTop);
    } else {
        u8g2_DrawUTF8(m_u8g2, new_x, m_y + u8g2_GetFontAscent(m_u8g2), m_labelTop);
        u8g2_DrawUTF8(m_u8g2, new_x, m_y + m_height + u8g2_GetFontDescent(m_u8g2), m_labelBottom);
    }

    // Draw a cute little box around everything
    u8g2_DrawFrame(m_u8g2, m_x, m_y, m_width, m_height);

    // reset clip window
    u8g2_SetMaxClipWindow(m_u8g2);

    // 'pop' existing font
    u8g2_SetFont(m_u8g2, stashedFont);
}
void Box::UpdateValue(int32_t val) {
    m_value.i = val;
    Render();
}
void Box::UpdateValue(uint32_t val) {
    m_value.u = val;
    Render();
}
void Box::UpdateValue(float val) {
    m_value.f = val;
    Render();
}
