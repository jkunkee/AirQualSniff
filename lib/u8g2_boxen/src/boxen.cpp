
#include "boxen.h"

Box::Box(u8g2_t *u8g2, u8g2_int_t x, u8g2_int_t y, u8g2_int_t width, u8g2_int_t height, const uint8_t *labelFont, const char *labelTop, const char *labelBottom, const uint8_t *valueFont, uint8_t digits)
:
m_u8g2(u8g2), m_x(x), m_y(y), m_width(width), m_height(height), m_labelFont(labelFont), m_labelTop(labelTop), m_labelBottom(labelBottom), m_valueFont(valueFont), m_digits(digits)
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

void Box::Render(BoxValueType boxType) {
    constexpr size_t bufLen = 20;
    char buf[bufLen];
    switch (boxType) {
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
    case BoxTypeIntUnsigned:
        snprintf(buf, bufLen, "%lu", m_value.u);
        break;
    default:
    case BoxTypeUnset:
        snprintf(buf, bufLen, "?");
        break;
    }

    // 'push' existing font
    FontData *stashedFont = m_u8g2->font;

    // Don't draw outside the specified box
    u8g2_SetClipWindow(m_u8g2, m_x, m_y, m_x + m_width, m_y + m_height);

    // Don't overwrite what's underneath
    u8g2_SetFontMode(m_u8g2, 1);

    // Measure up the data field
    setFontWrapper(m_u8g2, m_valueFont);
    u8g2_int_t numberBaseline;
    if (m_height - 2*2 < u8g2_GetFontAscent(m_u8g2)) {
        numberBaseline = m_y + m_height - 2;
    } else {
        numberBaseline = m_y + 2 + (m_height - 2*2 - u8g2_GetFontAscent(m_u8g2)) / 2 + u8g2_GetFontAscent(m_u8g2);
    }
    u8g2_int_t numberWidth = u8g2_GetStrWidth(m_u8g2, buf);

    // Measure up the label field(s)
    setFontWrapper(m_u8g2, m_labelFont);
    u8g2_int_t labelWidth = max(u8g2_GetStrWidth(m_u8g2, m_labelTop), u8g2_GetStrWidth(m_u8g2, m_labelBottom));

    // Draw the label field(s)
    if (strlen(m_labelBottom) == 0) {
        u8g2_SetFontPosBaseline(m_u8g2);
        u8g2_DrawUTF8(m_u8g2, m_x + m_width - 2 - labelWidth, numberBaseline, m_labelTop);
    } else {
        u8g2_SetFontPosBottom(m_u8g2);
        u8g2_DrawUTF8(m_u8g2, m_x + m_width - 2 - labelWidth, m_y + m_height / 2, m_labelTop);
        u8g2_SetFontPosTop(m_u8g2);
        u8g2_DrawUTF8(m_u8g2, m_x + m_width - 2 - labelWidth, m_y + m_height / 2, m_labelBottom);
    }

    // Draw the data field
    setFontWrapper(m_u8g2, m_valueFont);
    u8g2_SetFontPosBaseline(m_u8g2);
    u8g2_DrawUTF8(m_u8g2, m_x + m_width - 2 - labelWidth - 1 - numberWidth, numberBaseline, buf);

    // Draw a cute little box around everything
    u8g2_DrawFrame(m_u8g2, m_x, m_y, m_width, m_height);

    // reset clip window
    u8g2_SetMaxClipWindow(m_u8g2);

    // 'pop' existing font
    u8g2_SetFont(m_u8g2, stashedFont);
}
void Box::UpdateValue(int32_t val) {
    m_value.i = val;
    Render(BoxTypeIntSigned);
}
void Box::UpdateValue(uint32_t val) {
    m_value.u = val;
    Render(BoxTypeIntUnsigned);
}
void Box::UpdateValue(float val) {
    m_value.f = val;
    Render(BoxTypeFloat);
}
void Box::UpdateValue() {
    Render(BoxTypeUnset);
}
