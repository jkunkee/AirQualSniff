
#pragma once

#include <Arduino.h>
#include "decimator.h"
#include <U8g2lib.h>

void RenderSparkline(u8g2_t *u8g2, FIFO &dec, uint8_t x, uint8_t y, uint8_t w, uint8_t h, bool squish_instead_of_truncate, float min_h_step = 0.0f);
