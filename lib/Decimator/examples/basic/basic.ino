
#include <Wire.h>
#include <U8g2lib.h>

#include "decimator.h"
#include "sparkline.h"

// https://docs.wokwi.com/parts/board-ssd1306
// https://github.com/olikraus/u8g2/wiki/u8g2reference
// https://docs.wokwi.com/guides/diagram-editor

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u(U8G2_R0);

#define DEC_DEPTH 15

Decimator d(DEC_DEPTH, DEC_DEPTH, DEC_DEPTH);

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  u.begin();
  u.clear();
  Serial.begin(115200);
}

#define DEC_ROW 20

void loop() {
  // clear screen
  //u.clear();

  // pull data
  //int datum = random(DEC_ROW-1);
  static int datum = 0;
  datum = (datum + 1) % DEC_ROW;
  //int datum = DEC_ROW-1;

  d.push(datum);

  // render decimator state
  u.setDrawColor(0);
  u.drawBox(0, 0, 6*(DEC_DEPTH+2)+DEC_DEPTH, DEC_ROW+1+3);
  u.setDrawColor(1);
  u.drawLine(0*(DEC_DEPTH+2), DEC_ROW+1, 0*(DEC_DEPTH+2)+DEC_DEPTH-1, DEC_ROW+1);
  u.drawLine(1*(DEC_DEPTH+2), DEC_ROW+1, 1*(DEC_DEPTH+2)+DEC_DEPTH-1, DEC_ROW+1);
  u.drawLine(2*(DEC_DEPTH+2), DEC_ROW+1, 2*(DEC_DEPTH+2)+DEC_DEPTH-1, DEC_ROW+1);
  for (int idx = 0; idx < DEC_DEPTH; idx++) {
    float pt;
    d.fine.peek_raw(idx, &pt);
    u.drawPixel(0*(DEC_DEPTH+2)+idx, DEC_ROW-pt);
    d.mid.peek_raw(idx, &pt);
    u.drawPixel(1*(DEC_DEPTH+2)+idx, DEC_ROW-pt);
    d.coarse.peek_raw(idx, &pt);
    u.drawPixel(2*(DEC_DEPTH+2)+idx, DEC_ROW-pt);
  }
  u.drawPixel(0*(DEC_DEPTH+2)+d.fine.m_head_idx, DEC_ROW+2);
  u.drawPixel(1*(DEC_DEPTH+2)+d.mid.m_head_idx, DEC_ROW+2);
  u.drawPixel(2*(DEC_DEPTH+2)+d.coarse.m_head_idx, DEC_ROW+2);
  u.drawPixel(0*(DEC_DEPTH+2)+d.fine.m_tail_idx, DEC_ROW+3);
  u.drawPixel(1*(DEC_DEPTH+2)+d.mid.m_tail_idx, DEC_ROW+3);
  u.drawPixel(2*(DEC_DEPTH+2)+d.coarse.m_tail_idx, DEC_ROW+3);

  for (int idx = 0; idx < DEC_DEPTH; idx++) {
    float pt;
    d.fine.peek(idx, &pt);
    u.drawPixel(4*(DEC_DEPTH+2)+idx, DEC_ROW-pt);
    d.mid.peek(idx, &pt);
    u.drawPixel(5*(DEC_DEPTH+2)+idx, DEC_ROW-pt);
    d.coarse.peek(idx, &pt);
    u.drawPixel(6*(DEC_DEPTH+2)+idx, DEC_ROW-pt);
  }

  //if (d.is_full()) { float f; d.coarse.pop(&f); }

  // render sparkline
  constexpr uint8_t SPARK_ROW = DEC_ROW + 10;
  constexpr uint8_t SPARK_W = DEC_DEPTH - 5;
  constexpr uint8_t SPARK_H = 5;
  FIFO fifo(20);
  fifo.push(1.0);
  fifo.push(2.0);
  fifo.push(3.0);
  fifo.push(4.0);
  fifo.push(5.0);
  //u.setDrawColor(0);
  u.drawFrame(0, SPARK_ROW, SPARK_W+2, SPARK_H+2);
  u.setDrawColor(1);
  //RenderSparkline(u.getU8g2(), fifo, 1, SPARK_ROW+1, SPARK_W, SPARK_H);
  RenderSparkline(u.getU8g2(), d.fine, 1, SPARK_ROW+1, SPARK_W, SPARK_H, true);
  RenderSparkline(u.getU8g2(), d.fine, 1+SPARK_W+2, SPARK_ROW+1, SPARK_W, SPARK_H, false);

  u.updateDisplay();

  delay(100);
}
