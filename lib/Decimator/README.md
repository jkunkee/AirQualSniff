
# Decimator

This library provides the Decimator class, which wraps up the processes of pushing data to a FIFO and sucessively pushing averages of that FIFO into two coarser FIFOs. It also includes a simple u8g2-based sparkline renderer for the included FIFO class in sparkline.h/.cpp.

See `examples/basic/basic.ino` for a test rig for an Arduino Mega with an SSD1306. This can also be found [on Wokwi](https://wokwi.com/projects/355516643308376065).

1. Include `decimator.h`
2. Instantiate `Decimator` with the three desired FIFO sizes
3. Call `Decimator::push` with new data
4. Call `Decimator::*` to view, interpret, or clear the data
4. (optional) Instantiate U8G2 and call `RenderSparkline` with `Decimator::fine`, `med`, or `coarse` to visualize the data using a sparkline
