
#include "sparkline.h"

void RenderSparkline(u8g2_t *u, FIFO &f, uint8_t x, uint8_t y, uint8_t w, uint8_t h) {
    bool success;
    int pixel_idx = 0;

    // Clear graph area
    uint8_t curColor = u8g2_GetDrawColor(u);
    u8g2_SetDrawColor(u, 0);
    u8g2_DrawBox(u, x, y, w, h);
    u8g2_SetDrawColor(u, curColor);

    // h is the number of buckets to discretize the data values into
    uint8_t value_bins = h;
    int *binned_values = (int*)malloc(sizeof(int) * f.size());
    if (binned_values == NULL) {
        // OOM
        goto cleanup;
    }
    success = f.discretize(value_bins, binned_values, f.size());
    if (!success) {
        // FIFO is empty or binned_values was too small
        goto cleanup;
    }
    // Instead of scaling horizontally, truncate first samples
    for (int data_idx = max(0, f.size() - w); data_idx < f.size(); data_idx++) {
        u8g2_DrawPixel(u, x + pixel_idx++, y+h-1-binned_values[data_idx]);
    }

cleanup:
    if (binned_values != NULL) {
        free(binned_values);
    }
}
