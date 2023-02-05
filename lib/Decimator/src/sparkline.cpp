
#include "sparkline.h"

void RenderSparkline(u8g2_t *u, FIFO &f, uint8_t x, uint8_t y, uint8_t w, uint8_t h, bool squish_instead_of_truncate) {
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
    if (squish_instead_of_truncate && w < f.max_size()) {
        // Divide the samples into w ranges
        for (int chunk_no = 0; chunk_no < w; chunk_no++) {
            // This style of interval calculation is better than using
            // max_size()/w because it actually handles coprime sizes.
            int prev_postindex = (chunk_no * f.max_size()) / w;
            int cur_postindex = ((chunk_no+1) * f.max_size()) / w;
            // Select a value to represent the range
            int bin_no = 0;
            // mean bin number makes discontinuities look almost continuous
            // simple downsample - take the first, middle, last, etc. - is jumpy
            // as textured data flows through
            // max of interval behaves fairly well; I like it
            int data_idx;
            for (data_idx = prev_postindex; data_idx < cur_postindex; data_idx++) {
                bin_no = max(bin_no, binned_values[data_idx]);
            }
            // don't render points with no backing data
            if (data_idx <= f.size()) {
                u8g2_DrawPixel(u, x+chunk_no, y+h-1-bin_no);
            }
        }
    } else {
        // Instead of scaling horizontally, truncate first samples
        for (int data_idx = max(0, f.size() - w); data_idx < f.size(); data_idx++) {
            u8g2_DrawPixel(u, x + pixel_idx++, y+h-1-binned_values[data_idx]);
        }
    }

cleanup:
    if (binned_values != NULL) {
        free(binned_values);
    }
}
