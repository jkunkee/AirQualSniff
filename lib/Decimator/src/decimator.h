
//
// Decimator
//
// The idea is to have a place to stuff data and have the underlying stats and storage managed neatly.
//

#pragma once

class FIFO {
public:
    FIFO(int depth);
    FIFO(float* buf, int depth);
    ~FIFO();

    float mean_val();
    float max_val();
    float min_val();

    bool is_empty() { return m_size == 0; }
    bool is_full() { return m_size >= m_storage_size; }
    int size() { return m_size; }
    int max_size() { return m_storage_size; }

    bool push(float in);
    bool pop(float* out);
    bool peek(int idx, float* out);
    void clear();
    bool discretize(int num_buckets, int *result, int result_size);

    bool peek_raw(int idx, float* out);

private:
    bool m_buffer_allocated_by_constructor;
    float *m_storage;
    int m_storage_size;
    int m_head_idx; // index into storage for next insertion
    int m_tail_idx; // index into storage for next removal
    int m_size;
};

class Decimator {
public:
    Decimator(int fine_tier_count, int mid_tier_count, int coarse_tier_count);
    Decimator(float* fine_tier_buf, int fine_tier_count,
              float* mid_tier_buf, int mid_tier_count,
              float* coarse_tier_buf, int coarse_tier_count);
    ~Decimator();

    bool push(float in);
    bool is_full();
    bool decimate_and_clear(float *out);
    void clear();
    int capacity();

    FIFO fine;
    FIFO mid;
    FIFO coarse;

private:
    bool m_decimation_complete;
    float m_decimated_value;
};
