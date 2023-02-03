
#include "decimator.h"

#include <Arduino.h>

FIFO::FIFO(int depth) {
    m_buffer_allocated_by_constructor = true;
    m_storage = (float*)malloc(depth * sizeof(float));
    m_storage_size = depth;
    m_head_idx = 0; // index into storage for next insertion
    m_tail_idx = 0; // index into storage for next removal
    m_size = 0;
    m_bookmark = 0;
}

FIFO::FIFO(float* buf, int depth) {
    m_buffer_allocated_by_constructor = false;
    m_storage = buf;
    m_storage_size = depth;
    m_head_idx = 0; // index into storage for next insertion
    m_tail_idx = 0; // index into storage for next removal
    m_size = 0;
    m_bookmark = 0;
}

FIFO::~FIFO() {
    if (m_buffer_allocated_by_constructor) {
        free(m_storage);
    }
}

float FIFO::mean_val() {
    if (is_empty()) {
        return 0;
    }
    float sum = 0.0;
    for (int idx = 0; idx < size(); idx++) {
        float elem;
        peek(idx, &elem);
        sum += elem;
    }
    return sum / size();
}

float FIFO::max_val() {
    if (is_empty()) {
        return 0;
    }
    float max;
    peek(0, &max);
    for (int idx = 0; idx < size(); idx++) {
        float elem;
        peek(idx, &elem);
        if (elem > max) {
            max = elem;
        }
    }
    return max;
}

float FIFO::min_val() {
    if (is_empty()) {
        return 0;
    }
    float min;
    peek(0, &min);
    for (int idx = 0; idx < size(); idx++) {
        float elem;
        peek(idx, &elem);
        if (elem < min) {
            min = elem;
        }
    }
    return min;
}

bool FIFO::push(float in) {
    if (is_full()) {
        return false;
    }
    m_storage[m_head_idx] = in;
    m_head_idx = (m_head_idx + 1) % m_storage_size;
    m_size++;
    return true;
}

bool FIFO::pop(float* out) {
    if (out == NULL || is_empty()) {
        return false;
    }
    *out = m_storage[m_tail_idx];
    m_tail_idx = (m_tail_idx + 1) % m_storage_size;
    m_size--;
    return true;
}

bool FIFO::peek(int idx, float* out) {
    if (idx < 0 || out == NULL || idx >= m_storage_size) {
        return false;
    }
    *out = m_storage[(m_tail_idx + idx) % m_storage_size];
    return true;
}

bool FIFO::peek_raw(int idx, float* out) {
    if (idx < 0 || out == NULL || idx >= m_storage_size) {
        return false;
    }
    *out = m_storage[idx];
    return true;
} 

void FIFO::clear() {
    m_tail_idx = m_head_idx;
    m_size = 0;
}

bool FIFO::discretize(int num_buckets, int *results, int results_size) {
    if (size() == 0 || results_size < size() || num_buckets <= 0) {
        return false;
    }

    float local_min = min_val();
    float local_max = max_val();
    float step = (local_max - local_min) / num_buckets;

    bool success = true;
    for (int idx = 0; idx < size(); idx++) {
        float val;
        success = peek(idx, &val);
        if (!success) {
            break;
        }
        results[idx] = 0;

        for (int bucket_idx = 0; bucket_idx < num_buckets; bucket_idx++) {
            if (val < local_min + (bucket_idx + 1) * step) {
                results[idx] = bucket_idx;
                break;
            }
        }
    }
    return success;
}

bool FIFO::is_at_bookmark() {
    return m_head_idx == m_bookmark;
}

void FIFO::set_bookmark(int m) {
    m_bookmark = m % m_storage_size;
}

Decimator::Decimator(int fine_tier_count, int mid_tier_count, int coarse_tier_count) :
    fine(fine_tier_count), mid(mid_tier_count), coarse(coarse_tier_count),
    m_decimation_complete(false), m_decimated_value(0.0f) {}

Decimator::Decimator(float* fine_tier_buf, int fine_tier_count,
                     float* mid_tier_buf, int mid_tier_count,
                     float* coarse_tier_buf, int coarse_tier_count) :
                     fine(fine_tier_buf, fine_tier_count),
                     mid(mid_tier_buf, mid_tier_count),
                     coarse(coarse_tier_buf, coarse_tier_count),
                     m_decimation_complete(false), m_decimated_value(0.0f) {}

Decimator::~Decimator() {}

bool Decimator::push(float in) {
    float f;
    if (fine.is_full()) {
        fine.pop(&f);
    }
    fine.push(in);
    // rotate up
    if (fine.is_full() && fine.is_at_bookmark()) {
        if (mid.is_full()) {
            mid.pop(&f);
        }
        mid.push(fine.mean_val());
        if (mid.is_full() && mid.is_at_bookmark()) {
            if (coarse.is_full()) {
                coarse.pop(&f);
            }
            coarse.push(mid.mean_val());
            if (coarse.is_full() && coarse.is_at_bookmark()) {
                m_decimated_value = coarse.mean_val();
                m_decimation_complete = true;
            }
        }
    }
    return true;
}

bool Decimator::is_full() {
    return m_decimation_complete;
}

bool Decimator::decimate_and_clear(float *out) {
    if (out == NULL || !is_full()) {
        return false;
    }
    *out = m_decimated_value;
    clear();
    return true;
}

void Decimator::clear() {
    m_decimation_complete = false;
    fine.clear();
    mid.clear();
    coarse.clear();
}

int Decimator::capacity() {
    // return the number of samples represented by the output of decimate_and_clear
    return fine.max_size() * mid.max_size() * coarse.max_size();
}
