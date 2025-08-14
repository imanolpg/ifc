#include "median.h"
#include <math.h>

esp_err_t median_init_int(median_filter_int_t *f, int32_t *buffer,
                          int *sorted_idx, int size) {
  f->window_size = size;
  f->idx = 0;
  f->count = 0;
  f->buffer = buffer;
  f->sorted_idx = sorted_idx;
  for (int i = 0; i < size; ++i) {
    buffer[i] = 0;
    sorted_idx[i] = i;
  }
  return ESP_OK;
}

esp_err_t median_init_float(median_filter_float_t *f, float *buffer,
                            int *sorted_idx, int size) {
  f->window_size = size;
  f->idx = 0;
  f->count = 0;
  f->buffer = buffer;
  f->sorted_idx = sorted_idx;
  for (int i = 0; i < size; ++i) {
    buffer[i] = 0.0f;
    sorted_idx[i] = i;
  }
  return ESP_OK;
}

int32_t median_filter_int(median_filter_int_t *f, int32_t new_sample) {
  int N = f->window_size;
  int old = f->idx;
  f->idx = (f->idx + 1) % N;
  f->buffer[old] = new_sample;

  if (f->count < N) {
    f->count++;
    return new_sample;
  }

  int *idx = f->sorted_idx;
  for (int gap = N / 2; gap > 0; gap /= 2) {
    for (int i = gap; i < N; ++i) {
      int tmp = idx[i];
      int j = i;
      while (j >= gap && f->buffer[idx[j - gap]] > f->buffer[tmp]) {
        idx[j] = idx[j - gap];
        j -= gap;
      }
      idx[j] = tmp;
    }
  }
  return f->buffer[idx[N / 2]];
}

float median_filter_float(median_filter_float_t *f, float new_sample) {
  int N = f->window_size;
  int old = f->idx;
  f->idx = (f->idx + 1) % N;
  f->buffer[old] = new_sample;

  if (f->count < N) {
    f->count++;
    return new_sample;
  }

  int *idx = f->sorted_idx;
  for (int gap = N / 2; gap > 0; gap /= 2) {
    for (int i = gap; i < N; ++i) {
      int tmp = idx[i];
      int j = i;
      while (j >= gap && f->buffer[idx[j - gap]] > f->buffer[tmp]) {
        idx[j] = idx[j - gap];
        j -= gap;
      }
      idx[j] = tmp;
    }
  }
  return f->buffer[idx[N / 2]];
}