#ifndef MEDIAN_TYPED_H
#define MEDIAN_TYPED_H

#include <esp_err.h>
#include <stdint.h>

// Integer-based median filter using 32-bit samples
typedef struct {
  int window_size; // must be odd
  int idx;         // circular buffer index
  int count;       // num of elements in the array
  int32_t *buffer; // raw int32_t samples
  int *sorted_idx; // sorted indices into buffer
} median_filter_int_t;

// Float-based median filter
typedef struct {
  int window_size; // must be odd
  int idx;         // circular buffer index
  int count;       // num of elements in the array
  float *buffer;   // raw float samples
  int *sorted_idx; // sorted indices into buffer
} median_filter_float_t;

/**
 * Initialize integer median filter
 * @param f          pointer to filter struct
 * @param buffer     pointer to int32_t array of size 'size'
 * @param sorted_idx pointer to int array of size 'size'
 * @param size       window size (must be odd)
 * @return ESP_OK on success
 */
esp_err_t median_init_int(median_filter_int_t *f, int32_t *buffer,
                          int *sorted_idx, int size);

/**
 * Initialize float median filter
 * @param f          pointer to filter struct
 * @param buffer     pointer to float array of size 'size'
 * @param sorted_idx pointer to int array of size 'size'
 * @param size       window size (must be odd)
 * @return ESP_OK on success
 */
esp_err_t median_init_float(median_filter_float_t *f, float *buffer,
                            int *sorted_idx, int size);

/**
 * Run integer median filter on new_sample
 * @param f          pointer to filter struct
 * @param new_sample new sample to insert
 * @return median of window (int32)
 */
int32_t median_filter_int(median_filter_int_t *f, int32_t new_sample);

/**
 * Run float median filter on new_sample
 * @param f          pointer to filter struct
 * @param new_sample new sample to insert
 * @return median of window (float)
 */
float median_filter_float(median_filter_float_t *f, float new_sample);


#endif // MEDIAN_TYPED_H