#include "utils/kalman_filter.h"

#include <esp_err.h>

esp_err_t kalman1d_init(kalman1d_t *kf, float init_pressure,
                        float process_noise, float measurement_noise) {
  kf->x = init_pressure;
  kf->P = 1.0f;
  kf->Q = process_noise;
  kf->R = measurement_noise;
  return ESP_OK;
}

float kalman1d_update(kalman1d_t *kf, float z) {
  // Predict step: increase uncertainty.
  kf->P += kf->Q;

  // Compute Kalman gain.
  float K = kf->P / (kf->P + kf->R);

  // Update estimate with measurement residual.
  kf->x += K * (z - kf->x);

  // Update error covariance.
  kf->P *= (1.0f - K);

  return kf->x;
}