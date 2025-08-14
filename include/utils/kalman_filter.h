#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <esp_err.h>

/**
 * @brief One-dimensional Kalman filter state.
 *
 * x: Estimated value (state) after the last update.
 * P: Estimated uncertainty (error covariance) of the state.
 * Q: Process noise covariance (uncertainty introduced by the system dynamics).
 * R: Measurement noise covariance (uncertainty in the sensor measurements).
 */
typedef struct {
  float x; ///< State estimate (e.g., pressure).
  float P; ///< Estimate uncertainty (error covariance).
  float Q; ///< Process noise covariance.
  float R; ///< Measurement noise covariance.
} kalman1d_t;

/**
 * @brief Initialize a 1D Kalman filter instance.
 *
 * Sets the initial state and noise parameters for the filter.
 *
 * Larger Q values let the filter adapt faster to changes.
 * Larger R values make the filter trust measurements less.
 *
 * @param kf               Pointer to the filter instance to initialize.
 * @param init_pressure    Initial estimate for the state.
 * @param process_noise    Process noise covariance (Q).
 * @param measurement_noise Measurement noise covariance (R).
 * @return esp_err_t        `ESP_OK` on success.
 */
esp_err_t kalman1d_init(kalman1d_t *kf, float init_pressure,
                        float process_noise, float measurement_noise);

/**
 * @brief Perform a single Kalman filter update with a new measurement.
 *
 * Applies the predict and update steps of the Kalman filter:
 * 1. Predict the new state estimate and error covariance.
 * 2. Compute the Kalman gain.
 * 3. Update the state estimate and error covariance with the incoming
 * measurement.
 *
 * @param kf  Pointer to the filter instance.
 * @param z   New measurement.
 * @return float  Updated state estimate.
 */
float kalman1d_update(kalman1d_t *kf, float z);

#endif // KALMAN_FILTER_H