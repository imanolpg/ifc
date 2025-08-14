#include "sensors_reading/ms5611/ms5611_c.h"
#include "median.h"
#include "ms5611.h"
#include "sensors_reading/sensors_reading.h"
#include "utils/kalman_filter.h"

#include <esp_log.h>
#include <math.h>

static median_filter_int_t ms5611_prs_filter;
static int32_t ms5611_prs_raw_buf[MEDIAN_SIZE_PRS];
static int ms5611_prs_sorted_idx[MEDIAN_SIZE_PRS];

static median_filter_float_t ms5611_tmp_filter;
static float ms5611_tmp_raw_buf[MEDIAN_SIZE_TMP];
static int ms5611_tmp_sorted_idx[MEDIAN_SIZE_TMP];

static kalman1d_t kf_pressure;
static kalman1d_t kf_temperature;

esp_err_t ms5611_c_init(ms5611_t *dev) {
  ESP_ERROR_CHECK(ms5611_init_desc(dev, MS5611_ADDR_CSB_HIGH, I2C_NUM,
                                   I2C_SDA_IO, I2C_SCL_IO));
  ESP_ERROR_CHECK(ms5611_init(dev, MS5611_OSR));

  ESP_ERROR_CHECK(median_init_int(&ms5611_prs_filter, ms5611_prs_raw_buf,
                                  ms5611_prs_sorted_idx, MEDIAN_SIZE_PRS));
  ESP_ERROR_CHECK(median_init_float(&ms5611_tmp_filter, ms5611_tmp_raw_buf,
                                    ms5611_tmp_sorted_idx, MEDIAN_SIZE_TMP));

  ESP_ERROR_CHECK(kalman1d_init(&kf_pressure, 0.00f, 0.001f, 0.003f));
  ESP_ERROR_CHECK(kalman1d_init(&kf_temperature, 0.00f, 0.001f, 0.003f));

  return ESP_OK;
}

void ms5611_read_pressure_temperature_and_altitude(ms5611_t *dev,
                                                   int32_t *pressure,
                                                   float *temperature,
                                                   float *altitude) {


  if (ms5611_get_sensor_data(dev, pressure, temperature) != ESP_OK)
    return;

  // Pass through a median filter.
  *pressure = median_filter_int(&ms5611_prs_filter, *pressure);
  *temperature = median_filter_float(&ms5611_tmp_filter, *temperature);

  // Pass through Kalman filter.
  *pressure = kalman1d_update(&kf_pressure, *pressure);
  *temperature = kalman1d_update(&kf_temperature, *temperature);

  // Calculate altitude from pressure.
  *altitude = 44330.0f * (1.0f - powf(*pressure / 101325.0f, 0.19029495f));

  ESP_LOGI(TAG_MS5611, "Altitude: %.2f meters", *altitude);
  ESP_LOGI(TAG_MS5611, "Pressure: %ld Pa", *pressure);
  ESP_LOGI(TAG_MS5611, "Temperature: %.2f °C", *temperature);
}
