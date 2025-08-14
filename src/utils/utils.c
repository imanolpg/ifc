#include "utils/utils.h"
#include "sensors_reading/mpu6050/mpu6050_c.h"
#include "sensors_reading/sensors_reading.h"

#include <esp_err.h>
#include <esp_log.h>
#include <unistd.h>

float calculate_vertical_speed(float current_altitude, float last_altitude,
                               int64_t current_ms, int64_t last_ms) {
  return (current_altitude - last_altitude) /
         ((float)(current_ms - last_ms) / 1000.0);
}

esp_err_t format_log_line(char *line_buf, size_t buff_size, int64_t ms,
                          ROCKET_STATUS_T rocket_status, int32_t pressure,
                          float altitude, float vertical_speed,
                          float temperature, mpu6050_acceleration_t acc,
                          mpu6050_rotation_t rot) {

  int len = snprintf(line_buf, buff_size,
                     "%lld, %i, %5.3ld, %6.3f, %.3f, %.2f, %.3f, %.3f, %.3f, "
                     "%.3f, %.3f, %.3f",
                     ms, rocket_status, pressure, altitude, vertical_speed,
                     temperature, acc.x, acc.y, acc.z, rot.x, rot.y, rot.z);

  if (len > 0)
    return ESP_OK;
  else
    ESP_LOGE(TAG_SENSORS_READING, "Failed to format log line");

  return -1;
}
