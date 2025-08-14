#ifndef UTILS_H
#define UTILS_H

#include "mpu6050.h"
#include "sensors_reading/sensors_reading.h"

#include <esp_err.h>
#include <esp_log.h>
#include <unistd.h>

/**
 * @brief Calculate the vertical speed of the rocket in meters per second
 * (m/s).
 *
 * @param current_altitude  last recorded altitude in meters.
 * @param last_altitude previous recorded altitude in meters.
 * @param current_ms    current miliseconds since system boot.
 * @param last_ms   miliseconds since system boot where the last_altitude was
 * recorded.
 * @return float    vertical speed of the rocket in meters per second (m/s).
 */
float calculate_vertical_speed(float current_altitude, float last_altitude,
                               int64_t current_ms, int64_t last_ms);

/**
 * @brief Format the line that will be printed into the SD card file. This file
 * contains all the flight recorded data.
 *
 * @param line_buf  pointer to the buffer where the line will be written.
 * @param buff_size size of the buffer string in bytes.
 * @param ms    milliseconds from system bootup where the data was recorded.
 * @param rocket_status rocket status.
 * @param pressure  recorded pressures.
 * @param altitude  recorded altitude.
 * @param vertical_speed    calculated vertical speed.
 * @param temperature   recorded temperature.
 * @param acc   recorded acceleration.
 * @param rot   recorded rotation speed.
 * @return esp_err_t    `ESP_OK` on success.
 */
esp_err_t format_log_line(char *line_buf, size_t buff_size, int64_t ms,
                          ROCKET_STATUS_T rocket_status, int32_t pressure,
                          float altitude, float vertical_speed,
                          float temperature, mpu6050_acceleration_t acc,
                          mpu6050_rotation_t rot);

#endif // UTILS_H