#ifndef MPU6050_C_H
#define MPU6050_C_H

#include "mpu6050.h"

#define TAG_MPU6050 "MPU6050"

// Set the desired acceleration range.
#define ACCELERATION_RANGE_CONFIG_MP6050 MPU6050_ACCEL_RANGE_16

/**
 * @brief Initialize MPU6050 descriptor for reading acceleration.
 *
 * @param dev pointer where the descriptor will be stored.
 * @return esp_err_t `ESP_OK` if success.
 */
esp_err_t mpu6050_c_init(mpu6050_dev_t *dev);

/**
 * @brief Read data from MPU6050 and store linear acceleration (m/s^2) and
 * angular acceleration (º/s^2).
 *
 * @param dev pointer to the MPU6050 device descriptor.
 * @param accel pointer where the linear acceleration readings will be stored
 * (m/s^2).
 * @param rot pointer where the angular acceleration readings will be stored
 * (º/s^2).
 */
void mpu6050_c_read(mpu6050_dev_t *dev, mpu6050_acceleration_t *accel,
                    mpu6050_rotation_t *rot);


#endif // MPU6050_C_H