#include "sensors_reading/mpu6050/mpu6050_c.h"
#include "mpu6050.h"
#include "sensors_reading/sensors_reading.h"

#include "freertos/task.h"
#include <esp_log.h>

esp_err_t mpu6050_c_init(mpu6050_dev_t *dev) {
  ESP_ERROR_CHECK(
      mpu6050_init_desc(dev,
                        MPU6050_I2C_ADDRESS_LOW, // or MPU6050_I2C_ADDRESS_HIGH
                        I2C_NUM,                 // I2C port
                        I2C_SDA_IO, I2C_SCL_IO));

  ESP_ERROR_CHECK(mpu6050_init(dev));

  // Configure reading range up to 16g.
  ESP_ERROR_CHECK(
      mpu6050_set_full_scale_accel_range(dev, MPU6050_ACCEL_RANGE_16));

  vTaskDelay(pdMS_TO_TICKS(10));

  // Check for the accel range.
  if (dev->ranges.accel != MPU6050_ACCEL_RANGE_16) {
    ESP_LOGE(TAG_MPU6050,
             "Accel range not set to desired +-16g. Currently set to %ig",
             dev->ranges.accel);
  } else {
    ESP_LOGI(TAG_MPU6050, "Accel range set to +-16g");
  }

  return ESP_OK;
}

void mpu6050_c_read(mpu6050_dev_t *dev, mpu6050_acceleration_t *accel,
                    mpu6050_rotation_t *rot) {
  if (mpu6050_get_motion(dev, accel, rot) != ESP_OK)
    return;

  ESP_LOGI(TAG_MPU6050, "Accel: x=%.4f y=%.4f z=%.4f", accel->x, accel->y,
           accel->z);
  ESP_LOGI(TAG_MPU6050, "Gyro:  x=%.4f y=%.4f z=%.4f", rot->x, rot->y, rot->z);
}