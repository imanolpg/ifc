#include "sensors_reading/sensors_reading.h"
#include "ble/ble_c.h"
#include "sd/sd_c.h"
#include "sensors_reading/mpu6050/mpu6050_c.h"
#include "sensors_reading/ms5611/ms5611_c.h"
#include "utils/kalman_filter.h"
#include "utils/utils.h"

#include "freertos/queue.h" // For xQueueSend
#include <esp_log.h>        // For ESP_LOGX
#include <esp_timer.h>      // For esp_timer_get_time
#include <inttypes.h>       // For int64_t


// Sensors objects.
ms5611_t ms5611;
mpu6050_dev_t mpu6050 = {0};

// File where the data will be stored. It is inside the SD card.
FILE *logFile = NULL;

// Measured or calculated flight data.
ROCKET_STATUS_T rocket_status = PREPARING_FOR_FLIGHT;
mpu6050_acceleration_t rocket_accel = {0};
mpu6050_rotation_t rocket_rot = {0};
int32_t rocket_pressure;
float rocket_temperature, rocket_altitude, rocket_speed;

// Queues for sending data over tasks and auxiliary buffers.
extern QueueHandle_t xSensorQueue;
extern QueueHandle_t xUpdatedStatusFromFlightController;
ROCKET_STATUS_T buff_rocket_status; // Buffer to store rocket status;

void sensors_reading() {
  // Variable to calculate speed.
  float previous_rocket_altitude = 0.0;


  int64_t ms;          // Milliseconds count since the program start.
  int64_t previous_ms; // Milliseconds in the previous loop of the program. Used
                       // to calculate speed of the rocket.


  // Initialize SD.
  if (ACTIVATE_SD) {
    sd_c_init(&logFile);
  }

  // Install I2C drivers if the protocol is needed.
  if (ACTIVATE_MPU6050 || ACTIVATE_MS5611) {
    ESP_ERROR_CHECK(i2cdev_init());
  }

  // Initialize MPU6050.
  if (ACTIVATE_MPU6050) {
    mpu6050_c_init(&mpu6050);
  }

  // Initialize MS5611.
  if (ACTIVATE_MS5611) {
    ms5611_c_init(&ms5611);
    ms5611_read_pressure_temperature_and_altitude(
        &ms5611, &rocket_pressure, &rocket_temperature, &rocket_altitude);
  }

  // Initialize BLE.
  if (ACTIVATE_BLE) {
    ESP_ERROR_CHECK(ble_init());
  }

  // Do the first check to initialize milliseconds.
  previous_ms = esp_timer_get_time() / 1000;

  // Control variable to transmit data over BLE lower than 2 Hz (2 times per
  // second).
  // The tasks is running faster than 2 times per seconds and the BLE
  // connection can not handle that. For this reason data over BLE will be send
  // 2 times per second or slower. This variable helps calculating the last
  // time the data was sent.
  int64_t last_ms_of_BLE_transmission = 0;


  // Infinite loop with the main logic.
  while (1) {

    // Check to see if the flight controller task is reporting a new Rocket
    // Status value.
    if (xQueueReceive(xUpdatedStatusFromFlightController, &buff_rocket_status,
                      0) == pdPASS) {
      // The flight controller is changing the status of the flight status
      // variable. It needs to be stored in order to save it into the CSV file.
      rocket_status = buff_rocket_status;
    }

    // Calculate the milliseconds elapsed since the program started.
    ms = esp_timer_get_time() / 1000;

    // Read MPU6050 data (linear acceleration and angular rotation).
    if (ACTIVATE_MPU6050) {
      mpu6050_c_read(&mpu6050, &rocket_accel, &rocket_rot);
    }

    // Read MS5611 data (pressure and temperature). Also estimate the altitude
    // based on the pressure and vertical speed.
    if (ACTIVATE_MS5611) {
      // Read data.
      ms5611_read_pressure_temperature_and_altitude(
          &ms5611, &rocket_pressure, &rocket_temperature, &rocket_altitude);

      // Calculate rocket vertical speed.
      rocket_speed = calculate_vertical_speed(
          rocket_altitude, previous_rocket_altitude, ms, previous_ms);

      previous_ms = ms;
      previous_rocket_altitude = rocket_altitude;
    }

    // Log into the SD card.
    if (ACTIVATE_SD) {
      // Buffer to store the string that will be written into the SD card.
      char line[256];
      // Format the string that will be printed into the CSV file.
      esp_err_t format_log_line_error =
          format_log_line(line, sizeof(line), ms, rocket_status,
                          rocket_pressure, rocket_altitude, rocket_speed,
                          rocket_temperature, rocket_accel, rocket_rot);

      // Print the line into the CSV file if everything has been successful.
      if (!format_log_line_error)
        write_to_file_c(logFile, line);
    }

    // Put new data into the queue to send it to the flight controller task.
    SensorDataQueue_t txData;
    txData.rocket_status = rocket_status;
    txData.rocket_altitude = rocket_altitude;
    txData.rocket_acc = rocket_accel;
    xQueueOverwrite(xSensorQueue, &txData);


    // Broadcast over BLE new data.
    if (ACTIVATE_BLE) {

      // Control BLE transmission amount. In order not to saturate the BLE
      // connection, no more than 2 transmissions per second can be done.
      if (ms - last_ms_of_BLE_transmission >= 500) {

        update_rocket_status(rocket_status);

        if (ACTIVATE_MPU6050) {
          update_rocket_acc_x(rocket_accel.x);
          update_rocket_acc_y(rocket_accel.y);
          update_rocket_acc_z(rocket_accel.z);
          update_rocket_rot_x(rocket_rot.x);
          update_rocket_rot_y(rocket_rot.y);
          update_rocket_rot_z(rocket_rot.z);
        }

        if (ACTIVATE_MS5611) {
          update_rocket_altitude(rocket_altitude);
          update_rocket_speed(rocket_speed);
          update_rocket_pressure(rocket_pressure);
          update_rocket_temperature(rocket_temperature);
        }

        last_ms_of_BLE_transmission = ms;
      }
    }

    ESP_LOGI(TAG_SENSORS_READING, "Rocket status: %i", rocket_status);

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}