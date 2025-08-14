#include "flight_controller/flight_controller.h"
#include "sd/sd_c.h"
#include "sensors_reading/mpu6050/mpu6050_c.h"
#include "sensors_reading/sensors_reading.h"


#include <driver/gpio.h> // for gpio_num_t
#include <esp_log.h>     // For ESP_LOGX
#include <esp_timer.h>   // For esp_timer_get_time

extern QueueHandle_t xSensorQueue;
extern QueueHandle_t xUpdatedStatusFromFlightController;

bool detect_liftoff(mpu6050_acceleration_t rocket_acc) {
  // Check if the acceleration is greater than threshold.
  if (rocket_acc.x >= ACCELERATION_THRESHOLD_FOR_LIFTOFF_DETECTION ||
      rocket_acc.x <= -ACCELERATION_THRESHOLD_FOR_LIFTOFF_DETECTION ||
      rocket_acc.y >= ACCELERATION_THRESHOLD_FOR_LIFTOFF_DETECTION ||
      rocket_acc.y <= -ACCELERATION_THRESHOLD_FOR_LIFTOFF_DETECTION ||
      rocket_acc.z >= ACCELERATION_THRESHOLD_FOR_LIFTOFF_DETECTION ||
      rocket_acc.z <= -ACCELERATION_THRESHOLD_FOR_LIFTOFF_DETECTION) {
    return true;
  }
  return false;
}

float update_rocket_altitude_mean(float rocket_altitude,
                                  float *altitude_mean_array,
                                  int *oldest_element_pointer) {

  // Remove oldest measurement and add the newest one.
  altitude_mean_array[*oldest_element_pointer] = rocket_altitude;

  // Point to the oldest element and check overflow.
  *oldest_element_pointer += 1;
  if (*oldest_element_pointer >= WINDOW_SIZE_FOR_ALTITUDE_MEAN) {
    *oldest_element_pointer = 0;
  }

  // Calculate new mean.
  float new_mean = 0.0;
  for (int x = 0; x < WINDOW_SIZE_FOR_ALTITUDE_MEAN; x++) {
    new_mean += altitude_mean_array[x];
  }

  return new_mean / WINDOW_SIZE_FOR_ALTITUDE_MEAN;
}

bool should_drogue_chute_be_deployed(float altitude_mean,
                                     float highest_altitude_mean_detected,
                                     uint64_t ms, uint64_t ms_liftoff) {
  // First check for sweet spot with timers.
  uint64_t ms_since_liftoff = ms - ms_liftoff;
  ESP_LOGI(TAG_FLIGHT_CONTROLLER, "CHECK_FOR_APOGEE: ms_since_liftoff %llu",
           ms_since_liftoff);

  if (ms_since_liftoff < MIN_MS_DROGUE_DEPLOYMENT) {
    return false;
  } else if (ms_since_liftoff >= MAX_MS_DROGUE_DEPLOYMENT) {
    return true;
  }

  // Deploy parachute when the rocket has fallen more that what is set in
  // ALTITUDE_DIFFERENCE_TO_CONSIDER_APOGEE.
  if (highest_altitude_mean_detected - altitude_mean >=
      ALTITUDE_DIFFERENCE_TO_CONSIDER_APOGEE) {
    return true;
  }
  return false;
}

bool should_main_chute_be_deployed(float mean_altitude,
                                   float launching_altitude, uint64_t ms,
                                   uint64_t ms_liftoff) {
  // First check for sweet spot with timers.
  uint64_t ms_since_liftoff = ms - ms_liftoff;

  ESP_LOGI(TAG_FLIGHT_CONTROLLER,
           "CHECK_FOR_MAIN_DEPLOYMENT: ms_since_liftoff %llu",
           ms_since_liftoff);

  if (ms_since_liftoff < MIN_MS_MAIN_DEPLOYMENT) {
    return false;
  } else if (ms_since_liftoff > MAX_MS_MAIN_DEPLOYMENT) {
    return true;
  }

  // Timers have set the sweet spot for main deployment.
  // Deploy the parachute when the rocket is under 200 meters.
  if (mean_altitude - launching_altitude <= MAIN_CHUTE_DEPLOYMENT_HEIGHT) {
    return true;
  }
  return false;
}

bool detect_touchdown(mpu6050_acceleration_t rocket_acc) {
  // Check if the acceleration is greater than threshold.
  if (rocket_acc.x >= ACCELERATION_THRESHOLD_FOR_TOUCHDOWN_DETECTION ||
      rocket_acc.x <= -ACCELERATION_THRESHOLD_FOR_TOUCHDOWN_DETECTION ||
      rocket_acc.y >= ACCELERATION_THRESHOLD_FOR_TOUCHDOWN_DETECTION ||
      rocket_acc.y <= -ACCELERATION_THRESHOLD_FOR_TOUCHDOWN_DETECTION ||
      rocket_acc.z >= ACCELERATION_THRESHOLD_FOR_TOUCHDOWN_DETECTION ||
      rocket_acc.z <= -ACCELERATION_THRESHOLD_FOR_TOUCHDOWN_DETECTION) {
    return true;
  }
  return false;
}

void flight_controller() {

  // Variables to calculate apogee.
  float mean_altitude;
  float mean_altitude_array[WINDOW_SIZE_FOR_ALTITUDE_MEAN] = {0.0};
  float highest_altitude_mean_detected = 0.0;
  int mean_oldest_pointer = 0;

  // Variable to register when liftoff occurred.
  uint64_t ms_liftoff = 0.0;

  // Variable to register the initial launching altitude.
  float launching_altitude = 0.0;

  // Configure pins for the chutes.
  const gpio_num_t drogue_chute_pin = GPIO_NUM_16;
  const gpio_num_t main_chute_pin = GPIO_NUM_17;

  // Set the pins to be output pins.
  gpio_reset_pin(drogue_chute_pin);
  gpio_set_direction(drogue_chute_pin, GPIO_MODE_OUTPUT);

  gpio_reset_pin(main_chute_pin);
  gpio_set_direction(main_chute_pin, GPIO_MODE_OUTPUT);

  // Disconnect ejectors for safety.
  gpio_set_level(drogue_chute_pin, 0);
  gpio_set_level(main_chute_pin, 0);

  // Time in ms since the program started.
  int64_t ms;

  ROCKET_STATUS_T rocket_status;
  mpu6050_acceleration_t rocket_accel = {0};
  float rocket_altitude;

  // Set the data structure that will receive the data.
  SensorDataQueue_t rxData;
  bool read_data = false; // Helper variable for initial data state.

  typedef enum {
    READY_TO_IGNITE,
    IGNITING,
    IGNITION_DONE
  } chute_charge_execution_status_t;

  chute_charge_execution_status_t drogue_chute_charge_execution_status =
      READY_TO_IGNITE;
  chute_charge_execution_status_t main_chute_charge_execution_status =
      READY_TO_IGNITE;

  int64_t ms_drogue_chute_deploy; // This variable will record the miliseconds
  // value when drogue chute is deployed.
  int64_t ms_main_chute_deploy; // This variable will record the miliseconds
                                // value when main chute is deployed.

  while (1) {
    // Read any available data from the queue.
    if (xQueueReceive(xSensorQueue, &rxData, portMAX_DELAY) == pdPASS) {
      rocket_status = rxData.rocket_status;
      rocket_accel = rxData.rocket_acc;
      rocket_altitude = rxData.rocket_altitude;

      read_data = true;
    }

    // Do not execute the task if data is not initialized.
    if (!read_data) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    // Calculate the milliseconds from the program start.
    ms = esp_timer_get_time() / 1000;

    // Calculate the current mean altitude.
    mean_altitude = update_rocket_altitude_mean(
        rocket_altitude, mean_altitude_array, &mean_oldest_pointer);

    // Update highest mean.
    if (highest_altitude_mean_detected < mean_altitude) {
      highest_altitude_mean_detected = mean_altitude;
    }

    switch (rocket_status) {
    case PREPARING_FOR_FLIGHT:
      break;

    case READY_TO_FLY: {
      // Detect liftoff.
      if (detect_liftoff(rocket_accel)) {
        rocket_status = ASCENDING;
        launching_altitude = mean_altitude;
        ms_liftoff = esp_timer_get_time() / 1000;
      }

      break;
    }

    case ASCENDING: {
      // Check for the best moment for opening the drogue chute.
      if (should_drogue_chute_be_deployed(
              mean_altitude, highest_altitude_mean_detected, ms, ms_liftoff)) {
        if (drogue_chute_charge_execution_status == READY_TO_IGNITE) {
          gpio_set_level(drogue_chute_pin, 1);
          ESP_LOGI(TAG_FLIGHT_CONTROLLER, "Drogue chute ignited %lld",
                   (long long)ms);
          ms_drogue_chute_deploy = ms;
          drogue_chute_charge_execution_status = IGNITING;
        } else if (drogue_chute_charge_execution_status == IGNITING) {
          if (ms - ms_drogue_chute_deploy >= MS_EXECUTING_CHUTE_CHARGES) {
            gpio_set_level(drogue_chute_pin, 0);
            ESP_LOGI(TAG_FLIGHT_CONTROLLER, "Drogue chute ignition stoped %lld",
                     (long long)ms);
            drogue_chute_charge_execution_status = IGNITION_DONE;
            rocket_status = DESCENDING_WITH_DROGUE;
          }
        } else {
          ESP_LOGI(TAG_FLIGHT_CONTROLLER, "Drogue chute already deployed.");
          rocket_status = DESCENDING_WITH_DROGUE;
        }
      }

      break;
    }

    case DESCENDING_WITH_DROGUE: {
      // Check for the best moment for opening the main chute.
      if (should_main_chute_be_deployed(mean_altitude, launching_altitude, ms,
                                        ms_liftoff)) {
        if (main_chute_charge_execution_status == READY_TO_IGNITE) {
          gpio_set_level(main_chute_pin, 1);
          ESP_LOGI(TAG_FLIGHT_CONTROLLER, "Main chute ignited %lld",
                   (long long)ms);
          ms_main_chute_deploy = ms;
          main_chute_charge_execution_status = IGNITING;
        } else if (main_chute_charge_execution_status == IGNITING) {
          if (ms - ms_main_chute_deploy >= MS_EXECUTING_CHUTE_CHARGES) {
            gpio_set_level(main_chute_pin, 0);
            ESP_LOGI(TAG_FLIGHT_CONTROLLER, "Main chute ignition stoped %lld",
                     (long long)ms);
            main_chute_charge_execution_status = IGNITION_DONE;
            rocket_status = DESCENDING_WITH_MAIN;
          }
        } else {
          ESP_LOGI(TAG_FLIGHT_CONTROLLER, "Main chute already deployed.");
          rocket_status = DESCENDING_WITH_MAIN;
        }
      }

      break;
    }

    case DESCENDING_WITH_MAIN: {
      // Detect touchdown.
      if (detect_touchdown(rocket_accel)) {
        rocket_status = LANDED;
      }

      break;
    }

    case LANDED:
      break;

    default:
      break;
    }

    // Check if the rocket status has been updated by the flight controller
    // (current task). In case it has been updated, write a new data to notify
    // the sensors task. This sensors task needs to be notified in order to send
    // over BLE the new value.
    if (rxData.rocket_status != rocket_status) {
      // Send new value to the queue.
      while (xQueueSend(xUpdatedStatusFromFlightController, &rocket_status,
                        pdMS_TO_TICKS(10)) != pdPASS) {

        // Failed to send new rocket status to sensor task.
        ESP_LOGE(TAG_FLIGHT_CONTROLLER, "Failed to send to sensor task");

        vTaskDelay(pdMS_TO_TICKS(10));
      }

      // New rocket status sent to sensor task.
      ESP_LOGI(TAG_FLIGHT_CONTROLLER, "New rocket status sent to sensor task");
    }

    ESP_LOGI(TAG_FLIGHT_CONTROLLER, "Rocket status: %i", rocket_status);

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}
