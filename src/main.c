#include "flight_controller/flight_controller.h"
#include "sensors_reading/sensors_reading.h"

#include "freertos/queue.h"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Handle for the queue.
QueueHandle_t xSensorQueue = NULL;
QueueHandle_t xUpdatedStatusFromFlightController = NULL;

void app_main() {

  // Setup log levels.
  esp_log_level_set("*", ESP_LOG_DEBUG);
  esp_log_level_set("BLE", ESP_LOG_ERROR);
  esp_log_level_set("SD", ESP_LOG_ERROR);
  esp_log_level_set("Flight Controller", ESP_LOG_DEBUG);
  esp_log_level_set("Sensors Reading", ESP_LOG_ERROR);
  esp_log_level_set("MPU6050", ESP_LOG_ERROR);
  esp_log_level_set("MS5611", ESP_LOG_ERROR);

  // Create a queue for sending data between tasks.
  // Queue to send sensors readings from SensorsReading to FlightController.
  xSensorQueue = xQueueCreate(1, sizeof(SensorDataQueue_t));
  if (xSensorQueue == NULL) {
    printf("Queue xSensorQueue creation failed!\n");
    return;
  }

  // Queue to notify the SensorsReading task of any new values of the rocket
  // status.
  xUpdatedStatusFromFlightController = xQueueCreate(1, sizeof(ROCKET_STATUS_T));
  if (xUpdatedStatusFromFlightController == NULL) {
    printf("Queue xUpdatedStatusFromFlightController creation failed!\n");
    return;
  }

  // Create the tasks.
  xTaskCreate(sensors_reading, "SensorsReadingTask", 4096, NULL, 5, NULL);
  xTaskCreate(flight_controller, "FlightControllerTask", 4096, NULL, 3, NULL);
}