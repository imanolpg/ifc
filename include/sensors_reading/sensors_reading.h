#ifndef SENSORS_READING_H
#define SENSORS_READING_H

#include "mpu6050.h"

/**
 * @def ACTIVATE_MPU6050
 * @brief Enable MPU6050 sensor for linear acceleration and angular rotation.
 */
#define ACTIVATE_MPU6050 1

/**
 * @def ACTIVATE_MS5611
 * @brief Enable MS5611 pressure and temperature sensor.
 */
#define ACTIVATE_MS5611 1

/**
 * @def ACTIVATE_SD
 * @brief Enable SD card logging for saving flight data.
 */
#define ACTIVATE_SD 1

/**
 * @def ACTIVATE_BLE
 * @brief Enable BLE control and telemetry transmission.
 */
#define ACTIVATE_BLE 1

/**
 * @enum ROCKET_STATUS_T
 * @brief Rocket flight status states.
 *
 * Enumerates the possible phases of the rocket's flight sequence.
 */
typedef enum {
  PREPARING_FOR_FLIGHT,   ///< Manipulating the rocket.
  READY_TO_FLY,           ///< Set in the launch rod ready for liftoff.
  ASCENDING,              ///< Rocket is ascending.
  DESCENDING_WITH_DROGUE, ///< Apogee detected and falling with drogue chute.
  DESCENDING_WITH_MAIN,   ///< Falling with main chute.
  LANDED                  ///< Rocket landed.
} ROCKET_STATUS_T;

/**
 * @brief I2C bus configuration for sensor communication with MS5611 and
 * MPU6050.
 */
#define I2C_SCL_IO 22
#define I2C_SDA_IO 21
#define I2C_NUM I2C_NUM_0
#define I2C_FREQ_HZ 400000

#define TAG_SENSORS_READING "Sensors Reading"

/**
 * @brief Data packet structure for passing sensor readings between tasks.
 *
 * Contains the current flight status, altitude, and IMU readings.
 */
typedef struct {
  ROCKET_STATUS_T rocket_status;
  float rocket_altitude;
  mpu6050_acceleration_t rocket_acc;
} SensorDataQueue_t;

/**
 * @brief Main FreeRTOS task function for reading all sensors.
 *
 * Initializes peripherals, reads sensor data in an infinite loop and pushes new
 * data into the queue to notify the flight controller.
 */
void sensors_reading();

#endif // SENSORS_READING_H