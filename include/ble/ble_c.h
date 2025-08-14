#ifndef BLE_C_H
#define BLE_C_H

#include "esp_err.h"
#include "esp_gatts_api.h"
#include "sensors_reading/mpu6050/mpu6050_c.h"
#include "sensors_reading/sensors_reading.h"

#include <stdint.h>

#ifndef LO_UINT16
#define LO_UINT16(a) ((uint8_t)((a) & 0xFF))
#endif
#ifndef HI_UINT16
#define HI_UINT16(a) ((uint8_t)(((a) >> 8) & 0xFF))
#endif


#define GATTS_SERVICE_UUID 0x00FF              // Service UUID
#define GATTS_CHAR_UUID_ROCKET_STATUS 0xFF01   // Characteristic Rocket Status
#define GATTS_CHAR_UUID_ROCKET_ALTITUDE 0xFF02 // Characteristic Rocket altitude
#define GATTS_CHAR_UUID_ROCKET_SPEED 0xFF03    // Characteristic Rocket speed
#define GATTS_CHAR_UUID_ROCKET_PRESSURE 0xFF04 // Characteristic Rocket pressure
#define GATTS_CHAR_UUID_ROCKET_TEMPERATURE                                     \
  0xFF05                                    // Characteristic Rocket temperature
#define GATTS_CHAR_UUID_ROCKET_ACC_X 0xFF06 // Characteristic Rocket acc x
#define GATTS_CHAR_UUID_ROCKET_ACC_Y 0xFF07 // Characteristic Rocket acc y
#define GATTS_CHAR_UUID_ROCKET_ACC_Z 0xFF08 // Characteristic Rocket acc z
#define GATTS_CHAR_UUID_ROCKET_ROT_X 0xFF09 // Characteristic Rocket rot x
#define GATTS_CHAR_UUID_ROCKET_ROT_Y 0xFF10 // Characteristic Rocket rot y
#define GATTS_CHAR_UUID_ROCKET_ROT_Z 0xFF11 // Characteristic Rocket rot z

#define GATTS_NUM_HANDLE 25

extern ROCKET_STATUS_T rocket_status;
extern mpu6050_acceleration_t rocket_accel;
extern mpu6050_rotation_t rocket_rot;
extern float rocket_temperature, rocket_altitude;
extern int32_t rocket_pressure;

#define TAG_BLE "BLE"


/**
 * @brief Initializes BLE application with all the needed drivers and
 * components.
 *
 * @return esp_err_t `ESP_OK` if success.
 */
esp_err_t ble_init(void);

/**
 * @brief Callback function for GATT services.
 *
 * @param event
 * @param gatts_if
 * @param param
 */
void ble_gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                             esp_ble_gatts_cb_param_t *param);

/**
 * @brief Create a and add service object.
 *
 */
void create_and_add_service(void);

/**
 * @brief Create the characteristic.
 *
 */
void crete_characteristics(void);

/**
 * @brief Notify a float value.
 *
 * @param value
 * @param char_handler
 * @return esp_err_t
 */
esp_err_t ble_notify_float(float value, uint16_t char_handler);

/**
 * @brief Notify a uint8_t value.
 *
 * @param value
 * @param char_handler
 * @return esp_err_t
 */
esp_err_t ble_notify_uint8(uint8_t value, uint16_t char_handler);

// External functions for notifying new values.
/**
 * @brief Notify rocket status over BLE.
 *
 * @param value new rocket status.
 * @return esp_err_t `ESP_OK` if success.
 */
esp_err_t update_rocket_status(int value);

/**
 * @brief Notify altitude over BLE.
 *
 * @param value new altitude value.
 * @return esp_err_t
 */
esp_err_t update_rocket_altitude(float value);

/**
 * @brief Notify speed over BLE.
 *
 * @param value new rocket speed value.
 * @return esp_err_t `ESP_OK` if success.
 */
esp_err_t update_rocket_speed(float value);

/**
 * @brief Notify pressure over BLE.
 *
 * @param value new pressure value.
 * @return esp_err_t `ESP_OK` if success.
 */
esp_err_t update_rocket_pressure(int32_t value);

/**
 * @brief Notify temperature over BLE.
 *
 * @param value new temperature value.
 * @return esp_err_t `ESP_OK` if success.
 */
esp_err_t update_rocket_temperature(float value);

/**
 * @brief Notify new X-axis linear acceleration.
 *
 * @param value new X-axis linear acceleration value.
 * @return esp_err_t `ESP_OK` if success.
 */
esp_err_t update_rocket_acc_x(float value);

/**
 * @brief Notify new Y-axis linear acceleration.
 *
 * @param value new Y-axis linear acceleration value.
 * @return esp_err_t `ESP_OK` if success.
 */
esp_err_t update_rocket_acc_y(float value);

/**
 * @brief Notify new Z-axis linear acceleration.
 *
 * @param value new Z-axis linear acceleration value.
 * @return esp_err_t `ESP_OK` if success.
 */
esp_err_t update_rocket_acc_z(float value);

/**
 * @brief Notify new X-axis angular acceleration.
 *
 * @param value new X-axis angular acceleration value.
 * @return esp_err_t `ESP_OK` if success.
 */
esp_err_t update_rocket_rot_x(float value);

/**
 * @brief Notify new Y-axis angular acceleration.
 *
 * @param value new Y-axis angular acceleration value.
 * @return esp_err_t `ESP_OK` if success.
 */
esp_err_t update_rocket_rot_y(float value);

/**
 * @brief Notify new Z-axis angular acceleration.
 *
 * @param value new Z-axis angular acceleration value.
 * @return esp_err_t `ESP_OK` if success.
 */
esp_err_t update_rocket_rot_z(float value);

#endif // BLE_C_H