#include "ble/ble_c.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_common_api.h"
#include "esp_gattc_api.h"
#include "esp_gatts_api.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include <string.h>


// Variables for storeing the characteristic handler.
static uint16_t char_handle_rocket_status = 0;
static uint16_t char_handle_rocket_altitude = 0;
static uint16_t char_handle_rocket_speed = 0;
static uint16_t char_handle_rocket_pressure = 0;
static uint16_t char_handle_rocket_temperature = 0;
static uint16_t char_handle_rocket_acc_x = 0;
static uint16_t char_handle_rocket_acc_y = 0;
static uint16_t char_handle_rocket_acc_z = 0;
static uint16_t char_handle_rocket_rot_x = 0;
static uint16_t char_handle_rocket_rot_y = 0;
static uint16_t char_handle_rocket_rot_z = 0;

// Count for how many characteristics have been registerd.
static int registered_char_count = 0;

static uint16_t service_handle = 0;
static esp_gatt_if_t gatts_if_global = 0;
static uint16_t conn_id = 0;
static bool is_connected = false;
static uint16_t cccd_handle = 0;

esp_err_t ble_init(void) {
  esp_err_t err;

  // Initialize non-volatile memory.
  err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
      err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);

  // Initialize the controller.
  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  err = esp_bt_controller_init(&bt_cfg);
  if (err) {
    ESP_LOGE(TAG_BLE, "BT init failed: %s", esp_err_to_name(err));
    return err;
  }
  err = esp_bt_controller_enable(ESP_BT_MODE_BLE);
  if (err) {
    ESP_LOGE(TAG_BLE, "BT enable failed: %s", esp_err_to_name(err));
    return err;
  }

  // Initialize Bluedroid stack.
  err = esp_bluedroid_init();
  if (err) {
    ESP_LOGE(TAG_BLE, "Bluedroid init failed: %s", esp_err_to_name(err));
    return err;
  }
  err = esp_bluedroid_enable();
  if (err) {
    ESP_LOGE(TAG_BLE, "Bluedroid enable failed: %s", esp_err_to_name(err));
    return err;
  }

  // Declare that no callback is needed for GAP event.
  esp_ble_gap_register_callback(NULL);

  // Register GATT callback.
  esp_ble_gatts_register_callback(ble_gatts_event_handler);

  esp_ble_gatts_app_register(0);

  return ESP_OK;
}

void ble_gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                             esp_ble_gatts_cb_param_t *param) {
  switch (event) {

  // Called when the application has registered.
  // Here a function is called to create the service.
  case ESP_GATTS_REG_EVT:
    gatts_if_global = gatts_if;
    create_and_add_service();
    break;

  // Called when the service has finished creating.
  // Here characteristics are created.
  case ESP_GATTS_CREATE_EVT:
    service_handle = param->create.service_handle;
    crete_characteristics();
    break;

  // Called when a characteristic has been added.
  case ESP_GATTS_ADD_CHAR_EVT:
    // Register the handler of the characteristics.
    switch (param->add_char.char_uuid.uuid.uuid16) {

    case GATTS_CHAR_UUID_ROCKET_STATUS:
      char_handle_rocket_status = param->add_char.attr_handle;
      registered_char_count++;
      break;

    case GATTS_CHAR_UUID_ROCKET_ALTITUDE:
      char_handle_rocket_altitude = param->add_char.attr_handle;
      registered_char_count++;
      break;

    case GATTS_CHAR_UUID_ROCKET_SPEED:
      char_handle_rocket_speed = param->add_char.attr_handle;
      registered_char_count++;
      break;

    case GATTS_CHAR_UUID_ROCKET_PRESSURE:
      char_handle_rocket_pressure = param->add_char.attr_handle;
      registered_char_count++;
      break;

    case GATTS_CHAR_UUID_ROCKET_TEMPERATURE:
      char_handle_rocket_temperature = param->add_char.attr_handle;
      registered_char_count++;
      break;

    case GATTS_CHAR_UUID_ROCKET_ACC_X:
      char_handle_rocket_acc_x = param->add_char.attr_handle;
      registered_char_count++;
      break;

    case GATTS_CHAR_UUID_ROCKET_ACC_Y:
      char_handle_rocket_acc_y = param->add_char.attr_handle;
      registered_char_count++;
      break;

    case GATTS_CHAR_UUID_ROCKET_ACC_Z:
      char_handle_rocket_acc_z = param->add_char.attr_handle;
      registered_char_count++;
      break;

    case GATTS_CHAR_UUID_ROCKET_ROT_X:
      char_handle_rocket_rot_x = param->add_char.attr_handle;
      registered_char_count++;
      break;

    case GATTS_CHAR_UUID_ROCKET_ROT_Y:
      char_handle_rocket_rot_y = param->add_char.attr_handle;
      registered_char_count++;
      break;

    case GATTS_CHAR_UUID_ROCKET_ROT_Z:
      char_handle_rocket_rot_z = param->add_char.attr_handle;
      registered_char_count++;
      break;

    default:
      break;
    }

    // Start the service once the characteristic handles have been registered.
    if (registered_char_count == 11) {
      ESP_LOGI(TAG_BLE, "Done char count");
      esp_ble_gatts_start_service(service_handle);
    }

    break;


  // Called when a descriptor has been added to a characteristic.
  case ESP_GATTS_ADD_CHAR_DESCR_EVT:
    if (param->add_char_descr.descr_uuid.uuid.uuid16 ==
        ESP_GATT_UUID_CHAR_CLIENT_CONFIG) {
      cccd_handle = param->add_char_descr.attr_handle;
      ESP_LOGI(TAG_BLE, "CCCD handle = %u", cccd_handle);
    }
    break;

  // Called when a new client has connected to the server.
  case ESP_GATTS_CONNECT_EVT:
    conn_id = param->connect.conn_id;
    is_connected = true;
    ESP_LOGI(TAG_BLE, "Client connected (conn_id=%d)", conn_id);
    esp_ble_gatt_set_local_mtu(517);
    esp_ble_gattc_send_mtu_req(gatts_if, param->connect.conn_id);
    break;

  // Called when a client has disconnected from the server.
  case ESP_GATTS_DISCONNECT_EVT:
    is_connected = false;
    ESP_LOGI(TAG_BLE, "Client disconnected");

    // Restart advertisement.
    esp_ble_gap_start_advertising(&(esp_ble_adv_params_t){
        .adv_int_min = 0x20,
        .adv_int_max = 0x40,
        .adv_type = ADV_TYPE_IND,
        .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
        .channel_map = ADV_CHNL_ALL,
        .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
    });
    break;

  // Called when a client wants to read a value.
  case ESP_GATTS_READ_EVT:
    ESP_LOGI(TAG_BLE, "En ESP_GATT_READ_EVT");
    // If no response is needed, exit early (stack handles it automatically).
    if (!param->read.need_rsp)
      return;

    // Prepare and send response.
    esp_gatt_rsp_t rsp;
    memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
    rsp.attr_value.handle = param->read.handle;
    rsp.attr_value.len = sizeof(rocket_status);

    // Copy value into the response buffer in little-endian order.
    rsp.attr_value.value[0] = (uint8_t)(rocket_status & 0xFF);
    rsp.attr_value.value[1] = (uint8_t)((rocket_status >> 8) & 0xFF);
    rsp.attr_value.value[2] = (uint8_t)((rocket_status >> 16) & 0xFF);
    rsp.attr_value.value[3] = (uint8_t)((rocket_status >> 24) & 0xFF);

    esp_ble_gatts_send_response(gatts_if, param->read.conn_id,
                                param->read.trans_id, ESP_GATT_OK, &rsp);

    break;

  // Called when a client wants to write a value.
  case ESP_GATTS_WRITE_EVT:
    ESP_LOGI(TAG_BLE, "ESP_GATTS_WRITE_EVT, handle %u, len %u",
             param->write.handle, param->write.len);

    // Handle writes to the rocket status characteristic.
    if (param->write.handle == char_handle_rocket_status &&
        param->write.len == sizeof(uint32_t)) {
      uint32_t new_status = (uint32_t)param->write.value[0] |
                            ((uint32_t)param->write.value[1] << 8) |
                            ((uint32_t)param->write.value[2] << 16) |
                            ((uint32_t)param->write.value[3] << 24);

      // Set rocket status enum.
      switch (new_status) {
      case 0:
        rocket_status = PREPARING_FOR_FLIGHT;
        break;
      case 1:
        rocket_status = READY_TO_FLY;
        break;
      case 2:
        rocket_status = ASCENDING;
        break;
      case 3:
        rocket_status = DESCENDING_WITH_DROGUE;
        break;
      case 4:
        rocket_status = DESCENDING_WITH_MAIN;
        break;
      case 5:
        rocket_status = LANDED;
        break;

      default:
        break;
      }
      ESP_LOGI(TAG_BLE, "Updated rocket_status to %u", rocket_status);
    }

    // Send a write response if requested.
    if (param->write.need_rsp) {
      esp_ble_gatts_send_response(gatts_if_global, param->write.conn_id,
                                  param->write.trans_id, ESP_GATT_OK, NULL);
    }

    break;

  default:
    break;
  }
}

void create_and_add_service(void) {
  esp_err_t ret;

  // Create the service.
  esp_gatt_srvc_id_t svc_id = {
      .is_primary = true,
      .id.inst_id = 0,
      .id.uuid.len = ESP_UUID_LEN_16,
      .id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID,

  };
  esp_ble_gatts_create_service(gatts_if_global, &svc_id, GATTS_NUM_HANDLE);


  // 16-byte UUID.
  static uint8_t adv_service_uuid128[16] = {0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00,
                                            0x00, 0x80, 0x00, 0x10, 0x00, 0x00,
                                            0xFF, 0x00, 0x00, 0x00};

  // Configure advertising data.
  esp_ble_adv_data_t adv_data = {
      .set_scan_rsp = false,
      .include_name = false,    // Do not set a device name in advertisement.
      .include_txpower = false, // No TX power level in advertisement.
      .min_interval = 0x0006,   // Optional: 7.5 ms.
      .max_interval = 0x0010,   // Optional: 20 ms (these can be 0 if not used).
      .appearance = 0x00,       // Optional: appearance field (0 = unknown).
      .manufacturer_len = 0,
      .p_manufacturer_data = NULL,
      .service_data_len = 0,
      .p_service_data = NULL,
      .service_uuid_len =
          sizeof(adv_service_uuid128),       // MUST be 16 for one 16-bit UUID.
      .p_service_uuid = adv_service_uuid128, // Pointer to 16-byte UUID array.
      .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
  };

  // Configure the response by the device once a client has sent an active scan
  // request.
  esp_ble_adv_data_t scan_rsp_data = {
      .set_scan_rsp = true,
      .include_name = true,     // Put the device name in scan response.
      .include_txpower = false, // Optional: TX power in scan response.
      .manufacturer_len = 0,
      .p_manufacturer_data = NULL,
      .service_data_len = 0,
      .p_service_data = NULL,
      .service_uuid_len = 0,
      .p_service_uuid = NULL, // Do not send UUIDs in scan response.
      .flag = 0, // Flags are only in the primary adv, not in scan response.
  };

  // Set the GAP device name.
  ret = esp_ble_gap_set_device_name("IZAR Flight Controller");
  if (ret != ESP_OK) {
    ESP_LOGE(TAG_BLE, "Failed to set device name: %s", esp_err_to_name(ret));
  }

  ret = esp_ble_gap_config_adv_data(&adv_data);
  ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG_BLE, "Failed to configure adv and rsp data: %s",
             esp_err_to_name(ret));
  }

  // Start advertising.
  esp_ble_gap_start_advertising(&(esp_ble_adv_params_t){
      .adv_int_min = 0x20,
      .adv_int_max = 0x40,
      .adv_type = ADV_TYPE_IND,
      .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
      .channel_map = ADV_CHNL_ALL,
      .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
  });
}

void add_characteristic(int identifier) {
  esp_ble_gatts_add_char(service_handle,
                         &(esp_bt_uuid_t){
                             .len = ESP_UUID_LEN_16,
                             .uuid.uuid16 = identifier,
                         },
                         ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                         ESP_GATT_CHAR_PROP_BIT_READ |
                             ESP_GATT_CHAR_PROP_BIT_WRITE |
                             ESP_GATT_CHAR_PROP_BIT_NOTIFY,
                         NULL, NULL);
}

void crete_characteristics(void) {
  // Add all characteristics.
  add_characteristic(GATTS_CHAR_UUID_ROCKET_STATUS);
  add_characteristic(GATTS_CHAR_UUID_ROCKET_ALTITUDE);
  add_characteristic(GATTS_CHAR_UUID_ROCKET_SPEED);
  add_characteristic(GATTS_CHAR_UUID_ROCKET_PRESSURE);
  add_characteristic(GATTS_CHAR_UUID_ROCKET_TEMPERATURE);
  add_characteristic(GATTS_CHAR_UUID_ROCKET_ACC_X);
  add_characteristic(GATTS_CHAR_UUID_ROCKET_ACC_Y);
  add_characteristic(GATTS_CHAR_UUID_ROCKET_ACC_Z);
  add_characteristic(GATTS_CHAR_UUID_ROCKET_ROT_X);
  add_characteristic(GATTS_CHAR_UUID_ROCKET_ROT_Y);
  add_characteristic(GATTS_CHAR_UUID_ROCKET_ROT_Z);

  // Add characteristic descriptor.
  esp_bt_uuid_t descr_uuid = {
      .len = ESP_UUID_LEN_16,
      .uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG,
  };
  esp_ble_gatts_add_char_descr(service_handle, &descr_uuid,
                               ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL,
                               NULL);
}

esp_err_t update_rocket_status(int value) {
  return ble_notify_uint8(value, char_handle_rocket_status);
}

esp_err_t update_rocket_altitude(float value) {
  return ble_notify_float(value, char_handle_rocket_altitude);
}

esp_err_t update_rocket_speed(float value) {
  return ble_notify_float(value, char_handle_rocket_speed);
}

esp_err_t update_rocket_pressure(int32_t value) {
  return ble_notify_float(value, char_handle_rocket_pressure);
}

esp_err_t update_rocket_temperature(float value) {
  return ble_notify_float(value, char_handle_rocket_temperature);
}

esp_err_t update_rocket_acc_x(float value) {
  return ble_notify_float(value, char_handle_rocket_acc_x);
}

esp_err_t update_rocket_acc_y(float value) {
  return ble_notify_float(value, char_handle_rocket_acc_y);
}

esp_err_t update_rocket_acc_z(float value) {
  return ble_notify_float(value, char_handle_rocket_acc_z);
}

esp_err_t update_rocket_rot_x(float value) {
  return ble_notify_float(value, char_handle_rocket_rot_x);
}

esp_err_t update_rocket_rot_y(float value) {
  return ble_notify_float(value, char_handle_rocket_rot_y);
}

esp_err_t update_rocket_rot_z(float value) {
  return ble_notify_float(value, char_handle_rocket_rot_z);
}

esp_err_t ble_notify_float(float value, uint16_t char_handler) {
  esp_err_t err;
  if (!is_connected) {
    return ESP_FAIL;
  }
  err = esp_ble_gatts_send_indicate(gatts_if_global, conn_id, char_handler,
                                    sizeof(value), (uint8_t *)&value, false);
  if (err)
    ESP_LOGE(TAG_BLE, "Err notify float: %d", err);
  return err;
}

esp_err_t ble_notify_uint8(uint8_t value, uint16_t char_handler) {
  esp_err_t err;
  if (!is_connected) {
    return ESP_FAIL;
  }
  err = esp_ble_gatts_send_indicate(gatts_if_global, conn_id, char_handler,
                                    sizeof(value), (uint8_t *)&value, false);
  if (err)
    ESP_LOGE(TAG_BLE, "Err notify uint8: %d", err);
  return err;
}