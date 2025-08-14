#include "sd.h"

#include <driver/i2c.h>
#include <driver/sdspi_host.h>
#include <errno.h>
#include <esp_log.h>
#include <esp_vfs_fat.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

/**
 * Initialize SD card reader.
 */
esp_err_t init_sd_card(FILE **logFile) {
  esp_err_t ret;

  // Configure SDSPI host (VSPI).
  sdmmc_host_t host = SDSPI_HOST_DEFAULT();
  host.slot = VSPI_HOST;

  // Initialize the SPI bus.
  spi_bus_config_t bus_cfg = {
      .mosi_io_num = GPIO_NUM_23,
      .miso_io_num = GPIO_NUM_19,
      .sclk_io_num = GPIO_NUM_18,
  };

  ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);

  if (ret != ESP_OK) {
    ESP_LOGE(TAG_SD, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
    return SD_ERR_SPI_INIT;
  }

  // Mount FAT filesystem
  esp_vfs_fat_sdmmc_mount_config_t mount_config = {
      .format_if_mount_failed = false, .max_files = 5};

  sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
  slot_config.host_id = host.slot;
  slot_config.gpio_cs = GPIO_NUM_5;
  slot_config.gpio_cd = GPIO_NUM_NC;

  sdmmc_card_t *card;
  ret = esp_vfs_fat_sdspi_mount("/sdcard", &host, &slot_config, &mount_config,
                                &card);

  if (ret != ESP_OK) {
    ESP_LOGE(TAG_SD, "Failed to mount SD card: %s", esp_err_to_name(ret));
    *logFile = NULL;
    return SD_ERR_MOUNT_FAIL;
  }

  ESP_LOGI(TAG_SD, "SD card mounted successfully");

  /* Find a unique log filename: LOG_1.csv, LOG_2.csv, etc. */
  char filename[64];
  int idx = 1;
  struct stat st;
  while (1) {
    snprintf(filename, sizeof(filename), "/sdcard/log%d.csv", idx);
    if (stat(filename, &st) == 0) {
      // File exists, try next index
      idx++;
    } else {
      // File does not exist, use this name
      break;
    }
  }
  *logFile = fopen(filename, "w");

  if (!*logFile) {
    // Todo: check if there is a better way of getting the error without the
    // errno variable.
    int err = errno;
    ESP_LOGE(TAG_SD, "File open failed: errno=%d (%s)", err, strerror(err));
    return SD_ERR_FILE_OPEN;
  }
  ESP_LOGI(TAG_SD, "SD card file opened successfully");

  return ESP_OK;
}

/**
 * Writes the message to the file and saves it.
 */
esp_err_t write_to_file(FILE *file, const char *message, const char *endl) {
  // Check if endl has a value.
  if (!endl) {
    endl = "";
  }

  // Check if file is open.
  if (!file) {
    ESP_LOGE(TAG_SD, "Can not write to the file. File is closed.");
    return SD_ERR_FILE_CLOSED;
  }

  // Add text to the file.
  fprintf(file, "%s%s", message, endl);

  // Save the file into the SD card. This is necessary because when the ESP32 is
  // disconnected from power, if the file is not saved into the SD card all the
  // data will be lost.
  fflush(file);
  fsync(fileno(file));

  return ESP_OK;
}

char *sd_err_to_name(esp_err_t err) {
  switch (err) {
  case ESP_OK:
    return "OK";
  case (esp_err_t)SD_ERR_SPI_INIT:
    return "SPI initialization failed";
  case (esp_err_t)SD_ERR_MOUNT_FAIL:
    return "FAT mount failed";
  case (esp_err_t)SD_ERR_FILE_OPEN:
    return "File open failed";
  case (esp_err_t)SD_ERR_WRITE_FAIL:
    return "Writing to file failed";
  case (esp_err_t)SD_ERR_FILE_CLOSED:
    return "File is closed.";
  default:
    return "Unknown SD error";
  }
}