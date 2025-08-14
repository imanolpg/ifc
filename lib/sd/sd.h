#ifndef SD_H
#define SD_H

#include <esp_log.h>
#include <freertos/FreeRTOS.h>

#define TAG_SD "SD"

// Define error codes.
#define SD_ERR_BASE 0x200

typedef enum {
  SD_ERR_SPI_INIT = SD_ERR_BASE + 1,    ///< spi_bus_initialize() failed
  SD_ERR_MOUNT_FAIL = SD_ERR_BASE + 2,  ///< esp_vfs_fat_sdspi_mount() failed
  SD_ERR_FILE_OPEN = SD_ERR_BASE + 3,   ///< fopen() returned NULL
  SD_ERR_WRITE_FAIL = SD_ERR_BASE + 4,  ///< fwrite() or flush failed
  SD_ERR_FILE_CLOSED = SD_ERR_BASE + 5, ///< file is closed
} sd_err_t;

esp_err_t init_sd_card(FILE **logFile);
esp_err_t write_to_file(FILE *file, const char *message, const char *endl);
char *sd_err_to_name(esp_err_t err);

#endif // SD_H