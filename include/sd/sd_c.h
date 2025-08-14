#ifndef SD_C_H
#define SD_C_H

#include <esp_err.h>
#include <stdio.h>

#define TAG_SD "SD"

/**
 * @brief Initialize the SD card and open the file to write data.
 *
 * @param logFile pointer to the file that needs to be open.
 * @return esp_err_t `ESP_OK` if success.
 */
esp_err_t sd_c_init(FILE **logFile);

/**
 * @brief Write text to file.
 *
 * @param file pointer to the file descriptor to write the text.
 * @param text array of characters to write to the file.
 */
void write_to_file_c(FILE *file, char *text);


#endif // SD_C_H
