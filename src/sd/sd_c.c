#include "sd/sd_c.h"
#include "sd.h"

#include <unistd.h>

esp_err_t sd_c_init(FILE **file) {

  // Initialize SD card.
  ESP_ERROR_CHECK(init_sd_card(file));

  // Write CSV column titles to file.
  ESP_ERROR_CHECK(
      write_to_file(*file,
                    "Time(ms),RocketStatus,Pressure(Pa),Altitude(m),"
                    "VerticalSpeed(m/s),Temperature(ºC),AccX(g),"
                    "AccY(g),AccZ(g),RotX(º/s),RotY(º/s),RotZ(º/s)",
                    "\n"));

  return ESP_OK;
}

void write_to_file_c(FILE *file, char *text) {
  write_to_file(file, text, "\n");
}