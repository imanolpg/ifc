#ifndef MS5611_C_H
#define MS5611_C_H

#include "median.h"
#include "ms5611.h"

#define TAG_MS5611 "MS5611"

#define MS5611_OSR MS5611_OSR_4096

#define MEDIAN_SIZE_PRS 50
#define MEDIAN_SIZE_TMP 50

/**
 * @brief Initialize MS5611 descriptor for pressure and temperature readings.
 *
 * @param dev pointer where the descriptor will be stored.
 * @return esp_err_t `ESP_OK` if success.
 */
esp_err_t ms5611_c_init(ms5611_t *dev);

/**
 * @brief Read data from MS5611 and store pressure (Pa), temperature (ºC) and
 * altitude (m) from sea level.
 *
 * @param dev pointer to MS5611 device descriptor.
 * @param pressure pointer where pressure reading will be stored (Pa).
 * @param temperature pointer where temperature reading will be stored (ºC).
 * @param altitude pointer where altitude estimation will be stored (m).
 */
void ms5611_read_pressure_temperature_and_altitude(ms5611_t *dev,
                                                   int32_t *pressure,
                                                   float *temperature,
                                                   float *altitude);

#endif // MS5611_C_H