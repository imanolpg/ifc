#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include "sensors_reading/mpu6050/mpu6050_c.h"

#include <inttypes.h> // For int64_t
#include <stdbool.h>  // For bool

// Set acceleration threshold to detect liftoff (in g-forces).
#define ACCELERATION_THRESHOLD_FOR_LIFTOFF_DETECTION 2.0

// Set window size to calculate the altitude mean.
#define WINDOW_SIZE_FOR_ALTITUDE_MEAN 10

// Altitude threshold (in meters) from the highest recorded altitude to consider
// it the apogee.
#define ALTITUDE_DIFFERENCE_TO_CONSIDER_APOGEE 1

// Configure height for deploying main chute (in meters).
#define MAIN_CHUTE_DEPLOYMENT_HEIGHT 150.0

// Configure deployment times. This opens windows for chutes deployment.
#define MIN_MS_DROGUE_DEPLOYMENT 9500
#define MAX_MS_DROGUE_DEPLOYMENT 1300
#define MIN_MS_MAIN_DEPLOYMENT 20000
#define MAX_MS_MAIN_DEPLOYMENT 35000

// Number of milliseconds that the chute deployment charges will be ignited.
#define MS_EXECUTING_CHUTE_CHARGES 1000

// Set acceleration threshold to detect touchdown (in g-forces).
#define ACCELERATION_THRESHOLD_FOR_TOUCHDOWN_DETECTION 2.0

#define TAG_FLIGHT_CONTROLLER "Flight Controller"

/**
 * @brief Main FreeRTOS task function for the flight controller.
 *
 * Evaluates rocket status and determines when to deploy the drogue chute and
 * the main chute based on the data available from the rocket sensors.
 */
void flight_controller();

/**
 * @brief Detect if liftoff has happen. Works by detecting if the currect linear
 * acceleration is higher than the stablish level.
 *
 * @param rocket_acc linear acceleration.
 * @return true if liftoff has been detected.
 * @return false if there is not enough acceleration to consider liftoff.
 */
bool detect_liftoff(mpu6050_acceleration_t rocket_acc);

/**
 * @brief Create a mobile mean to filter rocket altitude.
 *
 * @param rocket_altitude calculated rocket altitude.
 * @param altitude_mean_array pointer to where all the values for the mean
 * mobile are being stored.
 * @param oldest_element_pointer pointer to the oldest altitude reading.
 * @return float new mobile mean.
 */
float update_rocket_altitude_mean(float rocket_altitude,
                                  float *altitude_mean_array,
                                  int *oldest_element_pointer);

/**
 * @brief Check if the drogue chute should be deployed.
 *
 * It takes into acount:
 * 1. The flight time must be between the stablish window for drogue chute
 * deployment. The chute will be deployed if time exceeds the maximum time set
 * in MAX_MS_DROGUE_DEPLOYMENT.
 * 2. The difference between the biggest recorded mean altitude value (apogee)
 * and the current mean altitude value should be of at least what defined in
 * ALTITUDE_DIFFERENCE_TO_CONSIDER_APOGEE.
 *
 * @param altitude_mean current altitude mean value.
 * @param highest_altitude_mean_detected biggest recorded mean value.
 * @param ms current milliseconds.
 * @param ms_liftoff milliseconds when liftoff happened.
 * @return true if sweet spot for drogue chute deployment has occurred now or in
 * the past.
 * @return false if sweet spot for drogue chute deployment has not occurred yet.
 */
bool should_drogue_chute_be_deployed(float altitude_mean,
                                     float highest_altitude_mean_detected,
                                     uint64_t ms, uint64_t ms_liftoff);

/**
 * @brief Check if the main chute should be deployed.
 *
 * It takes into acount:
 * 1. The flight time must be between the stablish window for main chute
 * deployment. The chute will be deployed if time exceeds the maximum time set
 * in MAX_MS_MAIN_DEPLOYMENT.
 * 2. The difference between the current mean altitude and the launching
 * altitude value should be less than what is defined in
 * MAIN_CHUTE_DEPLOYMENT_HEIGHT.
 *
 * @param mean_altitude current altitude mean value.
 * @param launching_altitude altitude recorded when liftoff occurred.
 * @param ms current milliseconds.
 * @param ms_liftoff millisecconds when liftoff occurred.
 * @return true if sweet spot for main chute deployment has occurred now or in
 * the past.
 * @return false if sweet spot for main chute deployment has not occurred yet.
 */
bool should_main_chute_be_deployed(float mean_altitude,
                                   float launching_altitude, uint64_t ms,
                                   uint64_t ms_liftoff);

/**
 * @brief Detect if the rocket has landed.
 *
 * Touchdown is detected when any of the axes linear acceleration data detects
 * an acceleration grater than what is set in
 * ACCELERATION_THRESHOLD_FOR_TOUCHDOWN_DETECTION.
 *
 * @param rocket_acc linear acceleration data.
 * @return true if landing has been detected.
 * @return false if landing has not been detected.
 */
bool detect_touchdown(mpu6050_acceleration_t rocket_acc);

#endif // FLIGHT_CONTROLLER_H