/** 
  * @file actors.h
  * @brief Function declaration of actor processes.
  *
  * @note
  *
  * Main process (drone_sys.c) is responsible for handling the main loop. Dead children will
  * be respawned and their residues cleaned.
  **/

#pragma once

#ifndef ACTORS_H
#define ACTORS_H

#include "proj_types.h"

/**
 * @brief Main accelerometer loop function.
 *
 * Does the following:
 * - Reads motor values. Simulates accelerometer data based on motor values.
 *
 **/
void accel_loop(drone_shared_t *shm_ptr); 

/**
  * @brief Main battery loop function.
  *
  * Does the following:
  * - Reads monotonic clock value;
  * - Based on current drone state, discharges each 2 seconds or charges each 500 milliseconds.
  * - Mutates state if charge is lower than 15 %.
  * - Simulates system shutdown when charge is 0% by sending SIGTERM to all related processes.
  * - Updates global charge value via atomic operations.
  *
  **/
void battery_loop(drone_shared_t *shm_ptr);

/**
  * @brief Main GPS loop function.
  *
  * Does the following:
  * - Acts as a producer that writes upcoming NMEA strings to shared circular buffer.
  *
  **/
void gps_loop(drone_shared_t *shm_ptr);

/**
  * @brief Flight controller loop function.
  *
  * Does the following:
  * - Mutates state based on data obtained from the operator.
  * - Mutates `motors[4]` based on accelerometer data and current state.
  * - Turns off `motors[4]` slowly and changes to `Charge` state, if battery is less than 15%.
  *
  **/
void flight_loop(drone_shared_t *shm_ptr);

/**
  * @brief telemetry sender loop function.
  *
  * Does the following:
  * - Reads all types of data within the shared memory region.
  * - If operator is ready, sends data packages for real time samples of the system.
  * - Not all resources are guaranteed to be sent in each package.
  * - Only sends GPS data when internal state is `SampleGPS` (consumer).
  *
  **/
void telemetry_loop(drone_shared_t *shm_ptr);

/**
  * @brief Watchdog loop.
  *
  * Does the following:
  * - For all other PIDs, checks their stack trace to decide, whether they were caught in a deadlock.
  * - If at least one deadlock occurs, signals the parent process to reinit locks, and restart all children.
  *
  * @note Internal state of data within the shared memory is preserved. Only locks are reinitialized.
  **/
void watchdog_loop(drone_shared_t *shm_ptr);

#endif // !ACTORS_H

