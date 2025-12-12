/** 
  * @file proj_types.h
  * @brief Defines shared data types between all actors and mail loops definitions.
  *
  * @note
  **/

#pragma once

#ifndef PROJ_TYPES_H
#define PROJ_TYPES_H

// STD
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdatomic.h>
#include <stdbool.h>

// System ctrl.
#include <sys/prctl.h>
#include <sys/wait.h>
#include <sys/socket.h>
#include <semaphore.h>
#include <unistd.h>
#include <fcntl.h>

// OPENSSL (TLS)
#include <arpa/inet.h>
#include <openssl/ssl.h>
#include <openssl/tls1.h>
#include <openssl/bio.h>
#include <openssl/err.h>

// MISC
#include <sys/mman.h>
#include <string.h>
#include <signal.h>
#include <errno.h>
#include <time.h>
#include <math.h>
#include <ctype.h>

#define SHM_NAME                "drone_shm"

#define NANOSECONDS_IN_MS       1000000L
#define NANOSECONDS_IN_SEC      1000000000L

/**
  * @brief Subtype representation of battery charge.
  *
  * @note Shall always be in range of <0 ... 100>. Other values shall be considered as errors.
  **/
typedef _Atomic(uint8_t) bat_charge_t; 

/**
  * @brief Drone state type. 
  *
  *                v----------------*
  * SampleGPS <-> Fly <-> Land  -> Idle <-> Charge
  *    |           |       |        ^
  *    |           |       v        |
  *    |           *----> Abort ----*
  *    *-------------------^
  **/
typedef enum {
    Reserved    = 1 << 0,
    // Same as `Fly`. Forces the GPS controller to provide samples.
    SampleGPS   = 1 << 1,
    // Motors work at full speed until stabilization.
    Fly         = 1 << 2,
    // Motors are being stopped slowly.
    Land        = 1 << 3,
    // Idle state on floor.
    Idle        = 1 << 4,
    // Same as idle but charging it's battery.
    Charge      = 1 << 5,
    // Same as `Land`, but cannot be changed until set to `Idle`.
    Abort       = 1 << 6,
} current_action_t;

/**
  * @brief Drone's acceleration on all axes in g-units. 
  **/
typedef struct {
    float x, y, z;
} acceleration_t;

/**
  * @brief Drone's motors PWM ratio. 
  **/
typedef struct {
    float motors[4];
} motors_t;

#define GPS_BUFFER_SIZE (128 * 10)

/**
  * @brief NMEA string circular buffer. 
  **/
typedef struct {
    char buf[GPS_BUFFER_SIZE];
} nmea_t;

/**
  * @brief Semaphore based implementation of RWLock for multiple readers and multiple writers.
  **/
typedef struct {
    sem_t read, write;
    uint8_t read_counter;
} rw_lock_t;

/**
  * @brief Initialization for RWLock.
  **/
void rwlock_init(rw_lock_t *rwlock);

/**
  * @brief Implementation of RWLock reader lock algorithm.
  **/
void rwlock_read_lock(rw_lock_t *rwlock);

/**
  * @brief Implementation of RWLock reader unlock algorithm.
  **/
void rwlock_read_unlock(rw_lock_t *rwlock);

/**
  * @brief Implementation of RWLock writer lock algorithm.
  **/
void rwlock_write_lock(rw_lock_t *rwlock);

/**
  * @brief Implementation of RWLock writer unlock algorithm.
  **/
void rwlock_write_unlock(rw_lock_t *rwlock);

/**
  * @brief Table of PIDs for all drone subsystem processes.
  *
  * @note   Each process must update its PID entry in shared memory during startup.
  *         It allows parent process to respawn them when they are killed or crashed.
  **/
typedef struct {
    pid_t flight_ctrl, accel, battery, gps_ctrl, telemetry, wdg;
} drone_pids_t;


/**
  * @brief  Table of counters, where each actor increments them individually.
  **/
typedef struct {
    uint32_t flight_ctrl, accel, battery, gps_ctrl, telemetry;
} wdg_counters_t;

/**
  * @brief Contains all data, which are shared through the POSIX shared memory API. 
  *
  * @note Besides raw types, contains all required synchronization primitives.
  **/
typedef struct {
    // PID of each process. Used for signals.
    drone_pids_t pids;

    // Operator server IP.
    char operator_ip[INET_ADDRSTRLEN], drone_ip[INET_ADDRSTRLEN];
    // Telemetry and flight controller port.
    uint16_t telemetry_port, flight_ctrl_port;

    // Counters for watchdog process.
    wdg_counters_t wdg;

    // Multiple-writers multiple-readers => requires RWLock.
    struct {
        rw_lock_t lock;         // RWlock.
        current_action_t type;  // Raw data type.
    } action;

    // Single-writer, multiple-readers => one mutex for all.
    struct {
        sem_t mutex;                    // Mutex lock.
        acceleration_t acceleration;    // Raw data type.
    } accel;

    // Single-writer, multiple-readers => one mutex for all.
    struct {
        sem_t mutex;                    // Mutex lock.
        motors_t motors;                // Raw data type.
    } pwm;

    // Producer-consumer problem with circular buffer and indexes.
    struct {
        sem_t mutex, full, empty;           // Semaphores with termination conditions to prevent busy loop.
        size_t write, read;                 // Local counter for producer and consumer.
        nmea_t nmea;                        // Raw buffer.
    } gps;

    // Atomical value => no extra synchronization primitive.
    bat_charge_t battery;
} drone_shared_t;

#define __PRINTACT_HELPER(name) \
    case name:                  \
        msg = #name;            \
        break;                  \

/**
  * @brief Prints current action to STDIN.
  */
static void printactln(current_action_t a) {
    const char *msg = "Undefined";
    switch (a) {
        __PRINTACT_HELPER(Reserved)
        __PRINTACT_HELPER(Fly)
        __PRINTACT_HELPER(SampleGPS)
        __PRINTACT_HELPER(Land)
        __PRINTACT_HELPER(Idle)
        __PRINTACT_HELPER(Charge)
        __PRINTACT_HELPER(Abort)
    }
    printf("%s\n", msg);
}

#endif
