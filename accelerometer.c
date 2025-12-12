/** 
  * @file accelerometer.c
  * @brief Provides accelerometer data to the rest of the system.
  *
  * Main tasks:
  * - Read motors PWM values and simulate accelerometer based on it.
  * - Mutate accelerometer data within the shared memory with additional noise.
  *
  * @note
  *
  * Only this module changes internal accelerometer values. There are two readers of
  * acceleration values (flight_ctrl.c and telemetry.c).
  **/

#include "proj_types.h"

#define MAX_THRUST      9.81f * 2.0f        // 2g max upward acceleration.
#define DIFF_FACTOR     0.2f                // Motor imbalance on X/Y tilt.
#define NOISE_XY_STD    0.02f
#define NOISE_Z_STD     0.05f

static bat_charge_t current_battery;
static acceleration_t acc;
static motors_t m;

/* Simulation purpose noise for sensor. (Box-Muller) */
static float gauss_noise(float stddev) {
    float u1 = (rand() + 1.0f) / (RAND_MAX + 2.0f);
    float u2 = (rand() + 1.0f) / (RAND_MAX + 2.0f);
    float mag = stddev * sqrtf(-2.0f * logf(u1));
    return mag * cosf(2.0f * M_PI * u2);
} 

/**
  * @brief Main accelerometer loop function.
  *
  * Does the following:
  * - Reads motor values. Simulates accelerometer data based on motor values.
  *
  **/
void accel_loop(drone_shared_t *shm_ptr) { 
    sem_wait(&shm_ptr->pwm.mutex); 
    m = shm_ptr->pwm.motors;
    sem_post(&shm_ptr->pwm.mutex);

    float m0 = m.motors[0];
    float m1 = m.motors[1];
    float m2 = m.motors[2];
    float m3 = m.motors[3];

    /* Upward thrust (Z axis) — sum of 4 motors */
    float thrust = (m0 + m1 + m2 + m3) * MAX_THRUST;

    /* Differential thrust → tilt acceleration */
    float roll_acc  = (m1 + m3 - m0 - m2) * (MAX_THRUST * DIFF_FACTOR);
    float pitch_acc = (m2 + m3 - m0 - m1) * (MAX_THRUST * DIFF_FACTOR);

    /* Gravity always pulls down */
    acc.x = roll_acc + gauss_noise(NOISE_XY_STD);
    acc.y = pitch_acc + gauss_noise(NOISE_XY_STD);
    acc.z = thrust - 9.81f + gauss_noise(NOISE_Z_STD);

    printf("Accelerometer sample: [x: %f, y: %f, z: %f];\n", acc.x, acc.y, acc.z);

    sem_wait(&shm_ptr->accel.mutex); 
    shm_ptr->accel.acceleration = acc;
    sem_post(&shm_ptr->accel.mutex);

    shm_ptr->wdg.accel++;
    usleep(10000);
}
