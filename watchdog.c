/** 
  * @file watchdog.c
  * @brief Checks whether system is stalled by trying to acquire different semaphores from time to time.
  *
  * Main tasks:
  * - Checks all other children if they are waiting for a semaphore.
  * - Forces the main process to reinit a shared memory region when a timeout has reached.
  *
  * @note
  *
  **/

#include "proj_types.h"

#define WDG_SLEEP_US            100000
#define WDG_TIMEOUT_MS          2000

/* Helper to get current time in milliseconds */
uint32_t get_time_ms() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (tv.tv_sec * 1000UL) + (tv.tv_usec / 1000UL);
}

/**
  * @brief Watchdog loop.
  *
  * Does the following:
  * - For all other PIDs, checks their stack trace to decide, whether they were caught in a deadlock.
  * - If at least one deadlock occurs, signals the parent process to reinit locks, and restart all children.
  *
  * @note Internal state of data within the shared memory is preserved. Only locks are reinitialized.
  **/
void watchdog_loop(drone_shared_t *shm_ptr) {
    static uint32_t old[5] = {0};
    // Keep last time heartbeat changed for each process (in milliseconds)
    static unsigned long last_change_time[5] = {0};

    // Initialize last_change_time on first run
    for (int i = 0; i < 5; ++i) {
        last_change_time[i] = get_time_ms();
        old[i] = 0;
    }

    while (1) {
        uint32_t new[5] = {
            shm_ptr->wdg.accel,
            shm_ptr->wdg.battery,
            shm_ptr->wdg.gps_ctrl,
            shm_ptr->wdg.telemetry,
            shm_ptr->wdg.flight_ctrl,
        };

        unsigned long now = get_time_ms();

        for (int i = 0; i < 5; ++i) {
            if (new[i] != old[i]) {
                last_change_time[i] = now;
            } else {
                if (now - last_change_time[i] >= WDG_TIMEOUT_MS) {
                    pid_t ppid = getppid();
                    printf("Process %d heartbeat timeout! Sending SIGUSR1 to parent %d\n", i, ppid);
                    kill(ppid, SIGUSR1);
                    return;
                }
            }
            old[i] = new[i];
        }

        usleep(WDG_SLEEP_US);
    }
}
