/** 
  * @file battery.c
  * @brief Battery charge sensor task. Controls drone's behaviour based on the current charge.
  *
  * Main tasks:
  * - Give battery charge information in range <0 ... 100> [%];
  * - Puts drone in `Abort` state when charge is < 15%.
  * - Charges the battery when in `Charge` state.
  * - Charge information is shared between processes via POSIX shared-memory API.
  *
  * @note
  *
  * This task is the only writer in the system that updates the battery-charge value.
  * The value is stored in an atomic variable, which makes single-writer / multiple-reader
  * access safe on all modern platforms that support C11 atomic operations, including all UNIX-like 
  * systems implementing the POSIX API.
  **/

#include "proj_types.h"

#define DISCHARGE_INTERVAL_MS 2000
#define CHARGE_INTERVAL_MS 500

static struct timespec last_time, now;
static bat_charge_t current_battery;
static current_action_t current_action;
static bool init = true;

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
void battery_loop(drone_shared_t *shm_ptr) {
    clock_gettime(CLOCK_MONOTONIC, &now);

    // One time initialization boolean.
    if (init) {
        last_time = now;
        init = false;
    }

    // Calculate elapsed time since last charge
    long elapsed_ms = (now.tv_sec - last_time.tv_sec) * 1000 
        + (now.tv_nsec - last_time.tv_nsec) / NANOSECONDS_IN_MS;

    // Battery data is always changed and read atomically.
    current_battery = atomic_load_explicit(&shm_ptr->battery, memory_order_acquire);

    rwlock_read_lock(&shm_ptr->action.lock);
    current_action = shm_ptr->action.type;
    rwlock_read_unlock(&shm_ptr->action.lock);

    if (current_action == Charge) {
        if (elapsed_ms >= CHARGE_INTERVAL_MS) {
            last_time = now;
            if (current_battery < 100)
                atomic_store_explicit(&shm_ptr->battery, current_battery + 1, memory_order_release);
            printf("Charging: Battery value (%u%%)\n", current_battery);
        }
    } else {
        if (elapsed_ms >= DISCHARGE_INTERVAL_MS) {
            last_time = now;
            if (current_battery > 0) {
                atomic_store_explicit(&shm_ptr->battery, current_battery - 1, memory_order_release);                    
                printf("Discharging: Battery value %u%%\n", current_battery);

                if (current_battery < 15 && current_action != Abort) {
                    printf("Battery low (%u%%). Switching to Abort state.\n", current_battery);

                    // Changing global drone state to `Abort`.
                    rwlock_write_lock(&shm_ptr->action.lock);
                    shm_ptr->action.type = Abort;
                    rwlock_write_unlock(&shm_ptr->action.lock);
                }
            } else {
                perror("Battery charge is 0. Hard system shutdown.");
                kill(0, SIGTERM);    // Kills all processes within the subsystem (including the parent).
            }
        }
    }

    shm_ptr->wdg.battery++;
    usleep(100);
}
