/** 
  * @file telemetry.c
  * @brief Connects to operator and sends all sensor data via TCP.
  *
  * Main tasks:
  *  - Connect to operator TCP server (IP/port from shared memory).
  *  - Periodically send battery, accel, and action state.
  *
  */

#include "proj_types.h"

#define TELEMETRY_TIMEOUT_US    10000
#define TELEMETRY_BUF_SIZE      512 
#define CONNECTION_TIMEOUT_MS   10000
#define GPS_WAIT_TIMEOUT_S      5

#define PERCENT(f) (int)(f * 100 + 0.5f)

static int sock_fd = -1;
static bool init = true;

/**
  * @brief Tries to connect to operator's TCP server via data stored in shared memory. 
  **/
static bool try_connect(drone_shared_t *shm_ptr) {
    struct sockaddr_in op_addr;

    if (sock_fd != -1)
        close(sock_fd);

    sock_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (sock_fd < 0) {
        perror("telemetry socket()");
        return false;
    }

    memset(&op_addr, 0, sizeof(op_addr));
    op_addr.sin_family      = AF_INET;
    op_addr.sin_port        = htons(shm_ptr->telemetry_port);

    if (inet_pton(AF_INET, shm_ptr->operator_ip, &op_addr.sin_addr) <= 0) {
        perror("telemetry inet_pton");
        close(sock_fd);
        sock_fd = -1;
        return false;
    }

    printf("Trying TCP connect to %s:%d...\n",
           shm_ptr->operator_ip,
           shm_ptr->telemetry_port);

    if (connect(sock_fd, (struct sockaddr*)&op_addr, sizeof(op_addr)) < 0) {
        perror("Telemetry connect");
        close(sock_fd);
        sock_fd = -1;
        return false;
    }

    printf("Connected to operator.\n");
    return true;
}

/* Helper function to fill TCP message buffer. */
#define BUF_APPEND(msg, ptr, fmt, ...)                                                                          \
    do {                                                                                                        \
        if ((ptr) < sizeof(msg)) {                                                                              \
            int _written = snprintf((msg) + (ptr), sizeof(msg) - (ptr), fmt "\n", ##__VA_ARGS__);               \
            if (_written > 0) {                                                                                 \
                (ptr) += (_written < (int)(sizeof(msg) - (ptr)) ? _written : (int)(sizeof(msg) - (ptr) - 1));   \
            }                                                                                                   \
        }                                                                                                       \
    } while (0)

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
void telemetry_loop(drone_shared_t *shm_ptr) {
    char msg[256] = {0};
    size_t ptr = 0;
    bat_charge_t battery;
    acceleration_t accel;
    current_action_t action;
    motors_t m;

    if (init) {
        if (try_connect(shm_ptr))
            init = false;
        else
            goto _wdg;
    }

    battery = atomic_load_explicit(&shm_ptr->battery, memory_order_acquire);
    BUF_APPEND(msg, ptr, "BAT = %d%%", battery);

    if (sem_trywait(&shm_ptr->accel.mutex) == 0) {
        accel = shm_ptr->accel.acceleration;
        sem_post(&shm_ptr->accel.mutex);
        BUF_APPEND(msg, ptr, "ACCEL = (x: %.6f, y: %.6f, z: %.6f)", accel.x, accel.y, accel.z);
    }

    if (sem_trywait(&shm_ptr->pwm.mutex) == 0) {
        m = shm_ptr->pwm.motors;
        sem_post(&shm_ptr->pwm.mutex);
        BUF_APPEND(msg, ptr, "MOTORS PWM = [%d%%, %d%%, %d%%, %d%%]", 
            PERCENT(m.motors[0]),
            PERCENT(m.motors[1]),
            PERCENT(m.motors[2]),
            PERCENT(m.motors[3])
        );
    }

    rwlock_read_lock(&shm_ptr->action.lock);
    action = shm_ptr->action.type;
    rwlock_read_unlock(&shm_ptr->action.lock);
    BUF_APPEND(msg, ptr, "ACTION = %d", action);

    // Telemetry unit is the consumer for GPS data.
    if (action == SampleGPS) {
        BUF_APPEND(msg, ptr, "GPS {\n");
        printf("Obtained GPS strings: ");

        while (ptr < sizeof(msg) - 2) {
            char c;
            struct timespec ts;

            clock_gettime(CLOCK_MONOTONIC, &ts);
            ts.tv_sec += GPS_WAIT_TIMEOUT_S;

            // Wait with timeout for new GPS data
            if (sem_timedwait(&shm_ptr->gps.full, &ts) == -1) {
                if (errno == ETIMEDOUT) {
                    printf("\n[GPS timeout: no new data]\n");
                    BUF_APPEND(msg, ptr, "NO FIX.");

                    // Set to abort state.
                    rwlock_write_lock(&shm_ptr->action.lock);
                    shm_ptr->action.type = Abort;
                    rwlock_write_unlock(&shm_ptr->action.lock);

                    break;
                } else {
                    perror("sem_timedwait(full)");
                    break;
                }
            }
            if (sem_timedwait(&shm_ptr->gps.mutex, &ts) == -1) {
                perror("sem_timedwait(mutex)");
                sem_post(&shm_ptr->gps.full);
                break;
            }

            // Read single char
            c = shm_ptr->gps.nmea.buf[shm_ptr->gps.read];
            msg[ptr++] = c;
            shm_ptr->gps.read = (shm_ptr->gps.read + 1) % GPS_BUFFER_SIZE;

            sem_post(&shm_ptr->gps.mutex);
            sem_post(&shm_ptr->gps.empty);

            if (c == '\n')
                break;
        }

        BUF_APPEND(msg, ptr, "\n}");
        printf("\n");
    }

    // Sends the message via connected TCP socket.
    int n = send(sock_fd, msg, ptr, MSG_NOSIGNAL);  // MSG_NOSIGNAL prevents SIGPIPE when operator crashes during communication.
    if (n <= 0) {
        fprintf(stderr, "Telemetry send failed, connection lost\n");
        close(sock_fd);
        sock_fd = -1;
        init = true;
        return;
    }

_wdg:
    shm_ptr->wdg.telemetry++;
    usleep(TELEMETRY_TIMEOUT_US);
}
