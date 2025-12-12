/** 
  * @file flight_ctrl.c
  * @brief Obtains data from other actors and controls drone motor accordingly.
  *
  * Main tasks:
  * - Controls internal state of the drone based on it's state command.
  * - Responds to operator's command via UDP (non-blocking). 
  *
  * @note
  *
  **/

#include "proj_types.h"

#define DELTA_SIMULATION_US 50000
#define DELTA_DECREASE      0.01f
#define DELTA_INCREASE      0.005f
#define MAGNITUDE_THRESH    1.0f
#define FLY_THRESH          0.7f
#define BIND_RETRY_MS       2000 

#define MAX_FLY_TIMEOUT     10

#define ACCEL_STABILIZATION_THRESHOLD 0.5f

static bat_charge_t current_battery;
static current_action_t last_action = Reserved;
static acceleration_t last_accel;
static bool init = true;

static int sockfd;
static struct sockaddr_in serveraddr;
static struct timespec now, last_time;

/**
  * @brief Tries to bind for operator's UDP traffic.
  **/
bool try_bind(drone_shared_t *shm_ptr) {
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        perror("socket");
        return false;
    }

    memset(&serveraddr, 0, sizeof(serveraddr));
    serveraddr.sin_family = AF_INET;

    if (inet_pton(AF_INET, shm_ptr->drone_ip, &serveraddr.sin_addr) <= 0) {
        perror("Wrong drone IP address value.");
        return false;
    }

    serveraddr.sin_port = htons(shm_ptr->flight_ctrl_port);

    // Binding for listening from operator.
    if (bind(sockfd, (struct sockaddr*)&serveraddr, sizeof(serveraddr)) < 0) {
        perror("Unable to bind for listening.");
        close(sockfd);
        return false;
    }

    // Getting current socket flags.
    int flags = fcntl(sockfd, F_GETFL, 0);
    if (flags < 0) {
        perror("fctrl");
        close(sockfd);
        return false;
    }

    // Making socket non-blocking.
    if (fcntl(sockfd, F_SETFL, flags | O_NONBLOCK) < 0) {
        perror("fcntl");
        close(sockfd);
        return false;
    }

    return true;
}

/**
  * @brief Flight controller loop function.
  *
  * Does the following:
  * - Mutates state based on data obtained from the operator.
  * - Mutates `motors[4]` based on accelerometer data and current state.
  * - Turns off `motors[4]` slowly and changes to `Charge` state, if battery is less than 15%.
  *
  **/
void flight_loop(drone_shared_t *shm_ptr) {
    static socklen_t len;
    static uint8_t fly_timeout = 0;
    motors_t tmp_m = {0};
    current_action_t current_action, operator_cmd = Reserved;
    float avg_pwm;
    ssize_t n;

    clock_gettime(CLOCK_MONOTONIC, &now);

    // Calculate elapsed time since last charge
    long elapsed_ms = (now.tv_sec - last_time.tv_sec) * 1000 
        + (now.tv_nsec - last_time.tv_nsec) / NANOSECONDS_IN_MS;

    if (init) {
        if (elapsed_ms >= BIND_RETRY_MS) {
            printf("Connection is not initialized. Trying to bind... ");
            if (try_bind(shm_ptr)) {
                printf("Socket bind complete..\n");
                init = false;
                len = sizeof(serveraddr);
                goto _binded;
            }
        }
        last_time = now;
    } else {
_binded:
        /* Trying to get new command from operator. This part is non-blocking. */
        n = recvfrom(sockfd, &operator_cmd, sizeof(operator_cmd), MSG_DONTWAIT, (struct sockaddr*)&serveraddr, &len);
        if (n < 0) {                                                // UDP receive error.
            if (errno == EWOULDBLOCK) {} else    // Doing nothing when no data can be read.
            if (errno == EAGAIN || errno == EINTR) { goto _binded; }                   // Socket read interrupted. Retrying.
            else { 
                // Communication error. Set to Abort state.
                rwlock_write_lock(&shm_ptr->action.lock);
                shm_ptr->action.type = Abort;
                rwlock_write_unlock(&shm_ptr->action.lock);
                perror("recvfrom"); 
                init = true; 
            }               
        } else if (n == sizeof(operator_cmd)) {                     // Received data!
            printf("Obtained command from operator: %d.\n", operator_cmd);
        }
    }

    rwlock_read_lock(&shm_ptr->action.lock);
    current_action = shm_ptr->action.type;
    rwlock_read_unlock(&shm_ptr->action.lock);

    if (current_action != last_action) {
        printf("Current state: ");
        printactln(current_action);
        last_action = current_action;
    }

    /* Mutating system state based on current action. */
    switch (current_action) {
        case Fly:       // Fly -> Read accelerometer data and adjust motors.
            // Calculate average motor PWM
            avg_pwm = 0.0f;

            sem_wait(&shm_ptr->pwm.mutex);
            tmp_m = shm_ptr->pwm.motors;
            sem_post(&shm_ptr->pwm.mutex);

            for (int i = 0; i < 4; ++i) {
                avg_pwm += tmp_m.motors[i];
            }
            avg_pwm /= 4.0f;

            // If PWM below threshold, start flying higher.
            if (avg_pwm < FLY_THRESH) {
                for (int i = 0; i < 4; ++i) {
                    tmp_m.motors[i] += DELTA_INCREASE;
                    if (tmp_m.motors[i] > 1.0f)
                        tmp_m.motors[i] = 1.0f;
                }
            }

            sem_wait(&shm_ptr->accel.mutex);
            acceleration_t accel = shm_ptr->accel.acceleration;
            sem_post(&shm_ptr->accel.mutex);

            // Stabilize when in air.
            if (avg_pwm >= ACCEL_STABILIZATION_THRESHOLD) {

                tmp_m.motors[0] -= accel.x + accel.y;
                tmp_m.motors[1] -= accel.x + accel.y;
                tmp_m.motors[2] -= accel.x + accel.y;
                tmp_m.motors[3] -= accel.x + accel.y;

                for (int i = 0; i < 4; ++i) {
                    if (tmp_m.motors[i] > 1.0f)
                        tmp_m.motors[i] = 1.0f;
                    if (tmp_m.motors[i] < 0.0f)
                        tmp_m.motors[i] = 0.0f;
                }
            }

            sem_wait(&shm_ptr->pwm.mutex);
            shm_ptr->pwm.motors = tmp_m;
            sem_post(&shm_ptr->pwm.mutex);

            if (
                    accel.x == last_accel.x && 
                    accel.y == last_accel.y && 
                    accel.z == last_accel.z
               ) {
                fly_timeout++;
                // If accelerometer data is not changing, switching to abort.
                if (fly_timeout >= MAX_FLY_TIMEOUT) {
                    fprintf(stderr, "Too much same accelerometer data. Unable to predict current drone movement. Aborting...");
                    rwlock_write_lock(&shm_ptr->action.lock);
                    shm_ptr->action.type = Abort;
                    rwlock_write_unlock(&shm_ptr->action.lock);
                    fly_timeout = 0;
                }
            } else {
                fly_timeout = 0;
            }
            last_accel = accel;

            if (operator_cmd & (SampleGPS | Land | Abort)) {
                rwlock_write_lock(&shm_ptr->action.lock);
                shm_ptr->action.type = operator_cmd;
                rwlock_write_unlock(&shm_ptr->action.lock);
            }
            break; 
        case SampleGPS: // SampleGPS -> Flags the telemetry unit to send newest NMEA data.
            if (operator_cmd & (Fly | Abort)) {
                rwlock_write_lock(&shm_ptr->action.lock);
                shm_ptr->action.type = operator_cmd;
                rwlock_write_unlock(&shm_ptr->action.lock);
            }
            break;
        case Idle:      // Idle -> Wait for command.
            if (operator_cmd & (Fly | Charge | Abort)) {
                rwlock_write_lock(&shm_ptr->action.lock);
                shm_ptr->action.type = operator_cmd;
                rwlock_write_unlock(&shm_ptr->action.lock);
            }
            break;
        case Charge:    // Charge -> Ignore commands until at least charged above 15%. Otherwise change command.
            if (operator_cmd & (Idle | Abort)) {
                current_battery = atomic_load_explicit(&shm_ptr->battery, memory_order_acquire);
                if (current_battery >= 15) {
                    // Battery sufficiently charged, allow operator commands
                    rwlock_write_lock(&shm_ptr->action.lock);
                    shm_ptr->action.type = operator_cmd;
                    rwlock_write_unlock(&shm_ptr->action.lock);
                } else {
                    printf("Charging: Battery below 15%%, ignoring operator commands.\n");
                };
            }
            break;
        case Abort:     // Abort -> Turn off the motors and land with additional prerequisites. This state specially ignores operator commands.
            // Maybe charge is less than 15%
            current_battery = atomic_load_explicit(&shm_ptr->battery, memory_order_acquire);
            if (current_battery < 15) {
                // If idle on ground, charging immediately.
                rwlock_write_lock(&shm_ptr->action.lock);
                shm_ptr->action.type = Charge;
                rwlock_write_unlock(&shm_ptr->action.lock);
                break;
            } else {
                printf("Changing to previous action.");
                rwlock_write_lock(&shm_ptr->action.lock);
                shm_ptr->action.type = last_action;
                rwlock_write_unlock(&shm_ptr->action.lock);
            }
        case Land:      // Land -> Turn off the motors and land.
            if (operator_cmd & (Fly | Abort)) {
                rwlock_write_lock(&shm_ptr->action.lock);
                shm_ptr->action.type = operator_cmd;
                rwlock_write_unlock(&shm_ptr->action.lock);
                break;
            }

            sem_wait(&shm_ptr->pwm.mutex);
            float avg = 0;

            // Decreasing PWM for each motor.
            for (int i = 0; i < 4; ++i) {
                float current = shm_ptr->pwm.motors.motors[i]; 
                current -= DELTA_DECREASE;
                if (current < 0.) {
                    current = 0.;
                }

                avg += current;
                shm_ptr->pwm.motors.motors[i] = current;
            }
            avg /= 4;
            printf("Landing: Average motor PWM: %f%%.\n", avg);

            if (avg == 0.) {    // Changing to idle when landed.
                rwlock_write_lock(&shm_ptr->action.lock);
                if (current_action == Abort) {
                    printf("Landing while Abort: Set to Charge.\n");
                    shm_ptr->action.type = Charge;
                } else {
                    printf("Landing: Set to Idle.\n");
                    shm_ptr->action.type = Idle;
                }
                rwlock_write_unlock(&shm_ptr->action.lock);
            }

            sem_post(&shm_ptr->pwm.mutex);

            break;
        default:
            fprintf(stderr, "Unexpected state value obtained: %d. Switching to `Abort` due to undefined behavior.\n.", current_action);
            rwlock_write_lock(&shm_ptr->action.lock);
            shm_ptr->action.type = Abort;
            rwlock_write_unlock(&shm_ptr->action.lock);
    }

    shm_ptr->wdg.flight_ctrl++;
    usleep(DELTA_SIMULATION_US);
}
