/** 
  * @file drone_sys.c
  * @brief Initializes shared memory region and spawns actors. Controls the integrity of the whole system.
  *
  * Main tasks:
  * - Parse input arguments to obtain IPs and ports.
  * - Initialize shared memory region and memory maps it.
  * - Spawns children subprocesses. Controls their lifecycle by respawning them when killed.
  *
  * @note
  *
  * All modifications made to shared memory by main process is done before forking. That data shall be considered
  * as read-only afterwards, therefore it needs no synchronization between processes.
  **/

#include "proj_types.h"
#include "actors.h"

/**  
  * Shared memory file descriptor and memory mapped pointer are global, but a whole duplicate will be created for each child, 
  * that must close it after some error.
  **/ 
int shm_fd;
drone_shared_t *shm_ptr;

// SIGTERM Flag.
volatile sig_atomic_t sigterm = 0;
// SIGCHLD Flag. Used to restart crashed children.
volatile sig_atomic_t sigchld = 0;

/**
  * @brief Spawns actor by forking current program and starting required main loop.
  *
  * Does the following:
  * - Forks process to create child subprocess;
  * - Changes it's name, prepares log files and redirects output;
  * - Enters child's main loop;
  * - Parent continues by leaving the function;
  **/
pid_t spawn_actor(
    void (*main_loop)(drone_shared_t *shm_ptr), 
    drone_shared_t *dsptr, 
    char *name
) {
    pid_t pid = fork();

    if (pid < 0) {
        perror("fork");
        return -1;
    }

    // If child - starts the main loop.
    if (pid == 0) {
        char file[64];
        int fd;

        prctl(PR_SET_NAME, name, 0, 0, 0);          // Swapping child name.

        snprintf(file, sizeof(file), "./build/%s.log", name);   // Preparing log file for child. 
        fd = open(file, O_WRONLY | O_CREAT | O_TRUNC, 0666);    // Creating | opening for writing.
        dup2(fd, STDOUT_FILENO);                                // Redirecting STDOUT.
        dup2(fd, STDERR_FILENO);                                // Redirecting STDERR.
        close(fd);

        while(!sigterm) {
            main_loop(dsptr);                       // Performing child loop iteration.
        }

        munmap(dsptr, sizeof(drone_shared_t));
        close(shm_fd);
        _exit(0);
    }

    printf("Spawned child task with PID: [%d] of type: \"%s\".\n", pid, name);

    // Return child's PID
    return pid;
}

/**
  * @brief SIGCHDL handler.
  *
  * Restarts each child that crashed.
  **/
static void sigchld_handler(int _) {
    printf("SIGCHLD: Cleaning zombies...\n");
    sigchld = 1;
}

/**
  * @brief SIGTERM handler.
  *
  * Stops the entire system. Cleanups the resources.
  **/
static void sigterm_handler(int _) {
    printf("SIGTERM: Exiting...\n");
    sigterm = 1;
}

/**
  * @brief Used to init default drone lock values.
  *
  * @note Only first process that creates a shared memory shall initialize it's contents.
  **/
static void init_locks_shm(drone_shared_t *ptr) {
    // Initialization of synchronization primitives.
    rwlock_init(&ptr->action.lock);

    sem_init(&ptr->accel.mutex, 1, 1);  // One access at a time.
    sem_init(&ptr->pwm.mutex, 1, 1);    // One access at a time.

    sem_init(&ptr->gps.mutex, 1, 1);                // One access to critical section at a time. Started at 0 and appended when state is changed to `SampleGPS`.
    sem_init(&ptr->gps.empty, 1, GPS_BUFFER_SIZE);  // Initially all buffer slots are empty.
    sem_init(&ptr->gps.full, 1, 0);                 // Initially zero buffer slots are full.
}

/**
  * @brief Used to init default drone values within the shared memory region.
  *
  * @note Only first process that creates a shared memory shall initialize it's contents.
  **/
static void init_drone_shm(drone_shared_t *ptr) {
    // Makes sures that the whole memory region is zeroed.
    memset(ptr, 0, sizeof(drone_shared_t));

    // Default init values.
    ptr->battery = 100;
    ptr->action.type = Idle;
    ptr->accel.acceleration.x = ptr->accel.acceleration.y = ptr->accel.acceleration.z = 0.0f;

    init_locks_shm(ptr);
}

/**
  * @brief SIGUSR1 handler.
  *
  * Reinitis locks, respawns children.
  **/
static void sigusr1_handler(int _) {
    printf("SIGUSR1: Watchdog detected a deadlock...\n");

    kill(shm_ptr->pids.accel, SIGTERM);
    kill(shm_ptr->pids.battery, SIGTERM);
    kill(shm_ptr->pids.gps_ctrl, SIGTERM);
    kill(shm_ptr->pids.telemetry, SIGTERM);
    kill(shm_ptr->pids.flight_ctrl, SIGTERM);
    kill(shm_ptr->pids.wdg, SIGTERM);

    init_locks_shm(shm_ptr);
}

/**
  * @brief Main application entry point. 
  *
  * Does the following:
  * - Checks if a drone shared memory interface exists within the OS. If not, creates one.
  * - Creates SIGINT handler for clearing resources.
  * - Spawns dedicated actors as children.
  *
  **/
int main(int argc, char **argv) {
    struct sigaction sa;
    int created = 0, ret = 0;

    if (argc < 5) {
        fprintf(stderr, "Usage: %s <telemetry_ip> <telemetry_port> <drone_ip> <flight_ctrl_port>\n", argv[0]);
        return 1;
    }

    printf("SHM open...\n");

    /* Opens or creates shared memory object. */
    shm_fd = shm_open(SHM_NAME, O_RDWR | O_CREAT | O_EXCL, 0660);
    if (shm_fd < 0) {
        if (errno == EEXIST) {  // Already exists.
            shm_fd = shm_open(SHM_NAME, O_RDWR, 0);
            if (shm_fd < 0) {
                perror("shm_open existing");
                ret = 1;
                goto _shm_unlink;
            }
        } else {
            perror("shm_open");
            ret = 1;
            goto _shm_unlink;
        }
    } else {
        created = 1;
        // Truncating shared region size.
        if (ftruncate(shm_fd, sizeof(drone_shared_t)) < 0) {
            perror("ftruncate");
            ret = 1;
            goto _shm_close;
        }
    }

    printf("MMAP...\n");

    /* Memory mapping shared memory region. */
    shm_ptr = mmap(NULL, sizeof(drone_shared_t), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (shm_ptr == MAP_FAILED) {
        perror("mmap");
        ret = 1;
        goto _shm_close;
    }

    /* Initializing if we are the first process to init shared memory. */ 
    if (created)
        init_drone_shm(shm_ptr);

    /* Network related parameters are stored in SHM. */
    strncpy(shm_ptr->operator_ip, argv[1], INET_ADDRSTRLEN);
    shm_ptr->operator_ip[INET_ADDRSTRLEN - 1] = 0;
    shm_ptr->telemetry_port  = (uint16_t)atoi(argv[2]);

    strncpy(shm_ptr->drone_ip, argv[3], INET_ADDRSTRLEN);
    shm_ptr->drone_ip[INET_ADDRSTRLEN - 1] = 0;
    shm_ptr->flight_ctrl_port = (uint16_t)atoi(argv[4]);

    printf("Config stored in SHM: ip=%s tp=%u fp=%u\n",
        shm_ptr->operator_ip,
        shm_ptr->telemetry_port,
        shm_ptr->flight_ctrl_port
    );

    printf("Define SIGTERM handler...\n");
    /* Declaring SIGTERM handler. */
    sa.sa_handler = sigterm_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;


    if (sigaction(SIGTERM, &sa, NULL) == -1) {
        perror("sigaction");
        goto _shm_munmap;
    }
    
    printf("Define SIGINT handler...\n");

    // Does the same as SIGTERM.
    if (sigaction(SIGINT, &sa, NULL) == -1) {
        perror("sigaction");
        goto _shm_munmap;
    }

    printf("Spawning children processes.\n");

    /* Forking children */
    shm_ptr->pids.battery = spawn_actor(battery_loop, shm_ptr, "BATTERY");
    shm_ptr->pids.accel = spawn_actor(accel_loop, shm_ptr, "ACCELEROMETER");
    shm_ptr->pids.gps_ctrl = spawn_actor(gps_loop, shm_ptr, "GPS");
    shm_ptr->pids.flight_ctrl = spawn_actor(flight_loop, shm_ptr, "CTRL");
    shm_ptr->pids.telemetry = spawn_actor(telemetry_loop, shm_ptr, "TELEMETRY");
    shm_ptr->pids.wdg = spawn_actor(watchdog_loop, shm_ptr, "WATCHDOG");

    printf("Define SIGCHLD handler...\n");

    /* Declaring SIGCHLD handler. */
    sa.sa_handler = sigchld_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = SA_RESTART | SA_NOCLDSTOP;

    if (sigaction(SIGCHLD, &sa, NULL) < 0) {
        perror("sigaction");
        goto _shm_munmap;
    }

    /* Declaring SIGUSR1 handler. */
    sa.sa_handler = sigusr1_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = SA_RESTART | SA_NOCLDSTOP;

    if (sigaction(SIGUSR1, &sa, NULL) < 0) {
        perror("sigaction");
        goto _shm_munmap;
    }

    // Handling signals.
    for (;;) {
        // Restarting child.
        if (sigchld) {
            int cpid;
            while ((cpid = waitpid(-1, NULL, WNOHANG)) > 0) {
                printf("Child crashed with PID: %d, of type: ", cpid);

                if (cpid == shm_ptr->pids.accel) {
                    printf("ACCELEROMETER\n");
                    shm_ptr->pids.accel = spawn_actor(accel_loop, shm_ptr, "ACCELEROMETER"); 
                } else if (cpid == shm_ptr->pids.battery) {
                    printf("BATTERY\n");
                    shm_ptr->pids.battery = spawn_actor(battery_loop, shm_ptr, "BATTERY"); 
                } else if (cpid == shm_ptr->pids.gps_ctrl) {
                    printf("GPS\n");
                    shm_ptr->pids.gps_ctrl = spawn_actor(gps_loop, shm_ptr, "GPS"); 
                } else if (cpid == shm_ptr->pids.flight_ctrl) {
                    shm_ptr->pids.flight_ctrl = spawn_actor(flight_loop, shm_ptr, "CTRL");
                } else if (cpid == shm_ptr->pids.telemetry) {
                    shm_ptr->pids.telemetry = spawn_actor(telemetry_loop, shm_ptr, "TELEMETRY");
                } else if (cpid == shm_ptr->pids.wdg) {
                    shm_ptr->pids.wdg = spawn_actor(watchdog_loop, shm_ptr, "WATCHDOG");
                } else {
                    fprintf(stderr, "Unmarked PID child dead.\n");
                }
            }
        }

        // Exiting with cleanup.
        if (sigterm)
            break;

        printf("Pausing the main process.\n");
        pause();
    }

    // Terminate all children when shutting down. SIGTERM allows them to clean resources.
    killpg(getpgrp(), SIGTERM);

    // End cleanup.
_shm_munmap:
    munmap(shm_ptr, sizeof(drone_shared_t));
_shm_close:
    close(shm_fd);
_shm_unlink:
    shm_unlink(SHM_NAME);

    return ret;
}

/**
  * @brief Initialization for RWLock.
  **/
void rwlock_init(rw_lock_t *rwlock) {
    sem_init(&(rwlock->read), 1, 1);
    sem_init(&(rwlock->write), 1, 1);
    rwlock->read_counter = 0;
}

/**
  * @brief Implementation of RWLock reader lock algorithm.
  **/
void rwlock_read_lock(rw_lock_t *rwlock) {
    sem_wait(&rwlock->read);            // Enters critical section here.
    rwlock->read_counter++;             // -- Counter hidden behind critical section.
    if (rwlock->read_counter == 1)      // -- Only first reader locks writers. This also locks readers, if writers are already locked.
        sem_wait(&rwlock->write);
    sem_post(&rwlock->read);            // Leave critical section here.
}

/**
  * @brief Implementation of RWLock reader unlock algorithm.
  **/
void rwlock_read_unlock(rw_lock_t *rwlock) {
    sem_wait(&rwlock->read);            // Enters critical section here.
    (rwlock->read_counter)--;           // -- Counter hidden behind critical section.
    if (rwlock->read_counter == 0)      // -- Only last reader frees writers.
        sem_post(&rwlock->write);
    sem_post(&rwlock->read);            // Leave critical section here.
}

/**
  * @brief Implementation of RWLock writer lock algorithm.
  **/
void rwlock_write_lock(rw_lock_t *rwlock) {
    sem_wait(&rwlock->write);           // Enters critical section here. Or waits if any readers are left or other writers.
}

/**
  * @brief Implementation of RWLock writer unlock algorithm.
  **/
void rwlock_write_unlock(rw_lock_t *rwlock) {
    sem_post(&rwlock->write);           // Leave critical section here.
}
