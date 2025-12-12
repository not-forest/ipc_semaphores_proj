/** 
  * @file gps_ctrl.c
  * @brief Sends GPS NMEA string data via circular buffer.
  *
  * Main tasks:
  * - Send NMEA string data each second via circular buffer (producer).
  * - Only types new data when buffer is not full (consumer obtains data). Only happen when state is `SampleGPS`.
  *
  * @note
  **/

#include "proj_types.h"

#define NMEA_COUNT (sizeof(nmea_samples)/sizeof(nmea_samples[0]))

/* Simulation samples. Those are being sent in a loop. */
static const char *nmea_samples[] = {
    "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\n",
    "$GPGSA,A,3,04,05,09,12,24,25,29,30,31,,,1.8,1.0,1.5*33\n",
    "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\n",
    "$GPVTG,084.4,T,003.1,M,022.4,N,041.4,K*1F\n"
};

static int sample_index = 0;
static size_t nmea_samples_count = sizeof(nmea_samples) / sizeof(nmea_samples[0]);

/**
  * @brief Main GPS loop function.
  *
  * Does the following:
  * - Acts as a producer that writes upcoming NMEA strings to shared circular buffer.
  **/
void gps_loop(drone_shared_t *shm_ptr) {
    const char *msg = nmea_samples[sample_index];
    size_t count = 0;

    printf("Writing: ");

    while (count < strlen(msg)) {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        ts.tv_sec += 1;

        int s = sem_timedwait(&shm_ptr->gps.empty, &ts);
        if (s == -1) {
            if (errno == ETIMEDOUT)
                goto _wdg;
            else
                perror("sem_timedwait");
        } else {
            sem_wait(&shm_ptr->gps.mutex);

            shm_ptr->gps.nmea.buf[shm_ptr->gps.write] = msg[count];

            shm_ptr->gps.write = (shm_ptr->gps.write + 1) % GPS_BUFFER_SIZE;

            sem_post(&shm_ptr->gps.mutex);
            sem_post(&shm_ptr->gps.full);

            count++;
        }
    }

    sample_index = (sample_index + 1) % nmea_samples_count;

_wdg:
    shm_ptr->wdg.gps_ctrl++;
    sleep(1);
}
