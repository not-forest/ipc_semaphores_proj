/**
 * @file operator.c
 * @brief Operator console application for communicating with the drone.
 *
 * Main tasks:
 * - Telemetry: TCP server on operator side.
 * - Flight controller: UDP command sender
 */

#include "proj_types.h"

// SIGTERM Flag.
volatile sig_atomic_t sigterm = 0;

/**
  * @brief Converts obtained string to lower case.
  **/
static inline void str_tolower(char *s) {
    for (; *s; s++) *s = tolower((unsigned char)*s);
}

/**
  * @brief Converts obtained input string sequence into a drone command. 
  **/
static int get_action_from_cmd(const char *cmd, current_action_t *out_action) {
    char tmp[32];
    strncpy(tmp, cmd, sizeof(tmp) - 1);
    tmp[sizeof(tmp) - 1] = 0;

    // Removes trailing newline.
    size_t len = strlen(tmp);
    if (len > 0 && tmp[len - 1] == '\n') {
        tmp[len - 1] = 0;
    }

    str_tolower(tmp);

    if (strcmp(tmp, "samplegps") == 0) { *out_action = SampleGPS; return 1; }
    if (strcmp(tmp, "fly") == 0)       { *out_action = Fly; return 1; }
    if (strcmp(tmp, "land") == 0)      { *out_action = Land; return 1; }
    if (strcmp(tmp, "idle") == 0)      { *out_action = Idle; return 1; }
    if (strcmp(tmp, "charge") == 0)    { *out_action = Charge; return 1; }
    if (strcmp(tmp, "abort") == 0)     { *out_action = Abort; return 1; }

    return 0;
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
  * @brief Separate operator binary entry point.
  *
  * Does the following:
  * - Parses input arguments to set IP address and separate ports for telemetry and flight control.
  * - Prepares TCP server to communicate with drone for telemetry messages.
  * - Sends UDP messages to drone.
  *
  **/
int main(int argc, char **argv) {
    struct sigaction sa;
    struct sockaddr_in tel_addr, fc_addr, drone_addr;
    socklen_t drone_addr_len = sizeof(drone_addr);

    char buf[1024];
    int telemetry_listen_fd = -1;
    int telemetry_fd = -1;
    int udp_fd = -1;
    int n;
    int ret = 0;

    if (argc < 5) {
        fprintf(stderr,
            "Usage: %s <operator_ip> <telemetry_unit_port> <drone_ip> <flight_ctrl_port>\n",
            argv[0]
        );
        return 1;
    }

    printf("Starting operator console...\n");

    // Parsing operator's IP + port.
    memset(&tel_addr, 0, sizeof(tel_addr));
    if (inet_pton(AF_INET, argv[1], &tel_addr.sin_addr) <= 0) {
        perror("Bad operator IP address");
        ret = 1;
        goto _shutdown;
    }
    tel_addr.sin_family = AF_INET;
    tel_addr.sin_port   = htons(atoi(argv[2]));
    printf("Telemetry IP/port parsed.\n");

    // Parsing drone's IP + port.
    telemetry_listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (telemetry_listen_fd < 0) {
        perror("socket(TCP)");
        ret = 1;
        goto _shutdown;
    }

    // TCP bind.
    if (bind(telemetry_listen_fd, (struct sockaddr*)&tel_addr, sizeof(tel_addr)) < 0) {
        perror("bind(TCP)");
        ret = 1;
        goto _shutdown;
    }

    // TCP listen.
    if (listen(telemetry_listen_fd, 1) < 0) {
        perror("listen(TCP)");
        ret = 1;
        goto _shutdown;
    }
    printf("Telemetry TCP listener created.\n");

    // UDP socket.
    udp_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_fd < 0) {
        perror("socket(UDP)");
        ret = 1;
        goto _shutdown;
    }

    memset(&fc_addr, 0, sizeof(fc_addr));
    if (inet_pton(AF_INET, argv[3], &fc_addr.sin_addr) <= 0) {
        perror("Bad flight controller IP");
        ret = 1;
        goto _shutdown;
    }
    fc_addr.sin_family = AF_INET;
    fc_addr.sin_port   = htons(atoi(argv[4]));
    printf("UDP socket ready for Flight Controller commands.\n");

    // SIGINT | SIGTERM handlers.
    sa.sa_handler = sigterm_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;

    if (sigaction(SIGTERM, &sa, NULL) == -1 ||
        sigaction(SIGINT,  &sa, NULL) == -1) {
        perror("sigaction");
        ret = 1;
        goto _shutdown;
    }

    printf("Signal handlers installed.\n");

    fd_set rfds;

    while (!sigterm) {
        FD_ZERO(&rfds);
        FD_SET(telemetry_listen_fd, &rfds);
        FD_SET(STDIN_FILENO, &rfds);

        int maxfd = telemetry_listen_fd;
        if (telemetry_fd > 0) {
            FD_SET(telemetry_fd, &rfds);
            if (telemetry_fd > maxfd)
                maxfd = telemetry_fd;
        }

        int sel = select(maxfd+1, &rfds, NULL, NULL, NULL);
        if (sel < 0) {
            if (errno == EINTR)
                continue;
            perror("select");
            break;
        }

        /* Handle new telemetry connections. */
        if (FD_ISSET(telemetry_listen_fd, &rfds)) {
            telemetry_fd = accept(telemetry_listen_fd,
                                  (struct sockaddr*)&drone_addr,
                                  &drone_addr_len);
            if (telemetry_fd < 0) {
                perror("accept");
            } else {
                printf("Telemetry client connected.\n");
            }
        }

        /* Printing upcoming telemetry data. */
        if (telemetry_fd > 0 && FD_ISSET(telemetry_fd, &rfds)) {
            n = read(telemetry_fd, buf, sizeof(buf)-1);
            if (n > 0) {
                buf[n] = 0;
                printf("[TELEMETRY] {\n%s}\n", buf);
            } else {
                printf("Telemetry disconnected.\n");
                shutdown(telemetry_fd, SHUT_RDWR);
                close(telemetry_fd);
                telemetry_fd = -1;
            }
        }

        /* Sends UDP (non-blocking) when STDIN buffer has characters. */
        if (FD_ISSET(STDIN_FILENO, &rfds)) {
            char cmdline[32];
            if (fgets(cmdline, sizeof(cmdline), stdin)) {
                current_action_t action;
                if (get_action_from_cmd(cmdline, &action)) {
                    printactln(action);
                    ssize_t sent = 
                        sendto(udp_fd, &action, sizeof(action), 0, (struct sockaddr*)&fc_addr, sizeof(fc_addr));

                    if (sent == sizeof(action))
                        printf("Sent command '%s' via UDP.\n", cmdline);
                    else
                        perror("sendto");
                } else {
                    printf("Invalid command: %s", cmdline);
                    printf("Valid: fly, samplegps, land, idle, charge, abort\n");
                }
            }
        }
    }

    printf("\nShutting down cleanly...\n");

_shutdown:
    /* Close UDP socket */
    if (udp_fd >= 0) {
        close(udp_fd);
        udp_fd = -1;
    }

    /* Close telemetry client */
    if (telemetry_fd >= 0) {
        shutdown(telemetry_fd, SHUT_RDWR);
        close(telemetry_fd);
        telemetry_fd = -1;
    }

    /* Close telemetry listener */
    if (telemetry_listen_fd >= 0) {
        shutdown(telemetry_listen_fd, SHUT_RDWR);
        close(telemetry_listen_fd);
        telemetry_listen_fd = -1;
    }

    printf("All sockets closed. Exiting.\n");
    return ret;
}
