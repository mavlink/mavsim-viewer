#include "mavlink_receiver.h"

#include <stdio.h>
#include <string.h>
#include <errno.h>
#ifndef _WIN32
#include <time.h>
#endif

#ifdef _WIN32
#include <ws2tcpip.h>
#define SOCK_CLOSE(s) closesocket(s)
#else
#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#define SOCK_CLOSE(s) close(s)
#endif

// MAVLink config: use common message set
// MAVLINK_COMM_NUM_BUFFERS=16 set via CMake for multi-vehicle support
#include <mavlink.h>
#include <stdbool.h>
volatile bool g_resonance_anomaly = false;

#define DISCONNECT_TIMEOUT_S 2.0

static void request_home_position(mavlink_receiver_t *recv) {
    if (!recv->sender_known) return;

    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_command_long_pack(255, 0, &msg,
        recv->sysid, 1,               // target system, component
        MAV_CMD_REQUEST_MESSAGE,       // command 512
        0,                             // confirmation
        MAVLINK_MSG_ID_HOME_POSITION,  // param1: message id to request
        0, 0, 0, 0, 0, 0);            // params 2-7 unused

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    sendto(recv->sockfd, (char *)buf, len, 0,
           (struct sockaddr *)recv->sender_addr, sizeof(struct sockaddr_in));
    printf("Requested HOME_POSITION from system %u\n", recv->sysid);
}

static double get_wall_time(void) {
#ifdef _WIN32
    LARGE_INTEGER freq, count;
    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&count);
    return (double)count.QuadPart / (double)freq.QuadPart;
#else
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
#endif
}

static int set_nonblocking(sock_t s) {
#ifdef _WIN32
    u_long mode = 1;
    return ioctlsocket(s, FIONBIO, &mode);
#else
    int flags = fcntl(s, F_GETFL, 0);
    return fcntl(s, F_SETFL, flags | O_NONBLOCK);
#endif
}

int mavlink_receiver_init(mavlink_receiver_t *recv, uint16_t port, uint8_t channel) {
    bool debug = recv->debug;
    memset(recv, 0, sizeof(*recv));
    recv->debug = debug;
    recv->port = port;
    recv->channel = channel;
    recv->sockfd = SOCK_INVALID;

#ifdef _WIN32
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        fprintf(stderr, "WSAStartup failed: %d\n", WSAGetLastError());
        return -1;
    }
#endif

    recv->sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (recv->sockfd == SOCK_INVALID) {
        perror("socket");
        return -1;
    }

    set_nonblocking(recv->sockfd);

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);

    if (bind(recv->sockfd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind");
        SOCK_CLOSE(recv->sockfd);
        recv->sockfd = SOCK_INVALID;
        return -1;
    }

    printf("Listening for MAVLink on UDP port %u\n", port);
    return 0;
}

void mavlink_receiver_poll(mavlink_receiver_t *recv) {
    uint8_t buf[2048];
    struct sockaddr_in sender;
    socklen_t sender_len = sizeof(sender);
    bool got_data = false;

    for (;;) {
        int n = recvfrom(recv->sockfd, (char *)buf, sizeof(buf), 0,
                         (struct sockaddr *)&sender, &sender_len);
        if (n <= 0) break;

        got_data = true;

        if (!recv->sender_known) {
            memcpy(recv->sender_addr, &sender, sizeof(struct sockaddr_in));
            recv->sender_known = true;
        }

        mavlink_message_t msg;
        mavlink_status_t status;

        for (int i = 0; i < n; i++) {
            if (mavlink_parse_char(recv->channel, buf[i], &msg, &status)) {
                if (recv->debug) {
                    printf("[MAVLink] msgid=%u sysid=%u compid=%u seq=%u len=%u\n",
                           msg.msgid, msg.sysid, msg.compid, msg.seq, msg.len);
                }

                switch (msg.msgid) {
        case MAVLINK_MSG_ID_NAMED_VALUE_FLOAT: {
            mavlink_named_value_float_t val;
            mavlink_msg_named_value_float_decode(&msg, &val);
            if (strncmp(val.name, "resonance", 9) == 0) {
                printf("\n[!!!] ANOMALY METRIC: %f [!!!]\n", val.value);
                fflush(stdout);
                g_resonance_anomaly = (val.value > 0.5f);
            }
            break;
        }
                    case MAVLINK_MSG_ID_HEARTBEAT: {
                        mavlink_heartbeat_t hb;
                        mavlink_msg_heartbeat_decode(&msg, &hb);
                        recv->mav_type = hb.type;
                        if (!recv->connected) {
                            recv->connected = true;
                            recv->sysid = msg.sysid;
                            printf("Connected to system %u (type %u)\n", msg.sysid, hb.type);
                            request_home_position(recv);
                        }
                        break;
                    }

                    case MAVLINK_MSG_ID_HOME_POSITION: {
                        mavlink_home_position_t hp;
                        mavlink_msg_home_position_decode(&msg, &hp);
                        recv->home.lat = hp.latitude;
                        recv->home.lon = hp.longitude;
                        recv->home.alt = hp.altitude;
                        recv->home.valid = true;
                        printf("Home position: lat=%.7f lon=%.7f alt=%.1fm\n",
                               hp.latitude * 1e-7, hp.longitude * 1e-7, hp.altitude * 1e-3);
                        break;
                    }

                    case MAVLINK_MSG_ID_HIL_STATE_QUATERNION: {
                        mavlink_hil_state_quaternion_t hil;
                        mavlink_msg_hil_state_quaternion_decode(&msg, &hil);

                        recv->state.quaternion[0] = hil.attitude_quaternion[0];
                        recv->state.quaternion[1] = hil.attitude_quaternion[1];
                        recv->state.quaternion[2] = hil.attitude_quaternion[2];
                        recv->state.quaternion[3] = hil.attitude_quaternion[3];
                        recv->state.lat = hil.lat;
                        recv->state.lon = hil.lon;
                        recv->state.alt = hil.alt;
                        recv->state.vx = hil.vx;
                        recv->state.vy = hil.vy;
                        recv->state.vz = hil.vz;
                        recv->state.ind_airspeed = hil.ind_airspeed;
                        recv->state.true_airspeed = hil.true_airspeed;
                        recv->state.time_usec = hil.time_usec;
                        recv->state.valid = true;

                        if (recv->debug) {
                            printf("  HIL_STATE_Q: lat=%d lon=%d alt=%d q=[%.3f,%.3f,%.3f,%.3f]\n",
                                   hil.lat, hil.lon, hil.alt,
                                   hil.attitude_quaternion[0], hil.attitude_quaternion[1],
                                   hil.attitude_quaternion[2], hil.attitude_quaternion[3]);
                        }
                        break;
                    }
                }
            }
        }
    }

    if (got_data) {
        recv->last_msg_time = get_wall_time();
    } else if (recv->connected && recv->last_msg_time > 0) {
        double now = get_wall_time();
        if (now - recv->last_msg_time > DISCONNECT_TIMEOUT_S) {
            recv->connected = false;
            recv->state.valid = false;
            recv->home.valid = false;
            recv->sender_known = false;
            printf("Disconnected from system %u\n", recv->sysid);
        }
    }
}

void mavlink_receiver_close(mavlink_receiver_t *recv) {
    if (recv->sockfd != SOCK_INVALID) {
        SOCK_CLOSE(recv->sockfd);
        recv->sockfd = SOCK_INVALID;
    }
#ifdef _WIN32
    WSACleanup();
#endif
}
