#ifndef MAVLINK_RECEIVER_H
#define MAVLINK_RECEIVER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef _WIN32
typedef uintptr_t sock_t;
#define SOCK_INVALID (~(sock_t)0)
#else
typedef int sock_t;
#define SOCK_INVALID (-1)
#endif

typedef struct {
    float quaternion[4]; // w, x, y, z
    int32_t lat;         // degE7
    int32_t lon;         // degE7
    int32_t alt;         // mm
    int16_t vx, vy, vz;          // cm/s, NED
    uint16_t ind_airspeed;       // cm/s
    uint16_t true_airspeed;      // cm/s
    uint64_t time_usec;          // timestamp (time since boot), microseconds
    bool valid;
} hil_state_t;

typedef struct {
    sock_t sockfd;
    uint16_t port;
    uint8_t channel;
    bool connected;
    bool debug;
    uint8_t sysid;
    double last_msg_time;        // wall-clock time of last received message
    hil_state_t state;
} mavlink_receiver_t;

// Initialize UDP socket on given port with MAVLink parse channel. Returns 0 on success.
int mavlink_receiver_init(mavlink_receiver_t *recv, uint16_t port, uint8_t channel);

// Poll for new messages (non-blocking). Call once per frame.
void mavlink_receiver_poll(mavlink_receiver_t *recv);

// Cleanup socket.
void mavlink_receiver_close(mavlink_receiver_t *recv);

#endif
