#include "data_source.h"
#include "mavlink_receiver.h"

#include <stdlib.h>
#include <string.h>

static void mavlink_poll(data_source_t *ds, float dt) {
    (void)dt;
    mavlink_receiver_t *recv = (mavlink_receiver_t *)ds->impl;
    mavlink_receiver_poll(recv);

    // Copy state up to data_source fields
    ds->state = recv->state;
    ds->home = recv->home;
    ds->connected = recv->connected;
    ds->sysid = recv->sysid;
    ds->mav_type = recv->mav_type;
}

static void mavlink_close(data_source_t *ds) {
    mavlink_receiver_t *recv = (mavlink_receiver_t *)ds->impl;
    if (recv) {
        mavlink_receiver_close(recv);
        free(recv);
        ds->impl = NULL;
    }
}

static const data_source_ops_t mavlink_ops = {
    .poll = mavlink_poll,
    .close = mavlink_close,
};

int data_source_mavlink_create(data_source_t *ds, uint16_t port, uint8_t channel, bool debug_flag) {
    memset(ds, 0, sizeof(*ds));
    ds->ops = &mavlink_ops;

    mavlink_receiver_t *recv = (mavlink_receiver_t *)calloc(1, sizeof(mavlink_receiver_t));
    if (!recv) return -1;

    recv->debug = debug_flag;
    int ret = mavlink_receiver_init(recv, port, channel);
    if (ret != 0) {
        free(recv);
        return ret;
    }

    ds->impl = recv;
    ds->debug = debug_flag;
    ds->playback.speed = 1.0f;
    return 0;
}
