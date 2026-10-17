/*
 * OCP Emulator complete-transaction I2C bridge.
 *
 * Copyright (c) 2026 OCP Emulator contributors
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "migration/vmstate.h"
#include "qemu/bswap.h"
#include "qemu/cutils.h"
#include "qemu/main-loop.h"
#include "qemu/module.h"
#include "qemu/sockets.h"
#include "qemu/timer.h"
#include "hw/i2c/i2c.h"
#include "hw/i2c/ocp_emulator_i2c_bridge.h"
#include "hw/qdev-properties.h"

#include <sys/un.h>

#define BRIDGE_RECONNECT_MS 100
#define BRIDGE_DEFAULT_TIMEOUT_MS 1000
#define BRIDGE_MAX_TIMEOUT_MS 60000

OBJECT_DECLARE_SIMPLE_TYPE(OcpEmulatorI2CBridge,
                           OCP_EMULATOR_I2C_BRIDGE)

typedef enum BridgeInjectPhase {
    BRIDGE_INJECT_IDLE,
    BRIDGE_INJECT_WAIT_START,
    BRIDGE_INJECT_SEND,
} BridgeInjectPhase;

typedef struct BridgeDispatchIdentity {
    uint32_t transaction_id;
    uint32_t initiator_id;
    uint32_t timeout_ms;
    uint8_t address;
    uint8_t token[OCP_I2C_BRIDGE_TOKEN_SIZE];
} BridgeDispatchIdentity;

struct OcpEmulatorI2CBridge {
    I2CSlave parent_obj;

    I2CBus *bus;
    char *socket_path;
    uint32_t bus_id;
    uint32_t initiator_id;
    int socket_fd;
    bool socket_connected;
    QEMUTimer *reconnect_timer;
    QEMUTimer *transaction_timer;
    QEMUTimer *outbound_timer;
    QEMUBH *inject_bh;

    BridgeInjectPhase inject_phase;
    uint16_t inject_length;
    uint16_t inject_position;
    uint8_t inject_data[OCP_I2C_BRIDGE_MAX_DATA];
    /*
     * Owned by the sole active injection, never replaced while it is active.
     * BH/timer callbacks are serialized progress notifications, not cached
     * results. Teardown cancels both and removes pending bus ownership before
     * a new injection can reuse this state; finish snapshots identity first.
     */
    BridgeDispatchIdentity inject_identity;

    bool outbound_active;
    bool outbound_oversize;
    bool outbound_waiting_completion;
    uint16_t outbound_length;
    uint32_t next_transaction_id;
    uint32_t outbound_transaction_id;
    uint8_t outbound_address;
    uint8_t outbound_data[OCP_I2C_BRIDGE_MAX_DATA];
};

static uint16_t bridge_get_be16(const uint8_t *p)
{
    return ((uint16_t)p[0] << 8) | p[1];
}

static uint32_t bridge_get_be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) | p[3];
}

static void bridge_put_be16(uint8_t *p, uint16_t value)
{
    p[0] = value >> 8;
    p[1] = value;
}

static void bridge_put_be32(uint8_t *p, uint32_t value)
{
    p[0] = value >> 24;
    p[1] = value >> 16;
    p[2] = value >> 8;
    p[3] = value;
}

static bool bridge_bytes_nonzero(const uint8_t *bytes, size_t length)
{
    size_t i;

    for (i = 0; i < length; i++) {
        if (bytes[i] != 0) {
            return true;
        }
    }
    return false;
}

static void bridge_schedule_reconnect(OcpEmulatorI2CBridge *s)
{
    if (s->socket_path && s->reconnect_timer) {
        timer_mod(s->reconnect_timer,
                  qemu_clock_get_ms(QEMU_CLOCK_REALTIME) +
                  BRIDGE_RECONNECT_MS);
    }
}

static void bridge_release_injection_master(OcpEmulatorI2CBridge *s)
{
    qemu_bh_cancel(s->inject_bh);
    if (s->bus->bh == s->inject_bh) {
        if (i2c_bus_busy(s->bus)) {
            i2c_end_transfer(s->bus);
        }
        i2c_bus_release(s->bus);
    } else {
        i2c_cancel_pending_master(s->bus, s->inject_bh);
    }
}

static void bridge_disconnect(OcpEmulatorI2CBridge *s)
{
    timer_del(s->outbound_timer);
    s->socket_connected = false;
    if (s->socket_fd >= 0) {
        qemu_set_fd_handler(s->socket_fd, NULL, NULL, NULL);
        close(s->socket_fd);
        s->socket_fd = -1;
    }

    if (s->inject_phase != BRIDGE_INJECT_IDLE) {
        timer_del(s->transaction_timer);
        bridge_release_injection_master(s);
        s->inject_phase = BRIDGE_INJECT_IDLE;
        s->inject_length = 0;
        s->inject_position = 0;
    }
    s->outbound_active = false;
    s->outbound_waiting_completion = false;
    s->outbound_length = 0;
    s->outbound_oversize = false;
    bridge_schedule_reconnect(s);
}

static bool bridge_send_packet(OcpEmulatorI2CBridge *s,
                               const uint8_t *packet, size_t length)
{
    ssize_t ret;

    if (!s->socket_connected) {
        return false;
    }
    do {
        ret = send(s->socket_fd, packet, length, MSG_NOSIGNAL);
    } while (ret < 0 && errno == EINTR);

    if (ret == length) {
        return true;
    }
    if (ret < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        /* Complete transaction frames must never be silently dropped. */
        bridge_disconnect(s);
        return false;
    }
    bridge_disconnect(s);
    return false;
}

static void bridge_init_header(OcpEmulatorI2CBridge *s, uint8_t *packet,
                               uint8_t message_type,
                               const BridgeDispatchIdentity *identity,
                               uint16_t length, uint8_t status)
{
    memset(packet, 0, OCP_I2C_BRIDGE_HEADER_SIZE);
    memcpy(packet, OCP_I2C_BRIDGE_MAGIC, 4);
    packet[4] = OCP_I2C_BRIDGE_VERSION;
    packet[5] = message_type;
    packet[6] = OCP_I2C_BRIDGE_KIND_I2C_WRITE;
    packet[7] = OCP_I2C_BRIDGE_FLAG_STOP;
    bridge_put_be32(packet + 8, s->bus_id);
    bridge_put_be32(packet + 12, identity->transaction_id);
    bridge_put_be32(packet + 16, identity->initiator_id);
    packet[20] = identity->address;
    bridge_put_be32(packet + 24, identity->timeout_ms);
    bridge_put_be16(packet + 28, length);
    packet[30] = status;
    memcpy(packet + OCP_I2C_BRIDGE_TOKEN_NONCE_OFFSET, identity->token,
           sizeof(identity->token));
}

static void bridge_send_completion(OcpEmulatorI2CBridge *s,
                                   const BridgeDispatchIdentity *identity,
                                   uint8_t status)
{
    uint8_t packet[OCP_I2C_BRIDGE_HEADER_SIZE];

    bridge_init_header(s, packet, OCP_I2C_BRIDGE_MESSAGE_COMPLETION,
                       identity, 0, status);
    bridge_send_packet(s, packet, sizeof(packet));
}

static void bridge_finish_injection(OcpEmulatorI2CBridge *s, uint8_t status)
{
    const BridgeDispatchIdentity identity = s->inject_identity;

    timer_del(s->transaction_timer);
    bridge_release_injection_master(s);
    s->inject_phase = BRIDGE_INJECT_IDLE;
    s->inject_length = 0;
    s->inject_position = 0;
    bridge_send_completion(s, &identity, status);
}

static void bridge_inject_bh(void *opaque)
{
    OcpEmulatorI2CBridge *s = opaque;

    if (s->inject_phase == BRIDGE_INJECT_IDLE) {
        if (s->bus->bh == s->inject_bh) {
            i2c_bus_release(s->bus);
        }
        return;
    }

    if (s->inject_phase == BRIDGE_INJECT_WAIT_START) {
        if (i2c_start_send_async(s->bus, s->inject_identity.address)) {
            bridge_finish_injection(s, OCP_I2C_BRIDGE_STATUS_NACK);
            return;
        }
        s->inject_phase = BRIDGE_INJECT_SEND;
        return;
    }

    if (s->inject_position == s->inject_length) {
        bridge_finish_injection(s, OCP_I2C_BRIDGE_STATUS_SUCCESS);
        return;
    }

    if (i2c_send_async(s->bus, s->inject_data[s->inject_position])) {
        bridge_finish_injection(s, OCP_I2C_BRIDGE_STATUS_NACK);
        return;
    }
    s->inject_position++;
}

static void bridge_transaction_timeout(void *opaque)
{
    OcpEmulatorI2CBridge *s = opaque;

    if (s->inject_phase != BRIDGE_INJECT_IDLE) {
        bridge_finish_injection(s, OCP_I2C_BRIDGE_STATUS_TIMEOUT);
    }
}

static uint8_t bridge_validate_request(OcpEmulatorI2CBridge *s,
                                       const uint8_t *packet, size_t length,
                                       BridgeDispatchIdentity *identity,
                                       uint16_t *data_length,
                                       bool *fatal)
{
    uint8_t flags;

    *identity = (BridgeDispatchIdentity) {
        .transaction_id = length >= 16 ? bridge_get_be32(packet + 12) : 0,
        .initiator_id = length >= 20 ? bridge_get_be32(packet + 16) : 0,
        .address = length >= 21 ? packet[20] : 0,
        .timeout_ms = BRIDGE_DEFAULT_TIMEOUT_MS,
    };
    *data_length = 0;
    *fatal = true;
    if (length >= OCP_I2C_BRIDGE_HEADER_SIZE) {
        memcpy(identity->token, packet + OCP_I2C_BRIDGE_TOKEN_NONCE_OFFSET,
               sizeof(identity->token));
    }

    if (length < OCP_I2C_BRIDGE_HEADER_SIZE ||
        memcmp(packet, OCP_I2C_BRIDGE_MAGIC, 4) != 0 ||
        packet[4] != OCP_I2C_BRIDGE_VERSION ||
        packet[5] != OCP_I2C_BRIDGE_MESSAGE_REQUEST) {
        return OCP_I2C_BRIDGE_STATUS_MALFORMED;
    }

    if (!bridge_bytes_nonzero(identity->token,
                              OCP_I2C_BRIDGE_TOKEN_NONCE_SIZE) ||
        !bridge_bytes_nonzero(identity->token + OCP_I2C_BRIDGE_TOKEN_NONCE_SIZE,
                              8)) {
        return OCP_I2C_BRIDGE_STATUS_MALFORMED;
    }
    *fatal = false;

    *data_length = bridge_get_be16(packet + 28);
    identity->timeout_ms = bridge_get_be32(packet + 24);
    if (*data_length > OCP_I2C_BRIDGE_MAX_DATA ||
        length > OCP_I2C_BRIDGE_MAX_FRAME) {
        return OCP_I2C_BRIDGE_STATUS_OVERSIZE;
    }
    if (length != OCP_I2C_BRIDGE_HEADER_SIZE + *data_length ||
        bridge_get_be32(packet + 8) != s->bus_id || packet[20] > 0x7f ||
        packet[30] != 0 || packet[31] != 0) {
        return OCP_I2C_BRIDGE_STATUS_MALFORMED;
    }

    flags = packet[7];
    if (identity->address == I2C_BROADCAST ||
        packet[6] != OCP_I2C_BRIDGE_KIND_I2C_WRITE ||
        flags != OCP_I2C_BRIDGE_FLAG_STOP || packet[21] != 0 ||
        packet[22] != 0 || packet[23] != 0) {
        return OCP_I2C_BRIDGE_STATUS_UNSUPPORTED;
    }
    if (identity->timeout_ms == 0 ||
        identity->timeout_ms > BRIDGE_MAX_TIMEOUT_MS) {
        return OCP_I2C_BRIDGE_STATUS_UNSUPPORTED;
    }
    return OCP_I2C_BRIDGE_STATUS_SUCCESS;
}

static bool bridge_handle_completion(OcpEmulatorI2CBridge *s,
                                     const uint8_t *packet, size_t length)
{
    if (!s->outbound_waiting_completion ||
        length != OCP_I2C_BRIDGE_HEADER_SIZE ||
        memcmp(packet, OCP_I2C_BRIDGE_MAGIC, 4) != 0 ||
        packet[4] != OCP_I2C_BRIDGE_VERSION ||
        packet[5] != OCP_I2C_BRIDGE_MESSAGE_COMPLETION ||
        packet[6] != OCP_I2C_BRIDGE_KIND_I2C_WRITE ||
        packet[7] != OCP_I2C_BRIDGE_FLAG_STOP ||
        bridge_get_be32(packet + 8) != s->bus_id ||
        bridge_get_be32(packet + 12) != s->outbound_transaction_id ||
        bridge_get_be32(packet + 16) != s->initiator_id ||
        packet[20] != s->outbound_address || packet[21] != 0 ||
        packet[22] != 0 || packet[23] != 0 ||
        bridge_get_be32(packet + 24) != BRIDGE_DEFAULT_TIMEOUT_MS ||
        bridge_get_be16(packet + 28) != 0 ||
        packet[30] > OCP_I2C_BRIDGE_STATUS_DISCONNECTED || packet[31] != 0 ||
        bridge_bytes_nonzero(packet + OCP_I2C_BRIDGE_TOKEN_NONCE_OFFSET,
                             OCP_I2C_BRIDGE_TOKEN_SIZE)) {
        return false;
    }

    timer_del(s->outbound_timer);
    s->outbound_waiting_completion = false;
    return true;
}

static void bridge_outbound_timeout(void *opaque)
{
    OcpEmulatorI2CBridge *s = opaque;

    if (s->outbound_waiting_completion) {
        bridge_disconnect(s);
    }
}

static void bridge_socket_read(void *opaque)
{
    OcpEmulatorI2CBridge *s = opaque;
    uint8_t packet[OCP_I2C_BRIDGE_MAX_FRAME + 1];
    BridgeDispatchIdentity identity;
    uint16_t data_length;
    uint8_t status;
    bool fatal;
    ssize_t length;

    do {
        length = recv(s->socket_fd, packet, sizeof(packet), MSG_TRUNC);
    } while (length < 0 && errno == EINTR);

    if (length == 0 || (length < 0 && errno != EAGAIN &&
                        errno != EWOULDBLOCK)) {
        bridge_disconnect(s);
        return;
    }
    if (length < 0) {
        return;
    }

    if (length >= 6 &&
        packet[5] == OCP_I2C_BRIDGE_MESSAGE_COMPLETION) {
        if (!bridge_handle_completion(s, packet, length)) {
            bridge_disconnect(s);
        }
        return;
    }

    status = bridge_validate_request(s, packet, length, &identity,
                                     &data_length, &fatal);
    if (status != OCP_I2C_BRIDGE_STATUS_SUCCESS) {
        if (fatal) {
            bridge_disconnect(s);
        } else {
            bridge_send_completion(s, &identity, status);
        }
        return;
    }
    if (s->inject_phase != BRIDGE_INJECT_IDLE) {
        bridge_send_completion(s, &identity, OCP_I2C_BRIDGE_STATUS_UNSUPPORTED);
        return;
    }

    s->inject_identity = identity;
    s->inject_length = data_length;
    s->inject_position = 0;
    memcpy(s->inject_data, packet + OCP_I2C_BRIDGE_HEADER_SIZE, data_length);
    s->inject_phase = BRIDGE_INJECT_WAIT_START;
    timer_mod(s->transaction_timer,
              qemu_clock_get_ms(QEMU_CLOCK_REALTIME) + identity.timeout_ms);
    i2c_bus_master(s->bus, s->inject_bh);
    i2c_schedule_pending_master(s->bus);
}

static void bridge_connect_ready(void *opaque)
{
    OcpEmulatorI2CBridge *s = opaque;
    int error = 0;
    socklen_t length = sizeof(error);

    if (getsockopt(s->socket_fd, SOL_SOCKET, SO_ERROR, &error, &length) < 0 ||
        error != 0) {
        bridge_disconnect(s);
        return;
    }
    s->socket_connected = true;
    qemu_set_fd_handler(s->socket_fd, bridge_socket_read, NULL, s);
}

static void bridge_try_connect(void *opaque)
{
    OcpEmulatorI2CBridge *s = opaque;
    struct sockaddr_un address;
    int ret;

    if (s->socket_fd >= 0 || !s->socket_path) {
        return;
    }
    if (strlen(s->socket_path) >= sizeof(address.sun_path)) {
        return;
    }

    s->socket_fd = socket(AF_UNIX, SOCK_SEQPACKET, 0);
    if (s->socket_fd < 0) {
        bridge_schedule_reconnect(s);
        return;
    }
    qemu_set_cloexec(s->socket_fd);
    qemu_socket_set_nonblock(s->socket_fd);
    memset(&address, 0, sizeof(address));
    address.sun_family = AF_UNIX;
    pstrcpy(address.sun_path, sizeof(address.sun_path), s->socket_path);

    ret = connect(s->socket_fd, (struct sockaddr *)&address, sizeof(address));
    if (ret == 0) {
        s->socket_connected = true;
        qemu_set_fd_handler(s->socket_fd, bridge_socket_read, NULL, s);
    } else if (errno == EINPROGRESS) {
        qemu_set_fd_handler(s->socket_fd, NULL, bridge_connect_ready, s);
    } else {
        bridge_disconnect(s);
    }
}

static int bridge_i2c_event(I2CSlave *slave, enum i2c_event event)
{
    OcpEmulatorI2CBridge *s = OCP_EMULATOR_I2C_BRIDGE(slave);
    uint8_t packet[OCP_I2C_BRIDGE_MAX_FRAME];
    BridgeDispatchIdentity identity;
    bool sent;

    switch (event) {
    case I2C_START_SEND:
        if (s->bus->broadcast || !s->socket_connected ||
            s->outbound_waiting_completion || s->outbound_active) {
            s->outbound_active = false;
            s->outbound_oversize = false;
            s->outbound_length = 0;
            return -1;
        }
        s->outbound_active = true;
        s->outbound_oversize = false;
        s->outbound_length = 0;
        return 0;
    case I2C_START_RECV:
    case I2C_START_SEND_ASYNC:
        s->outbound_active = false;
        s->outbound_oversize = false;
        s->outbound_length = 0;
        return -1;
    case I2C_NACK:
        s->outbound_active = false;
        s->outbound_length = 0;
        return 0;
    case I2C_FINISH:
        if (!s->outbound_active) {
            return 0;
        }
        s->outbound_active = false;
        if (s->outbound_oversize) {
            s->outbound_length = 0;
            return 0;
        }
        identity = (BridgeDispatchIdentity) {
            .transaction_id = ++s->next_transaction_id,
            .initiator_id = s->initiator_id,
            .address = slave->address,
            .timeout_ms = BRIDGE_DEFAULT_TIMEOUT_MS,
        };
        bridge_init_header(s, packet, OCP_I2C_BRIDGE_MESSAGE_REQUEST,
                           &identity, s->outbound_length,
                           OCP_I2C_BRIDGE_STATUS_SUCCESS);
        memcpy(packet + OCP_I2C_BRIDGE_HEADER_SIZE, s->outbound_data,
               s->outbound_length);
        s->outbound_transaction_id = identity.transaction_id;
        s->outbound_address = slave->address;
        s->outbound_waiting_completion = true;
        timer_mod(s->outbound_timer,
                  qemu_clock_get_ms(QEMU_CLOCK_REALTIME) +
                  BRIDGE_DEFAULT_TIMEOUT_MS);
        sent = bridge_send_packet(
            s, packet, OCP_I2C_BRIDGE_HEADER_SIZE + s->outbound_length);
        if (!sent) {
            s->outbound_waiting_completion = false;
        }
        s->outbound_length = 0;
        return 0;
    }
    return -1;
}

static int bridge_i2c_send(I2CSlave *slave, uint8_t data)
{
    OcpEmulatorI2CBridge *s = OCP_EMULATOR_I2C_BRIDGE(slave);

    if (s->bus->broadcast) {
        return 0;
    }
    if (!s->outbound_active ||
        s->outbound_length == OCP_I2C_BRIDGE_MAX_DATA) {
        s->outbound_oversize = true;
        return -1;
    }
    s->outbound_data[s->outbound_length++] = data;
    return 0;
}

static uint8_t bridge_i2c_recv(I2CSlave *slave)
{
    return 0xff;
}

static void bridge_reset(DeviceState *dev)
{
    OcpEmulatorI2CBridge *s = OCP_EMULATOR_I2C_BRIDGE(dev);

    if (s->inject_phase != BRIDGE_INJECT_IDLE) {
        bridge_finish_injection(s, OCP_I2C_BRIDGE_STATUS_RESET);
    }
    s->outbound_active = false;
    timer_del(s->outbound_timer);
    s->outbound_waiting_completion = false;
    s->outbound_length = 0;
    s->outbound_oversize = false;
}

static void bridge_realize(DeviceState *dev, Error **errp)
{
    OcpEmulatorI2CBridge *s = OCP_EMULATOR_I2C_BRIDGE(dev);
    BusState *parent_bus = qdev_get_parent_bus(dev);

    if (!s->socket_path || !s->socket_path[0]) {
        error_setg(errp, "socket-path is required");
        return;
    }
    if (strlen(s->socket_path) >= sizeof(((struct sockaddr_un *)0)->sun_path)) {
        error_setg(errp, "socket-path is too long for a local Unix socket");
        return;
    }
    s->bus = I2C_BUS(parent_bus);
    s->inject_bh = qemu_bh_new_guarded(bridge_inject_bh, s,
                                       &dev->mem_reentrancy_guard);
    s->reconnect_timer = timer_new_ms(QEMU_CLOCK_REALTIME,
                                      bridge_try_connect, s);
    s->transaction_timer = timer_new_ms(QEMU_CLOCK_REALTIME,
                                        bridge_transaction_timeout, s);
    s->outbound_timer = timer_new_ms(QEMU_CLOCK_REALTIME,
                                     bridge_outbound_timeout, s);
    bridge_try_connect(s);
}

static void bridge_unrealize(DeviceState *dev)
{
    OcpEmulatorI2CBridge *s = OCP_EMULATOR_I2C_BRIDGE(dev);

    if (s->inject_phase != BRIDGE_INJECT_IDLE) {
        timer_del(s->transaction_timer);
        bridge_release_injection_master(s);
        s->inject_phase = BRIDGE_INJECT_IDLE;
    }
    if (s->socket_fd >= 0) {
        qemu_set_fd_handler(s->socket_fd, NULL, NULL, NULL);
        close(s->socket_fd);
        s->socket_fd = -1;
    }
    timer_free(s->outbound_timer);
    timer_free(s->transaction_timer);
    timer_free(s->reconnect_timer);
    qemu_bh_delete(s->inject_bh);
}

static const Property bridge_properties[] = {
    DEFINE_PROP_STRING("socket-path", OcpEmulatorI2CBridge, socket_path),
    DEFINE_PROP_UINT32("bus-id", OcpEmulatorI2CBridge, bus_id, 0),
    DEFINE_PROP_UINT32("initiator-id", OcpEmulatorI2CBridge, initiator_id, 0),
};

static const VMStateDescription bridge_vmstate = {
    .name = TYPE_OCP_EMULATOR_I2C_BRIDGE,
    .unmigratable = 1,
};

static void bridge_instance_init(Object *obj)
{
    OcpEmulatorI2CBridge *s = OCP_EMULATOR_I2C_BRIDGE(obj);

    s->socket_fd = -1;
}

static void bridge_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(klass);

    dc->desc = "OCP Emulator complete-transaction I2C bridge";
    dc->realize = bridge_realize;
    dc->unrealize = bridge_unrealize;
    device_class_set_legacy_reset(dc, bridge_reset);
    device_class_set_props(dc, bridge_properties);
    dc->vmsd = &bridge_vmstate;
    sc->event = bridge_i2c_event;
    sc->send = bridge_i2c_send;
    sc->recv = bridge_i2c_recv;
}

static const TypeInfo bridge_type_info = {
    .name = TYPE_OCP_EMULATOR_I2C_BRIDGE,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(OcpEmulatorI2CBridge),
    .instance_init = bridge_instance_init,
    .class_init = bridge_class_init,
};

static void bridge_register_types(void)
{
    type_register_static(&bridge_type_info);
}

type_init(bridge_register_types)
