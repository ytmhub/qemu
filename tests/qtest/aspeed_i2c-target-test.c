/*
 * QTests for the AST2600 complete-transaction I2C bridge.
 *
 * Copyright (c) 2026 OCP Emulator contributors
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "hw/i2c/bcm2835_i2c.h"
#include "hw/i2c/ocp_emulator_i2c_bridge.h"
#include "hw/sensor/tmp105_regs.h"
#include "libqtest-single.h"
#include "qtest_aspeed.h"

#include <poll.h>
#include <sys/un.h>

#define TEST_TARGET_ADDRESS 0x55
#define TEST_BRIDGE_ADDRESS 0x52
#define TEST_BUS_ID 7
#define TEST_INITIATOR_ID 0x11223344
#define TEST_REMOTE_INITIATOR_ID 0x55667788
#define TEST_GENERAL_CALL_ADDRESS 0
#define TEST_GENERAL_CALL_TARGET_ADDRESS 0x49
#define TEST_GENERAL_CALL_CONTROLLER_BASE 0x3f205000

#define I2CD_FUN_CTRL 0x00
#define I2CD_INTR_CTRL 0x0c
#define I2CD_INTR_STS 0x10
#define I2CD_CMD 0x14
#define I2CD_DEV_ADDR 0x18
#define I2CD_BYTE_BUF 0x20

#define I2CD_MASTER_EN BIT(0)
#define I2CD_SLAVE_EN BIT(1)
#define I2CD_INTR_RX_DONE BIT(2)
#define I2CD_INTR_NORMAL_STOP BIT(4)
#define I2CD_INTR_SLAVE_ADDR_RX_MATCH BIT(7)
#define I2CD_INTR_TX_NAK BIT(1)
#define I2CD_INTR_TX_ACK BIT(0)
#define I2CD_M_STOP_CMD BIT(5)
#define I2CD_M_TX_CMD BIT(1)
#define I2CD_M_START_CMD BIT(0)

#define TEST_TIMEOUT_MS 5000

static unsigned int socket_sequence;

typedef struct BridgeFixture {
    QTestState *qts;
    char *socket_path;
    int listener_fd;
    int peer_fd;
    uint32_t base;
} BridgeFixture;

static void put_be16(uint8_t *p, uint16_t value)
{
    p[0] = value >> 8;
    p[1] = value;
}

static void put_be32(uint8_t *p, uint32_t value)
{
    p[0] = value >> 24;
    p[1] = value >> 16;
    p[2] = value >> 8;
    p[3] = value;
}

static uint16_t get_be16(const uint8_t *p)
{
    return ((uint16_t)p[0] << 8) | p[1];
}

static uint32_t get_be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) | p[3];
}

static int wait_for_fd(int fd, short events, int timeout_ms)
{
    struct pollfd poll_fd = { .fd = fd, .events = events };
    int ret;

    do {
        ret = poll(&poll_fd, 1, timeout_ms);
    } while (ret < 0 && errno == EINTR);
    return ret > 0 ? poll_fd.revents : 0;
}

static int accept_peer(int listener_fd)
{
    int fd;

    g_assert_cmpint(wait_for_fd(listener_fd, POLLIN, TEST_TIMEOUT_MS), !=, 0);
    do {
        fd = accept(listener_fd, NULL, NULL);
    } while (fd < 0 && errno == EINTR);
    g_assert_cmpint(fd, >=, 0);
    return fd;
}

static void fixture_setup(BridgeFixture *f, gconstpointer data)
{
    struct sockaddr_un address;

    f->socket_path = g_strdup_printf("/tmp/ocp-i2c-bridge-%u-%u.sock",
                                     (unsigned int)getpid(),
                                     socket_sequence++);
    f->listener_fd = socket(AF_UNIX, SOCK_SEQPACKET, 0);
    g_assert_cmpint(f->listener_fd, >=, 0);
    memset(&address, 0, sizeof(address));
    address.sun_family = AF_UNIX;
    g_strlcpy(address.sun_path, f->socket_path, sizeof(address.sun_path));
    unlink(f->socket_path);
    g_assert_cmpint(bind(f->listener_fd, (struct sockaddr *)&address,
                         sizeof(address)), ==, 0);
    g_assert_cmpint(listen(f->listener_fd, 1), ==, 0);

    if (data) {
        f->qts = qtest_initf("-machine raspi2b "
                             "-device ocp-emulator-i2c-bridge,"
                             "bus=i2c-bus.0,address=0x52,"
                             "bus-id=7,initiator-id=0x11223344,socket-path=%s "
                             "-device tmp105,id=general-call-target,"
                             "bus=i2c-bus.0,address=0x49",
                             f->socket_path);
    } else {
        f->qts = qtest_initf("-machine ast2600-evb "
                             "-device ocp-emulator-i2c-bridge,"
                             "bus=aspeed.i2c.bus.0,address=0x52,"
                             "bus-id=7,initiator-id=0x11223344,socket-path=%s",
                             f->socket_path);
        f->base = ast2600_i2c_calc_bus_addr(0);
        qtest_writel(f->qts, AST2600_ASPEED_I2C_BASE_ADDR + 0x0c, 0);
    }
    f->peer_fd = accept_peer(f->listener_fd);
}

static void fixture_teardown(BridgeFixture *f, gconstpointer data)
{
    qtest_quit(f->qts);
    close(f->peer_fd);
    close(f->listener_fd);
    unlink(f->socket_path);
    g_free(f->socket_path);
}

static size_t make_request(uint8_t *packet, uint32_t transaction_id,
                           uint8_t address, const uint8_t *payload,
                           uint16_t payload_length, uint32_t timeout_ms)
{
    memset(packet, 0, OCP_I2C_BRIDGE_HEADER_SIZE + payload_length);
    memcpy(packet, OCP_I2C_BRIDGE_MAGIC, 4);
    packet[4] = OCP_I2C_BRIDGE_VERSION;
    packet[5] = OCP_I2C_BRIDGE_MESSAGE_REQUEST;
    packet[6] = OCP_I2C_BRIDGE_KIND_I2C_WRITE;
    packet[7] = OCP_I2C_BRIDGE_FLAG_STOP;
    put_be32(packet + 8, TEST_BUS_ID);
    put_be32(packet + 12, transaction_id);
    put_be32(packet + 16, TEST_REMOTE_INITIATOR_ID);
    packet[20] = address;
    put_be32(packet + 24, timeout_ms);
    put_be16(packet + 28, payload_length);
    if (payload_length) {
        memcpy(packet + OCP_I2C_BRIDGE_HEADER_SIZE, payload, payload_length);
    }
    return OCP_I2C_BRIDGE_HEADER_SIZE + payload_length;
}

static void make_completion(uint8_t *packet, uint32_t transaction_id,
                            uint8_t address, uint8_t status)
{
    memset(packet, 0, OCP_I2C_BRIDGE_HEADER_SIZE);
    memcpy(packet, OCP_I2C_BRIDGE_MAGIC, 4);
    packet[4] = OCP_I2C_BRIDGE_VERSION;
    packet[5] = OCP_I2C_BRIDGE_MESSAGE_COMPLETION;
    packet[6] = OCP_I2C_BRIDGE_KIND_I2C_WRITE;
    packet[7] = OCP_I2C_BRIDGE_FLAG_STOP;
    put_be32(packet + 8, TEST_BUS_ID);
    put_be32(packet + 12, transaction_id);
    put_be32(packet + 16, TEST_INITIATOR_ID);
    packet[20] = address;
    put_be32(packet + 24, TEST_TIMEOUT_MS);
    packet[30] = status;
}

static void send_frame(int fd, const uint8_t *packet, size_t length)
{
    ssize_t ret;

    do {
        ret = send(fd, packet, length, MSG_NOSIGNAL);
    } while (ret < 0 && errno == EINTR);
    g_assert_cmpint(ret, ==, length);
}

static size_t receive_frame(int fd, uint8_t *packet, size_t capacity)
{
    ssize_t ret;

    g_assert_cmpint(wait_for_fd(fd, POLLIN, TEST_TIMEOUT_MS), !=, 0);
    do {
        ret = recv(fd, packet, capacity, 0);
    } while (ret < 0 && errno == EINTR);
    g_assert_cmpint(ret, >, 0);
    return ret;
}

static void assert_completion(int fd, uint32_t transaction_id,
                              uint32_t initiator_id, uint8_t status)
{
    uint8_t packet[OCP_I2C_BRIDGE_MAX_FRAME];
    size_t length = receive_frame(fd, packet, sizeof(packet));

    g_assert_cmpuint(length, ==, OCP_I2C_BRIDGE_HEADER_SIZE);
    g_assert_cmpmem(packet, 4, OCP_I2C_BRIDGE_MAGIC, 4);
    g_assert_cmpuint(packet[4], ==, OCP_I2C_BRIDGE_VERSION);
    g_assert_cmpuint(packet[5], ==, OCP_I2C_BRIDGE_MESSAGE_COMPLETION);
    g_assert_cmpuint(get_be32(packet + 12), ==, transaction_id);
    g_assert_cmpuint(get_be32(packet + 16), ==, initiator_id);
    g_assert_cmpuint(get_be16(packet + 28), ==, 0);
    g_assert_cmpuint(packet[30], ==, status);
}

static uint32_t wait_for_status(BridgeFixture *f, uint32_t mask)
{
    int64_t deadline = g_get_monotonic_time() +
                       TEST_TIMEOUT_MS * G_TIME_SPAN_MILLISECOND;
    uint32_t status;

    do {
        status = qtest_readl(f->qts, f->base + I2CD_INTR_STS);
        if ((status & mask) == mask) {
            return status;
        }
        g_usleep(1000);
    } while (g_get_monotonic_time() < deadline);

    g_error("timed out waiting for I2C status 0x%x (last 0x%x)", mask,
            status);
}

static void configure_target(BridgeFixture *f)
{
    qtest_writel(f->qts, f->base + I2CD_DEV_ADDR, TEST_TARGET_ADDRESS);
    qtest_writel(f->qts, f->base + I2CD_INTR_CTRL,
                 I2CD_INTR_RX_DONE | I2CD_INTR_NORMAL_STOP);
    qtest_writel(f->qts, f->base + I2CD_FUN_CTRL, I2CD_SLAVE_EN);
}

static void receive_target_write(BridgeFixture *f, const uint8_t *expected,
                                 size_t expected_length)
{
    size_t i;

    g_assert_cmpuint(expected_length, >, 0);
    wait_for_status(f, I2CD_INTR_RX_DONE |
                       I2CD_INTR_SLAVE_ADDR_RX_MATCH);
    g_assert_cmphex(qtest_readl(f->qts, AST2600_ASPEED_I2C_BASE_ADDR) &
                    BIT(0), ==, BIT(0));
    g_assert_cmphex((qtest_readl(f->qts, f->base + I2CD_BYTE_BUF) >> 8) &
                    0xff, ==, TEST_TARGET_ADDRESS << 1);
    qtest_writel(f->qts, f->base + I2CD_INTR_STS, I2CD_INTR_RX_DONE);

    for (i = 0; i < expected_length; i++) {
        wait_for_status(f, I2CD_INTR_RX_DONE);
        g_assert_cmphex((qtest_readl(f->qts, f->base + I2CD_BYTE_BUF) >> 8) &
                        0xff, ==, expected[i]);
        qtest_writel(f->qts, f->base + I2CD_INTR_STS,
                     I2CD_INTR_RX_DONE);
    }
    wait_for_status(f, I2CD_INTR_NORMAL_STOP);
}

static void test_outbound_controller(BridgeFixture *f, gconstpointer data)
{
    const uint8_t expected[] = { 0x11, 0x22, 0x33 };
    const uint8_t second = 0x44;
    uint8_t packet[OCP_I2C_BRIDGE_MAX_FRAME];
    uint32_t transaction_id;
    size_t i, length;

    qtest_writel(f->qts, f->base + I2CD_INTR_CTRL,
                 I2CD_INTR_TX_ACK | I2CD_INTR_TX_NAK);
    qtest_writel(f->qts, f->base + I2CD_FUN_CTRL, I2CD_MASTER_EN);
    qtest_writel(f->qts, f->base + I2CD_BYTE_BUF, TEST_BRIDGE_ADDRESS << 1);
    qtest_writel(f->qts, f->base + I2CD_CMD,
                 I2CD_M_START_CMD | I2CD_M_TX_CMD);
    wait_for_status(f, I2CD_INTR_TX_ACK);

    for (i = 0; i < sizeof(expected); i++) {
        qtest_writel(f->qts, f->base + I2CD_BYTE_BUF, expected[i]);
        qtest_writel(f->qts, f->base + I2CD_CMD, I2CD_M_TX_CMD);
    }
    qtest_writel(f->qts, f->base + I2CD_CMD, I2CD_M_STOP_CMD);

    length = receive_frame(f->peer_fd, packet, sizeof(packet));
    g_assert_cmpuint(length, ==,
                     OCP_I2C_BRIDGE_HEADER_SIZE + sizeof(expected));
    g_assert_cmpuint(packet[5], ==, OCP_I2C_BRIDGE_MESSAGE_REQUEST);
    g_assert_cmpuint(packet[6], ==, OCP_I2C_BRIDGE_KIND_I2C_WRITE);
    g_assert_cmpuint(packet[7], ==, OCP_I2C_BRIDGE_FLAG_STOP);
    g_assert_cmpuint(get_be32(packet + 8), ==, TEST_BUS_ID);
    transaction_id = get_be32(packet + 12);
    g_assert_cmpuint(get_be32(packet + 16), ==, TEST_INITIATOR_ID);
    g_assert_cmpuint(packet[20], ==, TEST_BRIDGE_ADDRESS);
    g_assert_cmpuint(get_be16(packet + 28), ==, sizeof(expected));
    g_assert_cmpmem(packet + OCP_I2C_BRIDGE_HEADER_SIZE, sizeof(expected),
                    expected, sizeof(expected));

    make_completion(packet, transaction_id, TEST_BRIDGE_ADDRESS,
                    OCP_I2C_BRIDGE_STATUS_SUCCESS);
    send_frame(f->peer_fd, packet, OCP_I2C_BRIDGE_HEADER_SIZE);
    qtest_clock_step(f->qts, 1);

    qtest_writel(f->qts, f->base + I2CD_INTR_STS,
                 I2CD_INTR_TX_ACK | I2CD_INTR_TX_NAK);
    qtest_writel(f->qts, f->base + I2CD_BYTE_BUF,
                 TEST_BRIDGE_ADDRESS << 1);
    qtest_writel(f->qts, f->base + I2CD_CMD,
                 I2CD_M_START_CMD | I2CD_M_TX_CMD);
    wait_for_status(f, I2CD_INTR_TX_ACK);
    qtest_writel(f->qts, f->base + I2CD_BYTE_BUF, second);
    qtest_writel(f->qts, f->base + I2CD_CMD, I2CD_M_TX_CMD);
    qtest_writel(f->qts, f->base + I2CD_CMD, I2CD_M_STOP_CMD);

    length = receive_frame(f->peer_fd, packet, sizeof(packet));
    g_assert_cmpuint(length, ==, OCP_I2C_BRIDGE_HEADER_SIZE + 1);
    g_assert_cmpuint(get_be32(packet + 12), ==, transaction_id + 1);
    g_assert_cmpuint(get_be16(packet + 28), ==, 1);
    g_assert_cmpuint(packet[OCP_I2C_BRIDGE_HEADER_SIZE], ==, second);

    make_completion(packet, transaction_id + 1, TEST_BRIDGE_ADDRESS,
                    OCP_I2C_BRIDGE_STATUS_SUCCESS);
    send_frame(f->peer_fd, packet, OCP_I2C_BRIDGE_HEADER_SIZE);
}

static void test_outbound_general_call_ignored(BridgeFixture *f,
                                               gconstpointer data)
{
    const uint8_t payload[] = { TMP105_REG_CONFIG, 0x60 };
    uint32_t status;
    size_t i;

    qtest_writel(f->qts,
                 TEST_GENERAL_CALL_CONTROLLER_BASE + BCM2835_I2C_A,
                 TEST_GENERAL_CALL_ADDRESS);
    qtest_writel(f->qts,
                 TEST_GENERAL_CALL_CONTROLLER_BASE + BCM2835_I2C_DLEN,
                 sizeof(payload));
    qtest_writel(f->qts,
                 TEST_GENERAL_CALL_CONTROLLER_BASE + BCM2835_I2C_C,
                 BCM2835_I2C_C_I2CEN | BCM2835_I2C_C_ST |
                 BCM2835_I2C_C_CLEAR);
    for (i = 0; i < sizeof(payload); i++) {
        qtest_writel(f->qts,
                     TEST_GENERAL_CALL_CONTROLLER_BASE + BCM2835_I2C_FIFO,
                     payload[i]);
    }
    status = qtest_readl(f->qts,
                         TEST_GENERAL_CALL_CONTROLLER_BASE + BCM2835_I2C_S);
    g_assert_cmphex(status & (BCM2835_I2C_S_DONE | BCM2835_I2C_S_ERR), ==,
                    BCM2835_I2C_S_DONE);

    qtest_writel(f->qts,
                 TEST_GENERAL_CALL_CONTROLLER_BASE + BCM2835_I2C_A,
                 TEST_GENERAL_CALL_TARGET_ADDRESS);
    qtest_writel(f->qts,
                 TEST_GENERAL_CALL_CONTROLLER_BASE + BCM2835_I2C_DLEN, 1);
    qtest_writel(f->qts,
                 TEST_GENERAL_CALL_CONTROLLER_BASE + BCM2835_I2C_C,
                 BCM2835_I2C_C_I2CEN | BCM2835_I2C_C_ST |
                 BCM2835_I2C_C_CLEAR);
    qtest_writel(f->qts,
                 TEST_GENERAL_CALL_CONTROLLER_BASE + BCM2835_I2C_FIFO,
                 TMP105_REG_CONFIG);
    qtest_writel(f->qts,
                 TEST_GENERAL_CALL_CONTROLLER_BASE + BCM2835_I2C_DLEN, 1);
    qtest_writel(f->qts,
                 TEST_GENERAL_CALL_CONTROLLER_BASE + BCM2835_I2C_C,
                 BCM2835_I2C_C_I2CEN | BCM2835_I2C_C_ST |
                 BCM2835_I2C_C_CLEAR | BCM2835_I2C_C_READ);
    g_assert_cmphex(qtest_readl(
                        f->qts,
                        TEST_GENERAL_CALL_CONTROLLER_BASE + BCM2835_I2C_FIFO),
                    ==, payload[1]);
    g_assert_cmpint(wait_for_fd(f->peer_fd, POLLIN, 200), ==, 0);
}

static void assert_repeated_start_rejected(BridgeFixture *f, bool is_recv)
{
    const uint8_t payload = 0xa5;

    qtest_writel(f->qts, f->base + I2CD_INTR_STS, UINT32_MAX);
    qtest_writel(f->qts, f->base + I2CD_BYTE_BUF,
                 TEST_BRIDGE_ADDRESS << 1);
    qtest_writel(f->qts, f->base + I2CD_CMD,
                 I2CD_M_START_CMD | I2CD_M_TX_CMD);
    wait_for_status(f, I2CD_INTR_TX_ACK);

    qtest_writel(f->qts, f->base + I2CD_INTR_STS,
                 I2CD_INTR_TX_ACK | I2CD_INTR_TX_NAK);
    qtest_writel(f->qts, f->base + I2CD_BYTE_BUF, payload);
    qtest_writel(f->qts, f->base + I2CD_CMD, I2CD_M_TX_CMD);
    wait_for_status(f, I2CD_INTR_TX_ACK);

    qtest_writel(f->qts, f->base + I2CD_INTR_STS,
                 I2CD_INTR_TX_ACK | I2CD_INTR_TX_NAK);
    qtest_writel(f->qts, f->base + I2CD_BYTE_BUF,
                 (TEST_BRIDGE_ADDRESS << 1) | is_recv);
    qtest_writel(f->qts, f->base + I2CD_CMD,
                 I2CD_M_START_CMD | I2CD_M_TX_CMD);
    wait_for_status(f, I2CD_INTR_TX_NAK);
    qtest_writel(f->qts, f->base + I2CD_CMD, I2CD_M_STOP_CMD);

    g_assert_cmpint(wait_for_fd(f->peer_fd, POLLIN, 200), ==, 0);
}

static void test_repeated_start_rejected(BridgeFixture *f,
                                         gconstpointer data)
{
    qtest_writel(f->qts, f->base + I2CD_INTR_CTRL,
                 I2CD_INTR_TX_ACK | I2CD_INTR_TX_NAK);
    qtest_writel(f->qts, f->base + I2CD_FUN_CTRL, I2CD_MASTER_EN);

    assert_repeated_start_rejected(f, false);
    assert_repeated_start_rejected(f, true);
}

static void test_inbound_target(BridgeFixture *f, gconstpointer data)
{
    const uint8_t expected[] = { 0xa5, 0x5a, 0x7e };
    uint8_t packet[OCP_I2C_BRIDGE_MAX_FRAME];
    size_t length;

    configure_target(f);
    length = make_request(packet, 0x10203040, TEST_TARGET_ADDRESS, expected,
                          sizeof(expected), TEST_TIMEOUT_MS);
    send_frame(f->peer_fd, packet, length);
    receive_target_write(f, expected, sizeof(expected));
    assert_completion(f->peer_fd, 0x10203040, TEST_REMOTE_INITIATOR_ID,
                      OCP_I2C_BRIDGE_STATUS_SUCCESS);
}

static void test_nack_address_mismatch(BridgeFixture *f, gconstpointer data)
{
    const uint8_t payload = 0xaa;
    uint8_t packet[OCP_I2C_BRIDGE_MAX_FRAME];
    size_t length;

    configure_target(f);
    length = make_request(packet, 2, TEST_TARGET_ADDRESS + 1, &payload, 1,
                          TEST_TIMEOUT_MS);
    send_frame(f->peer_fd, packet, length);
    assert_completion(f->peer_fd, 2, TEST_REMOTE_INITIATOR_ID,
                      OCP_I2C_BRIDGE_STATUS_NACK);
}

static void test_maximum_and_oversize(BridgeFixture *f, gconstpointer data)
{
    uint8_t packet[OCP_I2C_BRIDGE_MAX_FRAME + 1];
    uint8_t payload[OCP_I2C_BRIDGE_MAX_DATA];
    size_t length;
    unsigned int i;

    for (i = 0; i < sizeof(payload); i++) {
        payload[i] = i;
    }
    configure_target(f);
    length = make_request(packet, 3, TEST_TARGET_ADDRESS, payload,
                          sizeof(payload), 10000);
    send_frame(f->peer_fd, packet, length);
    receive_target_write(f, payload, sizeof(payload));
    assert_completion(f->peer_fd, 3, TEST_REMOTE_INITIATOR_ID,
                      OCP_I2C_BRIDGE_STATUS_SUCCESS);

    length = make_request(packet, 4, TEST_TARGET_ADDRESS, payload,
                          sizeof(payload), TEST_TIMEOUT_MS);
    put_be16(packet + 28, OCP_I2C_BRIDGE_MAX_DATA + 1);
    packet[length] = 0xff;
    send_frame(f->peer_fd, packet, length + 1);
    assert_completion(f->peer_fd, 4, TEST_REMOTE_INITIATOR_ID,
                      OCP_I2C_BRIDGE_STATUS_OVERSIZE);
}

static void test_reset_during_transaction(BridgeFixture *f,
                                          gconstpointer data)
{
    const uint8_t payload[] = { 1, 2 };
    uint8_t packet[OCP_I2C_BRIDGE_MAX_FRAME];
    size_t length;

    configure_target(f);
    length = make_request(packet, 5, TEST_TARGET_ADDRESS, payload,
                          sizeof(payload), TEST_TIMEOUT_MS);
    send_frame(f->peer_fd, packet, length);
    wait_for_status(f, I2CD_INTR_RX_DONE |
                       I2CD_INTR_SLAVE_ADDR_RX_MATCH);
    qtest_system_reset(f->qts);
    assert_completion(f->peer_fd, 5, TEST_REMOTE_INITIATOR_ID,
                      OCP_I2C_BRIDGE_STATUS_RESET);
    g_assert_cmphex(qtest_readl(f->qts, f->base + I2CD_INTR_STS), ==, 0);
}

static void test_malformed_frame(BridgeFixture *f, gconstpointer data)
{
    const uint8_t malformed[] = { 0xde, 0xad, 0xbe };
    const uint8_t payload = 0xaa;
    uint8_t packet[OCP_I2C_BRIDGE_MAX_FRAME];
    size_t length;

    send_frame(f->peer_fd, malformed, sizeof(malformed));
    assert_completion(f->peer_fd, 0, 0,
                      OCP_I2C_BRIDGE_STATUS_MALFORMED);

    length = make_request(packet, 8, TEST_GENERAL_CALL_ADDRESS, &payload, 1,
                          TEST_TIMEOUT_MS);
    send_frame(f->peer_fd, packet, length);
    assert_completion(f->peer_fd, 8, TEST_REMOTE_INITIATOR_ID,
                      OCP_I2C_BRIDGE_STATUS_UNSUPPORTED);
}

static void test_disconnect_reconnect(BridgeFixture *f, gconstpointer data)
{
    const uint8_t payload[] = { 0xcc, 0xdd };
    uint8_t packet[OCP_I2C_BRIDGE_MAX_FRAME];
    size_t length;

    configure_target(f);
    length = make_request(packet, 6, TEST_TARGET_ADDRESS, payload,
                          sizeof(payload), TEST_TIMEOUT_MS);
    send_frame(f->peer_fd, packet, length);
    wait_for_status(f, I2CD_INTR_RX_DONE |
                       I2CD_INTR_SLAVE_ADDR_RX_MATCH);

    close(f->peer_fd);
    f->peer_fd = accept_peer(f->listener_fd);
    g_assert_cmpint(wait_for_fd(f->peer_fd, POLLIN, 200), ==, 0);

    length = make_request(packet, 7, TEST_TARGET_ADDRESS + 1, payload,
                          sizeof(payload), TEST_TIMEOUT_MS);
    send_frame(f->peer_fd, packet, length);
    assert_completion(f->peer_fd, 7, TEST_REMOTE_INITIATOR_ID,
                      OCP_I2C_BRIDGE_STATUS_NACK);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    g_test_add("/ast2600/i2c/bridge/outbound-controller", BridgeFixture,
               NULL, fixture_setup, test_outbound_controller,
               fixture_teardown);
    if (qtest_has_machine("raspi2b") && qtest_has_device("tmp105")) {
        g_test_add("/i2c/bridge/outbound-general-call-ignored",
                   BridgeFixture, GINT_TO_POINTER(1), fixture_setup,
                   test_outbound_general_call_ignored, fixture_teardown);
    }
    g_test_add("/ast2600/i2c/bridge/repeated-start-rejected", BridgeFixture,
               NULL, fixture_setup, test_repeated_start_rejected,
               fixture_teardown);
    g_test_add("/ast2600/i2c/bridge/inbound-target", BridgeFixture,
               NULL, fixture_setup, test_inbound_target, fixture_teardown);
    g_test_add("/ast2600/i2c/bridge/nack-address-mismatch", BridgeFixture,
               NULL, fixture_setup, test_nack_address_mismatch,
               fixture_teardown);
    g_test_add("/ast2600/i2c/bridge/maximum-and-oversize", BridgeFixture,
               NULL, fixture_setup, test_maximum_and_oversize,
               fixture_teardown);
    g_test_add("/ast2600/i2c/bridge/reset-during-transaction", BridgeFixture,
               NULL, fixture_setup, test_reset_during_transaction,
               fixture_teardown);
    g_test_add("/ast2600/i2c/bridge/malformed-frame", BridgeFixture,
               NULL, fixture_setup, test_malformed_frame, fixture_teardown);
    g_test_add("/ast2600/i2c/bridge/disconnect-reconnect", BridgeFixture,
               NULL, fixture_setup, test_disconnect_reconnect,
               fixture_teardown);
    return g_test_run();
}
