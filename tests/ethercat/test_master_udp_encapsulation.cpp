/**
 * @file test_master_udp_encapsulation.cpp
 * @brief Tests for EtherCAT-over-UDP encapsulation in the Master class.
 *
 * Verifies:
 *  - Config defaults (disabled by default)
 *  - TX: frames are correctly encapsulated with Ethernet/IPv4/UDP headers
 *  - TX: IP checksum is valid
 *  - TX: UDP ports and IP addresses match config
 *  - RX: UDP-encapsulated frames are correctly parsed (datagrams extracted)
 *  - RX: non-EtherCAT UDP frames are ignored
 *  - Size limits: oversized datagrams are rejected in UDP mode
 *
 * When TETHER_ENABLE_UDP_ENCAPSULATION is 0 (default), only a single test
 * verifying the feature is compiled out is included.
 */

#include <gtest/gtest.h>

#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Types.hpp"

#include <cstring>
#include <vector>
#include <functional>
#include <thread>
#include <atomic>
#include <chrono>

using namespace EtherCAT;

#if !TETHER_ENABLE_UDP_ENCAPSULATION

// ============================================================================
// When UDP encapsulation is compiled out, verify it's always disabled.
// ============================================================================

TEST(MasterUdpEncapsulation, CompiledOutAlwaysReturnsFalse) {
    Master master;
    EXPECT_FALSE(master.isUdpEncapsulationEnabled());
}

#else // TETHER_ENABLE_UDP_ENCAPSULATION

// ============================================================================
// Helpers
// ============================================================================

/// Build a direct Ethernet/EtherCAT frame (EtherType 0x88A4) with one datagram.
static std::vector<uint8_t> buildDirectEtherCATFrame(
    Command cmd, uint8_t idx,
    uint16_t adp, uint16_t ado,
    const uint8_t* payload, uint16_t payload_len,
    uint16_t wkc)
{
    const size_t dgram_total = 10 + payload_len + 2;
    const size_t frame_size = 14 + 2 + dgram_total;
    std::vector<uint8_t> frame(std::max(frame_size, (size_t)60), 0);

    // Ethernet header
    frame[0] = 0x01; frame[1] = 0x01; frame[2] = 0x05;
    frame[3] = 0x00; frame[4] = 0x00; frame[5] = 0x00;
    frame[6] = 0xAA; frame[7] = 0xBB; frame[8] = 0xCC;
    frame[9] = 0xDD; frame[10] = 0xEE; frame[11] = 0xFF;
    frame[12] = 0x88; frame[13] = 0xA4;

    // EtherCAT frame header
    uint16_t ec_raw = static_cast<uint16_t>((dgram_total & 0x07FFu) | (1u << 12));
    frame[14] = ec_raw & 0xFF;
    frame[15] = (ec_raw >> 8) & 0xFF;

    // Datagram header
    frame[16] = static_cast<uint8_t>(cmd);
    frame[17] = idx;
    frame[18] = adp & 0xFF;
    frame[19] = (adp >> 8) & 0xFF;
    frame[20] = ado & 0xFF;
    frame[21] = (ado >> 8) & 0xFF;
    uint16_t len_flags = payload_len & 0x07FFu;
    frame[22] = len_flags & 0xFF;
    frame[23] = (len_flags >> 8) & 0xFF;
    frame[24] = 0; frame[25] = 0;

    if (payload && payload_len > 0) {
        std::memcpy(&frame[26], payload, payload_len);
    }
    frame[26 + payload_len] = wkc & 0xFF;
    frame[26 + payload_len + 1] = (wkc >> 8) & 0xFF;

    return frame;
}

/// Build an Ethernet/IPv4/UDP/EtherCAT frame (EtherCAT-over-UDP encapsulation).
static std::vector<uint8_t> buildUdpEtherCATFrame(
    Command cmd, uint8_t idx,
    uint16_t adp, uint16_t ado,
    const uint8_t* payload, uint16_t payload_len,
    uint16_t wkc,
    uint32_t src_ip = 0xC0A80101,   // 192.168.1.1
    uint32_t dst_ip = 0xC0A801FF,   // 192.168.1.255
    uint16_t src_port = 0x88A4,
    uint16_t dst_port = 0x88A4)
{
    // First build the EtherCAT payload (frame header + datagram + WKC).
    const size_t dgram_total = 10 + payload_len + 2;
    const size_t ecat_len = 2 + dgram_total;

    // Ethernet(14) + IP(20) + UDP(8) + EtherCAT payload
    const size_t frame_size = 14 + 20 + 8 + ecat_len;
    std::vector<uint8_t> frame(std::max(frame_size, (size_t)60), 0);

    // Ethernet header
    frame[0] = 0x01; frame[1] = 0x01; frame[2] = 0x05;
    frame[3] = 0x00; frame[4] = 0x00; frame[5] = 0x00;
    frame[6] = 0xAA; frame[7] = 0xBB; frame[8] = 0xCC;
    frame[9] = 0xDD; frame[10] = 0xEE; frame[11] = 0xFF;
    frame[12] = 0x08; frame[13] = 0x00; // EtherType = IPv4

    // IPv4 header (20 bytes)
    size_t ip_off = 14;
    frame[ip_off + 0] = 0x45;  // version=4, IHL=5
    frame[ip_off + 1] = 0x00;  // TOS
    uint16_t ip_total_len = static_cast<uint16_t>(20 + 8 + ecat_len);
    frame[ip_off + 2] = (ip_total_len >> 8) & 0xFF;
    frame[ip_off + 3] = ip_total_len & 0xFF;
    frame[ip_off + 4] = 0x00; frame[ip_off + 5] = 0x01; // ID
    frame[ip_off + 6] = 0x40; frame[ip_off + 7] = 0x00; // DF, no offset
    frame[ip_off + 8] = 64;   // TTL
    frame[ip_off + 9] = 0x11; // protocol = UDP
    frame[ip_off + 10] = 0; frame[ip_off + 11] = 0; // checksum (placeholder)
    // Source IP (big-endian)
    frame[ip_off + 12] = (src_ip >> 24) & 0xFF;
    frame[ip_off + 13] = (src_ip >> 16) & 0xFF;
    frame[ip_off + 14] = (src_ip >> 8) & 0xFF;
    frame[ip_off + 15] = src_ip & 0xFF;
    // Destination IP (big-endian)
    frame[ip_off + 16] = (dst_ip >> 24) & 0xFF;
    frame[ip_off + 17] = (dst_ip >> 16) & 0xFF;
    frame[ip_off + 18] = (dst_ip >> 8) & 0xFF;
    frame[ip_off + 19] = dst_ip & 0xFF;

    // Compute IP checksum
    uint32_t sum = 0;
    for (int i = 0; i < 20; i += 2) {
        sum += static_cast<uint32_t>(frame[ip_off + i]) << 8;
        sum += static_cast<uint32_t>(frame[ip_off + i + 1]);
    }
    while (sum >> 16) sum = (sum & 0xFFFF) + (sum >> 16);
    uint16_t cksum = static_cast<uint16_t>(~sum & 0xFFFF);
    frame[ip_off + 10] = (cksum >> 8) & 0xFF;
    frame[ip_off + 11] = cksum & 0xFF;

    // UDP header (8 bytes)
    size_t udp_off = ip_off + 20;
    frame[udp_off + 0] = (src_port >> 8) & 0xFF;
    frame[udp_off + 1] = src_port & 0xFF;
    frame[udp_off + 2] = (dst_port >> 8) & 0xFF;
    frame[udp_off + 3] = dst_port & 0xFF;
    uint16_t udp_len = static_cast<uint16_t>(8 + ecat_len);
    frame[udp_off + 4] = (udp_len >> 8) & 0xFF;
    frame[udp_off + 5] = udp_len & 0xFF;
    frame[udp_off + 6] = 0; frame[udp_off + 7] = 0; // checksum = 0

    // EtherCAT frame header
    size_t ec_off = udp_off + 8;
    uint16_t ec_raw = static_cast<uint16_t>((dgram_total & 0x07FFu) | (1u << 12));
    frame[ec_off + 0] = ec_raw & 0xFF;
    frame[ec_off + 1] = (ec_raw >> 8) & 0xFF;

    // Datagram header
    size_t dg_off = ec_off + 2;
    frame[dg_off + 0] = static_cast<uint8_t>(cmd);
    frame[dg_off + 1] = idx;
    frame[dg_off + 2] = adp & 0xFF;
    frame[dg_off + 3] = (adp >> 8) & 0xFF;
    frame[dg_off + 4] = ado & 0xFF;
    frame[dg_off + 5] = (ado >> 8) & 0xFF;
    uint16_t len_flags = payload_len & 0x07FFu;
    frame[dg_off + 6] = len_flags & 0xFF;
    frame[dg_off + 7] = (len_flags >> 8) & 0xFF;
    frame[dg_off + 8] = 0; frame[dg_off + 9] = 0;

    if (payload && payload_len > 0) {
        std::memcpy(&frame[dg_off + 10], payload, payload_len);
    }
    frame[dg_off + 10 + payload_len] = wkc & 0xFF;
    frame[dg_off + 10 + payload_len + 1] = (wkc >> 8) & 0xFF;

    return frame;
}

/// Compute IPv4 header checksum from a 20-byte header.
static uint16_t computeExpectedChecksum(const uint8_t* ip) {
    uint32_t sum = 0;
    for (int i = 0; i < 20; i += 2) {
        sum += static_cast<uint32_t>(ip[i]) << 8;
        sum += static_cast<uint32_t>(ip[i + 1]);
    }
    while (sum >> 16) sum = (sum & 0xFFFF) + (sum >> 16);
    return static_cast<uint16_t>(~sum & 0xFFFF);
}

// ============================================================================
// Config defaults
// ============================================================================

TEST(MasterUdpEncapsulation, DefaultDisabled) {
    Master::Config config;
    EXPECT_FALSE(config.udp_encapsulation.enabled);
    Master master(config);
    EXPECT_FALSE(master.isUdpEncapsulationEnabled());
}

TEST(MasterUdpEncapsulation, DefaultPortIs34980) {
    Master::Config config;
    EXPECT_EQ(config.udp_encapsulation.destination_port, 0x88A4);
    EXPECT_EQ(config.udp_encapsulation.source_port, 0x88A4);
}

TEST(MasterUdpEncapsulation, DefaultDestinationIsBroadcast) {
    Master::Config config;
    EXPECT_EQ(config.udp_encapsulation.destination_ip, 0xFFFFFFFFu);
}

// ============================================================================
// TX: frame encapsulation verification
// ============================================================================

TEST(MasterUdpEncapsulation, TxProducesUdpEncapsulatedFrame) {
    Master::Config config;
    config.udp_encapsulation.enabled = true;
    config.udp_encapsulation.source_ip = 0xC0A80101;      // 192.168.1.1
    config.udp_encapsulation.destination_ip = 0xC0A801FF;  // 192.168.1.255
    config.udp_encapsulation.source_port = 0x1234;
    config.udp_encapsulation.destination_port = 0x88A4;
    Master master(config);

    std::vector<uint8_t> captured;
    NetworkInterface iface;
    iface.send = [&captured](const uint8_t* data, size_t len) -> bool {
        captured.assign(data, data + len);
        return true;
    };

    uint8_t mac[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
    master.start(iface, mac);

    uint8_t payload[2] = {0x04, 0x00}; // SAFE-OP
    ASSERT_TRUE(master.sendSingleDatagram(
        Command::APRD, 0x42, 0x0000, 0x0130, payload, 2, true));

    ASSERT_FALSE(captured.empty());
    ASSERT_GE(captured.size(), 60u); // minimum Ethernet frame

    // Verify Ethernet header
    EXPECT_EQ(captured[12], 0x08);  // EtherType = 0x0800 (IPv4)
    EXPECT_EQ(captured[13], 0x00);

    // Verify IPv4 header
    const uint8_t* ip = captured.data() + 14;
    EXPECT_EQ(ip[0], 0x45);  // version=4, IHL=5
    EXPECT_EQ(ip[9], 0x11);  // protocol = UDP
    // Source IP = 192.168.1.1
    EXPECT_EQ(ip[12], 192); EXPECT_EQ(ip[13], 168);
    EXPECT_EQ(ip[14], 1); EXPECT_EQ(ip[15], 1);
    // Destination IP = 192.168.1.255
    EXPECT_EQ(ip[16], 192); EXPECT_EQ(ip[17], 168);
    EXPECT_EQ(ip[18], 1); EXPECT_EQ(ip[19], 255);

    // Verify IP checksum: computing the checksum over a header with a valid
    // checksum field should yield 0 (the ones-complement sum is 0xFFFF → ~0 = 0).
    uint16_t verify = computeExpectedChecksum(ip);
    EXPECT_EQ(verify, 0u);

    // Verify UDP header
    const uint8_t* udp = ip + 20;
    uint16_t src_port = (static_cast<uint16_t>(udp[0]) << 8) | udp[1];
    uint16_t dst_port = (static_cast<uint16_t>(udp[2]) << 8) | udp[3];
    EXPECT_EQ(src_port, 0x1234);
    EXPECT_EQ(dst_port, 0x88A4);

    // Verify EtherCAT frame header (after UDP header)
    const uint8_t* ecat = udp + 8;
    uint16_t ec_raw = static_cast<uint16_t>(ecat[0]) |
                      (static_cast<uint16_t>(ecat[1]) << 8);
    uint16_t ec_len = ec_raw & 0x07FFu;
    uint16_t ec_type = (ec_raw >> 12) & 0x0Fu;
    EXPECT_EQ(ec_type, 1u);
    EXPECT_GE(ec_len, 14u); // datagram header + 2 bytes data + 2 bytes WKC

    // Verify datagram
    const uint8_t* dg = ecat + 2;
    EXPECT_EQ(dg[0], static_cast<uint8_t>(Command::APRD));
    EXPECT_EQ(dg[1], 0x42);
    uint16_t adp = static_cast<uint16_t>(dg[2]) | (static_cast<uint16_t>(dg[3]) << 8);
    uint16_t ado = static_cast<uint16_t>(dg[4]) | (static_cast<uint16_t>(dg[5]) << 8);
    EXPECT_EQ(adp, 0x0000);
    EXPECT_EQ(ado, 0x0130);

    // Verify payload
    EXPECT_EQ(dg[10], 0x04);
    EXPECT_EQ(dg[11], 0x00);

    master.stop();
}

TEST(MasterUdpEncapsulation, TxDisabledSendsDirectEtherCAT) {
    Master::Config config;
    config.udp_encapsulation.enabled = false;
    Master master(config);

    std::vector<uint8_t> captured;
    NetworkInterface iface;
    iface.send = [&captured](const uint8_t* data, size_t len) -> bool {
        captured.assign(data, data + len);
        return true;
    };

    uint8_t mac[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
    master.start(iface, mac);

    uint8_t payload[2] = {0x04, 0x00};
    ASSERT_TRUE(master.sendSingleDatagram(
        Command::APRD, 0x42, 0x0000, 0x0130, payload, 2, true));

    ASSERT_FALSE(captured.empty());
    // Should be direct EtherCAT (EtherType 0x88A4), not IPv4
    EXPECT_EQ(captured[12], 0x88);
    EXPECT_EQ(captured[13], 0xA4);

    master.stop();
}

TEST(MasterUdpEncapsulation, TxDefaultDestinationIsBroadcast) {
    Master::Config config;
    config.udp_encapsulation.enabled = true;
    // Leave destination_ip as default (0xFFFFFFFF = broadcast)
    Master master(config);

    std::vector<uint8_t> captured;
    NetworkInterface iface;
    iface.send = [&captured](const uint8_t* data, size_t len) -> bool {
        captured.assign(data, data + len);
        return true;
    };

    uint8_t mac[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
    master.start(iface, mac);

    uint8_t payload[2] = {0x04, 0x00};
    ASSERT_TRUE(master.sendSingleDatagram(
        Command::APRD, 0x42, 0x0000, 0x0130, payload, 2, true));

    ASSERT_FALSE(captured.empty());
    const uint8_t* ip = captured.data() + 14;
    // Destination IP should be 255.255.255.255
    EXPECT_EQ(ip[16], 0xFF); EXPECT_EQ(ip[17], 0xFF);
    EXPECT_EQ(ip[18], 0xFF); EXPECT_EQ(ip[19], 0xFF);

    master.stop();
}

// ============================================================================
// TX: IP identification increments
// ============================================================================

TEST(MasterUdpEncapsulation, TxIpIdentificationIncrements) {
    Master::Config config;
    config.udp_encapsulation.enabled = true;
    Master master(config);

    std::vector<std::vector<uint8_t>> captured_frames;
    NetworkInterface iface;
    iface.send = [&captured_frames](const uint8_t* data, size_t len) -> bool {
        captured_frames.emplace_back(data, data + len);
        return true;
    };

    uint8_t mac[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
    master.start(iface, mac);

    uint8_t payload[2] = {0x04, 0x00};
    ASSERT_TRUE(master.sendSingleDatagram(Command::APRD, 0x01, 0x0000, 0x0130, payload, 2, true));
    ASSERT_TRUE(master.sendSingleDatagram(Command::APRD, 0x02, 0x0000, 0x0130, payload, 2, true));

    ASSERT_EQ(captured_frames.size(), 2u);

    // Extract IP identification from both frames
    auto extract_ip_id = [](const std::vector<uint8_t>& f) -> uint16_t {
        return (static_cast<uint16_t>(f[14 + 4]) << 8) | f[14 + 5];
    };

    uint16_t id1 = extract_ip_id(captured_frames[0]);
    uint16_t id2 = extract_ip_id(captured_frames[1]);
    // IDs should differ (incrementing counter)
    EXPECT_NE(id1, id2);

    master.stop();
}

// ============================================================================
// TX: size limit enforcement
// ============================================================================

TEST(MasterUdpEncapsulation, TxRejectsOversizedInUdpMode) {
    Master::Config config;
    config.udp_encapsulation.enabled = true;
    Master master(config);

    NetworkInterface iface;
    iface.send = [](const uint8_t*, size_t) -> bool { return true; };

    uint8_t mac[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
    master.start(iface, mac);

    // In UDP mode, the max datalen is reduced by 28 bytes (IP+UDP overhead).
    // Direct mode max: 1514 - 26 (header) - 2 (WKC) = 1486
    // UDP mode max: 1486 - 28 = 1458
    // Use 1480 bytes which fits in direct mode but exceeds UDP mode.
    std::vector<uint8_t> big(1480, 0);
    EXPECT_FALSE(master.sendSingleDatagram(
        Command::APWR, 0x01, 0x0000, 0x0130, big.data(), 1480, false));

    master.stop();
}

// ============================================================================
// RX: parsing UDP-encapsulated frames
// ============================================================================

TEST(MasterUdpEncapsulation, RxParsesUdpEncapsulatedFrame) {
    Master master;

    uint8_t payload[2] = {0x02, 0x00}; // PRE_OP
    auto frame = buildUdpEtherCATFrame(
        Command::APRD, 0x55, 0x0000, 0x0130, payload, 2, 1);

    // This should be parsed correctly even though the master is not in UDP mode,
    // because parseEtherCATFrame handles both formats.
    // The packet will be unrouted (no waiter registered), but it should not crash.
    master.handleRxFrame(frame.data(), frame.size());
    // No crash = success. The datagram is routed to the rx_queue or dropped.
}

TEST(MasterUdpEncapsulation, RxParsesUdpEncapsulatedFrameWithWaiter) {
    Master master;

    uint8_t payload[2] = {0x04, 0x00}; // SAFE-OP
    auto frame = buildUdpEtherCATFrame(
        Command::APRD, 0x77, 0x0000, 0x0130, payload, 2, 1);

    // Start a waiter thread BEFORE feeding the frame (race-free).
    std::atomic<bool> got_response{false};
    RxDatagram rx{};
    std::thread waiter([&]() {
        got_response.store(master.waitForResponseIdx(0x77, 500, rx));
    });

    // Small delay to ensure the waiter is registered.
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    master.handleRxFrame(frame.data(), frame.size());

    waiter.join();
    ASSERT_TRUE(got_response.load());
    EXPECT_EQ(rx.wkc, 1u);
    EXPECT_EQ(rx.datalen, 2u);
    EXPECT_EQ(rx.data[0], 0x04);
    EXPECT_EQ(rx.data[1], 0x00);
}

TEST(MasterUdpEncapsulation, RxIgnoresNonEtherCatUdp) {
    Master master;

    // Build a UDP frame with a non-EtherCAT destination port
    auto frame = buildUdpEtherCATFrame(
        Command::APRD, 0x55, 0x0000, 0x0130, nullptr, 2, 1,
        0xC0A80101, 0xC0A801FF, 0x88A4, 12345); // wrong dst port

    // Start a waiter thread — it should time out (frame ignored).
    std::atomic<bool> got_response{true};
    RxDatagram rx{};
    std::thread waiter([&]() {
        got_response.store(master.waitForResponseIdx(0x55, 100, rx));
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    master.handleRxFrame(frame.data(), frame.size());

    waiter.join();
    EXPECT_FALSE(got_response.load()); // should time out — frame was ignored
}

TEST(MasterUdpEncapsulation, RxStillHandlesDirectEtherCAT) {
    // When UDP mode is enabled on the master, it should still accept
    // direct EtherCAT frames on RX (mixed-mode support).
    Master::Config config;
    config.udp_encapsulation.enabled = true;
    Master master(config);

    uint8_t payload[2] = {0x08, 0x00}; // OP
    auto frame = buildDirectEtherCATFrame(
        Command::APRD, 0x88, 0x0000, 0x0130, payload, 2, 1);

    std::atomic<bool> got_response{false};
    RxDatagram rx{};
    std::thread waiter([&]() {
        got_response.store(master.waitForResponseIdx(0x88, 500, rx));
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    master.handleRxFrame(frame.data(), frame.size());

    waiter.join();
    ASSERT_TRUE(got_response.load());
    EXPECT_EQ(rx.wkc, 1u);
    EXPECT_EQ(rx.data[0], 0x08);
    EXPECT_EQ(rx.data[1], 0x00);
}

// ============================================================================
// RX: edge cases
// ============================================================================

TEST(MasterUdpEncapsulation, RxUdpFrameTooShort) {
    Master master;

    // Build a valid-looking UDP frame but truncate it
    auto frame = buildUdpEtherCATFrame(
        Command::APRD, 0x55, 0x0000, 0x0130, nullptr, 2, 1);
    // Truncate to just the Ethernet + IP header (no UDP or EtherCAT)
    master.handleRxFrame(frame.data(), 14 + 20); // should not crash
}

TEST(MasterUdpEncapsulation, RxUdpFrameWithNonUdpProtocol) {
    Master master;

    auto frame = buildUdpEtherCATFrame(
        Command::APRD, 0x55, 0x0000, 0x0130, nullptr, 2, 1);
    // Change IP protocol from UDP (0x11) to TCP (0x06)
    frame[14 + 9] = 0x06;

    // Start a waiter thread — it should time out (frame ignored).
    std::atomic<bool> got_response{true};
    RxDatagram rx{};
    std::thread waiter([&]() {
        got_response.store(master.waitForResponseIdx(0x55, 100, rx));
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    master.handleRxFrame(frame.data(), frame.size());

    waiter.join();
    EXPECT_FALSE(got_response.load()); // should be ignored
}

#endif // TETHER_ENABLE_UDP_ENCAPSULATION
