/**
 * @file LinuxCan.cpp
 * @brief Linux SocketCAN implementation of the ICan interface.
 *
 * @details
 * Uses the Linux SocketCAN subsystem (PF_CAN, SOCK_RAW). The interface must
 * already exist and be up (e.g. via `ip link set can0 up type can bitrate
 * 500000` or `vcan0` for virtual CAN). This file is only compiled when
 * TETHER_ENABLE_KLIPPER_CAN is ON.
 */

#include "tether/hal/ICan.hpp"

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <cstring>
#include <string>
#include <atomic>
#include <mutex>
#include <deque>

namespace tether::hal {

class LinuxCan : public ICan {
public:
    LinuxCan() = default;
    ~LinuxCan() override { close(); }

    bool open(const CanConfig& config) override {
        if (open_) return true;
        sock_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (sock_ < 0) return false;

        // Bind to the named interface.
        struct ifreq ifr{};
        std::strncpy(ifr.ifr_name, config.interfaceName.c_str(), IFNAMSIZ - 1);
        if (::ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) {
            ::close(sock_); sock_ = -1; return false;
        }
        struct sockaddr_can addr{};
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (::bind(sock_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
            ::close(sock_); sock_ = -1; return false;
        }

        // Loopback / receive-own options.
        int loopback = config.loopback ? 1 : 0;
        ::setsockopt(sock_, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));
        int recvOwn = config.receiveOwn ? 1 : 0;
        ::setsockopt(sock_, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recvOwn, sizeof(recvOwn));

        // Set non-blocking.
        int flags = ::fcntl(sock_, F_GETFL, 0);
        ::fcntl(sock_, F_SETFL, flags | O_NONBLOCK);

        config_ = config;
        open_ = true;
        return true;
    }

    void close() override {
        if (!open_.exchange(false)) return;
        if (sock_ >= 0) { ::close(sock_); sock_ = -1; }
    }

    bool isOpen() const override { return open_; }

    bool send(const CanFrame& frame) override {
        if (!open_ || sock_ < 0) return false;
        if (frame.dlc > 8) return false;
        struct can_frame f{};
        f.can_id = frame.id & 0x7FF; // 11-bit standard ID
        f.can_dlc = frame.dlc;
        std::memcpy(f.data, frame.data, frame.dlc);
        ssize_t n = ::write(sock_, &f, sizeof(f));
        if (n < 0) { stats_.txErrors++; return false; }
        stats_.txFrames++;
        stats_.txBytes += frame.dlc;
        return true;
    }

    bool recv(CanFrame& out, bool canBlock) override {
        if (!open_ || sock_ < 0) return false;
        if (canBlock) {
            // Simple poll-based block.
            struct pollfd pfd{sock_, POLLIN, 0};
            while (open_) {
                int pr = ::poll(&pfd, 1, 100);
                if (pr > 0) break;
            }
        }
        struct can_frame f{};
        ssize_t n = ::read(sock_, &f, sizeof(f));
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) return false;
            stats_.rxErrors++;
            return false;
        }
        out.id = f.can_id & 0x7FF;
        out.dlc = f.can_dlc;
        std::memcpy(out.data, f.data, f.can_dlc);
        stats_.rxFrames++;
        stats_.rxBytes += f.can_dlc;
        return true;
    }

    CanStats stats() const override { return stats_; }

private:
    int sock_ = -1;
    std::atomic<bool> open_{false};
    CanConfig config_;
    CanStats stats_;
};

} // namespace tether::hal
