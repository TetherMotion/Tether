#pragma once

/**
 * @file FramedChild.hpp
 * @brief Spawned child process speaking a tiny length-prefixed frame
 *        protocol over stdin/stdout pipes — the transport Tether TUIs
 *        use to drive headless tool backends (e.g. an agent CLI's
 *        `--monitor` or `chat --rpc` modes).
 *
 * Wire format (both directions):
 *   `<KIND> <payload-len>\n<payload-len raw bytes>`
 * plus the bare line `QUIT` to detach (no payload).
 *
 * The child's stderr can either pass through to our own stderr or be
 * captured on a third pipe and drained into a log view — capture is
 * the right choice while a curses UI owns the terminal.
 *
 * All I/O on our side is non-blocking; call poll() from a UI tick.
 */

#include <cerrno>
#include <csignal>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include <fcntl.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include "tether/utils/FramePump.hpp"

namespace Tether {
namespace Utils {

/// Options for FramedChild::spawn().
struct FramedChildOpts {
    /// Capture stderr on a third pipe (drainErr()); else pass through.
    bool capture_stderr = true;
    /// Append "--monitor" right after argv[0]? No — callers control the
    /// full argv; this exists only to document that the child gets
    /// exe + args verbatim via execvp(exe, [exe, args...]).
};

class FramedChild : public IFramePump {
public:
    FramedChild() = default;
    FramedChild(const FramedChild&) = delete;
    FramedChild& operator=(const FramedChild&) = delete;
    ~FramedChild() { quit(); }

    /**
     * Spawn `exe` with arguments `args` (argv = [exe, args...]).
     * stdin/stdout become the frame pipes; stderr per `opts`.
     */
    bool spawn(const std::string& exe, const std::vector<std::string>& args,
               const FramedChildOpts& opts = {}) {
        int pin[2], pout[2], perr[2] = {-1, -1};
        if (pipe(pin) || pipe(pout)) return false;
        if (opts.capture_stderr && pipe(perr)) {
            close(pin[0]); close(pin[1]); close(pout[0]); close(pout[1]);
            return false;
        }
        pid_ = fork();
        if (pid_ < 0) return false;
        if (pid_ == 0) {
            dup2(pin[0], STDIN_FILENO);
            dup2(pout[1], STDOUT_FILENO);
            if (opts.capture_stderr) dup2(perr[1], STDERR_FILENO);
            for (int fd : {pin[0], pin[1], pout[0], pout[1], perr[0], perr[1]})
                if (fd >= 0) close(fd);
            std::vector<char*> argv;
            argv.push_back(const_cast<char*>(exe.c_str()));
            for (auto& a : args) argv.push_back(const_cast<char*>(a.c_str()));
            argv.push_back(nullptr);
            execvp(exe.c_str(), argv.data());
            _exit(127);
        }
        close(pin[0]);
        close(pout[1]);
        if (opts.capture_stderr) {
            close(perr[1]);
            err_ = perr[0];
            fcntl(err_, F_SETFL, O_NONBLOCK);
        }
        to_child_ = pin[1];
        from_child_ = pout[0];
        fcntl(from_child_, F_SETFL, O_NONBLOCK);
        return true;
    }

    bool alive() const override { return !dead_; }
    pid_t pid() const { return pid_; }

    /**Send one frame. */
    void sendFrame(const char* kind, const std::string& payload) {
        if (to_child_ < 0) return;
        std::string hdr = std::string(kind) + " " + std::to_string(payload.size()) + "\n";
        (void)!write(to_child_, hdr.data(), hdr.size());
        (void)!write(to_child_, payload.data(), payload.size());
    }

    /**Bare `QUIT` line, close stdin, wait briefly, SIGTERM the rest. */
    void quit() {
        if (to_child_ >= 0) {
            const char q[] = "QUIT\n";
            (void)!write(to_child_, q, sizeof(q) - 1);
            close(to_child_);
            to_child_ = -1;
        }
        if (pid_ > 0) {
            for (int i = 0; i < 20 && waitpid(pid_, nullptr, WNOHANG) == 0; ++i)
                usleep(50 * 1000);
            if (waitpid(pid_, nullptr, WNOHANG) == 0) kill(pid_, SIGTERM);
            waitpid(pid_, nullptr, 0);
            pid_ = -1;
        }
        if (from_child_ >= 0) { close(from_child_); from_child_ = -1; }
        if (err_ >= 0) { close(err_); err_ = -1; }
    }

    /**
     * Pump available bytes; returns every complete frame received
     * since the last call. Sets `dead_` on EOF/EBADF.
     */
    std::vector<Frame> poll() override {
        std::vector<Frame> out;
        if (from_child_ < 0) return out;
        char buf[16384];
        for (;;) {
            ssize_t n = read(from_child_, buf, sizeof(buf));
            if (n > 0) { rx_.append(buf, size_t(n)); continue; }
            if (n == 0) { dead_ = true; break; }
            if (errno != EAGAIN && errno != EWOULDBLOCK) dead_ = true;
            break;
        }
        for (;;) {
            auto nl = rx_.find('\n');
            if (nl == std::string::npos) break;
            std::string hdr = rx_.substr(0, nl);
            auto sp = hdr.find(' ');
            if (sp == std::string::npos) { rx_.erase(0, nl + 1); continue; }
            size_t len = 0;
            try { len = std::stoul(hdr.substr(sp + 1)); }
            catch (...) { rx_.erase(0, nl + 1); continue; }
            if (rx_.size() < nl + 1 + len) break; // wait for the payload
            Frame f;
            f.kind = hdr.substr(0, sp);
            f.payload = rx_.substr(nl + 1, len);
            rx_.erase(0, nl + 1 + len);
            out.push_back(std::move(f));
        }
        return out;
    }

    /**
     * Drain captured stderr (spawn with capture_stderr): returns the
     * raw bytes received since the last call — log output, panics,
     * backtraces. Complete lines are preferred by callers; trailing
     * partial line bytes are kept buffered here.
     */
    std::string drainErr() override {
        if (err_ < 0) return {};
        char buf[8192];
        for (;;) {
            ssize_t n = read(err_, buf, sizeof(buf));
            if (n > 0) { rxerr_.append(buf, size_t(n)); continue; }
            break; // EAGAIN or EOF — either way, take what we have
        }
        auto nl = rxerr_.rfind('\n');
        if (nl == std::string::npos) return {};
        std::string done = rxerr_.substr(0, nl + 1);
        rxerr_.erase(0, nl + 1);
        return done;
    }

private:
    pid_t pid_ = -1;
    int to_child_ = -1;
    int from_child_ = -1;
    int err_ = -1;
    std::string rx_;
    std::string rxerr_;
    bool dead_ = false;
};

}  // namespace Utils
}  // namespace Tether
