/**
 * @file test_helpers.hpp
 * @brief Shared test helpers for the Klipper test suite.
 *
 * @details
 * Provides common utilities used across multiple test files:
 *   - uniqueSocketPath(): Generate a unique UDS socket path per test.
 *   - createTempConfig(): Write a config string to a temp file and return the path.
 *   - TempDir: RAII wrapper for temporary directories.
 *   - TempFile: RAII wrapper for temporary files.
 *
 * This header eliminates the ~15 duplicated uniqueSocketPath() definitions
 * scattered across test files.
 */

#pragma once

#include <atomic>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>

namespace tether::klipper::test {

/// @brief Generate a unique UDS socket path for testing.
/// Uses PID + atomic counter to ensure uniqueness across tests.
inline std::string uniqueSocketPath(const std::string& prefix = "tether_test") {
    static std::atomic<size_t> counter{0};
    return "/tmp/" + prefix + "_" + std::to_string(getpid()) +
           "_" + std::to_string(counter++) + ".sock";
}

/// @brief Generate a unique temp directory path.
inline std::string uniqueTempDir(const std::string& prefix = "tether_test") {
    static std::atomic<size_t> counter{0};
    return "/tmp/" + prefix + "_dir_" + std::to_string(getpid()) +
           "_" + std::to_string(counter++);
}

/// @brief Generate a unique temp file path.
inline std::string uniqueTempFile(const std::string& prefix = "tether_test",
                                   const std::string& suffix = ".cfg") {
    static std::atomic<size_t> counter{0};
    return "/tmp/" + prefix + "_" + std::to_string(getpid()) +
           "_" + std::to_string(counter++) + suffix;
}

/// @brief Write content to a temp config file and return the path.
/// The caller is responsible for cleaning up the file.
inline std::string createTempConfig(const std::string& content,
                                     const std::string& prefix = "tether_test_cfg") {
    std::string path = uniqueTempFile(prefix, ".cfg");
    std::ofstream f(path);
    f << content;
    f.close();
    return path;
}

/// @brief RAII wrapper for a temporary directory.
/// Creates the directory on construction, removes it on destruction.
class TempDir {
public:
    explicit TempDir(const std::string& prefix = "tether_test")
        : path_(uniqueTempDir(prefix)) {
        std::filesystem::create_directories(path_);
    }
    ~TempDir() { std::filesystem::remove_all(path_); }

    TempDir(const TempDir&) = delete;
    TempDir& operator=(const TempDir&) = delete;

    const std::string& path() const { return path_; }

private:
    std::string path_;
};

/// @brief RAII wrapper for a temporary file.
/// Creates the file with given content on construction, removes it on destruction.
class TempFile {
public:
    TempFile(const std::string& content, const std::string& prefix = "tether_test")
        : path_(createTempConfig(content, prefix)) {}
    ~TempFile() { std::filesystem::remove(path_); }

    TempFile(const TempFile&) = delete;
    TempFile& operator=(const TempFile&) = delete;

    const std::string& path() const { return path_; }

private:
    std::string path_;
};

/// @brief RAII wrapper for a UDS socket path.
/// Removes the socket file on destruction if it exists.
class TempSocketPath {
public:
    explicit TempSocketPath(const std::string& prefix = "tether_test")
        : path_(uniqueSocketPath(prefix)) {}
    ~TempSocketPath() { std::filesystem::remove(path_); }

    TempSocketPath(const TempSocketPath&) = delete;
    TempSocketPath& operator=(const TempSocketPath&) = delete;

    const std::string& path() const { return path_; }

private:
    std::string path_;
};

} // namespace tether::klipper::test
