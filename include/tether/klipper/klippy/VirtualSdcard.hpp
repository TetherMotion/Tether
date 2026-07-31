#pragma once

/// @file VirtualSdcard.hpp
/// @brief Virtual SD card for G-code file management and playback

#include <filesystem>
#include <fstream>
#include <mutex>
#include <string>
#include <vector>

namespace tether::klipper::klippy {

/// @brief Virtual SD card for G-code file management and playback.
///
/// Implements the Klipper virtual_sdcard module: file listing, opening,
/// and G-code streaming via M23 (select), M24 (start), M25 (pause),
/// M27 (report), and M20 (list).
class VirtualSdcard {
public:
    struct FileInfo {
        std::string path;
        size_t size = 0;
    };

    explicit VirtualSdcard(std::string rootDir)
        : rootDir_(std::move(rootDir)) {}

    /// @brief List files in the virtual SD card root.
    std::vector<FileInfo> listFiles() const {
        std::vector<FileInfo> result;
        if (!std::filesystem::exists(rootDir_)) return result;
        for (const auto& entry : std::filesystem::directory_iterator(rootDir_)) {
            if (entry.is_regular_file()) {
                FileInfo info;
                info.path = entry.path().filename().string();
                info.size = static_cast<size_t>(entry.file_size());
                result.push_back(info);
            }
        }
        return result;
    }

    /// @brief Select a file for printing (M23).
    /// @param filename Filename relative to root.
    /// @return True if file exists and was opened.
    bool selectFile(const std::string& filename) {
        std::lock_guard<std::mutex> lock(mutex_);
        closeFile();
        std::filesystem::path p = std::filesystem::path(rootDir_) / filename;
        if (!std::filesystem::exists(p)) return false;
        filePath_ = filename;
        fileSize_ = static_cast<size_t>(std::filesystem::file_size(p));
        filePosition_ = 0;
        isActive_ = false;
        isPaused_ = false;
        return true;
    }

    /// @brief Start printing the selected file (M24).
    /// @return True if printing started.
    bool startPrint() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (filePath_.empty()) return false;
        isActive_ = true;
        isPaused_ = false;
        return true;
    }

    /// @brief Pause printing (M25).
    void pausePrint() {
        std::lock_guard<std::mutex> lock(mutex_);
        isPaused_ = true;
    }

    /// @brief Resume printing (M24 after pause).
    void resumePrint() {
        std::lock_guard<std::mutex> lock(mutex_);
        isPaused_ = false;
    }

    /// @brief Cancel printing.
    void cancelPrint() {
        std::lock_guard<std::mutex> lock(mutex_);
        isActive_ = false;
        isPaused_ = false;
        filePosition_ = 0;
    }

    /// @brief Close the current file.
    void closeFile() {
        filePath_.clear();
        fileSize_ = 0;
        filePosition_ = 0;
        isActive_ = false;
        isPaused_ = false;
    }

    /// @brief Get the next chunk of G-code from the file.
    /// @param maxLines Maximum lines to read.
    /// @return Lines read, or empty if at EOF / not active.
    std::vector<std::string> readChunk(size_t maxLines = 32) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!isActive_ || isPaused_ || filePath_.empty()) return {};
        std::filesystem::path p = std::filesystem::path(rootDir_) / filePath_;
        std::ifstream file(p);
        if (!file.is_open()) return {};
        file.seekg(static_cast<std::streamoff>(filePosition_));
        std::vector<std::string> lines;
        std::string line;
        size_t bytesRead = 0;
        for (size_t i = 0; i < maxLines && std::getline(file, line); ++i) {
            bytesRead += line.size() + 1;
            lines.push_back(line);
        }
        filePosition_ += bytesRead;
        if (lines.empty() || filePosition_ >= fileSize_) {
            isActive_ = false;
        }
        return lines;
    }

    /// @brief Reset file position to beginning.
    void resetPosition() {
        std::lock_guard<std::mutex> lock(mutex_);
        filePosition_ = 0;
    }

    /// @brief Seek to a specific position in the file.
    void seek(size_t position) {
        std::lock_guard<std::mutex> lock(mutex_);
        filePosition_ = std::min(position, fileSize_);
    }

    // Status accessors
    bool isActive() const { return isActive_; }
    bool isPaused() const { return isPaused_; }
    const std::string& filePath() const { return filePath_; }
    size_t fileSize() const { return fileSize_; }
    size_t filePosition() const { return filePosition_; }
    double progress() const {
        if (fileSize_ == 0) return 0.0;
        return static_cast<double>(filePosition_) / static_cast<double>(fileSize_);
    }

private:
    mutable std::mutex mutex_;
    std::string rootDir_;
    std::string filePath_;
    size_t fileSize_ = 0;
    size_t filePosition_ = 0;
    bool isActive_ = false;
    bool isPaused_ = false;
};

} // namespace tether::klipper::klippy
