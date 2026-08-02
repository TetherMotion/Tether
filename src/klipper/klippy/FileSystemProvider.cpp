/**
 * @file FileSystemProvider.cpp
 * @brief StdFileSystemProvider implementation using std::filesystem.
 */

#include "tether/klipper/klippy/FileSystemProvider.hpp"

#include <filesystem>
#include <fstream>

namespace tether::klipper::klippy {

bool StdFileSystemProvider::fileExists(const std::string& path) {
    return std::filesystem::exists(path) && std::filesystem::is_regular_file(path);
}

std::optional<std::string> StdFileSystemProvider::readFile(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return std::nullopt;
    std::string content((std::istreambuf_iterator<char>(f)),
                         std::istreambuf_iterator<char>());
    return content;
}

bool StdFileSystemProvider::writeFile(const std::string& path, const std::string& content) {
    std::ofstream f(path, std::ios::binary);
    if (!f) return false;
    f.write(content.data(), static_cast<std::streamsize>(content.size()));
    return f.good();
}

size_t StdFileSystemProvider::fileSize(const std::string& path) {
    std::error_code ec;
    auto size = std::filesystem::file_size(path, ec);
    if (ec) return 0;
    return static_cast<size_t>(size);
}

std::vector<FileInfo> StdFileSystemProvider::listFiles(const std::string& dirPath) {
    std::vector<FileInfo> result;
    if (!std::filesystem::exists(dirPath)) return result;
    for (const auto& entry : std::filesystem::directory_iterator(dirPath)) {
        if (entry.is_regular_file()) {
            FileInfo info;
            info.name = entry.path().filename().string();
            info.size = static_cast<size_t>(entry.file_size());
            result.push_back(std::move(info));
        }
    }
    return result;
}

bool StdFileSystemProvider::deleteFile(const std::string& path) {
    std::error_code ec;
    return std::filesystem::remove(path, ec);
}

} // namespace tether::klipper::klippy
