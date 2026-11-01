/**
 * @file FileSystemProvider.hpp
 * @brief Injectable filesystem interface for testability.
 *
 * @details
 * Provides an abstract interface for filesystem operations used by the
 * UDS server's file endpoints (load/save config, list files, etc.).
 * The default implementation uses std::filesystem. Tests can inject
 * an in-memory mock to avoid touching the real filesystem.
 */

#pragma once

#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace tether::klipper::klippy {

/// @brief File information.
struct FileInfo {
    std::string name;
    size_t size = 0;
};

/// @brief Abstract interface for filesystem operations.
class IFileSystemProvider {
public:
    virtual ~IFileSystemProvider() = default;

    /**
     * @brief Check if a file exists.
     * @param path File path relative to the provider root.
     * @return `true` when the file exists and is readable
     */
    virtual bool fileExists(const std::string& path) = 0;

    /**
     * @brief Read a file's contents.
     * @param path File path relative to the provider root.
     * @return Full file contents, or nullopt on failure
     */
    virtual std::optional<std::string> readFile(const std::string& path) = 0;

    /**
     * @brief Write data to a file.
     * @param path File path relative to the provider root.
     * @param content Data written (file is truncated first).
     * @return `true` on success
     */
    virtual bool writeFile(const std::string& path, const std::string& content) = 0;

    /**
     * @brief Get file size.
     * @param path File path relative to the provider root.
     * @return Size in bytes; 0 if the file doesn't exist
     */
    virtual size_t fileSize(const std::string& path) = 0;

    /**
     * @brief List files in a directory.
     * @param dirPath Directory path relative to the provider root.
     * @return Entries in the directory; empty on error
     */
    virtual std::vector<FileInfo> listFiles(const std::string& dirPath) = 0;

    /**
     * @brief Delete a file.
     * @param path File path relative to the provider root.
     * @return `true` on success
     */
    virtual bool deleteFile(const std::string& path) = 0;
};

/// @brief Default implementation using std::filesystem.
class StdFileSystemProvider : public IFileSystemProvider {
public:
    bool fileExists(const std::string& path) override;
    std::optional<std::string> readFile(const std::string& path) override;
    bool writeFile(const std::string& path, const std::string& content) override;
    size_t fileSize(const std::string& path) override;
    std::vector<FileInfo> listFiles(const std::string& dirPath) override;
    bool deleteFile(const std::string& path) override;
};

/// @brief In-memory mock for testing.
class MockFileSystemProvider : public IFileSystemProvider {
public:
    /// Map of path -> file content.
    std::map<std::string, std::string> files;

    bool fileExists(const std::string& path) override {
        return files.count(path) > 0;
    }

    std::optional<std::string> readFile(const std::string& path) override {
        auto it = files.find(path);
        if (it == files.end()) return std::nullopt;
        return it->second;
    }

    bool writeFile(const std::string& path, const std::string& content) override {
        files[path] = content;
        return true;
    }

    size_t fileSize(const std::string& path) override {
        auto it = files.find(path);
        if (it == files.end()) return 0;
        return it->second.size();
    }

    std::vector<FileInfo> listFiles(const std::string& dirPath) override {
        std::vector<FileInfo> result;
        std::string prefix = dirPath;
        if (!prefix.empty() && prefix.back() != '/') prefix += '/';
        for (const auto& [path, content] : files) {
            if (path.rfind(prefix, 0) == 0) {
                FileInfo info;
                info.name = path.substr(prefix.size());
                info.size = content.size();
                result.push_back(std::move(info));
            }
        }
        return result;
    }

    bool deleteFile(const std::string& path) override {
        return files.erase(path) > 0;
    }
};

} // namespace tether::klipper::klippy
