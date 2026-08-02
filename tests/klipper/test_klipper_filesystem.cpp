/**
 * @file test_klipper_filesystem.cpp
 * @brief Tests for injectable filesystem provider.
 */

#include <gtest/gtest.h>
#include "tether/klipper/klippy/FileSystemProvider.hpp"

#include <cstdlib>
#include <filesystem>
#include <string>

using namespace tether::klipper::klippy;

// --- MockFileSystemProvider tests ---

TEST(MockFileSystemTest, WriteAndReadFile) {
    MockFileSystemProvider fs;
    EXPECT_TRUE(fs.writeFile("/tmp/test.txt", "hello world"));
    auto content = fs.readFile("/tmp/test.txt");
    ASSERT_TRUE(content.has_value());
    EXPECT_EQ(*content, "hello world");
}

TEST(MockFileSystemTest, ReadNonExistentFile) {
    MockFileSystemProvider fs;
    auto content = fs.readFile("/nonexistent");
    EXPECT_FALSE(content.has_value());
}

TEST(MockFileSystemTest, FileExists) {
    MockFileSystemProvider fs;
    EXPECT_FALSE(fs.fileExists("/test.cfg"));
    fs.writeFile("/test.cfg", "content");
    EXPECT_TRUE(fs.fileExists("/test.cfg"));
}

TEST(MockFileSystemTest, FileSize) {
    MockFileSystemProvider fs;
    fs.writeFile("/test.txt", "12345");
    EXPECT_EQ(fs.fileSize("/test.txt"), 5u);
    EXPECT_EQ(fs.fileSize("/nonexistent"), 0u);
}

TEST(MockFileSystemTest, ListFiles) {
    MockFileSystemProvider fs;
    fs.writeFile("/dir/a.txt", "aaa");
    fs.writeFile("/dir/b.txt", "bbbb");
    fs.writeFile("/other/c.txt", "c");

    auto files = fs.listFiles("/dir");
    EXPECT_EQ(files.size(), 2u);
    // Check that both files are listed (order may vary).
    std::set<std::string> names;
    for (const auto& f : files) names.insert(f.name);
    EXPECT_TRUE(names.count("a.txt"));
    EXPECT_TRUE(names.count("b.txt"));
}

TEST(MockFileSystemTest, DeleteFile) {
    MockFileSystemProvider fs;
    fs.writeFile("/test.txt", "content");
    EXPECT_TRUE(fs.deleteFile("/test.txt"));
    EXPECT_FALSE(fs.fileExists("/test.txt"));
    EXPECT_FALSE(fs.deleteFile("/nonexistent"));
}

TEST(MockFileSystemTest, OverwriteFile) {
    MockFileSystemProvider fs;
    fs.writeFile("/test.txt", "original");
    fs.writeFile("/test.txt", "overwritten");
    auto content = fs.readFile("/test.txt");
    ASSERT_TRUE(content.has_value());
    EXPECT_EQ(*content, "overwritten");
}

// --- StdFileSystemProvider tests ---

class StdFileSystemTest : public ::testing::Test {
protected:
    void SetUp() override {
        testDir_ = "/tmp/tether_test_" + std::to_string(getpid());
        std::filesystem::create_directories(testDir_);
    }
    void TearDown() override {
        std::error_code ec;
        std::filesystem::remove_all(testDir_, ec);
    }
    std::string testDir_;
};

TEST_F(StdFileSystemTest, WriteAndReadFile) {
    StdFileSystemProvider fs;
    std::string path = testDir_ + "/test.txt";
    EXPECT_TRUE(fs.writeFile(path, "hello world"));
    auto content = fs.readFile(path);
    ASSERT_TRUE(content.has_value());
    EXPECT_EQ(*content, "hello world");
}

TEST_F(StdFileSystemTest, ReadNonExistentFile) {
    StdFileSystemProvider fs;
    auto content = fs.readFile(testDir_ + "/nonexistent");
    EXPECT_FALSE(content.has_value());
}

TEST_F(StdFileSystemTest, FileExists) {
    StdFileSystemProvider fs;
    std::string path = testDir_ + "/test.cfg";
    EXPECT_FALSE(fs.fileExists(path));
    fs.writeFile(path, "content");
    EXPECT_TRUE(fs.fileExists(path));
}

TEST_F(StdFileSystemTest, FileSize) {
    StdFileSystemProvider fs;
    std::string path = testDir_ + "/test.txt";
    fs.writeFile(path, "12345");
    EXPECT_EQ(fs.fileSize(path), 5u);
}

TEST_F(StdFileSystemTest, ListFiles) {
    StdFileSystemProvider fs;
    fs.writeFile(testDir_ + "/a.txt", "aaa");
    fs.writeFile(testDir_ + "/b.txt", "bbbb");
    auto files = fs.listFiles(testDir_);
    EXPECT_EQ(files.size(), 2u);
}

TEST_F(StdFileSystemTest, DeleteFile) {
    StdFileSystemProvider fs;
    std::string path = testDir_ + "/test.txt";
    fs.writeFile(path, "content");
    EXPECT_TRUE(fs.deleteFile(path));
    EXPECT_FALSE(fs.fileExists(path));
}
