/**
 * @file test_klipper_test_helpers.cpp
 * @brief Tests for the shared test helpers.
 */

#include <gtest/gtest.h>
#include "test_helpers.hpp"

#include <filesystem>
#include <fstream>

using namespace tether::klipper::test;

TEST(TestHelpersTest, UniqueSocketPathIsUnique) {
    std::string a = uniqueSocketPath();
    std::string b = uniqueSocketPath();
    EXPECT_NE(a, b);
}

TEST(TestHelpersTest, UniqueSocketPathHasCorrectPrefix) {
    std::string path = uniqueSocketPath("myprefix");
    EXPECT_NE(path.find("myprefix"), std::string::npos);
    EXPECT_NE(path.find("/tmp/"), std::string::npos);
}

TEST(TestHelpersTest, UniqueTempDirIsUnique) {
    std::string a = uniqueTempDir();
    std::string b = uniqueTempDir();
    EXPECT_NE(a, b);
}

TEST(TestHelpersTest, UniqueTempFileIsUnique) {
    std::string a = uniqueTempFile();
    std::string b = uniqueTempFile();
    EXPECT_NE(a, b);
}

TEST(TestHelpersTest, CreateTempConfigWritesContent) {
    std::string path = createTempConfig("[stepper_x]\nstep_distance=0.01\n");
    EXPECT_TRUE(std::filesystem::exists(path));
    std::ifstream f(path);
    std::string content((std::istreambuf_iterator<char>(f)),
                         std::istreambuf_iterator<char>());
    EXPECT_EQ(content, "[stepper_x]\nstep_distance=0.01\n");
    std::filesystem::remove(path);
}

TEST(TestHelpersTest, TempDirCreatesAndRemoves) {
    std::string path;
    {
        TempDir dir("tether_helper_test");
        path = dir.path();
        EXPECT_TRUE(std::filesystem::exists(path));
        EXPECT_TRUE(std::filesystem::is_directory(path));
    }
    EXPECT_FALSE(std::filesystem::exists(path));
}

TEST(TestHelpersTest, TempFileCreatesAndRemoves) {
    std::string path;
    {
        TempFile file("hello world", "tether_helper_test");
        path = file.path();
        EXPECT_TRUE(std::filesystem::exists(path));
        std::ifstream f(path);
        std::string content((std::istreambuf_iterator<char>(f)),
                             std::istreambuf_iterator<char>());
        EXPECT_EQ(content, "hello world");
    }
    EXPECT_FALSE(std::filesystem::exists(path));
}

TEST(TestHelpersTest, TempSocketPathCreatesAndRemoves) {
    std::string path;
    {
        TempSocketPath sock("tether_helper_test");
        path = sock.path();
        // The path should be a valid string but the socket file doesn't
        // exist until a server binds to it.
        EXPECT_FALSE(path.empty());
    }
    // After destruction, the file should not exist (it never did, but
    // the destructor should not crash).
    EXPECT_FALSE(std::filesystem::exists(path));
}

TEST(TestHelpersTest, MultipleTempDirsCoexist) {
    TempDir dir1("tether_helper_test");
    TempDir dir2("tether_helper_test");
    EXPECT_NE(dir1.path(), dir2.path());
    EXPECT_TRUE(std::filesystem::exists(dir1.path()));
    EXPECT_TRUE(std::filesystem::exists(dir2.path()));
}
