cmake_minimum_required(VERSION 3.10)
project(libslip_test CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Project root (one level up from test/)
set(PROJECT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/..")

include_directories(${PROJECT_ROOT}/include)

# Enable code coverage if requested
if(ENABLE_COVERAGE)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        add_compile_options(-fprofile-arcs -ftest-coverage)
        add_link_options(--coverage)
    endif()
endif()

# Use system-installed GoogleTest (expect it to be available on the host)
find_package(GTest REQUIRED)
add_executable(test_all
    test_all_main.cpp
    test_slipbuffer.cpp
    test_slipencoder.cpp
    test_comprehensive.cpp
    ${PROJECT_ROOT}/src/Buffer.cpp
    ${PROJECT_ROOT}/src/Decoder.cpp
    ${PROJECT_ROOT}/src/Encoder.cpp
)
target_include_directories(test_all PRIVATE ${PROJECT_ROOT}/include)
target_link_libraries(test_all PRIVATE GTest::gtest pthread)

# Link coverage library if coverage is enabled
if(ENABLE_COVERAGE)
    target_link_libraries(test_all PRIVATE --coverage)
endif()

add_test(NAME AllTests COMMAND test_all)
