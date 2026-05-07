cmake_minimum_required(VERSION 3.16)
project(DummyGCodeApp C CXX)

set(CMAKE_CXX_STANDARD 20)

# Add Tether explicitly excluding all its targets by default.
# Only the targets that this application actually links to will be compiled!
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../.. Tether EXCLUDE_FROM_ALL)

add_executable(dummy_gcode_app main.cpp)

# We link ONLY the gcode component. This automatically pulls in its dependencies
# (like tether_common) but avoids building EtherCAT, Motion, etc.
target_link_libraries(dummy_gcode_app PRIVATE tether::gcode)
