# Benchmarks CMakeLists.txt
#
# Built only when -DTETHER_BUILD_BENCHMARKS=ON.

# gcode_motion_benchmark — end-to-end G-code motion processing pipeline
# benchmark. Mirrors the WebGCodeViewer processing pipeline using only
# Tether primitives, timing every stage and sub-stage.
add_executable(gcode_motion_benchmark
    gcode_motion_benchmark.cpp)

target_link_libraries(gcode_motion_benchmark PRIVATE
    tether_motion_planner
    tether_gcode
    tether_export
    tether_common)

target_include_directories(gcode_motion_benchmark PRIVATE
    ${PROJECT_SOURCE_DIR}/include)

target_compile_features(gcode_motion_benchmark PRIVATE cxx_std_23)
