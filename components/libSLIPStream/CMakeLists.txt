# This CMakeLists.txt is intentionally an ESP-IDF component file.
# Use `test/CMakeLists.txt` for standalone C++ test builds.

idf_component_register(SRCS "src/Decoder.cpp" "src/Encoder.cpp" "src/Buffer.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES driver)
