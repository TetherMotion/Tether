cmake_minimum_required(VERSION 3.16)
project(extract_esi)

# This tool depends on tinyxml2 and the tether library
find_package(TinyXML2 QUIET)
if(NOT TinyXML2_FOUND)
    include(FetchContent)
    FetchContent_Declare(tinyxml2 GIT_REPOSITORY https://github.com/leethomason/tinyxml2.git GIT_TAG 9.0.0)
    FetchContent_MakeAvailable(tinyxml2)
    # tinyxml2 provides target tinyxml2::tinyxml2 in modern cmakes
    set(TinyXML2_FOUND TRUE)
endif()

add_executable(extract_esi
    extract_esi.cpp
)

target_include_directories(extract_esi PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include
)

target_link_libraries(extract_esi PRIVATE
    tinyxml2
    tether_ethercat_common
)

# Installable utility (optional)
install(TARGETS extract_esi RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
