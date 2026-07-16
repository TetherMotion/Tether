cmake_minimum_required(VERSION 3.16)
project(extract_esi)

# tether_esi provides ESIParser.cpp + tinyxml2 linkage
add_executable(extract_esi
    extract_esi.cpp
)

target_include_directories(extract_esi PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include
)

target_link_libraries(extract_esi PRIVATE
    tether_esi
)

# Installable utility (optional)
install(TARGETS extract_esi RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
