# Single source of truth for the Tether version.
# Included by the top-level CMakeLists.txt (which feeds project(VERSION)) and
# by CMakeLists_component.txt for ESP-IDF component builds, so the generated
# TetherConfig.hpp reports the same version on both build paths.
set(TETHER_VERSION_MAJOR 2)
set(TETHER_VERSION_MINOR 0)
set(TETHER_VERSION_PATCH 0)
set(TETHER_VERSION "${TETHER_VERSION_MAJOR}.${TETHER_VERSION_MINOR}.${TETHER_VERSION_PATCH}")
