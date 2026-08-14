# Stub Findcoz-profiler.cmake — Tether's HTTP server does not use coz-profiler.
set(coz-profiler_FOUND TRUE)
if(NOT TARGET coz-profiler::coz-profiler)
    add_library(coz-profiler::coz-profiler INTERFACE IMPORTED)
endif()
