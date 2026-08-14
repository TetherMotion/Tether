# Stub FindHiredis.cmake — Tether's HTTP server does not use Hiredis.
set(Hiredis_FOUND TRUE)
if(NOT TARGET Hiredis::Hiredis)
    add_library(Hiredis::Hiredis INTERFACE IMPORTED)
endif()
