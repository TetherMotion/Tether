# Stub FindBoost.cmake — Tether's HTTP server does not use Boost directly.
set(Boost_FOUND TRUE)
if(NOT TARGET Boost::boost)
    add_library(Boost::boost INTERFACE IMPORTED)
endif()
