# Stub FindBrotli.cmake — Tether's HTTP server does not use Brotli.
set(Brotli_FOUND TRUE)
if(NOT TARGET Brotli::Brotli)
    add_library(Brotli::Brotli INTERFACE IMPORTED)
endif()
