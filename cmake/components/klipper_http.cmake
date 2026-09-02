# Component: tether_klipper_http
# Native HTTP/WebSocket server for Mainsail/Fluidd frontends.

if(NOT TETHER_ENABLE_KLIPPER_HTTP)
    return()
endif()
#
# Implements the full Moonraker HTTP + WebSocket API directly in C++,
# using Drogon as the web server backend and Glaze for JSON serialization.
# This eliminates the need for a Moonraker process between Tether and the
# web frontends.
#
# Dependencies:
#   - tether_klipper (for KlippyServer endpoint handlers)
#   - Drogon (HTTP/WebSocket framework)
#   - Glaze (JSON serialization, already a submodule)
#
# This component is optional and gated behind TETHER_ENABLE_KLIPPER_HTTP.
# It links against tether_klipper and reuses all existing endpoint handlers
# via KlippyServer::callEndpoint().

file(GLOB_RECURSE TETHER_KLIPPER_HTTP_SOURCES CONFIGURE_DEPENDS
    "${TETHER_ROOT}/src/klipper/http/*.cpp"
)

# Filter to only existing files
set(TETHER_KLIPPER_HTTP_SOURCES_FILTERED "")
foreach(src ${TETHER_KLIPPER_HTTP_SOURCES})
    if(EXISTS ${src})
        list(APPEND TETHER_KLIPPER_HTTP_SOURCES_FILTERED ${src})
    endif()
endforeach()

# Create variant targets
set(_http_variants "")
if(TETHER_BUILD_SHARED_LIBS)
    add_library(tether_klipper_http_shared SHARED ${TETHER_KLIPPER_HTTP_SOURCES_FILTERED})
    list(APPEND _http_variants tether_klipper_http_shared)
endif()
if(TETHER_BUILD_STATIC_LIBS)
    add_library(tether_klipper_http_static STATIC ${TETHER_KLIPPER_HTTP_SOURCES_FILTERED})
    list(APPEND _http_variants tether_klipper_http_static)
endif()

foreach(_tgt IN LISTS _http_variants)
    target_include_directories(${_tgt}
        PUBLIC
            $<BUILD_INTERFACE:${TETHER_ROOT}/include>
            $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether>
            $<BUILD_INTERFACE:${GLAZE_PATH}/include>
        PRIVATE
            $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/klipper>
    )

    target_compile_features(${_tgt} PRIVATE cxx_std_23)

    # Link against tether_klipper (shared preferred, then static)
    if(TARGET tether_klipper_shared)
        target_link_libraries(${_tgt} PUBLIC tether_klipper_shared)
    elseif(TARGET tether_klipper_static)
        target_link_libraries(${_tgt} PUBLIC tether_klipper_static)
    endif()

    # Link against Drogon
    target_link_libraries(${_tgt} PUBLIC Drogon::Drogon)

    # Threads (for any internal threading)
    find_package(Threads REQUIRED)
    target_link_libraries(${_tgt} PRIVATE Threads::Threads)

    set_target_properties(${_tgt} PROPERTIES
        POSITION_INDEPENDENT_CODE ON
    )
endforeach()

# Export for install/collection
set(TETHER_KLIPPER_HTTP_TARGETS ${_http_variants})

# Alias targets for convenience (shared preferred, matches other components)
if(TARGET tether_klipper_http_shared)
    add_library(tether_klipper_http ALIAS tether_klipper_http_shared)
    add_library(tether::klipper_http ALIAS tether_klipper_http_shared)
    add_library(Tether::KlipperHttp ALIAS tether_klipper_http_shared)
elseif(TARGET tether_klipper_http_static)
    add_library(tether_klipper_http ALIAS tether_klipper_http_static)
    add_library(tether::klipper_http ALIAS tether_klipper_http_static)
    add_library(Tether::KlipperHttp ALIAS tether_klipper_http_static)
endif()
