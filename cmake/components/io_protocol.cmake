# Component: tether_io_protocol
# SLIP-based parameter/signal streaming protocol with transport layer

# ---------------------------------------------------------------------------
# libSLIPspeed submodule
# ---------------------------------------------------------------------------
set(LIBSLIPSPEED_DIR ${TETHER_ROOT}/dependencies/libSLIPspeed)
if(NOT EXISTS ${LIBSLIPSPEED_DIR}/include)
    message(WARNING "libSLIPspeed submodule not checked out at ${LIBSLIPSPEED_DIR}; skipping tether_io_protocol")
    return()
endif()

# Build libSLIPspeed variants
set(_slip_variants "")
if(TETHER_BUILD_SHARED_LIBS)
    add_library(slipspeed_shared SHARED
        ${LIBSLIPSPEED_DIR}/src/Buffer.cpp
        ${LIBSLIPSPEED_DIR}/src/Decoder.cpp
        ${LIBSLIPSPEED_DIR}/src/Encoder.cpp
    )
    list(APPEND _slip_variants slipspeed_shared)
endif()
if(TETHER_BUILD_STATIC_LIBS)
    add_library(slipspeed_static STATIC
        ${LIBSLIPSPEED_DIR}/src/Buffer.cpp
        ${LIBSLIPSPEED_DIR}/src/Decoder.cpp
        ${LIBSLIPSPEED_DIR}/src/Encoder.cpp
    )
    list(APPEND _slip_variants slipspeed_static)
endif()

foreach(_tgt IN LISTS _slip_variants)
    target_include_directories(${_tgt} PUBLIC
        $<BUILD_INTERFACE:${LIBSLIPSPEED_DIR}/include>
        $<INSTALL_INTERFACE:include>
    )
    set_target_properties(${_tgt} PROPERTIES
        POSITION_INDEPENDENT_CODE ON
        CXX_STANDARD 20
        CXX_STANDARD_REQUIRED ON
    )
endforeach()

if(TETHER_BUILD_SHARED_LIBS)
    add_library(slipspeed ALIAS slipspeed_shared)
elseif(TETHER_BUILD_STATIC_LIBS)
    add_library(slipspeed ALIAS slipspeed_static)
endif()

# ---------------------------------------------------------------------------
# Core IO protocol sources
# ---------------------------------------------------------------------------
set(TETHER_IO_PROTOCOL_SOURCES
    ${TETHER_ROOT}/src/io/Registry.cpp
    ${TETHER_ROOT}/src/io/ThresholdFilter.cpp
    ${TETHER_ROOT}/src/io/Datalogging.cpp
    ${TETHER_ROOT}/src/io/Session.cpp
    ${TETHER_ROOT}/src/io/Server.cpp
)

# Transport: TCP is always available; serial only on non-ESP (or with own
# driver on ESP)
list(APPEND TETHER_IO_PROTOCOL_SOURCES
    ${TETHER_ROOT}/src/io/TcpTransport.cpp
)

if(NOT ESP_PLATFORM)
    list(APPEND TETHER_IO_PROTOCOL_SOURCES
        ${TETHER_ROOT}/src/io/SerialTransport.cpp
        ${TETHER_ROOT}/src/io/SpiDriver.cpp
    )
endif()

# Create the IO protocol library variants
set(_variants "")
if(TETHER_BUILD_SHARED_LIBS)
    add_library(tether_io_protocol_shared SHARED ${TETHER_IO_PROTOCOL_SOURCES})
    list(APPEND _variants tether_io_protocol_shared)
endif()
if(TETHER_BUILD_STATIC_LIBS)
    add_library(tether_io_protocol_static STATIC ${TETHER_IO_PROTOCOL_SOURCES})
    list(APPEND _variants tether_io_protocol_static)
endif()

foreach(_tgt IN LISTS _variants)
    target_include_directories(${_tgt}
        PUBLIC
            $<BUILD_INTERFACE:${TETHER_ROOT}/include>
            $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether>
            $<INSTALL_INTERFACE:include>
            $<INSTALL_INTERFACE:include/tether>
        PRIVATE
            ${TETHER_ROOT}/src
    )

    target_link_libraries(${_tgt} PUBLIC tether_common slipspeed)
endforeach()

# Pthreads for Server/Session threading
find_package(Threads REQUIRED)
foreach(_tgt IN LISTS _variants)
    target_link_libraries(${_tgt} PUBLIC Threads::Threads)
    set_target_properties(${_tgt} PROPERTIES
        POSITION_INDEPENDENT_CODE ON
        CXX_STANDARD 20
        CXX_STANDARD_REQUIRED ON
    )
endforeach()

if(TETHER_BUILD_SHARED_LIBS)
    add_library(tether_io_protocol ALIAS tether_io_protocol_shared)
    add_library(tether::io_protocol ALIAS tether_io_protocol_shared)
elseif(TETHER_BUILD_STATIC_LIBS)
    add_library(tether_io_protocol ALIAS tether_io_protocol_static)
    add_library(tether::io_protocol ALIAS tether_io_protocol_static)
endif()

# Export for other components
set(TETHER_IO_PROTOCOL_LIBRARY tether_io_protocol)
set(TETHER_IO_PROTOCOL_TARGETS ${_variants})
set(TETHER_SLIPSPEED_LIBRARY slipspeed)
set(TETHER_SLIPSPEED_TARGETS ${_slip_variants})
