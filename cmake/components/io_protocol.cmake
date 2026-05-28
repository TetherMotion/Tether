# Component: tether_io_protocol
# SLIP-based parameter/signal streaming protocol with transport layer

# ---------------------------------------------------------------------------
# libSLIPspeed submodule
# ---------------------------------------------------------------------------
set(LIBSLIPSPEED_DIR ${TETHER_ROOT}/components/libSLIPspeed)
if(NOT EXISTS ${LIBSLIPSPEED_DIR}/include)
    message(WARNING "libSLIPspeed submodule not checked out at ${LIBSLIPSPEED_DIR}; skipping tether_io_protocol")
    return()
endif()

# Build libSLIPspeed as a static library
add_library(slipspeed STATIC
    ${LIBSLIPSPEED_DIR}/src/Buffer.cpp
    ${LIBSLIPSPEED_DIR}/src/Decoder.cpp
    ${LIBSLIPSPEED_DIR}/src/Encoder.cpp
)

target_include_directories(slipspeed PUBLIC
    $<BUILD_INTERFACE:${LIBSLIPSPEED_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

set_target_properties(slipspeed PROPERTIES
    POSITION_INDEPENDENT_CODE ON
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
)

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
    )
endif()

# Create the IO protocol library
add_library(tether_io_protocol STATIC ${TETHER_IO_PROTOCOL_SOURCES})
add_library(tether::io_protocol ALIAS tether_io_protocol)

target_include_directories(tether_io_protocol
    PUBLIC
        $<BUILD_INTERFACE:${TETHER_ROOT}/include>
        $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether>
        $<INSTALL_INTERFACE:include>
        $<INSTALL_INTERFACE:include/tether>
    PRIVATE
        ${TETHER_ROOT}/src
)

target_link_libraries(tether_io_protocol
    PUBLIC tether_common slipspeed
)

# Pthreads for Server/Session threading
find_package(Threads REQUIRED)
target_link_libraries(tether_io_protocol PUBLIC Threads::Threads)

set_target_properties(tether_io_protocol PROPERTIES
    POSITION_INDEPENDENT_CODE ON
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
)

# Export for other components
set(TETHER_IO_PROTOCOL_LIBRARY tether_io_protocol PARENT_SCOPE)
set(TETHER_SLIPSPEED_LIBRARY slipspeed PARENT_SCOPE)
