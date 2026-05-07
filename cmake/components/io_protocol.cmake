# Component: tether_io_protocol
# SLIP-based parameter/signal streaming protocol with transport layer

# ---------------------------------------------------------------------------
# libSLIPStream submodule
# ---------------------------------------------------------------------------
set(LIBSLIPSTREAM_DIR ${TETHER_ROOT}/components/libSLIPStream)
if(NOT EXISTS ${LIBSLIPSTREAM_DIR}/include)
    message(WARNING "libSLIPStream submodule not checked out at ${LIBSLIPSTREAM_DIR}; skipping tether_io_protocol")
    return()
endif()

# Build libSLIPStream as a static library
add_library(slipstream STATIC
    ${LIBSLIPSTREAM_DIR}/src/Buffer.cpp
    ${LIBSLIPSTREAM_DIR}/src/Decoder.cpp
    ${LIBSLIPSTREAM_DIR}/src/Encoder.cpp
)

target_include_directories(slipstream PUBLIC
    $<BUILD_INTERFACE:${LIBSLIPSTREAM_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

set_target_properties(slipstream PROPERTIES
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
    PUBLIC tether_common slipstream
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
set(TETHER_SLIPSTREAM_LIBRARY slipstream PARENT_SCOPE)
