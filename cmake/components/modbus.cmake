# Component: tether_modbus
# Modbus master/slave protocol stack for Tether.
#
# Subcomponents:
#   - protocol: Pdu, FunctionCode, ExceptionCode, Crc, BitPacker, Codec, Builder
#   - link:     ILink, SerialPortLink (termios/RS-485), TcpLink/TcpListener,
#               LoopbackLink, ScriptedLink, PtyPair; RtuTiming, framers, channels
#   - server:   DataStore, RequestDispatcher, RtuServer, TcpServer
#   - client:   IModbusInvoker, BusExecutor, TransactionManager, ModbusClient
#   - typed:    ScalarCodec, TypedRef, WordOrder
#   - robust:   assurance strategies (à-la-carte feature flags)
#   - sim:      SimBus/SimSlave/SimInvoker
#   - bridge:   Tether IO / Klipper adapters (optional)
#
# Feature flags (independent, à-la-carte — see MODBUS-PLAN.md §27):
#   TETHER_MODBUS_ASCII, TETHER_MODBUS_UDP
#   TETHER_MODBUS_ROBUST_READ_TWICE / THRICE_VOTE / QUORUM / MEDIAN /
#     UNTIL_STABLE / STICKY / WRITE_CONFIRM / WRITE_THRICE_CONFIRM /
#     WRITE_WATCHDOG / BCAST_FOLLOWUP / STATS
#
# Dependencies: common (Expected/error), io (optional transport adapters).

file(GLOB_RECURSE TETHER_MODBUS_SOURCES CONFIGURE_DEPENDS
    "${TETHER_ROOT}/src/modbus/*.cpp"
)

set(TETHER_MODBUS_SOURCES_FILTERED "")
foreach(src ${TETHER_MODBUS_SOURCES})
    if(EXISTS ${src})
        list(APPEND TETHER_MODBUS_SOURCES_FILTERED ${src})
    endif()
endforeach()

# Create variant targets
set(_variants "")
if(TETHER_BUILD_SHARED_LIBS)
    add_library(tether_modbus_shared SHARED ${TETHER_MODBUS_SOURCES_FILTERED})
    list(APPEND _variants tether_modbus_shared)
endif()
if(TETHER_BUILD_STATIC_LIBS)
    add_library(tether_modbus_static STATIC ${TETHER_MODBUS_SOURCES_FILTERED})
    list(APPEND _variants tether_modbus_static)
endif()

foreach(_tgt IN LISTS _variants)
    target_include_directories(${_tgt}
        PUBLIC
            $<BUILD_INTERFACE:${TETHER_ROOT}/include>
            $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether>
        PRIVATE
            $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/modbus>
    )

    target_compile_features(${_tgt} PRIVATE cxx_std_23)
    target_link_libraries(${_tgt} PUBLIC Threads::Threads)

    if(TARGET tether_common_static)
        target_link_libraries(${_tgt} PUBLIC tether_common_static)
    elseif(TARGET tether_common_shared)
        target_link_libraries(${_tgt} PUBLIC tether_common_shared)
    endif()

    # ---- à-la-carte robust-strategy feature flags --------------------------
    # Optional tether::io bridge: enabled only when the IO protocol
    # component is actually being built (component include order puts
    # io_protocol before modbus).
    if(TETHER_MODBUS_IO_BRIDGE)
        if(TARGET tether_io_protocol_static)
            target_link_libraries(${_tgt} PUBLIC tether_io_protocol_static)
            target_compile_definitions(${_tgt} PUBLIC TETHER_MODBUS_IO_BRIDGE=1)
        elseif(TARGET tether_io_protocol_shared)
            target_link_libraries(${_tgt} PUBLIC tether_io_protocol_shared)
            target_compile_definitions(${_tgt} PUBLIC TETHER_MODBUS_IO_BRIDGE=1)
        endif()
    endif()

    # Optional Klipper objects: klipper headers are interface-only for
    # the objects we implement (PrinterObject/GcodeLine).
    if(TARGET tether_klipper_shared OR TARGET tether_klipper_static)
        if(TARGET tether_klipper_static)
            target_link_libraries(${_tgt} PUBLIC tether_klipper_static)
        else()
            target_link_libraries(${_tgt} PUBLIC tether_klipper_shared)
        endif()
        target_compile_definitions(${_tgt} PUBLIC TETHER_MODBUS_KLIPPER=1)
    endif()

    foreach(_flag
            TETHER_MODBUS_ASCII
            TETHER_MODBUS_UDP
            TETHER_MODBUS_ESP_UART
            TETHER_MODBUS_ROBUST_READ_TWICE
            TETHER_MODBUS_ROBUST_THRICE_VOTE
            TETHER_MODBUS_ROBUST_QUORUM
            TETHER_MODBUS_ROBUST_MEDIAN
            TETHER_MODBUS_ROBUST_UNTIL_STABLE
            TETHER_MODBUS_ROBUST_STICKY
            TETHER_MODBUS_ROBUST_WRITE_CONFIRM
            TETHER_MODBUS_ROBUST_WRITE_THRICE_CONFIRM
            TETHER_MODBUS_ROBUST_WRITE_WATCHDOG
            TETHER_MODBUS_ROBUST_BCAST_FOLLOWUP
            TETHER_MODBUS_ROBUST_STATS)
        if(${_flag})
            target_compile_definitions(${_tgt} PUBLIC ${_flag}=1)
        endif()
    endforeach()
endforeach()

# Alias for examples/tests
if(TARGET tether_modbus_static AND NOT TARGET tether_modbus)
    add_library(tether_modbus ALIAS tether_modbus_static)
elseif(TARGET tether_modbus_shared AND NOT TARGET tether_modbus)
    add_library(tether_modbus ALIAS tether_modbus_shared)
endif()
