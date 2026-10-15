cmake_minimum_required(VERSION 3.16)

# tether_io_client_lib — shared logic for the CLI exe and the gtest module.
add_library(tether_io_client_lib STATIC
    client_session.cpp
    value_text.cpp
    commands.cpp
)

target_include_directories(tether_io_client_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(tether_io_client_lib PUBLIC
    tether_io_protocol
    tether_common
    Threads::Threads
)

# tether_io_client — the actual binary (argparse + shell + tui + main).
add_executable(tether_io_client
    main.cpp
    tui.cpp
)

target_link_libraries(tether_io_client PRIVATE
    tether_io_client_lib
    tether_io_protocol
    tether_common
    Threads::Threads
)

if(TARGET tether_terminal_ui)
    target_link_libraries(tether_io_client PRIVATE tether_terminal_ui)
    target_compile_definitions(tether_io_client PRIVATE TIO_HAS_TUI=1)
else()
    message(STATUS "tether_terminal_ui not built — tui command will be unavailable")
endif()

set_target_properties(tether_io_client tether_io_client_lib PROPERTIES
    CXX_STANDARD 23
    CXX_STANDARD_REQUIRED ON
)

install(TARGETS tether_io_client RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
