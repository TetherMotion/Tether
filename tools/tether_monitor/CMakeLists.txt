cmake_minimum_required(VERSION 3.16)

# tether-monitor — ncurses progress UI for agent CLIs speaking the
# framed event protocol (`<exe> --monitor <args...>`; no Tether IO).
add_executable(tether-monitor monitor_tui.cpp)

target_link_libraries(tether-monitor PRIVATE tether_terminal_ui)

set_target_properties(tether-monitor PROPERTIES
    CXX_STANDARD 23
    CXX_STANDARD_REQUIRED ON
)

install(TARGETS tether-monitor RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
