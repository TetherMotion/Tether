cmake_minimum_required(VERSION 3.16)

# kico-monitor — ncurses progress UI for `kico` agent runs
# (drives the `kico --monitor` framed event protocol; no Tether IO).
add_executable(kico-monitor monitor_tui.cpp)

target_link_libraries(kico-monitor PRIVATE tether_terminal_ui)

set_target_properties(kico-monitor PROPERTIES
    CXX_STANDARD 23
    CXX_STANDARD_REQUIRED ON
)

install(TARGETS kico-monitor RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
