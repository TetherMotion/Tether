cmake_minimum_required(VERSION 3.16)

# tether-chat — ncurses chat frontend for agent CLIs speaking the
# framed stdio protocol (`<exe> chat --rpc`; no Tether IO involved).
add_executable(tether-chat chat_tui.cpp)

target_link_libraries(tether-chat PRIVATE tether_terminal_ui)

set_target_properties(tether-chat PROPERTIES
    CXX_STANDARD 23
    CXX_STANDARD_REQUIRED ON
)

install(TARGETS tether-chat RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
