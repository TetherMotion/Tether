# Component: tether_terminal_ui
# Reusable ncurses terminal-UI building blocks (session lifecycle,
# navigable TreeView, captured LogPane, composed TreeScreen) shared by
# the interactive examples.
#
# Depends only on tether_common — the public headers keep ncurses out of
# sight (opaque TermWindow*), so the ncurses macros never leak into
# consumers.  Skipped silently when no curses library is present.

find_package(Curses)

if(NOT CURSES_FOUND)
    message(STATUS "ncurses not found; tether_terminal_ui disabled")
    set(TETHER_TERMINAL_UI_TARGETS "")
    return()
endif()

file(GLOB TETHER_TERMINAL_UI_SOURCES CONFIGURE_DEPENDS
    "${TETHER_ROOT}/src/terminal_ui/*.cpp")

set(_variants "")
if(TETHER_BUILD_SHARED_LIBS)
    add_library(tether_terminal_ui_shared SHARED ${TETHER_TERMINAL_UI_SOURCES})
    list(APPEND _variants tether_terminal_ui_shared)
endif()
if(TETHER_BUILD_STATIC_LIBS)
    add_library(tether_terminal_ui_static STATIC ${TETHER_TERMINAL_UI_SOURCES})
    list(APPEND _variants tether_terminal_ui_static)
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
            ${CURSES_INCLUDE_DIRS}
    )
    target_link_libraries(${_tgt} PUBLIC tether_common ${CURSES_LIBRARIES})
    target_compile_definitions(${_tgt} PUBLIC TETHER_HAS_TERMINAL_UI=1)
    set_target_properties(${_tgt} PROPERTIES
        POSITION_INDEPENDENT_CODE ON
        CXX_STANDARD 23
        CXX_STANDARD_REQUIRED ON
    )
endforeach()

if(TETHER_BUILD_SHARED_LIBS)
    add_library(tether_terminal_ui ALIAS tether_terminal_ui_shared)
    add_library(tether::terminal_ui ALIAS tether_terminal_ui_shared)
elseif(TETHER_BUILD_STATIC_LIBS)
    add_library(tether_terminal_ui ALIAS tether_terminal_ui_static)
    add_library(tether::terminal_ui ALIAS tether_terminal_ui_static)
endif()

set(TETHER_TERMINAL_UI_LIBRARY tether_terminal_ui)
set(TETHER_TERMINAL_UI_TARGETS ${_variants})
