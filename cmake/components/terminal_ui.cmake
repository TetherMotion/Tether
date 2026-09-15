# Component: tether_terminal_ui
# Reusable ncurses terminal-UI building blocks (session lifecycle,
# navigable TreeView, captured LogPane, composed TreeScreen) shared by
# the interactive examples.
#
# Depends only on tether_common — the public headers keep ncurses out of
# sight (opaque TermWindow*), so the ncurses macros never leak into
# consumers.  Skipped silently when no curses library is present.

set(CURSES_NEED_NCURSES TRUE CACHE BOOL "Require ncurses" FORCE)
set(CURSES_NEED_WIDE TRUE CACHE BOOL "Require wide/Unicode ncurses" FORCE)

# FindCurses caches CURSES_*_LIBRARY paths.  If an earlier configure ran
# without CURSES_NEED_WIDE (e.g. before this component was enabled, or in
# a parent project that called find_package(Curses) first), the cache
# still points at the narrow library — which cannot emit UTF-8 (every
# multibyte char is rendered as M-b~T~@ escapes).  Detect the stale
# result, drop it, and re-search once.
if(CURSES_NCURSES_LIBRARY AND
   NOT CURSES_NCURSES_LIBRARY MATCHES "(ncursesw|cursesw)" AND
   NOT TETHER_CURSES_WIDE_RETRIED)
    message(STATUS "tether_terminal_ui: cached CURSES_NCURSES_LIBRARY="
                   "${CURSES_NCURSES_LIBRARY} is the narrow build — "
                   "re-searching for the wide (UTF-8-capable) library")
    set(TETHER_CURSES_WIDE_RETRIED TRUE CACHE INTERNAL
        "curses re-search after CURSES_NEED_WIDE was enabled")
    unset(CURSES_NCURSES_LIBRARY CACHE)
    unset(CURSES_CURSES_LIBRARY CACHE)
    unset(CURSES_FORM_LIBRARY CACHE)
    unset(CURSES_INCLUDE_PATH CACHE)
    unset(CURSES_HAVE_CURSES_H CACHE)
    unset(CURSES_HAVE_NCURSES_H CACHE)
    unset(CURSES_HAVE_NCURSES_NCURSES_H CACHE)
    unset(CURSES_HAVE_NCURSES_CURSES_H CACHE)
endif()
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
