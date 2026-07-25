# Component: tether_identification
# System identification: step response, least-squares, frequency identification,
# friction identification, polynomial models, subspace identification, rigid-body
# identification, adaptive observers, nonlinear identification, advanced friction
# models, dense linear algebra.
#
# Split from tether_motion_control to keep the motion-control module focused on
# real-time motion generation and CiA 402 motion logic. Identification is fully
# self-contained (no #include of other tether/ headers outside its own subtree) —
# a true opt-in module.

file(GLOB_RECURSE TETHER_IDENTIFICATION_SOURCES CONFIGURE_DEPENDS
    "${TETHER_ROOT}/src/identification/*.cpp"
)

# Filter to only existing files
set(TETHER_IDENTIFICATION_SOURCES_FILTERED "")
foreach(src ${TETHER_IDENTIFICATION_SOURCES})
    if(EXISTS ${src})
        list(APPEND TETHER_IDENTIFICATION_SOURCES_FILTERED ${src})
    endif()
endforeach()

# Create variant targets
set(_variants "")
if(TETHER_BUILD_SHARED_LIBS)
    add_library(tether_identification_shared SHARED ${TETHER_IDENTIFICATION_SOURCES_FILTERED})
    list(APPEND _variants tether_identification_shared)
endif()
if(TETHER_BUILD_STATIC_LIBS)
    add_library(tether_identification_static STATIC ${TETHER_IDENTIFICATION_SOURCES_FILTERED})
    list(APPEND _variants tether_identification_static)
endif()

foreach(_tgt IN LISTS _variants)
    target_include_directories(${_tgt}
        PUBLIC
            $<BUILD_INTERFACE:${TETHER_ROOT}/include>
            $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether>
            $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/identification>
            $<INSTALL_INTERFACE:include>
            $<INSTALL_INTERFACE:include/tether>
            $<INSTALL_INTERFACE:include/tether/identification>
        PRIVATE
            ${TETHER_ROOT}/src
    )

    target_link_libraries(${_tgt} PUBLIC tether_common)

    set_target_properties(${_tgt} PROPERTIES
        POSITION_INDEPENDENT_CODE ON
        CXX_STANDARD 20
        CXX_STANDARD_REQUIRED ON
    )
endforeach()

if(TETHER_BUILD_SHARED_LIBS)
    add_library(tether_identification ALIAS tether_identification_shared)
    add_library(tether::identification ALIAS tether_identification_shared)
elseif(TETHER_BUILD_STATIC_LIBS)
    add_library(tether_identification ALIAS tether_identification_static)
    add_library(tether::identification ALIAS tether_identification_static)
endif()

set(TETHER_IDENTIFICATION_LIBRARY tether_identification)
set(TETHER_IDENTIFICATION_TARGETS ${_variants})
