# Component: tether_destabilizer
# Adversarial stability testing: perturbation search, instability metrics, optimizers

set(TETHER_DESTABILIZER_SOURCES
    ${TETHER_ROOT}/src/destabilizer/Perturbation.cpp
    ${TETHER_ROOT}/src/destabilizer/ConstraintProjector.cpp
    ${TETHER_ROOT}/src/destabilizer/InstabilityMetrics.cpp
    ${TETHER_ROOT}/src/destabilizer/Optimizers.cpp
    ${TETHER_ROOT}/src/destabilizer/DestabilizerEngine.cpp
    ${TETHER_ROOT}/src/destabilizer/DefaultLimits.cpp
)

# Filter to only existing files
set(TETHER_DESTABILIZER_SOURCES_FILTERED "")
foreach(src ${TETHER_DESTABILIZER_SOURCES})
    if(EXISTS ${src})
        list(APPEND TETHER_DESTABILIZER_SOURCES_FILTERED ${src})
    endif()
endforeach()

# Create variant targets
set(_variants "")
if(TETHER_BUILD_SHARED_LIBS)
    add_library(tether_destabilizer_shared SHARED ${TETHER_DESTABILIZER_SOURCES_FILTERED})
    list(APPEND _variants tether_destabilizer_shared)
endif()
if(TETHER_BUILD_STATIC_LIBS)
    add_library(tether_destabilizer_static STATIC ${TETHER_DESTABILIZER_SOURCES_FILTERED})
    list(APPEND _variants tether_destabilizer_static)
endif()

foreach(_tgt IN LISTS _variants)
    target_include_directories(${_tgt}
        PUBLIC
            $<BUILD_INTERFACE:${TETHER_ROOT}/include>
            $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether>
            $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/destabilizer>
            $<INSTALL_INTERFACE:include>
            $<INSTALL_INTERFACE:include/tether>
            $<INSTALL_INTERFACE:include/tether/destabilizer>
        PRIVATE
            ${TETHER_ROOT}/src
    )

    target_link_libraries(${_tgt} PUBLIC tether_common tether_simulation)

    set_target_properties(${_tgt} PROPERTIES
        POSITION_INDEPENDENT_CODE ON
        CXX_STANDARD 20
        CXX_STANDARD_REQUIRED ON
    )
endforeach()

if(TETHER_BUILD_SHARED_LIBS)
    add_library(tether_destabilizer ALIAS tether_destabilizer_shared)
    add_library(tether::destabilizer ALIAS tether_destabilizer_shared)
elseif(TETHER_BUILD_STATIC_LIBS)
    add_library(tether_destabilizer ALIAS tether_destabilizer_static)
    add_library(tether::destabilizer ALIAS tether_destabilizer_static)
endif()

set(TETHER_DESTABILIZER_LIBRARY tether_destabilizer)
set(TETHER_DESTABILIZER_TARGETS ${_variants})
