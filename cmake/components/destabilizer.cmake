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

# Create the destabilizer library
add_library(tether_destabilizer STATIC ${TETHER_DESTABILIZER_SOURCES_FILTERED})
add_library(tether::destabilizer ALIAS tether_destabilizer)

target_include_directories(tether_destabilizer
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

target_link_libraries(tether_destabilizer
    PUBLIC tether_common tether_simulation
)

set_target_properties(tether_destabilizer PROPERTIES
    POSITION_INDEPENDENT_CODE ON
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
)

set(TETHER_DESTABILIZER_LIBRARY tether_destabilizer PARENT_SCOPE)
