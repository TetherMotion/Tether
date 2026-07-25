# Component: tether_autotuning
# Controller autotuning framework: classical methods (Ziegler-Nichols, Cohen-Coon,
# Tyreus-Luyben, Chien-Hrones-Reswick, Lopez, Lambda, SIMC, AMIGO, Astrom-Hagglund),
# model-based methods (IMC, pole placement, loop shaping, direct synthesis, Smith
# predictor, Dahlin, deadbeat, minimum-variance), hybrid methods, industrial
# autotuners, LQR tuning, mu-synthesis, QFT, sliding mode, optimization, adaptive.
#
# Split from tether_controls to keep the core controllers module lean.
# Autotuning is fully self-contained (no #include of other tether/ headers
# outside its own subtree) — a true opt-in module.

file(GLOB_RECURSE TETHER_AUTOTUNING_SOURCES CONFIGURE_DEPENDS
    "${TETHER_ROOT}/src/control/autotuning/*.cpp"
)

# Exclude monolithic stub files whose implementations have been split into
# individual files under classical/ and model_based/. These stubs have
# incomplete namespace openings and would cause compile errors; the real
# implementations live in the per-method .cpp files.
list(FILTER TETHER_AUTOTUNING_SOURCES EXCLUDE REGEX "ClassicalTuningMethods\\.cpp$")
list(FILTER TETHER_AUTOTUNING_SOURCES EXCLUDE REGEX "ModelBasedMethods\\.cpp$")

# Filter to only existing files
set(TETHER_AUTOTUNING_SOURCES_FILTERED "")
foreach(src ${TETHER_AUTOTUNING_SOURCES})
    if(EXISTS ${src})
        list(APPEND TETHER_AUTOTUNING_SOURCES_FILTERED ${src})
    endif()
endforeach()

# Create variant targets
set(_variants "")
if(TETHER_BUILD_SHARED_LIBS)
    add_library(tether_autotuning_shared SHARED ${TETHER_AUTOTUNING_SOURCES_FILTERED})
    list(APPEND _variants tether_autotuning_shared)
endif()
if(TETHER_BUILD_STATIC_LIBS)
    add_library(tether_autotuning_static STATIC ${TETHER_AUTOTUNING_SOURCES_FILTERED})
    list(APPEND _variants tether_autotuning_static)
endif()

foreach(_tgt IN LISTS _variants)
    target_include_directories(${_tgt}
        PUBLIC
            $<BUILD_INTERFACE:${TETHER_ROOT}/include>
            $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether>
            $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/control>
            $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/control/autotuning>
            $<INSTALL_INTERFACE:include>
            $<INSTALL_INTERFACE:include/tether>
            $<INSTALL_INTERFACE:include/tether/control>
            $<INSTALL_INTERFACE:include/tether/control/autotuning>
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
    add_library(tether_autotuning ALIAS tether_autotuning_shared)
    add_library(tether::autotuning ALIAS tether_autotuning_shared)
elseif(TETHER_BUILD_STATIC_LIBS)
    add_library(tether_autotuning ALIAS tether_autotuning_static)
    add_library(tether::autotuning ALIAS tether_autotuning_static)
endif()

set(TETHER_AUTOTUNING_LIBRARY tether_autotuning)
set(TETHER_AUTOTUNING_TARGETS ${_variants})
