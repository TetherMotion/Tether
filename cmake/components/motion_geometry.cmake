# Component: tether_motion_geometry
# Lightweight geometry core: NurbsCurve, PiecewiseNurbsPath, Bernstein,
# PointCurveDistance. No dependency on the blend layer or the export
# layer, so it can be linked by both tether_export and
# tether_motion_planner without creating a cycle.

set(TETHER_MOTION_GEOMETRY_SOURCES
    ${TETHER_ROOT}/src/motion_planner/geometry/NurbsCurve.cpp
    ${TETHER_ROOT}/src/motion_planner/geometry/Bernstein.cpp
    ${TETHER_ROOT}/src/motion_planner/geometry/PointCurveDistance.cpp
    ${TETHER_ROOT}/src/motion_planner/geometry/PiecewiseNurbsPath.cpp
    ${TETHER_ROOT}/src/motion_planner/geometry/CertifiedCurvatureSampler.cpp
)

# Filter to only existing files
set(TETHER_MOTION_GEOMETRY_SOURCES_FILTERED "")
foreach(src ${TETHER_MOTION_GEOMETRY_SOURCES})
    if(EXISTS ${src})
        list(APPEND TETHER_MOTION_GEOMETRY_SOURCES_FILTERED ${src})
    endif()
endforeach()

# Create variant targets
set(_variants "")
if(TETHER_BUILD_SHARED_LIBS)
    add_library(tether_motion_geometry_shared SHARED ${TETHER_MOTION_GEOMETRY_SOURCES_FILTERED})
    list(APPEND _variants tether_motion_geometry_shared)
endif()
if(TETHER_BUILD_STATIC_LIBS)
    add_library(tether_motion_geometry_static STATIC ${TETHER_MOTION_GEOMETRY_SOURCES_FILTERED})
    list(APPEND _variants tether_motion_geometry_static)
endif()

foreach(_tgt IN LISTS _variants)
    target_include_directories(${_tgt}
        PUBLIC
            $<BUILD_INTERFACE:${TETHER_ROOT}/include>
            $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether>
            $<INSTALL_INTERFACE:include>
            $<INSTALL_INTERFACE:include/tether>
    )

    target_link_libraries(${_tgt} PUBLIC tether_common)

    set_target_properties(${_tgt} PROPERTIES
        POSITION_INDEPENDENT_CODE ON
        CXX_STANDARD 20
        CXX_STANDARD_REQUIRED ON
    )
endforeach()

if(TETHER_BUILD_SHARED_LIBS)
    add_library(tether_motion_geometry ALIAS tether_motion_geometry_shared)
    add_library(tether::motion_geometry ALIAS tether_motion_geometry_shared)
elseif(TETHER_BUILD_STATIC_LIBS)
    add_library(tether_motion_geometry ALIAS tether_motion_geometry_static)
    add_library(tether::motion_geometry ALIAS tether_motion_geometry_static)
endif()

set(TETHER_MOTION_GEOMETRY_LIBRARY tether_motion_geometry)
set(TETHER_MOTION_GEOMETRY_TARGETS ${_variants})
