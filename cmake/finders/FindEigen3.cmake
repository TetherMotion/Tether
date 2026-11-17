# FindEigen3.cmake — satisfies find_package(Eigen3) (e.g. inside
# pinocchioConfig.cmake's find_dependency) using Tether's vendored Eigen
# submodule. Provides the Eigen3::Eigen imported interface target.

get_filename_component(_TETHER_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
set(_EIGEN_INC "${_TETHER_ROOT}/dependencies/eigen")

if(EXISTS "${_EIGEN_INC}/Eigen/src/Core/util/Macros.h")
    if(NOT TARGET Eigen3::Eigen)
        add_library(Eigen3::Eigen INTERFACE IMPORTED GLOBAL)
        set_target_properties(Eigen3::Eigen PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${_EIGEN_INC}")
    endif()
    set(Eigen3_FOUND TRUE)
    set(EIGEN3_FOUND TRUE)
    set(EIGEN3_INCLUDE_DIR "${_EIGEN_INC}")
    set(EIGEN3_INCLUDE_DIRS "${_EIGEN_INC}")
    # Extract version from Macros.h for find_dependency(Eigen3 x.y)
    file(STRINGS "${_EIGEN_INC}/Eigen/src/Core/util/Macros.h" _ver
         REGEX "#define EIGEN_(WORLD|MAJOR|MINOR)_VERSION")
    foreach(_v ${_ver})
        if(_v MATCHES "EIGEN_WORLD_VERSION ([0-9]+)")
            set(_w ${CMAKE_MATCH_1})
        elseif(_v MATCHES "EIGEN_MAJOR_VERSION ([0-9]+)")
            set(_maj ${CMAKE_MATCH_1})
        elseif(_v MATCHES "EIGEN_MINOR_VERSION ([0-9]+)")
            set(_min ${CMAKE_MATCH_1})
        endif()
    endforeach()
    if(DEFINED _w AND DEFINED _maj AND DEFINED _min)
        set(Eigen3_VERSION "${_w}.${_maj}.${_min}")
        set(EIGEN3_VERSION_STRING "${Eigen3_VERSION}")
    endif()
else()
    set(Eigen3_FOUND FALSE)
    if(Eigen3_FIND_REQUIRED)
        message(FATAL_ERROR "Vendored Eigen submodule not found at "
                            "${_EIGEN_INC}")
    endif()
endif()
