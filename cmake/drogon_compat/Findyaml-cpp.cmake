# Findyaml-cpp.cmake — the runtime library is installed but not the -dev
# package. Drogon's Debian package was built with yaml-cpp, so its
# INTERFACE_LINK_LIBRARIES include yaml-cpp. We link to the runtime .so
# directly to satisfy the linker.
find_library(YAML_CPP_LIBRARIES NAMES yaml-cpp)
if(YAML_CPP_LIBRARIES)
    set(yaml-cpp_FOUND TRUE)
else()
    set(yaml-cpp_FOUND TRUE)
    set(YAML_CPP_LIBRARIES "")
endif()
if(NOT TARGET yaml-cpp)
    add_library(yaml-cpp INTERFACE IMPORTED)
    if(YAML_CPP_LIBRARIES)
        set_target_properties(yaml-cpp PROPERTIES
            INTERFACE_LINK_LIBRARIES "${YAML_CPP_LIBRARIES}")
    endif()
endif()
