# Stub FindUUID.cmake for systems where uuid dev headers are not installed.
# Drogon links libuuid at build time. With modern glibc, uuid functions are
# often available in libc or libuuid. This stub provides an empty target.
find_library(UUID_LIBRARIES NAMES uuid)
if(UUID_LIBRARIES)
    set(UUID_FOUND TRUE)
else()
    set(UUID_FOUND TRUE)
    set(UUID_LIBRARIES "")
endif()
if(NOT TARGET UUID::UUID)
    add_library(UUID::UUID INTERFACE IMPORTED)
    if(UUID_LIBRARIES)
        set_target_properties(UUID::UUID PROPERTIES INTERFACE_LINK_LIBRARIES "${UUID_LIBRARIES}")
    endif()
endif()
