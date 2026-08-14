# Stub FindFilesystem.cmake — with C++23 and GCC 13+, std::filesystem is
# part of libstdc++ and does not need a separate library. This stub
# provides the std::filesystem target that Drogon's CMake config expects.
set(Filesystem_FOUND TRUE)
if(NOT TARGET std::filesystem)
    add_library(std::filesystem INTERFACE IMPORTED)
endif()
set(FS_LIBRARY "")
set(FS_FOUND TRUE)
