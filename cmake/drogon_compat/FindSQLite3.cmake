# Stub FindSQLite3.cmake for systems where Drogon's Debian package requires
# SQLite3 but it is not installed. Tether's HTTP server does not use SQLite3.
set(SQLite3_FOUND TRUE)
set(SQLite3_LIBRARIES "")
set(SQLite3_INCLUDE_DIRS "")
if(NOT TARGET SQLite::SQLite3)
    add_library(SQLite::SQLite3 INTERFACE IMPORTED)
endif()
