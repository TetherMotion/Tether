# Dummy Apps

These examples demonstrate how to consume individual parts of the Tether library in your own applications, avoiding the compilation of unused modules.

## How it works

When embedding Tether via CMake (`add_subdirectory(...)`), you can add the `EXCLUDE_FROM_ALL` parameter. This will instruct CMake to skip building all targets in Tether by default.

```cmake
add_subdirectory(path/to/Tether Tether EXCLUDE_FROM_ALL)
```

Then, you only link against the specific ALIAS target(s) your application needs:

```cmake
target_link_libraries(my_app PRIVATE tether::gcode)
```

CMake resolves the target dependencies, taking care of compiling **only** the `tether_gcode` library and its direct dependencies (like `tether_common`). Unrelated components (e.g. `tether_ethercat_master`) remain uncompiled, dramatically reducing your build times.
