cmake_minimum_required(VERSION 3.13)

# Two-mode build: host (native, unit tests) vs pico (arm, firmware).
# Selected by -DBUILD_TARGET=host|pico (default host so core/ TDD is the default path).
set(BUILD_TARGET "host" CACHE STRING "host | pico")

if(BUILD_TARGET STREQUAL "host")
    project(prober_host CXX)
    enable_language(C) # vendored FatFs (external/fatfs/ff.c) is C
    enable_testing()
    add_subdirectory(tests)
else()
    include(pico_sdk_import.cmake)
    project(prober C CXX ASM)
    # Use our directly-vendored TinyUSB (external/tinyusb) instead of the SDK's
    # nested lib/tinyusb, so no recursive submodule fetch is needed. Keep the
    # external/tinyusb pin in sync with the pico-sdk version (see .gitmodules).
    set(PICO_TINYUSB_PATH ${CMAKE_CURRENT_LIST_DIR}/external/tinyusb)
    pico_sdk_init()
    add_subdirectory(src)

    # HIL self-test firmware (target-only bring-up gates over USB-CDC). Off by
    # default so normal .uf2 builds are unaffected; enable with -DPROBER_HILTEST=ON.
    option(PROBER_HILTEST "Build HIL self-test firmware" OFF)
    if(PROBER_HILTEST)
        add_subdirectory(hiltest)
    endif()
endif()
