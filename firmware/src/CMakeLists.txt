# Reusable hal static library so both `prober` and the `hiltest` harness can
# link the same GPIO + PIO capture code. The pickup is emitted on the CPU via the
# header-only SoftPattern (hal/soft_pattern.hpp); there is no PIO pattern generator.
add_library(prober_hal STATIC hal/gpio.cpp hal/capture.cpp core/capture_decode.cpp)
pico_generate_pio_header(prober_hal ${CMAKE_CURRENT_LIST_DIR}/hal/capture.pio)

# `src` on the include path so hal/*.cpp resolve "hal/..." and "core/..."
# includes the same way the host unit tests do.
target_include_directories(prober_hal PUBLIC ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(prober_hal PUBLIC pico_stdlib hardware_pio hardware_dma)

# FatFs (ELM ChaN) + the portable facade + the flash diskio backend. Kept in
# its own lib so the vendored C sources compile with their own warning posture
# and the RAM-disk backend (diskio_ram.cpp) stays host-only.
add_library(
    prober_fatfs
    STATIC
    hal/flashfs.cpp
    hal/diskio_flash.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../external/fatfs/ff.c
    ${CMAKE_CURRENT_LIST_DIR}/../external/fatfs/ffsystem.c
)
target_include_directories(
    prober_fatfs
    PUBLIC ${CMAKE_CURRENT_LIST_DIR} ${CMAKE_CURRENT_LIST_DIR}/../external/fatfs
)
target_link_libraries(prober_fatfs PUBLIC pico_stdlib hardware_flash hardware_sync)

# TinyUSB CDC+MSC composite: custom descriptors + MSC callbacks that
# expose the FatFs flash volume as a USB drive. Kept in its own lib so the
# tusb_config.h include dir and TinyUSB link deps are scoped to USB consumers
# (e.g. the hil_msc gate) and never leak into `prober`/`hil_selftest`, which use
# the SDK's stdio_usb and would otherwise get duplicate USB descriptors.
add_library(prober_usb STATIC hal/usb_descriptors.c hal/usb_msc.cpp)
target_include_directories(
    prober_usb
    PUBLIC ${CMAKE_CURRENT_LIST_DIR} ${CMAKE_CURRENT_LIST_DIR}/hal # hal/ for tusb_config.h
)
target_link_libraries(prober_usb PUBLIC tinyusb_device tinyusb_board prober_fatfs)

# Core-1 deterministic pipeline + the pure core/ math and app/ schedule
# it reuses. core1_main.cpp is TARGET-only (PIO/DMA/multicore); the pure core/ and
# app/run_modes sources are the same ones the host unit tests compile.
add_library(
    prober_app
    STATIC
    app/core1_main.cpp
    app/run_modes.cpp
    core/angle.cpp
    core/stats.cpp
    core/csv.cpp
    core/tach.cpp
    # Core-0 pure logic used by the super-loop: FSM + config load +
    # CSV/result serialization. All SDK-free (same sources the host tests build).
    core/fsm.cpp
    core/fileio.cpp
    core/config.cpp
    core/ini.cpp
    core/parse.cpp
)
target_include_directories(prober_app PUBLIC ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(prober_app PUBLIC pico_stdlib pico_multicore prober_hal)

# Core-0 super-loop: main.cpp drives the FSM/USB/flash; flashfs_backend
# bridges core/fileio to the flash FAT. prober_usb brings the custom CDC+MSC
# composite (the flash volume exposed to the host) -- prober does NOT enable
# pico stdio_usb, so there is no duplicate-descriptor collision.
add_executable(prober app/main.cpp hal/flashfs_backend.cpp)
target_link_libraries(
    prober
    PRIVATE pico_stdlib pico_multicore prober_app prober_hal prober_fatfs prober_usb
)

# Embedded firmware version (git describe). Generated into the build dir (never
# committed) by cmake/GitVersion.cmake: once now so the header exists for the
# first compile / clangd, and again on every build via gen_version (write-if-
# changed, so it only forces a rebuild when the version actually changes).
set(VERSION_HPP ${CMAKE_BINARY_DIR}/generated/version.hpp)
execute_process(
    COMMAND
        ${CMAKE_COMMAND} -DOUT=${VERSION_HPP} -DSRC=${CMAKE_SOURCE_DIR} -P
        ${CMAKE_SOURCE_DIR}/cmake/GitVersion.cmake
)
add_custom_target(
    gen_version
    BYPRODUCTS ${VERSION_HPP}
    COMMAND
        ${CMAKE_COMMAND} -DOUT=${VERSION_HPP} -DSRC=${CMAKE_SOURCE_DIR} -P
        ${CMAKE_SOURCE_DIR}/cmake/GitVersion.cmake
    COMMENT "Embedding firmware version (git describe)"
    VERBATIM
)
add_dependencies(prober gen_version)
target_include_directories(prober PRIVATE ${CMAKE_BINARY_DIR}/generated)

# Bound RAM-hot `.time_critical` code at link time.
# The override incl brackets `*(.time_critical*)` (folded into `.data` by
# pico-sdk 2.x) with __time_critical_start__/__time_critical_end__; the extra
# `-T` fragment asserts the 12 KB cap so oversized RAM-hot code fails the build.
pico_add_linker_script_override_path(prober ${CMAKE_CURRENT_LIST_DIR}/../linker)
target_link_options(
    prober
    PRIVATE "LINKER:-T,${CMAKE_CURRENT_LIST_DIR}/../linker/prober_sections.ld"
)
set_property(
    TARGET prober
    APPEND
    PROPERTY
        LINK_DEPENDS
            ${CMAKE_CURRENT_LIST_DIR}/../linker/prober_sections.ld
            ${CMAKE_CURRENT_LIST_DIR}/../linker/section_default_data.incl
)

pico_add_extra_outputs(prober) # produces prober.uf2
