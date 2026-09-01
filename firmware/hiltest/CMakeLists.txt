add_executable(
    hil_selftest
    selftest.cpp
    ${CMAKE_SOURCE_DIR}/src/core/angle.cpp
    ${CMAKE_SOURCE_DIR}/src/core/stats.cpp
    ${CMAKE_SOURCE_DIR}/src/core/csv.cpp
    ${CMAKE_SOURCE_DIR}/src/app/run_modes.cpp
)
target_link_libraries(hil_selftest PRIVATE pico_stdlib prober_hal)
# Observation over USB-CDC; no UART.
pico_enable_stdio_usb(hil_selftest 1)
pico_enable_stdio_uart(hil_selftest 0)
pico_add_extra_outputs(hil_selftest)

# MSC gate: brings up the custom TinyUSB CDC+MSC composite exposing the
# FAT volume. Uses prober_usb's own descriptors -- so NO pico_enable_stdio_usb here
# (that would collide with duplicate tud_descriptor_* symbols).
add_executable(hil_msc msc_main.cpp)
target_link_libraries(hil_msc PRIVATE pico_stdlib prober_hal prober_fatfs prober_usb)
pico_add_extra_outputs(hil_msc)

# Isolation gate: flashfs over the real flash diskio, observed via SDK USB-CDC
# (no USB-MSC). Diagnoses on-device flash write/read-back independent of MSC.
add_executable(hil_flashfs flashfs_main.cpp)
target_link_libraries(hil_flashfs PRIVATE pico_stdlib prober_fatfs)
pico_enable_stdio_usb(hil_flashfs 1)
pico_enable_stdio_uart(hil_flashfs 0)
pico_add_extra_outputs(hil_flashfs)

# Gate F: dual-core integration (multicore + SPSC + flush-to-flash + MSC). Uses
# the custom composite (prober_usb) -- no pico_enable_stdio_usb.
add_executable(
    hil_multicore
    multicore_main.cpp
    ${CMAKE_SOURCE_DIR}/src/core/angle.cpp
    ${CMAKE_SOURCE_DIR}/src/core/stats.cpp
    ${CMAKE_SOURCE_DIR}/src/core/csv.cpp
    ${CMAKE_SOURCE_DIR}/src/app/run_modes.cpp
)
target_link_libraries(
    hil_multicore
    PRIVATE pico_stdlib pico_multicore prober_hal prober_fatfs prober_usb
)
pico_add_extra_outputs(hil_multicore)

# Gate G: ramp run mode dry-run (continuous SoftPattern retime + per-spark RampRow
# + SPSC + ramp flush + MSC). Uses the custom composite (prober_usb).
add_executable(
    hil_ramp
    ramp_main.cpp
    ${CMAKE_SOURCE_DIR}/src/core/angle.cpp
    ${CMAKE_SOURCE_DIR}/src/core/csv.cpp
    ${CMAKE_SOURCE_DIR}/src/app/run_modes.cpp
)
target_link_libraries(
    hil_ramp
    PRIVATE pico_stdlib pico_multicore prober_hal prober_fatfs prober_usb
)
pico_add_extra_outputs(hil_ramp)

# Gate H (real ignitor): production core1 pipeline vs the live
# ignitor, auto-run stepped sweep -> MAP_H01.CSV over MSC. Links prober_app for
# app::core1_entry + g_ic.
add_executable(hil_ignitor ignitor_main.cpp)
target_link_libraries(
    hil_ignitor
    PRIVATE pico_stdlib pico_multicore prober_hal prober_app prober_fatfs prober_usb
)
pico_add_extra_outputs(hil_ignitor)
