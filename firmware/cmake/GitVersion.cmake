# Generate a version header from `git describe`, invoked at BUILD time via
#   cmake -DOUT=<path> -DSRC=<repo dir> -P cmake/GitVersion.cmake
# Writes OUT only when the string changes, so an unchanged version does not
# trigger a rebuild of everything that includes it. Falls back to "unknown" when
# git or the repo history is unavailable (e.g. a tarball build).
if(NOT DEFINED OUT)
    message(FATAL_ERROR "GitVersion.cmake: -DOUT=<path> is required")
endif()
if(NOT DEFINED SRC)
    set(SRC "${CMAKE_CURRENT_LIST_DIR}")
endif()

set(version "unknown")
find_package(Git QUIET)
if(GIT_FOUND)
    execute_process(
        COMMAND "${GIT_EXECUTABLE}" describe --tags --dirty --always
        WORKING_DIRECTORY "${SRC}"
        OUTPUT_VARIABLE version
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
        RESULT_VARIABLE rc
    )
    if(NOT rc EQUAL 0 OR version STREQUAL "")
        set(version "unknown")
    endif()
endif()

set(content
    "#pragma once\n// GENERATED at build time by cmake/GitVersion.cmake -- do not edit or commit.\nnamespace build {\ninline constexpr char kVersion[] = \"${version}\";\n} // namespace build\n"
)

set(old "")
if(EXISTS "${OUT}")
    file(READ "${OUT}" old)
endif()
if(NOT old STREQUAL content)
    file(WRITE "${OUT}" "${content}")
    message(STATUS "firmware version: ${version}")
endif()
