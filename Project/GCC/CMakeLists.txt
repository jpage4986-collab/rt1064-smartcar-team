# SeekFree RT1064 Library GCC Porting
# Copyright 2021 lookas <i@18kas.com>
# All rights reserved.
# Licensed under MIT 

# CROSS COMPILER SETTING
SET(CMAKE_SYSTEM_NAME Generic)
CMAKE_MINIMUM_REQUIRED(VERSION 3.1.1)

# THE VERSION NUMBER
SET(Tutorial_VERSION_MAJOR 1)
SET(Tutorial_VERSION_MINOR 0)

# ENABLE ASM
ENABLE_LANGUAGE(ASM)

SET(CMAKE_STATIC_LIBRARY_PREFIX)
SET(CMAKE_STATIC_LIBRARY_SUFFIX)

SET(CMAKE_EXECUTABLE_LIBRARY_PREFIX)
SET(CMAKE_EXECUTABLE_LIBRARY_SUFFIX)

# CURRENT DIRECTORY
SET(ProjDirPath ${CMAKE_CURRENT_SOURCE_DIR})

SET(EXECUTABLE_OUTPUT_PATH ${ProjDirPath}/${CMAKE_BUILD_TYPE})
SET(LIBRARY_OUTPUT_PATH ${ProjDirPath}/${CMAKE_BUILD_TYPE})

project(tutorial)

set(MCUX_SDK_PROJECT_NAME tutorial.elf)

# Keep the established standalone 0x70000000 build as the default. Wireless
# transport jobs explicitly opt into the BOOT2-compatible application layout.
option(SU7_WIRELESS_APP "Build application for wireless recovery loader" OFF)
option(SU7_WIRELESS_LOADER "Build resident wireless recovery loader" OFF)
if(SU7_WIRELESS_APP AND SU7_WIRELESS_LOADER)
    message(FATAL_ERROR "Select either wireless application or loader, not both")
endif()
set(RT1064_LINKER_SCRIPT "${ProjDirPath}/MIMXRT1064xxxxx_seekfree.ld")
if(SU7_WIRELESS_APP OR SU7_WIRELESS_LOADER)
    file(READ "${RT1064_LINKER_SCRIPT}" wireless_linker)
    function(su7_replace_once input token replacement output)
        string(REGEX MATCHALL "${token}" matches "${input}")
        list(LENGTH matches count)
        if(NOT count EQUAL 1)
            message(FATAL_ERROR "Expected one linker token '${token}', got ${count}")
        endif()
        string(REPLACE "${token}" "${replacement}" updated "${input}")
        set(${output} "${updated}" PARENT_SCOPE)
    endfunction()
    if(SU7_WIRELESS_APP)
        su7_replace_once("${wireless_linker}" "ORIGIN = 0x70000000" "ORIGIN = 0x70020000" wireless_linker)
        su7_replace_once("${wireless_linker}" "ORIGIN = 0x70001000" "ORIGIN = 0x70021000" wireless_linker)
        su7_replace_once("${wireless_linker}" "ORIGIN = 0x70002000" "ORIGIN = 0x70022000" wireless_linker)
        su7_replace_once("${wireless_linker}" "ORIGIN = 0x70002400" "ORIGIN = 0x70022400" wireless_linker)
        su7_replace_once("${wireless_linker}" "LENGTH = 0x003FDC00" "LENGTH = 0x001DDC00" wireless_linker)
        set(EXECUTABLE_OUTPUT_PATH "${ProjDirPath}/wireless-app")
        add_compile_definitions(SU7_WIRELESS_APP=1)
    else()
        su7_replace_once("${wireless_linker}" "LENGTH = 0x003FDC00" "LENGTH = 0x0001DC00" wireless_linker)
        set(EXECUTABLE_OUTPUT_PATH "${ProjDirPath}/wireless-loader")
        add_compile_definitions(SU7_WIRELESS_LOADER=1)
    endif()
    set(RT1064_LINKER_SCRIPT "${CMAKE_CURRENT_BINARY_DIR}/wireless.ld")
    file(CONFIGURE OUTPUT "${RT1064_LINKER_SCRIPT}" CONTENT "${wireless_linker}" @ONLY)
endif()

include(${ProjDirPath}/flags.cmake)

include(${ProjDirPath}/config.cmake)

file(GLOB SRCS
    "${ProjDirPath}/../USER/src/*.c"
)
if(SU7_WIRELESS_LOADER)
    set(SRCS "${ProjDirPath}/../tools/firmware/wireless_loader.c")
endif()

add_executable(${MCUX_SDK_PROJECT_NAME} 
    ${SRCS}
)
# These image planners must keep frame latency bounded in the debug build.
set_source_files_properties(
    "${ProjDirPath}/../USER/src/track_path12.c"
    "${ProjDirPath}/../USER/src/track_ring13.c"
    "${ProjDirPath}/../USER/src/track_ring14.c"
    "${ProjDirPath}/../USER/src/track_ring15.c"
    "${ProjDirPath}/../USER/src/track_mode16.c"
    PROPERTIES COMPILE_OPTIONS "-O2")
set_property(TARGET ${MCUX_SDK_PROJECT_NAME} APPEND PROPERTY LINK_DEPENDS
    "${RT1064_LINKER_SCRIPT}")

target_include_directories(${MCUX_SDK_PROJECT_NAME} PRIVATE
    ${ProjDirPath}/../USER/inc
)

include(${ProjDirPath}/files.cmake)

TARGET_LINK_LIBRARIES(${MCUX_SDK_PROJECT_NAME} PRIVATE -Wl,--start-group)

target_link_libraries(${MCUX_SDK_PROJECT_NAME} PRIVATE m)

target_link_libraries(${MCUX_SDK_PROJECT_NAME} PRIVATE c)

target_link_libraries(${MCUX_SDK_PROJECT_NAME} PRIVATE gcc)

target_link_libraries(${MCUX_SDK_PROJECT_NAME} PRIVATE nosys)

target_link_libraries(${MCUX_SDK_PROJECT_NAME} PRIVATE
    ${ProjDirPath}/../vendor/seekfree-camera-config/zf_device_config.lib)

TARGET_LINK_LIBRARIES(${MCUX_SDK_PROJECT_NAME} PRIVATE -Wl,--end-group)
